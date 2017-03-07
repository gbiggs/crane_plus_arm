#!/usr/bin/env python

"""Controller node for the CRANE+ gripper."""

from math import asin, sin
import sys

import rospy
import rosgraph
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64


def goal_achieved(current, goal, delta):
    """Return True if the value of current is within delta of goal."""
    print('Goal check: current {} goal {} delta {}'.format(current, goal,
        delta))
    if abs(goal - current) <= delta:
        print('Returning true')
        return True
    return False


class GripperActionServer:
    """The action server that handles gripper commands."""

    def __init__(self,
                 servo_name,
                 movement_radius,
                 closed_angle,
                 timeout,
                 delta):
        """Initialise the gripper action server.

        Args:
            servo_name (str): The name of the servo controlling the gripper.
                This will be used as the namespace for the topics for that
                servo. For example, if the name is given as
                "gripper_servo_controller", this node will publish to the
                "gripper_servo_controller/command" topic and subscribe to the
                "gripper_servo_controller/state" topic.
            movement_radius (float): The radius of the circle described by the
                tip of the moving finger. This circle is centred on the gripper
                servo's axle.
            closed_angle (float): The angle in radians that the gripper servo
                is commanded to move to when the gripper is fully closed. This
                is required to shift between the absolute angle of the finger
                and the servo's coordinate frame.
            timeout (float): The maximum time to wait for the gripper to reach
                the goal position.
            delta (float): The allowable error in the gripper's position, in
                metres.

        """
        self._movement_radius = movement_radius
        self._closed_angle = closed_angle
        self._timeout = timeout
        self._delta = delta

        self._as = actionlib.SimpleActionServer('crane_plus_gripper',
                                                GripperCommandAction,
                                                auto_start=False)
        self._as.register_goal_callback(self._handle_command)
        self._as.register_preempt_callback(self._handle_preempt)

        self._command_pub = rospy.Publisher(
            rosgraph.names.ns_join(servo_name, 'command'),
            Float64,
            queue_size=1)
        self._state_sub = rospy.Subscriber(
            rosgraph.names.ns_join(servo_name, 'state'),
            JointState, self._state_update)
        self._timer = None
        self._last_state = None

        self._as.start()

    def _last_state_as_result(self):
        result = GripperCommandResult()
        result.position = self.angle_to_width(self._last_state.current_pos)
        result.effort = self._last_state.load
        if goal_achieved(self._last_state.current_pos,
                         self._last_state.goal_pos,
                         self._delta):
            result.stalled = False
            result.reached_goal = True
        else:
            result.stalled = True if self._last_state.velocity == 0 else False
            result.reached_goal = False

    def _handle_command(self):
        self._timer = rospy.Timer(rospy.Duration(self._timeout),
                                  self._timed_out,
                                  oneshot=True)

        goal = self._as.accept_new_goal()
        if self._as.is_preempt_requested():
            self._handle_preempt()
            return
        # Calculate the servo angle for the desired position
        goal_angle = self.width_to_angle(goal.command.position)
        rospy.loginfo('Gripper command to {} mm; setting servo to {} '
                      'rad'.format(goal.command.position, goal_angle))
        # Start the gripper servo moving to the goal position
        self._command_pub.publish(goal_angle)

    def _handle_preempt(self):
        if self._timer:
            self._timer.shutdown()
            self._timer = None
        rospy.loginfo('Gripper movement preempted')
        self._as.set_preempted(result=self._last_state_as_result())

    def _timed_out(self, te):
        rospy.loginfo('Gripper movement timed out ({})'.format(te))
        result = GripperCommandResult()
        result.position = self._feedback.position
        result.effort = self._feedback.effort
        self._as.set_aborted(result=self._last_state_as_result(),
                             text='Timed out')

    def _state_update(self, state):
        if not self._as.is_active():
            return
        rospy.loginfo('Got servo state update:\n{}'.format(state))
        self._last_state = state
        feedback = GripperCommandFeedback()
        feedback.position = self.angle_to_width(state.current_pos)
        feedback.effort = state.load
        if goal_achieved(state.current_pos, state.goal_pos, self._delta):
            feedback.stalled = False
            feedback.reached_goal = True
        else:
            feedback.stalled = True if state.velocity == 0 else False
            feedback.reached_goal = False
        self._as.publish_feedback(feedback)
        if feedback.reached_goal:
            if self._timer:
                self._timer.shutdown()
                self._timer = None
            self._as.set_succeeded(result=self._last_state_as_result(),
                                   text='Reached goal')

    def width_to_angle(self, width):
        """Calculate the angle to achieve a distance between the fingers."""
        # The gripper servo ranges from -0.6 rad to 0.65 rad, which is a width
        # range of 108 mm to 0 mm.
        # However, because one finger moves around a revolute joint while the other
        # is fixed, the distance is not linearly proportionate to the angle.
        # We must calculate it using the length of the chord between the two
        # points, taking the circle as the rotation of the moving finger about the
        # servo's axis.
        theta = 2 * asin(width / (2 * self._movement_radius))
        # Theta must be shifted to the gripper servo's coordinate frame
        return -1 * (theta - self._closed_angle)

    def angle_to_width(self, theta):
        """Calculate the distance between the fingers for a servo angle."""
        # See the comments for servo_angle_for_width for the calculation method,
        # which this function uses the inverse of.
        # The given angle must be shifted to a range where zero is the gripper
        # closed position
        return 2 * self._movement_radius * sin(abs(theta - self._closed_angle) / 2.0)


def main():
    """Main function."""
    rospy.init_node('crane_plus_gripper')
    servo_name = rospy.get_param(rosgraph.names.ns_join(rospy.get_name(),
                                                        'servo_name'),
                                 'gripper_servo_controller')
    # The radius of the circle drawn by the moving finger's tip is 93 mm
    movement_radius = rospy.get_param(rosgraph.names.ns_join(rospy.get_name(),
                                                             'movement_radius'),
                                      0.093)
    # The angle where the gripper is fully closed is 0.65 rad
    closed_angle = rospy.get_param(rosgraph.names.ns_join(rospy.get_name(),
                                                          'servo_closed_angle'),
                                   0.65)
    # Default timeout of 10 seconds
    timeout = rospy.get_param(rosgraph.names.ns_join(rospy.get_name(),
                                                     'timeout'),
                              10)
    # Default error delta of 1 mm
    delta = rospy.get_param(rosgraph.names.ns_join(rospy.get_name(),
                                                   'delta'),
                            0.001)

    server = GripperActionServer(servo_name,
                                 movement_radius,
                                 closed_angle,
                                 timeout,
                                 delta)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
