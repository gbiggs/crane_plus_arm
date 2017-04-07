#!/usr/bin/env python

"""Controller node for the CRANE+ gripper."""

import sys

import rospy
import rosgraph
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64


def goal_achieved(current, goal, delta):
    """Return True if the value of current is within delta of goal."""
    diff = abs(goal - current)
    reached = False
    if diff <= delta:
        reached = True
    rospy.logdebug('Goal reach check:\n\tCurrent position: {}\n\tGoal: '
                   '{}\n\tDifference: {}\n\tDelta {}\n\tReached: '
                   '{}'.format(current, goal, diff, delta, reached))
    return reached


class GripperActionServer:
    """The action server that handles gripper commands."""

    def __init__(self,
                 servo_name,
                 movement_radius,
                 closed_angle,
                 open_angle,
                 timeout,
                 delta,
                 overload_limit):
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
            open_angle (float): The angle in radians that the gripper servo
                is commanded to move to when the gripper is fully open.
            timeout (float): The maximum time to wait for the gripper to reach
                the goal position.
            delta (float): The allowable error in the gripper's position, in
                metres.
            overload_limit (float): The maximum current to allow the servo to
                draw; when this is exceeded, the gripper is assumed to have
                stalled due to a blockage and the servo is commanded to its
                current position to stop it moving.

        """
        self._movement_radius = movement_radius
        self._closed_angle = closed_angle
        self._open_angle = open_angle
        self._timeout = timeout
        self._delta = delta
        self._overload = overload_limit

        rospy.loginfo(
            'CRANE+ gripper action server configuration:\n' +
            '\tMovement radius: {} m\n'.format(self._movement_radius) +
            '\tClosed angle: {} rad\n'.format(self._closed_angle) +
            '\tOpen angle: {} rad\n'.format(self._open_angle) +
            '\tMovement timeout: {} s\n'.format(self._timeout) +
            '\tPosition error delta: {} m\n'.format(self._delta) +
            '\tOverload limit: {} A'.format(self._overload))

        self._as = actionlib.SimpleActionServer(
            rosgraph.names.ns_join(rospy.get_name(), 'gripper_command'),
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
        rospy.loginfo('Gripper action server waiting for goals')

    def _state_to_msg(self, state, msg):
        msg.position = self.angle_to_width(state.current_pos)
        msg.effort = state.load
        if goal_achieved(
                self.angle_to_width(state.current_pos),
                self._goal.command.position,
                self._delta) and \
           not state.is_moving:
            msg.stalled = False
            msg.reached_goal = True
        else:
            msg.stalled = not state.is_moving
            msg.reached_goal = False
        return msg

    def _handle_command(self):
        self._goal = self._as.accept_new_goal()
        if self._as.is_preempt_requested():
            self._handle_preempt()
            return
        # Calculate the servo angle for the desired position
        self._goal_angle = self.width_to_angle(self._goal.command.position)
        # Range check the goal angle
        if self._goal_angle < self._open_angle or \
           self._goal_angle > self._closed_angle:
            rospy.logwarn(
                'Gripper commanded to {} rad, which is outside the '
                'allowable range of [{}, {}] rad'.format(
                    self._goal_angle,
                    self._open_angle,
                    self._closed_angle))
            # TODO: Switch to ActionServer to allow rejecting goals
            self._as.set_aborted(text='Out of range')
            return
        rospy.loginfo('Gripper commanded to {} m; setting servo to {} '
                      'rad'.format(self._goal.command.position, self._goal_angle))
        # Start the gripper servo moving to the goal position
        self._command_pub.publish(self._goal_angle)
        # Set up the timer to prevent running forever
        self._timer = rospy.Timer(rospy.Duration(self._timeout),
                                  self._timed_out,
                                  oneshot=True)

    def _handle_preempt(self):
        if self._timer:
            self._timer.shutdown()
            self._timer = None
        rospy.loginfo('Gripper movement preempted')
        if not self._last_state:
            self._last_state = JointState()
        result = GripperCommandResult()
        self._as.set_preempted(
            result=self._state_to_msg(self._last_state, result),
            text='Preempted')

    def _timed_out(self, te):
        rospy.loginfo('Gripper movement timed out')
        self._timer = None
        if not self._last_state:
            self._last_state = JointState()
        result = GripperCommandResult()
        self._as.set_aborted(
            result=self._state_to_msg(self._last_state, result),
            text='Timed out')

    def _state_update(self, state):
        if not self._as.is_active():
            return
        rospy.logdebug('Got servo state update:\n{}'.format(state))
        self._last_state = state
        feedback = GripperCommandFeedback()
        self._state_to_msg(state, feedback)
        self._as.publish_feedback(feedback)
        if feedback.reached_goal:
            rospy.loginfo('Determined from state update that gripper goal has '
                          'been reached')
            if self._timer:
                self._timer.shutdown()
                self._timer = None
            result = GripperCommandResult()
            self._as.set_succeeded(
                result=self._state_to_msg(state, result), text='Reached goal')
        elif feedback.stalled:
            rospy.loginfo('Determined from state update that gripper has '
                          'stalled')
            if self._timer:
                self._timer.shutdown()
                self._timer = None
            result = GripperCommandResult()
            self._as.set_aborted(
                result=self._state_to_msg(state, result), text='Stalled')
        elif abs(state.load) > self._overload:
            rospy.loginfo('Determined from state update that gripper is '
                          'blocked (overload protection)')
            if self._timer:
                self._timer.shutdown()
                self._timer = None
            self._command_pub.publish(state.current_pos)
            result = GripperCommandResult()
            self._as.set_aborted(
                result=self._state_to_msg(state, result), text='Blocked')

    def width_to_angle(self, width):
        """Calculate the angle to achieve a distance between the fingers."""
        # The gripper servo ranges from -0.6 rad to 0.65 rad, which is a width
        # range of 108 mm to 0 mm.
        # However, because one finger moves around a revolute joint while the other
        # is fixed, the distance is not linearly proportionate to the angle.
        # We must calculate it using the length of the segment between the two
        # points, taking the circle as the rotation of the moving finger about the
        # servo's axis.
        theta = width / self._movement_radius
        # Theta must be shifted to the gripper servo's coordinate frame
        return -1 * (theta - self._closed_angle)

    def angle_to_width(self, theta):
        """Calculate the distance between the fingers for a servo angle."""
        # See the comments for servo_angle_for_width for the calculation method,
        # which this function uses the inverse of.
        # The given angle must be shifted to a range where zero is the gripper
        # closed position
        return self._movement_radius * abs(theta - self._closed_angle)


def main():
    """Main function."""
    rospy.init_node('crane_plus_gripper')
    servo_name = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'servo_name'),
        'finger_servo_controller')
    # The radius of the circle drawn by the moving finger's tip is 93 mm
    movement_radius = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'movement_radius'),
        0.093)
    # The angle where the gripper is fully closed is 0.65 rad by default
    closed_angle = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'servo_closed_angle'),
        0.65)
    # The angle where the gripper is fully open is -0.6 rad by default
    open_angle = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'servo_open_angle'),
        -0.6)
    # Default timeout of 5 seconds
    timeout = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'timeout'),
        5)
    # Default error delta of 1 mm
    delta = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'delta'),
        0.001)
    # Overload protection limit
    overload_limit = rospy.get_param(
        rosgraph.names.ns_join(rospy.get_name(), 'overload_limit'),
        0.9)

    server = GripperActionServer(servo_name,
                                 movement_radius,
                                 closed_angle,
                                 open_angle,
                                 timeout,
                                 delta,
                                 overload_limit)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
