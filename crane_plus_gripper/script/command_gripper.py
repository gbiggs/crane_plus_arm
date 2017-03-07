#!/usr/bin/env python

"""Control the width that a gripper is open."""

import sys

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


states = {2: 'Preempted', 3: 'Succeeded', 4: 'Aborted'}


def _feedback_cb(feedback):
    print('[Feedback] Position: {}'.format(feedback.position))
    print('[Feedback] Effort: {}'.format(feedback.effort))
    print('[Feedback] Stalled: {}'.format(feedback.stalled))
    print('[Feedback] Reached goal: {}'.format(feedback.reached_goal))


def main():
    """Main function."""
    rospy.init_node('gripper_command_client')

    args = rospy.myargv(argv=sys.argv)
    if len(args) != 2:
        print('Usage: {} <width>\n\tWhere <width> is a value in '
              'metres'.format(args[0]))
        return 1

    client = actionlib.SimpleActionClient('crane_plus_gripper',
                                          GripperCommandAction)
    client.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.position = float(args[1])
    client.send_goal(goal, feedback_cb=_feedback_cb)

    client.wait_for_result()
    print('[Result] State: {}'.format(states[client.get_state()]))
    print('[Result] Status: {}'.format(client.get_goal_status_text()))
    print('[Result] Position: {}'.format(client.get_result().position))
    print('[Result] Effort: {}'.format(client.get_result().effort))
    print('[Result] Stalled: {}'.format(client.get_result().stalled))
    print('[Result] Reached goal: {}'.format(client.get_result().reached_goal))


if __name__ == '__main__':
    sys.exit(main())
