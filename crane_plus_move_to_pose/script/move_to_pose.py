#!/usr/bin/env python

"""Move the end effector to a given pose."""

import argparse
import sys

import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        'Move the end effector to a given pose.')
    parser.add_argument(
        '-x',
        type=float,
        default=0,
        help='x-axis coordinate of the desired pose')
    parser.add_argument(
        '-y',
        type=float,
        default=0,
        help='y-axis coordinate of the desired pose')
    parser.add_argument(
        '-z',
        type=float,
        default=0,
        help='z-axis coordinate of the desired pose')
    parser.add_argument(
        '-r',
        '--roll',
        type=float,
        default=0,
        help='roll of the desired pose')
    parser.add_argument(
        '-p',
        '--pitch',
        type=float,
        default=0,
        help='pitch of the desired pose')
    parser.add_argument(
        '-a',
        '--yaw',
        type=float,
        default=0,
        help='yaw of the desired pose')
    parser.add_argument(
        '-g',
        '--group',
        type=str,
        default='arm',
        help='name of the move group to command')
    parser.add_argument(
        '-f',
        '--ref-frame',
        type=str,
        default='world',
        help='reference frame for the pose')
    my_args = rospy.myargv()[1:]
    args = parser.parse_args(my_args)

    moveit_commander.roscpp_initialize(my_args)
    rospy.init_node('move_to_pose')
    arm = moveit_commander.MoveGroupCommander(args.group)
    arm.set_pose_reference_frame(args.ref_frame)

    orientation = Quaternion(*tf.transformations.quaternion_from_euler(
        args.roll,
        args.pitch,
        args.yaw))
    pose = Pose(Point(args.x, args.y, args.z), orientation)
    rospy.loginfo('[move_to_pose] Moving arm to ' + str(pose))
    arm.set_pose_target(pose)
    arm.go()


if __name__ == '__main__':
    sys.exit(main())
