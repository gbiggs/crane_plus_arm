#!/usr/bin/env python

"""Joint state publishing node for the CRANE+ manipulator."""

import sys

import rospy
import rosgraph.names

from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as DynamixelJointState


def joint_state_cb(msg, js_publisher):
    """Callback that converts a Dynamixel JointState to a standard JointState."""
    js = JointState()
    js.header.stamp = msg.header.stamp
    js.name = [msg.name]
    js.position = [msg.current_pos]
    js.velocity = [msg.velocity]
    js.effort = [msg.load]
    js_publisher.publish(js)


def subscribe_to_servo_controller(joint, namespace, js_publisher):
    """Subscribe to the state topic for the specified servo controller."""
    topic_name = '/'.join([namespace, joint, 'state'])
    return rospy.Subscriber(topic_name,
                            DynamixelJointState,
                            callback=joint_state_cb,
                            callback_args=js_publisher)


def main():
    """Main function."""
    rospy.init_node('crane_plus_joint_state_publisher')

    try:
        servo_controllers = rospy.get_param(
            rosgraph.names.ns_join(rospy.get_name(), 'servo_controller_names'))
    except KeyError:
        rospy.logerr('Configuration error: Could not find list of servo '
                     'controllers on the parameter server')
        return 1
    namespace = rospy.get_param('namespace', '')

    if not servo_controllers:
        rospy.logerr('No servo controller names specified; nothing to publish')
        return 1

    js_pub = rospy.Publisher('joint_states', JointState)

    servo_subs = []
    for s in servo_controllers:
        servo_subs.append(subscribe_to_servo_controller(s, namespace, js_pub))

    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())