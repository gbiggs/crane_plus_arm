#!/usr/bin/env python

"""Joint state publishing node for the CRANE+ manipulator."""

import sys

import rospy

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


def publish_virtual_joint_cb_maker(joint_name, pub):
    """Make a callback for publishing a zero'd JointState for a given joint."""
    def publisher(event):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [joint_name]
        js.position = [0]
        js.velocity = [0]
        js.effort = [0]
        pub.publish(js)
    return publisher


def main():
    """Main function."""
    rospy.init_node('crane_plus_joint_state_publisher')

    try:
        servo_controllers = rospy.get_param('~servo_controller_names')
    except KeyError:
        rospy.logerr('Configuration error: Could not find list of servo '
                     'controllers on the parameter server')
        return 1
    namespace = rospy.get_param('namespace', '')
    publish_virtual_joint = rospy.get_param('~publish_virtual_joint', True)

    if not servo_controllers:
        rospy.logerr('No servo controller names specified; nothing to publish')
        return 1

    js_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    servo_subs = []
    for s in servo_controllers:
        servo_subs.append(subscribe_to_servo_controller(s, namespace, js_pub))

    if publish_virtual_joint:
        # Publish the virtual gripper joint regularly
        rospy.Timer(
            rospy.Duration(1),
            publish_virtual_joint_cb_maker(publish_virtual_joint, js_pub))

    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
