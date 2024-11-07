#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_state():
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['xPrismatic1', 'yPrismatic', 'zPrismatic']
    joint_state.position = [-0.4, 0.1, 0.2]
    joint_state.velocity = []
    joint_state.effort = []

    while not rospy.is_shutdown():
      joint_state.header.stamp = rospy.Time.now()
      pub.publish(joint_state)
      rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_state()
    except rospy.ROSInterruptException:
        pass