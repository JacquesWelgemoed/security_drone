#!/usr/bin/env python

#imports
import rospy
import mavros
from geometry_msgs.msg import PoseStamped


new_pos_pub = rospy.Publisher('mavros/setpoint_position/local/new', PoseStamped, queue_size=10)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 0

def update_pos(pose):
    print('Enter desired x: ')
    pose.pose.position.x = input()
    print('Enter desired y: ')
    pose.pose.position.y = input()
    print('Enter desired z: ')
    pose.pose.position.z = input()


def talker():
    new_pos_pub = rospy.Publisher('mavros/setpoint_position/local/new', PoseStamped, queue_size=10)
    rospy.init_node('new_poses', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        update_pos(pose)
        new_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
