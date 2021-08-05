#!/usr/bin/env python 

import rospy
from mavros_msgs.msg import MountControl
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class Gimbal_test():
    def __init__(self):

        self.message_pub = rospy.Publisher("/mavros/mount_control/command", MountControl, queue_size=10)
        self.mount_control = MountControl()
   

    def run(self, pitch_in):
       
        #Populate MountControl message
        self.mount_control.header.stamp = rospy.Time.now()  
        self.mount_control.mode = 2
        self.mount_control.pitch = pitch_in
        self.mount_control.roll = 0
        self.mount_control.yaw = 0

        #Publish MountControl message to move gimbal as desired
        self.message_pub.publish(self.mount_control)


if __name__ == '__main__':

    rospy.init_node('gimbal_test', anonymous=True, log_level= rospy.INFO)
    gt = Gimbal_test()
    gt.run(-90)
    rospy.sleep(1)
    gt.run(-90)
    rospy.sleep(1)
