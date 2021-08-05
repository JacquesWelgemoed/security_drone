#!/usr/bin/env python
#imports
import roslib
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

rospy.init_node('marker_follower')

distance = Twist() #created Twist object
transformData = TransformStamped() #created TransformStamped object
tfBuffer = tf2_ros.Buffer() #created Buffer object
listener = tf2_ros.TransformListener(tfBuffer) #created TransformListener object
stop = True

#callback for AlvarMarkers message
def callback(data):
    global stop
    if len(data.markers) == 0:
        stop = True
    else:
        stop = False

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) #publish velocity commands
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback) #subscriber ar marker
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            transformData = tfBuffer.lookup_transform("base_footprint", "ar_marker_0", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        #if no new marker is found, stop the turtlebot
        if stop:
            distance.linear.x = 0
            distance.angular.z = 0
        else:
            distance.linear.x = ((int(transformData.transform.translation.z * 100)/100.0) - 1)
            distance.angular.z = -(int(transformData.transform.translation.x * 100)/50.0)
        
        pub.publish(distance)
        rate.sleep()