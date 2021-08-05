#!/usr/bin/env python
#imports
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

rospy.init_node('landing_target_publisher')

pose = PoseStamped() #create PoseStamped objects
transformData = TransformStamped() #created TransformStamped object
tfBuffer = tf2_ros.Buffer() #created Buffer object
listener = tf2_ros.TransformListener(tfBuffer) #created TransformListener object
marker_name = rospy.get_param("landing_target_pub/marker_id", "ar_marker_105")
camera_frame_id = rospy.get_param("landing_target_pub/camera_frame", "cgo3_camera_link")

#X and Y are swapped since ar_track_alvar axes are swapped


#callback for AlvarMarkers message
def callback(data):
    if len(data.markers) == 0:
        pass
    else:
        #Do transformation of message here
        try:
            transformData = tfBuffer.lookup_transform(camera_frame_id, marker_name, rospy.Time(0)) 
           
            pose.pose.position.x = -float(transformData.transform.translation.y)
            pose.pose.position.y = -float(transformData.transform.translation.x)
            pose.pose.position.z = -float(transformData.transform.translation.z)


            pose.header.frame_id = 'landing_target'
            pose.header.stamp = rospy.Time.now()

            pub.publish(pose)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)    # subscribe to ar marker topic
pub = rospy.Publisher('/mavros/landing_target/pose', PoseStamped, queue_size=10)    # publish pose relative to landing target
rate = rospy.Rate(50)

while not rospy.is_shutdown():
    rospy.spin()
