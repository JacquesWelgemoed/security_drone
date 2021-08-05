#!/usr/bin/env python
#imports
import roslib
import time
import rospy
import tf2_ros
import mavros
import math
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

rospy.init_node('landing_node')

pose = PoseStamped() #create PoseStamped objects
prev_pose = PoseStamped()
transformData = TransformStamped() #created TransformStamped object
tfBuffer = tf2_ros.Buffer() #created Buffer object
listener = tf2_ros.TransformListener(tfBuffer) #created TransformListener object
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
stop = True


#callback for AlvarMarkers message
def callback(data):
    global stop
    if len(data.markers) == 0:
        stop = True
    else:
        stop = False

def old_pos_cb(old_pose):
    prev_pose.pose.position.x = old_pose.pose.position.x
    prev_pose.pose.position.y = old_pose.pose.position.y
    prev_pose.pose.position.z = old_pose.pose.position.z
    prev_pose.pose.orientation.x = old_pose.pose.orientation.x
    prev_pose.pose.orientation.y = old_pose.pose.orientation.y
    prev_pose.pose.orientation.z = old_pose.pose.orientation.z
    prev_pose.pose.orientation.w = old_pose.pose.orientation.w

def land():
    old_pos_sub = rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, old_pos_cb)
    new_pos_pub = rospy.Publisher('mavros/setpoint_position/local/new', PoseStamped, queue_size=10)
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback) #subscriber ar marker
    rate = rospy.Rate(10)
       
    
#####TEST
    while not rospy.is_shutdown():
        try:
            transformData = tfBuffer.lookup_transform("typhoon_h480__cgo3_camera_link", "ar_marker_0", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        #if no new marker is found don't move
        if stop:
            pose.pose.position.x = prev_pose.pose.position.x
            pose.pose.position.y = prev_pose.pose.position.y
            pose.pose.position.z = prev_pose.pose.position.z
            pose.pose.orientation.x = prev_pose.pose.orientation.x
            pose.pose.orientation.y = prev_pose.pose.orientation.y
            pose.pose.orientation.z = prev_pose.pose.orientation.z
            pose.pose.orientation.w = prev_pose.pose.orientation.w

        else:
            if((float(transformData.transform.translation.x)) * (float(transformData.transform.translation.y)) > 0 ):
                pose.pose.position.x =  prev_pose.pose.position.x - (float(transformData.transform.translation.x))
                pose.pose.position.y = prev_pose.pose.position.y - (float(transformData.transform.translation.y)) 
            else:
                pose.pose.position.x =  prev_pose.pose.position.x + (float(transformData.transform.translation.x))
                pose.pose.position.y = prev_pose.pose.position.y + (float(transformData.transform.translation.y)) 
            
            if(abs((float(transformData.transform.translation.x))) < 0.1 and abs((float(transformData.transform.translation.y))) < 0.1):
                pose.pose.position.z = prev_pose.pose.position.z - (float(transformData.transform.translation.z))

            else:
                pose.pose.position.z = prev_pose.pose.position.z

            if((float(transformData.transform.translation.z)) < 1):
                set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
             
            pose.pose.orientation.x = (float(transformData.transform.rotation.x)) 
            pose.pose.orientation.y = (float(transformData.transform.rotation.y)) 
            pose.pose.orientation.z = (float(transformData.transform.rotation.z)) 
            pose.pose.orientation.w = (float(transformData.transform.rotation.w)) 

        #xy_dist = math.sqrt((float(transformData.transform.translation.x))**2 + (float(transformData.transform.translation.y))**2)
        new_pos_pub.publish(pose)
        time.sleep(10)

#######TEST
if __name__ == '__main__':
    
    try:
        land()

    except rospy.ROSInterruptException:
        pass
