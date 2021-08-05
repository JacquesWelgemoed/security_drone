#!/usr/bin/env python
#imports
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers


#callback for AlvarMarkers message
def marker_callback(data):
    data.header.stamp = rospy.Time.now()
    if len(data.markers) == 0:
        return
    else:
        if(rospy.Time.now() - data.header.stamp > rospy.Duration(2.0)):
            return
        publish_vpe(data)


def publish_zero(t_event):
    global got_local_pos
    
    if(t_event.current_real - vpe_pose.header.stamp < pub_zero_timeout):
        return
    if(t_event.current_real - local_pose.header.stamp < pub_zero_timeout):
        if got_local_pos.is_zero():
            got_local_pos = t_event.current_real

        if(t_event.current_real - got_local_pos > pub_zero_duration):
            return

    else:
        got_local_pos = rospy.Time(0)
        
    rospy.loginfo_throttle(10, 'Publishing Zero')
    zero = PoseStamped()
    zero.header.frame_id = global_frame
    zero.header.stamp = t_event.current_real
    zero.pose.orientation.w = 1
    vpe_pub.publish(zero)
    
    
def publish_vpe(data):
   global vpe_pose
   try:
       # Get vpe pose from tf
       tf = TransformStamped()
       tf = tf_buffer.lookup_transform(marker_frame, mav_frame, data.header.stamp, rospy.Duration(0.02))
       
       vpe_pose.pose.position.x = tf.transform.translation.x
       vpe_pose.pose.position.y = tf.transform.translation.y
       vpe_pose.pose.position.z = tf.transform.translation.z
       vpe_pose.pose.orientation = tf.transform.rotation
       vpe_pose.header.stamp = rospy.Time.now()

        # apply offset of map to marker to get mav position in map frame
       vpe_offset = PoseStamped()
       vpe_offset = tf2_geometry_msgs.do_transform_pose(vpe_pose, offset)
       
       
       vpe = PoseStamped()
       vpe.pose.position.x = -vpe_offset.pose.position.x
       vpe.pose.position.y = vpe_offset.pose.position.y
       vpe.pose.position.z = -vpe_offset.pose.position.z
       vpe.pose.orientation = vpe_offset.pose.orientation
       vpe.header.frame_id = global_frame
       vpe.header.stamp = rospy.Time.now()
       vpe_pub.publish(vpe)
   except:
       pass
    

def local_pos_cb(data):
    global local_pose
    local_pose = data
    local_pose.header.stamp = rospy.Time.now()
         

#######TEST
if __name__ == '__main__':
    
    rospy.init_node('vpe_publisher')
    
    global_frame = 'map'
    marker_frame = 'ar_marker_105' #4
    mav_frame = 'base_link'	#camera_linkbase_link
    got_local_pos = rospy.Time(0)
    
    vpe_pose = PoseStamped() #create PoseStamped objects
    local_pose = PoseStamped()
    transform_data = TransformStamped() #created TransformStamped object
    stop = True
    
    
    tf_buffer = tf2_ros.Buffer() #created Buffer object
    tf_listener = tf2_ros.TransformListener(tf_buffer) #created TransformListener object
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    offset = tf_buffer.lookup_transform(global_frame, marker_frame, rospy.Time(0), rospy.Duration(0.1))
    offset.child_frame_id = marker_frame
    tf_broadcaster.sendTransform(offset)
    
    
    vpe_pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, marker_callback) #subscriber ar marker
    local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_pos_cb)
    
    #Still uncertain about this part
    zero_timer = rospy.Timer(rospy.Duration(0.1),publish_zero)
    pub_zero_timeout = rospy.Duration(5.0)
    pub_zero_duration = rospy.Duration(1.0)
    
    
    
    rospy.spin()    # Node should keep running and waiting for callbacks
    
        
