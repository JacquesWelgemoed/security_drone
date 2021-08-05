#!/usr/bin/env python

import rospy
from mavros_offboard_posctl_test import MavrosOffboardPosctlTest
from ar_track_alvar_msgs.msg import AlvarMarkers
from mission.msg import MissionState
from geometry_msgs.msg import PoseStamped


def marker_cb(msg):
    global marker_x, marker_y, marker_z
    if(len(msg.markers) != 0):
        marker_x = msg.markers[0].pose.pose.position.y # X and Y axes are swapped to work in local_position frame 
        marker_y = msg.markers[0].pose.pose.position.x
        marker_z = msg.markers[0].pose.pose.position.z

def mission_state_cb(msg):
    global running
    if msg.ExecutionState == "precision_land":
        running = True

def local_pose_cb(msg):
    global x_pos, y_pos, z_pos
    x_pos = msg.pose.position.x
    y_pos = msg.pose.position.y
    z_pos = msg.pose.position.z

def precision_land():
    '''Sends position commands to land on fiduical marker, commanded position are offset from local_position/pose.'''
    global running, x_pos, y_pos, z_pos

    # Create instance of position control test to use offboard position mode
    mavros = MavrosOffboardPosctlTest()
    mavros.wait_for_topics(10)
    
    mavros.set_mode("OFFBOARD", 5)
    # Move to point 5 meters above center of fiducial marker board
    mavros.reach_position((x_pos-marker_x), (y_pos-marker_y), 5, 0, 30)
    rospy.sleep(2)
    # Move to point 1 meter above center of fiducial marker board
    mavros.reach_position((x_pos-marker_x), (y_pos-marker_y), 1, 0, 30)
    
    # Change flight mode to land mode so that landing is finished by flight controller
    mavros.set_mode("AUTO.LAND", 10)

def main():
    global running
    # Initalise ROS node
    rospy.init_node('precision_landing')
    # Create all necessary publishers and subscribers
    marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, marker_cb)
    local_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_pose_cb)
    local_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    mission_state_sub = rospy.Subscriber('mission/state', MissionState, mission_state_cb)

    # Wait for Mission state message to call for a precision landing
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if(running):
            print("Executing Precision Landing")
            precision_land()
            running = False
        rate.sleep()
    


if __name__ == "__main__":
    
    # Instantiating all global variables to keep track of local_position/pose and pose relative to fiducial markers.
    x_pos = 0
    y_pos = 0
    z_pos = 0

    marker_x = 0
    marker_y = 0
    marker_z = 0

    running = False
    main()
    

