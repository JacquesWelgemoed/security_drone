#!/usr/bin/env python
import os
import rospy
from mavros_test_common import MavrosTestCommon
from mission_gen.srv import alarm, alarmRequest, alarmResponse
from mission.msg import MissionState
from mavros_msgs.msg import Waypoint, CommandCode
from mavros_msgs.srv import WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix

def handle_generate_mission(req):
    
    # Instantiate MavrosTestCommon object to use helper methods
    mavros = MavrosTestCommon()
    mavros.wait_for_topics(5)

    #Get current GPS Location
    home_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
    
    # Create list of waypoint to pass to flight controller
    wpList = []
    

    # Create Waypoints
    takeoff = Waypoint()
    takeoff.frame = Waypoint.FRAME_GLOBAL_REL_ALT#6
    takeoff.command = CommandCode.NAV_TAKEOFF#22
    takeoff.is_current = True
    takeoff.autocontinue = True
    takeoff.param1 = 15.0
    takeoff.param2 = 0.0
    takeoff.param3 = 0.0
    takeoff.param4 = float('nan')
    takeoff.x_lat = home_gps.latitude
    takeoff.y_long = home_gps.longitude
    takeoff.z_alt = 10

    goal = Waypoint()
    goal.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    goal.command = CommandCode.NAV_WAYPOINT
    goal.is_current = False
    goal.autocontinue = True    #Set to False if this is last waypoint
    goal.param1 = 0.0 
    goal.param2 = 0.0
    goal.param3 = 0.0
    goal.param4 = float('nan')
    goal.x_lat = req.latitude
    goal.y_long = req.longitude
    goal.z_alt = 10
    
    goal_speed = Waypoint()
    goal_speed.frame = Waypoint.FRAME_MISSION
    goal_speed.command = CommandCode.DO_CHANGE_SPEED
    goal_speed.is_current = False
    goal_speed.param1 = 1.0
    goal_speed.param2 = 10.0
    goal_speed.param3 = -1.0
    goal_speed.param4 = 0.0
    goal_speed.x_lat = 0.0
    goal_speed.y_long = 0.0
    goal_speed.z_alt = 0.0
    

    # Populate Waypoint list with mission 
    wpList.append(takeoff)
    wpList.append(goal)
    wpList.append(goal_speed)

    # Clear waypoints on Flight controller
    mavros.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mavros.send_wps(wpList, 5)

    # TODO: Write Mission to text file ?
    
    
    #Check if number of waypoint in flight controller matches number of waypoints in waypoint list and launch main mission execution node
    if(checked):
        msg = MissionState()
        msg.ExecutionState = "run"
        msg.alarmLatitude = req.latitude
        msg.alarmLongitude = req.longitude

        mission_state_pub.publish(msg)
        print("returning True")
        return alarmResponse(True) 
    else:
        print("returning false")
        return alarmResponse(False)
        

    

def generateMission():
    rospy.init_node('alarm_server')
    s = rospy.Service('generate_mission', alarm, handle_generate_mission)
    print("Ready to respond to alarms")
    rospy.spin()


if __name__ == "__main__":
    mission_state_pub = rospy.Publisher('mission/state', MissionState, queue_size=10)
    generateMission()