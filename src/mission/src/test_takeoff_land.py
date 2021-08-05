#!/usr/bin/env python
import rospy
from mission_test import MavrosMissionTest
from mission.msg import MissionState
from mavros_msgs.msg import Waypoint, CommandCode
from mavros_msgs.srv import WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64

def wait_for_mission(timeout, should_loiter):

    global allow_loiter
    loop_rate = 10
    rate = rospy.Rate(loop_rate)
    elapsed = 0

    # Waits for UAV to reach alarm location (Final Waypoint)
    while not rospy.is_shutdown(): #and (elapsed<timeout)
        rospy.sleep(2)
        if(mavros.mission_item_reached == (len(mavros.mission_wp.waypoints)-1)):
            print("Final Waypoint reached")
            if should_loiter:
                allow_loiter = True
                cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
                state_msg = MissionState()
                state_msg.alarmLatitude = cur_gps.latitude
                state_msg.alarmLongitude = cur_gps.longitude    
                state_msg.ExecutionState = "loiter"
                state_pub.publish(state_msg)
                rate.sleep()
            break
        rate.sleep()
        elapsed = elapsed + (1/loop_rate)


def loiter_commands_cb(msg):
    '''Receives loiter command messages and calls associated methods'''

    if allow_loiter:
        if ((msg.data == "left") or (msg.data == "right")):
            print("Doing yaw control")
            if(msg.data == "left"):
                do_yaw_ctl(False)
            else:
                do_yaw_ctl(True)
        
        elif msg.data == "return":
            land()



def yaw_mission(angle):
    '''Creates a waypoint with the current location's gps coordinated, 
    only changing the heading direction to coincide with the angle parameter'''

    wpList = []

    cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
    altitude = rospy.wait_for_message('mavros/global_position/rel_alt', Float64, timeout=5)

    loiter_wp = Waypoint()
    loiter_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    loiter_wp.command = CommandCode.NAV_WAYPOINT
    loiter_wp.is_current = True
    loiter_wp.autocontinue = False    #Set to False if this is last waypoint
    loiter_wp.param1 = 0.0 
    loiter_wp.param2 = 0.0
    loiter_wp.param3 = 0.0
    loiter_wp.param4 = angle
    loiter_wp.x_lat = cur_gps.latitude
    loiter_wp.y_long = cur_gps.longitude
    loiter_wp.z_alt = altitude.data

    wpList.append(loiter_wp)

    # Clear waypoints on Flight controller
    mavros.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mavros.send_wps(wpList, 5)
    
    if(checked):
        if (mavros.state.mode == "AUTO.MISSION"):
            print("Setting Mission Mode")
            mavros.set_mode("AUTO.MISSION", 10)
    else:
        rospy.logerr("Failed to send return to home waypoints to flight controller")


def do_yaw_ctl(clockwise):
    '''Updates the heading setpoint that is passed to the yaw_mission method'''
    print("Getting Yaw message")
    yaw = rospy.wait_for_message('mavros/global_position/compass_hdg', Float64, timeout=5)
    print("YAW: {}".format(yaw.data))
    if(clockwise):
        yaw_in = yaw.data + float(30.0)
        
    else:
        yaw_in = yaw.data - float(30.0)        
    print("Sending yaw: {}".format(yaw_in))
    yaw_mission(yaw_in)

def takeoff():
    '''Creates and executes a mission containing only a takeoff command, copter will loiter at that position until commanded otherwise'''

    global latitude,longitude

    #mavros.wait_for_topics(5)

    #Get current GPS Location
    home_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
    latitude = home_gps.latitude
    longitude = home_gps.longitude
    # Create list of waypoint to pass to flight controller
    wpList = []
    
    # Create Waypoints
    takeoff = Waypoint()
    takeoff.frame = Waypoint.FRAME_GLOBAL_REL_ALT#6
    takeoff.command = CommandCode.NAV_TAKEOFF#22
    takeoff.is_current = False
    takeoff.autocontinue = True
    takeoff.param1 = 15.0
    takeoff.param2 = 0.0
    takeoff.param3 = 0.0
    takeoff.param4 = float('nan')
    takeoff.x_lat = home_gps.latitude
    takeoff.y_long = home_gps.longitude
    takeoff.z_alt = 3
    
    takeoff2 = Waypoint()
    takeoff2.frame = Waypoint.FRAME_GLOBAL_REL_ALT#6
    takeoff2.command = CommandCode.NAV_WAYPOINT#22
    takeoff2.is_current = False
    takeoff2.autocontinue = False
    takeoff2.param1 = 0.0
    takeoff2.param2 = 0.1
    takeoff2.param3 = 0.0
    takeoff2.param4 = float('nan')
    takeoff2.x_lat = home_gps.latitude
    takeoff2.y_long = home_gps.longitude
    takeoff2.z_alt = 5

    wpList.append(takeoff)
    wpList.append(takeoff2)

    # Clear waypoints on Flight controller
    mavros.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mavros.send_wps(wpList, 5)

    if (checked):
        
        print("Authorize UAV to take off ?")
        user_input = raw_input("Type y to confirm or n to deny:")

        if(user_input != "y" and user_input != "Y"):    #TODO: Add timeout here
            print("You denied takeoff")
            return 
        else: 
            mavros.set_mode("AUTO.MISSION", 10)
            mavros.set_arm(True, 5)

    wait_for_mission(240, True)


def land():

    print("Do you want to land ?")
    user_input = raw_input("Type y to confirm or n to deny:")

    while(not user_input == "y" and not user_input == "Y"):
        print("Do you want to land ?")
        user_input = raw_input("Type y to confirm or n to deny:")
    else:
        print("You approved landing")

    # Create list of waypoint to pass to flight controller
    wpList = []
    
    # Create Waypoints
    landing = Waypoint()
    landing.frame = Waypoint.FRAME_GLOBAL_REL_ALT#6
    landing.command = CommandCode.NAV_WAYPOINT
    landing.is_current = False
    landing.autocontinue = True
    landing.param1 = 0.0
    landing.param2 = 0.1    # Acceptance radius in meters
    landing.param3 = 0.0
    landing.param4 = float('nan')
    landing.x_lat = latitude
    landing.y_long = longitude
    landing.z_alt = 5

    landing2 = Waypoint()
    landing2.frame = Waypoint.FRAME_GLOBAL_REL_ALT#6
    landing2.command = CommandCode.NAV_WAYPOINT
    landing2.is_current = False
    landing2.autocontinue = False
    landing2.param1 = 0.0
    landing2.param2 = 0.1    # Acceptance radius in meters
    landing2.param3 = 0.0
    landing2.param4 = float('nan')
    landing2.x_lat = latitude
    landing2.y_long = longitude
    landing2.z_alt = 3

    wpList.append(landing)
    wpList.append(landing2)

    # Clear waypoints on Flight controller
    mavros.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mavros.send_wps(wpList, 5)

    if (checked):
        if (mavros.state.mode == "AUTO.MISSION"):
            print("Setting Mission Mode")
            mavros.set_mode("AUTO.MISSION", 10)
    
    wait_for_mission(240, False)
    if (mavros.state.mode == "AUTO.MISSION"):
        print("Setting Land Mode")
        mavros.set_mode("AUTO.LAND", 10)
        
if __name__ == '__main__':

    allow_loiter = False
    latitude = None
    longitude = None
    yaw = 0

    rospy.init_node('Test_Takeoff')

    mavros = MavrosMissionTest()
    mavros.wait_for_topics(10)

    state_pub = rospy.Publisher('mission/state', MissionState, queue_size=10)
    loiter_sub = rospy.Subscriber('mission/loiter_commands', String, loiter_commands_cb)
    #loiter_sub = rospy.Subscriber('mission/loiter_commands_echo', String, loiter_commands_cb)    # Testing Telemetry radios

    takeoff()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and mavros.state.armed:
        rate.sleep()
    rospy.loginfo("Mission Completed")
    state_msg = MissionState()
    state_msg.alarmLatitude = 0
    state_msg.alarmLongitude = 0   
    state_msg.ExecutionState = "complete"
    state_pub.publish(state_msg)
    
