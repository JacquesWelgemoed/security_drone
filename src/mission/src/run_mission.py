#!/usr/bin/env python
import rospy
from mission.msg import MissionState
from mission_test import MavrosMissionTest
from mission.srv import Takeoff, TakeoffRequest, TakeoffResponse, Landing, LandingRequest, LandingResponse
from mavros_msgs.msg import State, Waypoint, CommandCode
from pymavlink import mavutil
from std_msgs.msg import String, Float64
from gimbal_test import Gimbal_test
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

def mission_state_cb(msg):
    '''Updates run variable to start missions'''

    global running, latitude, longitude
    if (msg.ExecutionState == "run"):
        latitude = msg.alarmLatitude
        longitude = msg.alarmLongitude
        running = True

def loiter_commands_cb(msg):
    '''Receives loiter command messages and calls associated methods'''

    global loiter_time, first_command, rtl_published
    loiter_time = 0
    first_command = True

    if msg.data == "return":
        do_rtl()
        
    elif ((msg.data == "left") or (msg.data == "right")):
        if(msg.data == "left"):
            do_yaw_ctl(False)
        else:
            do_yaw_ctl(True)

    else:
        if(msg.data == "up"):
            do_gimbal_control(True)
        else:
            do_gimbal_control(False)

def do_rtl():
    '''Creates a waypoint mission to return to take off location and executes that mission.'''

    wpList = []

    # Fly to takeoff location with an altitude of 10m
    home_wp = Waypoint()
    home_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    home_wp.command = CommandCode.NAV_WAYPOINT
    home_wp.is_current = True
    home_wp.autocontinue = True   
    home_wp.param1 = 0.0 
    home_wp.param2 = 0.0
    home_wp.param3 = 0.0
    home_wp.param4 = float('nan')
    home_wp.x_lat = home_gps.latitude
    home_wp.y_long = home_gps.longitude
    home_wp.z_alt = 10

    # Descend to a height of 5m above take off location
    loiter_wp = Waypoint()
    loiter_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    loiter_wp.command = CommandCode.NAV_WAYPOINT
    loiter_wp.is_current = False
    loiter_wp.autocontinue = False    #Set to False to cause UAV to loiter upon completion of mission
    loiter_wp.param1 = 0.0 
    loiter_wp.param2 = 0.0
    loiter_wp.param3 = 0.0
    loiter_wp.param4 = float('nan')
    loiter_wp.x_lat = home_gps.latitude
    loiter_wp.y_long = home_gps.longitude
    loiter_wp.z_alt = 5

    wpList.append(home_wp)
    wpList.append(loiter_wp)

    # Clear waypoints on Flight controller
    mission.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mission.send_wps(wpList, 5)
    
    if(checked):
        mission.set_mode("AUTO.MISSION", 10)
        do_landing()
    else:
        rospy.logerr("Failed to send return to home waypoints to flight controller")


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
    mission.clear_wps(5)
    
    #Send Mission to Flight Controller
    checked = mission.send_wps(wpList, 5)
    
    if(checked):
        mission.set_mode("AUTO.MISSION", 10)
    else:
        rospy.logerr("Failed to send return to home waypoints to flight controller")


def do_gimbal_control(up):
    '''Increments/Decrements gimbal pitch angle by 10 degrees depending on the value of the up parameter (True/False = Up/Down)'''

    global gimbal_pitch
    gimbal = Gimbal_test()
    if(up):
        pitch_in = gimbal_pitch + 10
        gimbal_pitch = pitch_in
        gimbal.run(pitch_in)
    else:
        pitch_in = gimbal_pitch - 10
        gimbal_pitch = pitch_in
        gimbal.run(pitch_in)


def look_down():
    '''Sets gimbal angle to -90 degrees to look downward for fiducial markers.'''

    gimbal = Gimbal_test()
    gimbal.run(-90)


def do_yaw_ctl(clockwise):
    '''Updates the heading setpoint that is passed to the yaw_mission method'''

    global yaw
    if(clockwise):
        yaw_in = yaw + 30
        yaw = yaw_in
        
    else:
        yaw_in = yaw - 30
        yaw = yaw_in        
    
    yaw_mission(yaw_in)


def do_landing():
    '''Waits for UAV to reach take off GPS location and requests a landing, if no fiducial markers are recognised the UAV is flown to 
    a rally point and landed there.'''

    rate = rospy.Rate(5)
    cur_gps = NavSatFix()
    # Waits for UAV to reach home
    while not rospy.is_shutdown():
       
        if(mission.mission_item_reached == (len(mission.mission_wp.waypoints)-1)): 
            look_down()
            look_down()
            rospy.sleep(5)
            look_down()     # Look down is called multiple times to ensure that MountControl message is received by flight controller
            state_msg = MissionState()
            state_msg.alarmLatitude = home_gps.latitude
            state_msg.alarmLongitude = home_gps.longitude    
            state_msg.ExecutionState = "landing"
            state_pub.publish(state_msg)
            break
        rate.sleep()

    # Generating a landing request
    altitude = rospy.wait_for_message('mavros/global_position/rel_alt', Float64, timeout=5)
    landing_request = rospy.ServiceProxy('landing_service', Landing)
    resp1 = landing_request(cur_gps.latitude, cur_gps.longitude, altitude.data, rospy.Time.now())
    
    # If the landing request response is "normal", move to alternate landing position and land there
    if(resp1.mode == "normal"):
        #TODO: Set rally_lat and rally_long to actual coordinates
        wpList = []

        cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
        altitude = rospy.wait_for_message('mavros/global_position/rel_alt', Float64, timeout=5)
        rally_lat = rospy.get_param("run_mission/rally_latitude")
        rally_long = rospy.get_param("run_mission/rally_longitude")

        rally_wp = Waypoint()
        rally_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        rally_wp.command = CommandCode.NAV_WAYPOINT
        rally_wp.is_current = True
        rally_wp.autocontinue = True   
        rally_wp.param1 = 0.0 
        rally_wp.param2 = 0.0
        rally_wp.param3 = 0.0
        rally_wp.param4 = 0.0
        rally_wp.x_lat = rally_lat
        rally_wp.y_long = rally_long
        rally_wp.z_alt = 20

        land_wp = Waypoint()
        land_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        land_wp.command = CommandCode.NAV_LAND
        land_wp.is_current = True
        land_wp.autocontinue = False    #Set to False if this is last waypoint
        land_wp.param1 = 0.0 
        land_wp.param2 = 0.0
        land_wp.param3 = 0.0
        land_wp.param4 = 0.0
        land_wp.x_lat = rally_lat
        land_wp.y_long = rally_long
        land_wp.z_alt = 0.0

        wpList.append(rally_wp)
        wpList.append(land_wp)

        # Clear waypoints on Flight controller
        mission.clear_wps(5)
        
        #Send Mission to Flight Controller
        checked = mission.send_wps(wpList, 5)
        
        if(checked):
            mission.set_mode("AUTO.MISSION", 10)
        else:
            rospy.logerr("Failed to send return to home waypoints to flight controller")

    # If landing service response is "precision", publish a MissionState mission with an ExecutionState of "precision_land" to initiate precision landing
    elif(resp1.mode == "precision"):
        state_msg = MissionState()
        state_msg.alarmLatitude = 0
        state_msg.alarmLongitude = 0   
        state_msg.ExecutionState = "precision_land"
        state_pub.publish(state_msg)
        

def main():

    # Wait for MissionState to be set to run
    global running
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if(running):
            print("Mission is running")
            run_mission()
            running = False
        rate.sleep()
   

def request_takeoff(latitude, longitude, stamp):
    '''Generate a takeoff_service request'''

    rospy.wait_for_service('takeoff_service')
    try:
        takeoff_request = rospy.ServiceProxy('takeoff_service', Takeoff)
        resp1 = takeoff_request(latitude, longitude, stamp)
        return resp1
    except rospy.ServiceException() as e:
        print("Service call failed: %s" & e)


def run_mission():
    '''Execute and monitor mission that was planned by mission_gen_server'''

    global running, loiter_time, rtl_published, home_gps, initial_battery
  
    # Wait for mission test topics and parameters
    mission.wait_for_topics(10)
    mission.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                10, -1)
    mission.wait_for_mav_type(10)

    home_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
    # Generate a takeoff request
    takeoff_approved = request_takeoff(latitude, longitude, rospy.Time.now())

    # If takeoff was not approved, return to main method
    if not takeoff_approved.approved:
        return

    #Start Mission
    mission.set_mode("AUTO.MISSION", 10)
    mission.set_arm(True, 5)

    loop_rate = 10
    rate = rospy.Rate(loop_rate)
    elapsed = 0
    

    # Waits for UAV to reach alarm location (Final Waypoint)
    while not rospy.is_shutdown():

        if(mission.mission_item_reached == 0):
            initial_battery = mission.battery.voltage

        if(mission.mission_item_reached == (len(mission.mission_wp.waypoints)-1)):
            rospy.loginfo("Final Waypoint reached in %f seconds" % elapsed)
            cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
            state_msg = MissionState()
            state_msg.alarmLatitude = cur_gps.latitude
            state_msg.alarmLongitude = cur_gps.longitude    
            state_msg.ExecutionState = "loiter"
            state_pub.publish(state_msg)
            break
        rate.sleep()
        elapsed = elapsed + (1/loop_rate)

    # Using 3.25V per cell as empty when battery is under load
    battery_consumed = ((initial_battery - mission.battery.voltage)/(initial_battery -13)) *100
    

    # This loop executes while the UAV loiters above the alarm location
    while not rospy.is_shutdown():
        
        # If UAV is disarmed, declare the mission as complete
        if(mission.state.armed == False):
            rospy.loginfo("Mission Completed")
            state_msg = MissionState()
            state_msg.alarmLatitude = 0
            state_msg.alarmLongitude = 0   
            state_msg.ExecutionState = "complete"
            state_pub.publish(state_msg)
            return
        
        # Check battery levels, if the amount consumed to reach alarm location is left return to home
        remaining_battery = (1 - (initial_battery - mission.battery.voltage)/(initial_battery -13)) * 100
        if(remaining_battery<= battery_consumed):
            cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
            state_msg = MissionState()
            state_msg.alarmLatitude = cur_gps.latitude
            state_msg.alarmLongitude = cur_gps.longitude    
            state_msg.ExecutionState = "return"
            state_pub.publish(state_msg)
            rtl_published = True
            do_rtl()

        # If no commands are sent to UAV during loiter phase for 2 minutes, return to home
        if (loiter_time>120 and (not rtl_published)):
            cur_gps = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
            state_msg = MissionState()
            state_msg.alarmLatitude = cur_gps.latitude
            state_msg.alarmLongitude = cur_gps.longitude    
            state_msg.ExecutionState = "return"
            state_pub.publish(state_msg)
            rtl_published = True
            do_rtl()

        rate.sleep()
        loiter_time = loiter_time + (1/loop_rate)
    

if __name__ == "__main__":
    try:

        # Initialise ROS Node
        rospy.init_node('run_mission')

        # Create all necessary publishers and subscribers
        state_pub = rospy.Publisher('mission/state', MissionState, queue_size=10)
        state_sub = rospy.Subscriber('mission/state', MissionState, mission_state_cb)    
        loiter_sub = rospy.Subscriber('mission/loiter_commands', String, loiter_commands_cb)

        # Instantiate global variables
        running = False
        initial_battery = 0
        latitude = 0
        longitude = 0
        yaw = -90 # TODO: Figure out how to read current yaw angle and increment from there instead of using global variable
        gimbal_pitch = 0
        loiter_time = 0
        home_gps = NavSatFix()
        first_command = False
        rtl_published = False

        # Create MavrosMisssionTest instance to use helper methods
        mission = MavrosMissionTest()

        # Run main method to wait for mission run command
        main()
    except rospy.ROSInterruptException:
        pass