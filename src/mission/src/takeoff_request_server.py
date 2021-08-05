#!/usr/bin/env python
import rospy
from mission.srv import Takeoff, TakeoffRequest, TakeoffResponse
from mission_test import MavrosMissionTest
from mavros_test_common import MavrosTestCommon
from mavros_msgs.msg import ExtendedState

def handle_confirmTakeoff(req):
    #Prompt user to authorize takeoff using command line here
    print("Authorize UAV to fly to alarm location with Latitude %f and Longitude %f ?" % (req.alarmLatitude, req.alarmLongitude))
    user_input = raw_input("Type y to confirm or n to deny:")

    if(user_input != "y" and user_input != "Y"):    #TODO: Add timeout here
        print("You denied takeoff")
        return TakeoffResponse(False)


    #Check if UAV has sufficient battery life and is ready to fly
    mavros = MavrosMissionTest()
    mavros.wait_for_topics(5)
    '''
    if (mavros.battery.voltage < 16): # 4V per cell on a 4 cell battery
        rospy.loginfo("Takeoff denied, battery voltage below 16V")
        return TakeoffResponse(False)
    '''

    # Check if UAV's flight controller thinks that it is landed on the ground
    if (mavros.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
        rospy.loginfo("Takeoff denied, make sure UAV is in landed state before attempting takeoff")
        return TakeoffResponse(False)

    #Check state of Docking Station Here
    #Wait for docking station to open and landing gear to be released
    
    #TODO Enter real values for flight range and alt for UAV
    alt = 40
    flight_range = 10000
    h_distance, v_distance = mavros.distance_to_wp(req.alarmLatitude, req.alarmLongitude, alt)

    if h_distance> flight_range:
        return TakeoffResponse(False)
    # TODO: What other checks should be performed here.


    # Once all preflight checks are passed return approved=True
    rospy.loginfo("Takeoff approved")
    return TakeoffResponse(True)



def confirmTakeoff():
    rospy.init_node('takeoff_server')
    s = rospy.Service('takeoff_service', Takeoff, handle_confirmTakeoff)
    print("Ready to process takeoff requests")
    rospy.spin()

if __name__ == "__main__":
    confirmTakeoff()
    

