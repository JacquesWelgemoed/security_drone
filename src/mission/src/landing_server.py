#!/usr/bin/env python

import rospy
from mission.srv import Landing, LandingRequest, LandingResponse
from mavros_test_common import MavrosTestCommon
from mavros_msgs.msg import ExtendedState
from ar_track_alvar_msgs.msg import AlvarMarkers

def handle_confirmLanding(req):
    
    mavros = MavrosTestCommon()
    mavros.wait_for_topics(5)
    
    # Checks if sufficient charge is left in the batteries for a precision landing
    if (mavros.battery.voltage < 13.2): # 3.3V per cell on a 4 cell battery
        rospy.loginfo("Landing next to station, battery too low to perform precision landing")
        return LandingResponse(True, "normal")


    # TODO: Check state of Docking Station Here
    # TODO: Wait for docking station to open 
    
    # TODO: What other checks should be performed here.(Wind Speed?)

    # Checks for fiducial markers for 5 seconds
    data = rospy.wait_for_message('ar_pose_marker', AlvarMarkers, timeout=5)
    if ((data is None) or (len(data.markers) == 0)):
        rospy.loginfo("Landing next to station, could not recognize markers")
        return LandingResponse(True, "normal")
    
    # Once all landing checks are passed return approved=True
    rospy.loginfo("Landing approved")
    return LandingResponse(True, "precision")



def confirmLanding():
    rospy.init_node('landing_server')
    s = rospy.Service('landing_service', Landing, handle_confirmLanding)
    print("Ready to process landing requests")
    rospy.spin()

if __name__ == "__main__":
    confirmLanding()
    