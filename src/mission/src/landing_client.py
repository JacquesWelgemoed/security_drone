#!/usr/bin/env python 
import sys
import rospy
from mission.srv import Landing, LandingRequest, LandingResponse


def takeoff_request_client(latitude, longitude, altitude, stamp):
    rospy.wait_for_service('landing_service')
    try:
        landing_request = rospy.ServiceProxy('landing_service', Landing)
        resp1 = landing_request(latitude, longitude, altitude, stamp)
        return resp1
    except rospy.ServiceException() as e:
        print("Service call failed: %s" & e)

def usage():
    return

if __name__ == "__main__":
    rospy.init_node('takeoff_client_test')
    if len(sys.argv) == 4:
        lat = float(sys.argv[1])
        long = float(sys.argv[2])
        alt = float(sys.argv[3])
    else:
        print("%s [latitude longitude altitude]" % sys.argv[0])
        sys.exit()
    print("Requesting (%s ; %s ; %s) " % (lat,long, alt) )
    s = takeoff_request_client(lat, long, alt, rospy.Time.now())
    