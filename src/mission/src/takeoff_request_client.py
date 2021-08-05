#!/usr/bin/env python 
import sys
import rospy
from mission.srv import Takeoff, TakeoffRequest, TakeoffResponse


def takeoff_request_client(latitude, longitude, stamp):
    rospy.wait_for_service('takeoff_service')
    try:
        takeoff_request = rospy.ServiceProxy('takeoff_service', Takeoff)
        resp1 = takeoff_request(latitude, longitude, stamp)
        return resp1
    except rospy.ServiceException() as e:
        print("Service call failed: %s" & e)

def usage():
    return

if __name__ == "__main__":
    rospy.init_node('takeoff_client_test')
    if len(sys.argv) == 3:
        lat = float(sys.argv[1])
        long = float(sys.argv[2])
    else:
        print("%s [latitude longitude]" % sys.argv[0])
        sys.exit()
    print("Requesting (%s ; %s) " % (lat,long) )
    s = takeoff_request_client(lat, long, rospy.Time.now())
    #print("%s * %s = %s " % (width, height, s))