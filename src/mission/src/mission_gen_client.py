#!/usr/bin/env python 

'''This script simulates the event where an alarm is triggered by an arbitrary sensor at a known location.'''

import sys
import rospy
from mission.srv import alarm, alarmRequest, alarmResponse


def generate_mission_client(latitude, longitude, stamp):
    rospy.wait_for_service('generate_mission')
    try:
        gen_mission = rospy.ServiceProxy('generate_mission', alarm)
        resp1 = gen_mission(latitude, longitude, stamp)
        return resp1
    except rospy.ServiceException() as e:
        print("Service call failed: %s" & e)

def usage():
    return

if __name__ == "__main__":
    rospy.init_node('alarm_client_node')
    if len(sys.argv) == 3:
        lat = float(sys.argv[1])
        long = float(sys.argv[2])
    else:
        print("%s [latitude longitude]" % sys.argv[0])
        sys.exit()
    print("Requesting (%s ; %s) " % (lat,long) )
    s = generate_mission_client(lat, long, rospy.Time.now())