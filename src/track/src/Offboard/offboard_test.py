#!/usr/bin/env python2

# imports
from offboard_posctl import OffboardPositionControl
import rospy
from pymavlink import mavutil

def test_flight():
    offb = OffboardPositionControl()
    offb.setUp()
    
    offb.wait_for_topics(60)
    offb.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
    
    offb.log_topic_vars()
    offb.set_mode("OFFBOARD", 5)
    offb.set_arming(True, 5)
    
    print("Flying Square")    
    positions = ((0, 0, 0), (5, 5, 2), (5, -5, 2), (-5, -5, 2), 
                     (0, 0, 2))
        
    for i in xrange(len(positions)):
        offb.nav_with_speed(positions[i][0], positions[i][1],
                                    positions[i][2], 2, 30)
            
    offb.set_mode("AUTO.LAND", 5)
    offb.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
    
    offb.set_arming(False, 5)
    
    

if __name__ == '__main__':
    try:
        rospy.init_node('offboard_test')
        test_flight()
    except rospy.ROSInterruptException as e:
        pass