#!/usr/bin/env python2

# imports
from __future__ import division

import rospy
import math
import numpy as np 
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_common import MavrosCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

class OffboardPositionControl(MavrosCommon, object):
    """ Allows offboard control by sending position setpoints via MAVROS """

    def setUp(self):
        super(OffboardPositionControl, self).setUp()
    
        self.pos = PoseStamped()
        self.radius = 1
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # Setpoints are sent in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    
    def tearDown(self):
        super(OffboardPositionControl, self).tearDown()

    # Helper Methods

    def send_pos(self):
        rate = rospy.Rate(10)
        self.pos.header = Header()
        self.pos.header.frame_id = "typhoon_h480__base_link"    # this frame is used for Gazebo Simulation

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)

            try:
                rate.sleep()
            except rospy.ROSInitException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}"
                       .format(self.local_position.pose.position.x, 
                               self.local_position.pose.position.y, 
                               self.local_position.pose.position.z))
        
        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x, 
                        self.local_position.pose.position.y, 
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset
        
    def reach_position(self, x, y, z, timeout):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z

       # rospy.loginfo("""attempting to reach position | x:{0}, y:{1}, z:{2} | current position x: {3:.2f},
       #                y:{4:.2f}, z:{5:.2f}""".format(x, y, z, self.local_position.pose.position.x, 
       #                                               self.local_position.pose.position.y, 
       #                                               self.local_position.pose.position.z))

        # For now, i will lock yaw toward north
        yaw_degrees = 0
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0,0,yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # checking if mav reaches setpoint position within 'timeout' seconds
        loop_freq = 40  # used to be 2
        rate = rospy.Rate(loop_freq)
        point_reached = False
        for i in xrange(loop_freq* timeout):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                #rospy.loginfo("position reached in {0} seconds of {1}".format(i/loop_freq, timeout))
                point_reached = True
                break
            
            try: 
                rate.sleep()
            except rospy.ROSException as e:
                pass
            
    def do_test_sequence(self):
        """Tests offboard position control"""
        
        # wait for topics to be available and mav landed state to be landed on ground
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arming(True, 5)
        
        rospy.loginfo("run mission")
        positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20), 
                     (0, 0, 20))
        
        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                       positions[i][2], 30)
            
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arming(False, 5)
        
    def get_distance(self, x, y, z):
        start_x = self.local_position.pose.position.x
        start_y = self.local_position.pose.position.y
        start_z = self.local_position.pose.position.z
       
        squared = (x - start_x)**2 + (y - start_y)**2 + (z - start_z)**2 
        return math.sqrt(squared)
         
    
    def constrain_variable(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))
    
     
    def nav_with_speed(self, x, y, z, speed, timeout):
        """Limit navigation speed to 'speed' | speed(float):m/s, timeout(int): seconds"""
        constrained_speed = self.constrain_variable(val=speed, min_val=0.1, max_val=1)
        dist = self.get_distance(x, y, z)
        time = float(dist/constrained_speed)
        rospy.loginfo("navigation speed set to {0}".format(constrained_speed))
        
        # Define starting point pose
        nav_start = PoseStamped()
        nav_start.header.stamp = rospy.Time.now()
        nav_start.pose.position.x = self.local_position.pose.position.x
        nav_start.pose.position.y = self.local_position.pose.position.y
        nav_start.pose.position.z = self.local_position.pose.position.z
        
        nav_setpoint = PoseStamped()
        nav_setpoint.header.stamp = rospy.Time.now()
        
        #loop_freq = 5
        #rate = rospy.Rate(loop_freq)
        
        start_time = rospy.Time.now()
        while not self.is_at_position(x, y, z, self.radius) and not rospy.is_shutdown():
            cur_time = rospy.Time.now()
            elapsed = float((cur_time.to_sec() - start_time.to_sec())/time)                    
            nav_setpoint.pose.position.x = nav_start.pose.position.x + (x - nav_start.pose.position.x) * elapsed
            nav_setpoint.pose.position.y = nav_start.pose.position.y + (y - nav_start.pose.position.y) * elapsed
            nav_setpoint.pose.position.z = nav_start.pose.position.z + (z - nav_start.pose.position.z) * elapsed
            
            self.reach_position(nav_setpoint.pose.position.x, nav_setpoint.pose.position.y, nav_setpoint.pose.position.z, 10)
                   
        print("Point x:{0}, y:{1}, z:{2} reached".format(x, y, z))