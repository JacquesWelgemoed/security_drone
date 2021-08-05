#!/usr/bin/env python
import rospy

start = rospy.Time.now()
rospy.sleep(10)
print(rospy.Time.now()-start)