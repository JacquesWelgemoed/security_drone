#!/usr/bin/env python

#imports
import rospy
import mavros


from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

#callback method for State subscriber
current_state = State()
offb_set_mode = SetMode

def update_pos_cb(new_pose):
    pose.pose.position.x = new_pose.pose.position.x
    pose.pose.position.y = new_pose.pose.position.y
    pose.pose.position.z = new_pose.pose.position.z


def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
new_pos_sub = rospy.Subscriber('mavros/setpoint_position/local/new', PoseStamped, update_pos_cb)
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 3



def position_control():
    rospy.init_node('offb_node', anonymous=True)
    rate = rospy.Rate(20.0)
    

    #Send setpoints before starting offboard mode
    for i in range (100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()


    if current_state.mode != "OFFBOARD": #and (now - last_request > rospy.Duration(1)):
        set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        print("OFFBOARD MODE")
    
    if not current_state.armed:# and (now - last_request > rospy.Duration(1)):
        arming_client(True)
        print("Arming")


    while not rospy.is_shutdown():
        
        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()

    except rospy.ROSInterruptException:
        pass


