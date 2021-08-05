#!/usr/bin/env python

'''Key press recognition code from https://www.codehaven.co.uk/python/using-arrow-keys-with-inputs-python/
'''


import rospy
import curses
from mission.msg import MissionState
from std_msgs.msg import String

def mission_state_cb(msg):
    global loiter
    if (msg.ExecutionState == "loiter"):
        loiter = True
    else:
        loiter = False

def main():
    global loiter
    # Wait for mission to be in loiter phase
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
       
        if(loiter):
            do_teleop()
            loiter = False
        rate.sleep()


        

def do_teleop():
    '''Read keyboard inputs and send corresponding commands to UAV'''
    global loiter
    # Curses setup
    # Get curses screen window
    screen = curses.initscr()
    # Turn off input echoing
    curses.noecho()
    # Don't wait for carriage return to respond to key press
    curses.cbreak()
    # map arrow keys to special values
    screen.keypad(True)
    msg_string = String()
    
    try:
        while True:
            char = screen.getch()
            if(char == ord('r')):
                msg_string = "return"
                teleop_pub.publish(msg_string)
                break
            elif char == curses.KEY_RIGHT:
                screen.addstr(0, 0, 'right')
                msg_string = "right"
                teleop_pub.publish(msg_string)
            elif char == curses.KEY_LEFT:
                screen.addstr(0, 0, 'left ')  
                msg_string = "left"
                teleop_pub.publish(msg_string)    
            elif char == curses.KEY_UP:
                screen.addstr(0, 0, 'up   ')  
                msg_string = "up"
                teleop_pub.publish(msg_string)     
            elif char == curses.KEY_DOWN:
                screen.addstr(0, 0, 'down ')
                msg_string = "down"
                teleop_pub.publish(msg_string)
    except rospy.ROSInterruptException:
        pass
    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        loiter = False

if __name__ == "__main__":
    rospy.init_node("arrow_teleop")
    mission_state_sub = rospy.Subscriber('mission/state', MissionState, mission_state_cb)
    teleop_pub = rospy.Publisher('mission/loiter_commands', String, queue_size=10)
    loiter = False
    main()
