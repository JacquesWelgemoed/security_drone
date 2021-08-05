#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
from mission.msg import MissionState

def mission_state_cb(msg):
    ser.write(b'state')
    rospy.sleep(1)
    msg_str = msg.data
    ser_str = state_dict(msg_str)
    ser.write(b'{0}' .format(ser_str))

def cmd_dict(argument):
    ''' Changes 5 character commands from serial connection back to normal words'''

    switcher = {
        "up___": "up",
        "down_": "down",
        "left_": "left",
        "right": "right",
    }
    return switcher.get(argument, "Invalid command")

#def state_dict_reverse(argument):
    '''Changes numbers from serial connection back to words'''
'''
    switcher = {
        "1": "run",
        "2": "loiter",
        "3": "return",
        "4": "landing",
        "5": "precision_land",
        "6": "complete",
    }
    return switcher.get(argument, "Invalid command")

'''
def state_dict(argument):
    ''' Changes state messages to numbers to send over serial port'''

    switcher = {
        "run": "1",
        "loiter": "2",
        "return": "3",
        "landing": "4",
        "precision_land": "5",
        "complete": "6",
    }
    return switcher.get(argument, "Invalid command")


def listen_serial():
    '''Reads data from serial port, converts it to loiter commands and publishes those commands'''
    rate = rospy.Rate(100)
    # Continuously read from serial port at 100Hz
    while not rospy.is_shutdown():
        serial_string = ser.read(5)
        ser_cmd = serial_string.decode()
        '''if(ser_cmd == 'state'):

            # Read one character from serial port
            state_str = ser.read(1)
            ser_state = state_str.decode()
            state_msg = state_dict(ser_state)

            # Create Mission state message and publish values received from serial port
            state = MissionState()
            state.alarmLatitude = 0
            state.alarmLongitude = 0
            state.ExecutionState = state_msg
            mission_state_pub.publish(state)
'''
        loiter_cmd = cmd_dict(ser_cmd)
        loiter_pub.publish(loiter_cmd)
        rate.sleep()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('serial_receiver')

    # Create publishers and subscribers
    loiter_pub = rospy.Publisher('mission/loiter_commands', String, queue_size=10)
    mission_state_sub = rospy.Subscriber('mission/state', MissionState, mission_state_cb)
    mission_state_pub = rospy.Publisher('mission/state', MissionState, queue_size=10)

    # Instantiate global variables
    loiter = False
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600)
    
    if(ser.is_open):
        listen_serial()
