#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String
from mission.msg import MissionState

def cmd_dict(argument):
    '''Changes loiter commands to five character strings, the size expected by the serial receiver'''
    switcher = {
        "up": "up___",
        "down": "down_",
        "left": "left_",
        "right": "right",
    }
    return switcher.get(argument, "Invalid command")


'''def state_dict(argument):
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

'''
def state_dict_reverse(argument):
    ''' Changes numbers from serial connection back to words'''

    switcher = {
        "1": "run",
        "2": "loiter",
        "3": "return",
        "4": "landing",
        "5": "precision_land",
        "6": "complete",
    }
    return switcher.get(argument, "Invalid command")


def loiter_commands_cb(msg):
    '''Subscribes to loiter commands, converts them to serial format and sends them over serial port'''

    str = cmd_dict(msg.data)
    ser.write(b'{0}' .format(str))
    print("Sending {0} to Rx" .format(str))


'''def mission_state_cb(msg):
    '''Subscribes to mission state messages and echoes them on serial port'''
    ser.write(b'state')
    rospy.sleep(1)
    msg_str = msg.data
    ser.write(b'{0}' .format(msg_str))
'''

def listen_serial():
    '''Reads data from serial port, converts it to loiter commands and publishes those commands'''
    rate = rospy.Rate(100)
    # Continuously read from serial port at 100Hz
    while not rospy.is_shutdown():
        serial_string = ser.read(5)
        ser_cmd = serial_string.decode()
        if(ser_cmd == 'state'):
            # Read one character from serial port
            state_str = ser.read(1)
            ser_state = state_str.decode()
            state_msg = state_dict_reverse(ser_state)

            # Create Mission state message and publish values received from serial port
            state = MissionState()
            state.alarmLatitude = 0
            state.alarmLongitude = 0
            state.ExecutionState = state_msg
            mission_state_pub.publish(state)
        rate.sleep()


if __name__ == '__main__':

    # Initialise ROS node
    rospy.init_node("serial_transmitter")

    # Create loiter commands subscriber
    loiter_sub = rospy.Subscriber('mission/loiter_commands', String, loiter_commands_cb)
    mission_state_sub = rospy.Subscriber('mission/state', MissionState, mission_state_cb)
    mission_state_pub = rospy.Publisher('mission/state', MissionState, queue_size=10)

    # Open serial port
    ser = serial.Serial(port='/dev/ttyUSB0',baudrate=57600)
    listen_serial()
    rospy.spin()