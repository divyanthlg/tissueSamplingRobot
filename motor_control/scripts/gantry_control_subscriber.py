#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import serial

class SerialWriterNode:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('serial_writer_node', anonymous=True)
        
        # Get parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        
        # Initialize serial connection
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        
        # Subscribe to the 'motor_control' topic
        self.subscription = rospy.Subscriber('motor_control', Int32, self.listener_callback)

    def listener_callback(self, msg):
        data_to_send = msg.data  
        rospy.loginfo(f"Received data: {data_to_send}")
        self.ser.write(f"{data_to_send}\n".encode())    

    def close_serial(self):
        self.ser.close()

if __name__ == '__main__':
    try:
        # Create an instance of SerialWriterNode
        node = SerialWriterNode()
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure serial connection is closed properly
        node.close_serial()
