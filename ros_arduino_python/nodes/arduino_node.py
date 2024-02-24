#!/usr/bin/env python3

import rospy
from ros_arduino_python.arduino_driver import Arduino
from geometry_msgs.msg import Twist
import os, time
import _thread
from serial.serialutil import SerialException

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 25))
        r = rospy.Rate(self.rate)

        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.omni_vel_sub = rospy.Subscriber('omni_vel', Twist, queue_size=1, callback=self.omni_vel_set)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)

        # Make the connection
        self.controller.connect()

        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        # Reserve a thread lock
        mutex = _thread.allocate_lock()

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            # msg=self.controller.get_encoder_counts()
            # print(msg)
            
            r.sleep()

    
    def omni_vel_set(self,data):
        msg=self.controller.drive(float(data.linear.x),float(data.linear.y))
        print("Sent",float(data.linear.x), float(data.linear.y))
        print(msg)

    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.controller.stop()
            rospy.sleep(2)
        except:
            pass

        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)
