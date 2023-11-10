#!/usr/bin/env python3

"""
    A ROS Node for the Arduino microcontroller

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_msgs.srv import *
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
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 25))
        r = rospy.Rate(self.rate)

        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, queue_size=1, callback=self.vel_set)

        self.omni_vel_sub = rospy.Subscriber('omni_vel', Twist, queue_size=1, callback=self.omni_vel_set)

        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)

        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)

        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)

        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)

        # A service to read the value of a digital sensor
        rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler)

        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)

        # A service to read the value of an analog sensor
        rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)

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
        msg=self.controller.drive(float(data.linear.x),float(data.linear.y),float(data.linear.z))
        print("Sent",float(data.linear.x), float(data.linear.y),float(data.linear.z))
        print(msg)

    def vel_set(self,data):
        print("R")
        vmx=data.linear.x
        vmy=data.linear.y
        wmp=data.angular.z
        sqrt3by2 = 0.8660254037844385965883020617184229195117950439453125
        L = 0.04
        v1_left = (L * wmp - (vmx / 2) - (sqrt3by2 * vmy))
        v2_back = (vmx + (L * wmp))
        v3_right = (L * wmp - (vmx / 2) + (sqrt3by2 * vmy))
        if(abs(v1_left)<1.8):
            v1_left=0
        elif abs(v1_left)>2.85:
            v1_left=v1_left/abs(v1_left)*2.85
        if abs(v2_back)<1.8:
            v2_back = 0
        elif abs(v2_back)>2.85:
            v2_back=v2_back/abs(v2_back)*2.85
        if abs(v3_right)<1.8:
            v3_right=0
        elif abs(v3_right)>2.85:
            v3_right=v3_right/abs(v3_right)*2.85
        
        # print("left",v1_left,"back",v2_back,"right",v3_right)
        msg = self.controller.drive(round(v1_left,2),round(v3_right,2),round(v2_back,2))
        if(msg): print("Sent",round(v1_left,2),round(v3_right,2),round(v2_back,2))

    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()

    def ServoReadHandler(self, req):
        pos = self.controller.servo_read(req.id)
        return ServoReadResponse(pos)

    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()

    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()

    def DigitalReadHandler(self, req):
        value = self.controller.digital_read(req.pin)
        return DigitalReadResponse(value)

    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()

    def AnalogReadHandler(self, req):
        value = self.controller.analog_read(req.pin)
        return AnalogReadResponse(value)

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
