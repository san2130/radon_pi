#!/usr/bin/env python3

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

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

import _thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
from pySerialTransfer import pySerialTransfer as txfer

SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/ttyACM0", baudrate=57600, timeout=0.5, motors_reversed=False):

        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
        self.motors_reversed = motors_reversed
        # Keep things _thread safe
        self.mutex = _thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            print("Connecting to Arduino on port", self.port, "...")
            self.port = txfer.SerialTransfer(port=self.port, baud=57600, timeout = 1)
            self.port.open()
            # The next line is necessary to give the firmware time to wake up.   
            time.sleep(2)
            test = self.get_baud()
            print("Connected at", self.baudrate)
            print("Arduino is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Arduino!")
            os._exit(1)

    def open(self):
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port.
        '''
        self.port.close()

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands
            below in a _thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        while not self.port.available():
                if self.port.status < 0:
                    if self.port.status == txfer.CRC_ERROR:
                        print('ERROR: CRC_ERROR')
                    elif self.port.status == txfer.PAYLOAD_ERROR:
                        print('ERROR: PAYLOAD_ERROR')
                    elif self.port.status == txfer.STOP_BYTE_ERROR:
                        print('ERROR: STOP_BYTE_ERROR')
                    else:
                        print('ERROR: {}'.format(self.port.status))

        rec_str_   = self.port.rx_obj(obj_type=type("a"),
                                     obj_byte_size=30,
                                     )
        # print('RCVD:',rec_str_)
        return rec_str_

    def execute(self, cmd):
        ''' _thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()

        ntries = 1
        attempts = 0
        try:
            send_size = 0
            str_size = self.port.tx_obj(cmd + '\r', send_size) - send_size
            send_size += str_size
            self.port.send(send_size)
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'ER' or value == None):
                try:
                    print("Sending again")
                    send_size = 0
                    str_size = self.port.tx_obj(cmd, send_size) - send_size
                    send_size += str_size
                    self.port.send(send_size)
                    value = self.recv(self.timeout)
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command: " + cmd)
            value = None

        self.mutex.release()
        return value

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print("Updating PID parameters")
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute(cmd)

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            x=self.execute('b')
            return x
        except:
            return None

    def get_encoder_counts(self):
        values = self.execute('e')
        return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute('r')[0:2]=="OK"

    def drive(self, right, left, back):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        if self.motors_reversed:
            left, right, back = -left, -right, -back
        return self.execute('m %d %d %d' %(right, left, back))

    def drive_closed(self, right, left, back):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)
        back_revs_per_second = float(back) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        back_ticks_per_loop  = int(back_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        return self.drive(right_ticks_per_loop , left_ticks_per_loop, back_ticks_per_loop )
    
    def drive_raw(self, right, left, back):
        return self.execute('o %d %d %d' %(right, left, back))[0:2] == "OK"

    def stop(self):
        ''' Stop both motors.
        '''
        msg=self.drive(0, 0, 0)
        if(msg):
            print("Stopped successfully")

    def analog_read(self, pin):
        return self.execute('a %d' %pin)

    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))

    def digital_read(self, pin):
        return self.execute('d %d' %pin)

    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))

    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''
        return self.execute_ack('s %d %d' %(id, min(SERVO_MAX, max(SERVO_MIN, degrees(pos)))))

    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''
        return radians(self.execute('t %d' %id))

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin)
