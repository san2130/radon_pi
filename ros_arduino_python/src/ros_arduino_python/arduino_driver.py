#!/usr/bin/env python3

import _thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
from pySerialTransfer import pySerialTransfer as txfer

class Arduino:

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

        self.mutex = _thread.allocate_lock()

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

    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        if self.motors_reversed:
            left, right= -left, -right
        return self.execute('m %d %d %d' %(right, left, 0))

    def drive_closed(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        return self.drive(right_ticks_per_loop , left_ticks_per_loop)
    
    def drive_raw(self, right, left):
        return self.execute('o %d %d %d' %(right, left, 0))[0:2] == "OK"

    def stop(self):
        ''' Stop both motors.
        '''
        msg=self.drive(0, 0)
        if(msg):
            print("Stopped successfully")
