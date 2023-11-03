#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

def nothing(x):
    pass

rospy.init_node("trackbar_speed")
vel_pub = rospy.Publisher('omni_vel', Twist, queue_size=10)
vel = Twist()
# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')



# create trackbars for color change

cv2.createTrackbar('LEFT','image',0,255,nothing)

cv2.createTrackbar('RIGHT','image',0,255,nothing)

cv2.createTrackbar('BACK','image',0,255,nothing)

cv2.setTrackbarMin('LEFT', 'image', -255)
cv2.setTrackbarMin('RIGHT', 'image', -255)
cv2.setTrackbarMin('BACK', 'image', -255)



# create switch for ON/OFF functionality

switch = '0 : OFF \n1 : ON'

cv2.createTrackbar(switch, 'image',0,1,nothing)



while(1):

    cv2.imshow('image',img)

    k = cv2.waitKey(1000//30) & 0xFF

    if k == 27:
        vel.linear.x=0
        vel.linear.y=0
        vel.linear.z=0
        vel_pub.publish(vel)
        break



    # get current positions of four trackbars

    r = cv2.getTrackbarPos('LEFT','image')

    g = cv2.getTrackbarPos('RIGHT','image')

    b = cv2.getTrackbarPos('BACK','image')

    s = cv2.getTrackbarPos(switch,'image')



    if s == 0:

        img[:] = 0
        vel.linear.x=0
        vel.linear.y=0
        vel.linear.z=0
        

    else:

        # img[:] = [b,g,r]
        # if r<256:
        #     r = r*(-1)
        # else:
        #     r=r-256
        # if b<256:
        #     b = b*(-1)
        # else:
        #     b=b-256
        # if g<256:
        #     g = g*(-1)
        # else:
        #     g=g-256
        
        vel.linear.x=r
        vel.linear.y=g
        vel.linear.z=b

    vel_pub.publish(vel)



cv2.destroyAllWindows()