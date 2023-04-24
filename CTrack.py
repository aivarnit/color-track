# This Python code uses OpenCV to track the position of a specific colored object
# in real-time video by creating a color mask, detecting the object contours, 
# and calculating the centroid of the object.
# Author: Anthony Varnit

import rospy
import cv2
import math
import sys
import time
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image            
from nav_msgs.msg import Odometry                
from tf.transformations import euler_from_quaternion

# Ros topic names for turtlebot_gazebo
safeMotionTopic= '/safe_cmd_vel'
poseTopic = '/odom' 
laserTopic  = '/scan'
imageTopic = '/camera/rgb/image_raw'



#global variables, to communicate with callbacks
gLoc = [0,0,0]   # x,y,yaw pose of robot
gGoal = [0,0]    # goalx and goaly, to be read from topic
imageTopic = '/camera/rgb/image_raw'
gCurrentImage = Image() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object
goalNumber = 0
gBumperLeft,gBumperRight= False, False # left/right close
start_time = time.time()
targetColors = {
    'orange': ([(0,30,75), (5,50,89)]),
    'white': ([(255,255,255), (255,255,255)]),
    'yellow': ([(0,80,80), (10,110,100)]),
    'black': ([(0,0,0), (20,20,20)])
}

# Callback for the image topic
def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge
    gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    return

# Euclidean distance
def dist(x1,y1,x2,y2):
    delx = x2-x1
    dely = y2-y1
    d = math.sqrt( delx*delx+dely*dely)
    return d




# procedure to center a colored region of the image by
# rotating the robot so that the colored region remains
# centered. The color information is a min and max 
# RGB value in targetCol=[minrgb,maxrgb]
def CTrack(targetCol):
    '''center the robot camera on the target if in view'''
    global gCurrentImage, targetColors, goalNumber, startTime

    rospy.init_node('CTrack',anonymous=True)
    # create windows to show camera and processing
    cv2.namedWindow('Turtlebot Camera', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Target', cv2.WINDOW_AUTOSIZE)

    imageSub = rospy.Subscriber(imageTopic,Image,callbackImage)
    vel_pub = rospy.Publisher(safeMotionTopic, Twist, queue_size=0)
    rospy.sleep(2) # wait for callbacks to catch up

    rate = rospy.Rate(10)
    msg=Twist()

    while not rospy.is_shutdown():
        #just show what the camera sees now
        cv2.imshow('Turtlebot Camera', cv2.resize(gCurrentImage,(320,240)))
        #get height h and width w of current image
        h,w = gCurrentImage.shape[0], gCurrentImage.shape[1]

        # make a binary image that is 0 except where the color is in range
        targetImage = cv2.inRange(gCurrentImage,targetColors[targetCol][0],targetColors[targetCol][1])
        cv2.imshow('Target',cv2.resize(targetImage,(320,240)))
        
        #tracking algorithm
        avel=0.2
        lvel=0.0
        # extract the moments of the target image
        m = cv2.moments(targetImage)
        threshold = 0
        if goalNumber == 1:
            threshold = 10000
        elif goalNumber == 3:
            threshold = 100000
        else:
            threshold = 0

        if m['m00']>threshold: # skip if the target image has non nonzero regions
            # how far is the X center of target  from X center of image
            delx = w/2 - m['m10']/m['m00'] 
            avel = 0.4*delx/w # use this to generate a proportional ang velocity
            dist= m['m00']/(h*w) # area as a fraction of the image size
            A,epi=40,10 # target area size, controls how close ti get to target
            if dist>A+epi:
                lvel = -0.1
            elif dist<A-epi:
                lvel=0.1
            else:
                print("target found")
                # Record proof of reaching the goal
                current_time = time.time()
                elapsed_time = current_time - start_time
                image_name = "goal" + str(goalNumber) + ".jpg"
                # Add information to the image
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(gCurrentImage, "Goal number: " + str(goalNumber), (10,20), font, 0.5, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(gCurrentImage, "X location: " + str(m['m10']/m['m00']), (10,40), font, 0.5, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(gCurrentImage, "Y location: " + str(m['m01']/m['m00']), (10,60), font, 0.5, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(gCurrentImage, "Elapsed time: " + str(elapsed_time), (10,80), font, 0.5, (255,255,255), 2, cv2.LINE_AA)
                cv2.imwrite(image_name, gCurrentImage)
                msg.linear.x,msg.angular.z=0,0
                vel_pub.publish(msg)
                rate.sleep()
                return
        msg.linear.x,msg.angular.z=lvel,avel # publish velocity
        vel_pub.publish(msg)
        cv2.waitKey(1) # need to do this for CV2 windows to show up
        rate.sleep()
    return


#
def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(safeMotionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return
#
if __name__ == '__main__':
    # identify/center the RGB color range of the target
    try:
        rospy.on_shutdown(callback_shutdown)
        for targetColor in targetColors:
            CTrack(targetColor)
            goalNumber+=1
    except  rospy.ROSInterruptException:
        pass
#