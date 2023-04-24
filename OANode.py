# This Python code is a ROS node that subscribes to a laser range topic,
# a pose topic, and a safe motion topic, and publishes twist messages to
# a motion topic based on the sensor inputs, controlling the motion of 
# a robot with bumper avoidance functionality.
# Author: Anthony Varnit

import rospy
import math

from geometry_msgs.msg import Twist,Vector3      # ROS Twist message
from sensor_msgs.msg import LaserScan            # ROS laser msg
from nav_msgs.msg import Odometry                # ROS odometry
from tf.transformations import euler_from_quaternion

motionTopic = 'cmd_vel'
safeMotionTopic= '/safe_cmd_vel'
poseTopic = '/odom' 
laserTopic  = '/scan'

#global variables, to communicate with callbacks
gLoc = [0,0,0]   # x,y,yaw pose of robot
gBumperLeft,gBumperRight= False, False # left/right close
safemsg = Twist()
lvel = 0
avel = 0


# this procedure is called to accept a new goal for MoveIt
#
def safeCallback(msg):
    global avel, lvel
    lvel = msg.linear.x
    avel = msg.angular.z
    return
#
# poseCallback
# this procedure is called to accept ROS pose topic info
# and make it available via the global gLoc
#
def poseCallback(data):
    global gLoc
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    gLoc[2]=yaw
    return
#
# callback for the laser range data
# sets two global variables gFrontLeft and Right
# if anything is too close
def laserCallback(msg):
    '''Call back function for laser range data'''
    global gBumperLeft,gBumperRight
    
    gBumperLeft,gBumperRight=False,False
    numRays = len(msg.ranges) # total num readings
    radPerIndex = math.radians(360)/numRays
    center = 0 # laser points to the front
    width = int(numRays/6) # left/right bumper 'window'
    tooClose=0.7 # threshold for bumper to activate
    
    for i in range(0,len(msg.ranges)):
        #rule out bad readings first
        if not math.isnan( msg.ranges[i] ) and \
           not math.isinf( msg.ranges[i] ) and msg.ranges[i]>0:
           # check for anything close left and right
           if msg.ranges[i]<tooClose:
               if i in range(0,width+1):
                   gBumperLeft=True
               elif i in range(numRays-width,numRays+1):
                   gBumperRight=True
    return

def OANode():
    global safemsg, gBumperLeft,gBumperRight
    rospy.init_node('OANode', anonymous=True)
    pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    rospy.Subscriber(poseTopic, Odometry, poseCallback)
    rospy.Subscriber(laserTopic, LaserScan, laserCallback)
    # subscribe to the NEW topic agreed upon (with the publisher node designer)
    rospy.Subscriber(safeMotionTopic, Twist, safeCallback)
    rospy.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): # infinite 'server' loop
            if lvel != 0:
                if gBumperLeft or gBumperRight:
                    if gBumperLeft:
                        print("bumper left")
                        safemsg.angular.z = -0.2
                        safemsg.linear.x = lvel / 2
                    if gBumperRight:
                        print("bumper right")
                        safemsg.angular.z = 0.2
                        safemsg.linear.x = lvel / 2
                pub.publish(safemsg)
                rate.sleep()
            safemsg.linear.x = lvel
            safemsg.angular.z = avel
            pub.publish(safemsg) 
            rate.sleep()
    return

#
def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return
#

if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        print("Waiting for twist() msg")
        OANode()
    except rospy.ROSInterruptException:
        pass