#!/usr/bin/env python
import rospy
import math
import cv2
import numpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib


global angKp
global linearKp
global rangeData
global goal_dist
global goal_ang
global follow_dist

global Kp
global Ki
global Kd

global myPose

global actualVelocityX
global actualAngularVelocityZ

linearKp = 0.3
angKp = 0.5
rangeData = []
goal_dist = 0
goal_ang = 0
follow_dist = 0.8

Kp = 0.2
Ki = 0.01
Kd = 0.01

actualVelocityX = 0
actualAngularVelocityZ = 0

#SLAM Stuff - Information Matrix and Vector:
global infoMatrix
global infoVector
global uList


global occGrid
#Covariance matrix of measurements
global Q


#Assumptions about variances (because hell if we're going to calibrate the sensors ourselves).
measVarianceDepth = 0.1
measVarianceOdom = 0.1 
landmarks = [] #To be defined later.

#Image Processing Fields:
global bluenessImage
global intenseImage
global bridge
bluenessImage = None
intenseImage = None
bridge = CvBridge()

#This function makes the initial prediction of our position, using the summation of our previous control signals.
def SLAMInit(uList): 

    poseEstimateList = [numpy.matrix([[0],[0],[0]])]
    for u,t in uList, range(len(uList)):
        poseEstPrev = poseEstimateList[t-1]
        #Take previous pose, add control signal (linear velocity and angular velocity) at that time.
        uLinVel = u[0]
        uAngVel = u[1]
        prevPoseAngle = poseEstPrev[2]
        poseEstimate_t = poseEstPrev + numpy.matrix([ [-(uLinVel/uAngVel) * sin(prevPoseAngle) +  (uLinVel/uAngVel) * sin(prevPoseAngle + uAngVel) ], [(uLinVel/uAngVel) * cos(prevPoseAngle) -  (uLinVel/uAngVel) * cos(prevPoseAngle+ uAngVel) ],  uAngVel  ]) 
        poseEstimateList.append(poseEstimate_t)
    return poseEstimateList 



def SLAMLinearize(uList, zList, cList, estList):
    global infoMatrix
    global infoVector


def SLAMReduce(infoMatrix, infoVector):
    reducedMatrix = infoMatrix
    reducedVector = infoVector



def SLAMSolve(reducedMatrix, reducedVector):

    #This function is responsible for the infoMatrix^-1 * infoVector step, which returns the best estimates of the poses and landmark locations.
    pass

def scanCallback(data):
    #We'll have a collection of depth data at first. We can refine this to include color and such by processing
    # multiple images - a depth image and a color image, for instance. 
    global rangeData
    rangeData = data.ranges[0:640] 

#This function's responsible for supplying a collection of blobs that fit the given criteria - for now, just "redness". 
def imageCallback(img_rgb):
    global bridge
    global bluenessImage
    global intenseImage
    maxIntensity = 255.0
    intensityRange = numpy.arange(maxIntensity)
    #Some preprocessing of the image to make it easier on the blob finder.
    phi = 1
    theta = 1

    cv_image = bridge.imgmsg_to_cv2(img_rgb, desired_encoding = "bgr8")
    newImage_deShadowed = (maxIntensity/phi) * (cv_image/(maxIntensity/theta)) ** 0.5
    newImage_deShadowed = numpy.array(newImage_deShadowed, dtype='uint8')

    cv2.imshow('newImage_deShadowed', newImage_deShadowed)

    rosIntenseImage = bridge.cv2_to_imgmsg(newImage_deShadowed, encoding='bgr8')
    intenseImage = rosIntenseImage

    cv_image = newImage_deShadowed

    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    rospy.loginfo("size of image:{}".format(img_hsv.shape))
    

    #These are the HSV ranges of the color Navy Blue. These will be the enemy, as jeans are a horrid plague upon the world of fashion.
    minBlueVals = numpy.array([110,80,30])
    maxBlueVals = numpy.array([130,255,255])

    bluMask = cv2.inRange(img_hsv, minBlueVals, maxBlueVals)
    cv_bluenessImage = cv2.bitwise_and(cv_image, cv_image, mask = bluMask)

    if(cv_bluenessImage is None):
        rospy.loginfo("Redness image is None for some reason.")
    else:   
        blobFinder = cv2.SimpleBlobDetector()
        #blobFinderParams = blobFinder.Params()

        keypoints = blobFinder.detect(cv_bluenessImage)
        rospy.loginfo("Keypoints of Blobs:{}".format(keypoints))
        for keypoint in keypoints:
            rospy.loginfo("Pixel X of Keypoint {}:{}".format(keypoint, keypoint.pt[0]))
            rospy.loginfo("Pixel Y of Keypoint {}:{}".format(keypoint, keypoint.pt[1]))
            #rospy.loginfo("Size of Image {}:{}".format(keypoint, keypoint.size))
        bgrImage = cv2.cvtColor(cv_bluenessImage, cv2.COLOR_HSV2BGR)
        rosImage = bridge.cv2_to_imgmsg(bgrImage, encoding="bgr8")
        bluenessImage = rosImage

#Stores odometry data into our global variables.
def odomCallback(data):
    global myPose
    global actualVelocityX
    global actualAngularVelocityZ
    actualVelocityX = data.twist.twist.linear.x
    actualAngularVelocityZ = data.twist.twist.angular.z
    myPose = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]

#Stores the occupancy grid created by the SLAM algorithm (third-party, perhaps later our own).
def mapCallback(data):
    global occGrid
    occGrid = data


def PIDController(integral, prevError, measuredVal, refVal):
    global Kp
    global Ki
    global Kd
    error = refVal - measuredVal
    integral += error
    derivative = error - prevError
    P = Kp * error
    I = Ki * integral
    D = Kd * derivative
        
    u = P + I + D
        
    prevError = error

    return u;

def main():
    global angKp
    global linearKp
    global rangeData
    global goal_dist
    global goal_ang 
    global follow_dist
    global bluenessImage
    global intenseImage
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    blueImgPub = rospy.Publisher('/blueImage', Image, queue_size=10)
    intensifiedImgPub = rospy.Publisher('/intenseImage', Image, queue_size=10)

    rospy.init_node('lab4', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scanCallback)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('camera/rgb/image_raw', Image, imageCallback)
    rate = rospy.Rate(10)  # Main loop: 10Hz

    vel=Twist()
    vel.linear.x=0
    vel.angular.z=0
    while not rospy.is_shutdown():
        #Do something: Probably try to approach the nearest thing to guard. That can come later, I guess.
        vel_pub.publish(vel)
        if(bluenessImage is not None):
            blueImgPub.publish(bluenessImage)
        if(intenseImage is not None):
            intensifiedImgPub.publish(intenseImage)
        rate.sleep()

      
if __name__ == "__main__":
    main()    