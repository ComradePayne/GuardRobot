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

def SLAMInit(uList): 

	#This function makes the initial prediction of our position, using the summation of our previous control signals.
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
	for j in range(len(landmarks)):
		#Get indices of 


def SLAMSolve(reducedMatrix, reducedVector):

	#This function is responsible for the infoMatrix^-1 * infoVector step, which returns the best estimates of the poses and landmark locations.

def visionCallback(data):
	#We'll have a collection of depth data at first. We can refine this to include color and such by processing
	# multiple images - a depth image and a color image, for instance. 
	global rangeData
    rangeData = data.ranges[0:640] 

def odomCallback(data):
	global myPose
	global actualVelocityX
    global actualAngularVelocityZ
    actualVelocityX = data.twist.twist.linear.x
    actualAngularVelocityZ = data.twist.twist.angular.z
    myPose = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z]


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
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.init_node('lab3_3', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, visionCallback)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rate = rospy.Rate(10)  # Main loop: 10Hz

    vel=Twist()
    vel.linear.x=0
    vel.angular.z=0
    while not rospy.is_shutdown():
    	#Do something: Probably try to approach the nearest thing to guard. That can come later, I guess.
    	vel_pub.publish(vel)
    	rate.sleep()

      
if __name__ == "__main__":
    main()    