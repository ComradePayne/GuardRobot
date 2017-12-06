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

from Entity import Entity


global angKp
global linearKp
global depthImage
global goal_dist
global goal_ang
global follow_dist

global Kp
global Ki
global Kd

global myPose
myPose = [0,0,0]

global actualVelocityX
global actualAngularVelocityZ

linearKp = 0.3
angKp = 0.5
depthImage = None
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
global intenseImage
global bluenessImage
global rednessImage
global greennessImage

global depthImage

global blueKeypoints
global redKeypoints
global greenKeypoints

global currentEnemy
global guardList
global waypointList

global bridge

bluenessImage = None
rednessImage = None
greennessImage = None
intenseImage = None
depthImage = None

blueKeypoints = []
redKeypoints = []
greenKeypoints = []

currentEnemy = None
guardList = []
waypointList = []

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

def IRCallback(depthImg):
    #We'll have a collection of depth data at first. We can refine this to include color and such by processing
    # multiple images - a depth image and a color image, for instance. 
    global depthImage
    depthImage = depthImg

#Takes the image, rotates the robot until the blob's average location is near the center of the screen. (screenWidth / 2, rounded to nearest pixel, + margin)
# Then, it uses the depth camera to figure out how far it is in front of the robot, then calculates the world coordinates of that spot.
def localizeKeypoints(blob_keyPoints, depthImage, category, currentPose):

    seenEntities = []

    degreesPerPixel = 1/11
    for blob in blob_keyPoints:
        blobX=blob.pt[0]
        blobY=blob.pt[1]

        blobXPixel = int(round(blobX))
        blobYPixel = int(round(blobY))

        #Data's in mm, so convert to meters.
        depthMeasurement = depthImage.data[blobYPixel][blobXPixel] / 1000

        if(isnan(depthMeasurement)):
            pass

        angleOfReading_deg = blobXRounded * degreesPerPixel
        angleOfReading_rad = numpy.pi/180 * angleOfReading_deg

        positionOfObj = [currentPose[0] + cos(angleOfReading_rad) * depthMeasurement, currentPose[1] + sin(angleOfReading_rad) * depthMeasurement]

        seenEntities.append(MapEntity(category, positionOfObj))
    return seenEntities


#Given an entity with a position and a category, it first figures out if the entity is likely to be the same entity as an already existing entity.
# The likilihood is inversely proportional to the distance between the new entity and an existing entity. 
def mapEntity(entity):
    global currentEnemy
    global guardList
    global waypointList
    probabilityThreshold = 0.8

    if entity.category == "Enemy":
        currentEnemy = entity
    elif entity.category == "Guard":
        matchesExisting = checkProbabilities(entity, guardList)
        if(not matchesExisting):
            guardList.append(entity)
    else:
        matchesExisting = checkProbabilities(entity, waypointList)
        if(not matchesExisting):
            waypointList.append(entity)


# True if a match, false if not.
def checkProbabilities(newEntity, entityList):
    probList = 1/numpy.hypot(newEntity.position[0] - entityList.position[0], newEntity.position[1] - entityList.position[1])
    maxProb = max(probList)
    minProb = min(probList)

    if maxProb > probabilityThreshold:
        return False

    else:
        return True



#This function's responsible for supplying a collection of blobs that fit the given criteria - for now, just "blueness". 
def imageCallback(img_rgb):
    global bridge
    global bluenessImage
    global intenseImage

    global blueKeypoints

    maxIntensity = 255.0
    intensityRange = numpy.arange(maxIntensity)
    #Some preprocessing of the image to make it easier on the blob finder.
    phi = 1
    theta = 1

    cv_image = bridge.imgmsg_to_cv2(img_rgb, desired_encoding = "bgr8")
    newImage_deShadowed = (maxIntensity/phi) * (cv_image/(maxIntensity/theta)) ** 0.5
    newImage_deShadowed = numpy.array(newImage_deShadowed, dtype='uint8')

    #Translate cv2 img to ros img, to display in rviz later.
    rosIntenseImage = bridge.cv2_to_imgmsg(newImage_deShadowed, encoding='bgr8')
    intenseImage = rosIntenseImage

    #Preprocessing done!
    cv_image = newImage_deShadowed

    #Convert to an HSV image.
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    rospy.loginfo("size of image:{}".format(img_hsv.shape))
    

    #These are the HSV ranges of the color Navy Blue. These will be the enemy, as jeans are a horrid plague upon the world of fashion.
    minBlueVals = numpy.array([110,80,30])
    maxBlueVals = numpy.array([130,255,255])

    bluMask = cv2.inRange(img_hsv, minBlueVals, maxBlueVals)
    cv_bluenessImage = cv2.bitwise_and(cv_image, cv_image, mask = bluMask)

    blobFinder = cv2.SimpleBlobDetector()
    #blobFinderParams = blobFinder.Params()

    blueKeypoints = blobFinder.detect(cv_bluenessImage)
    # rospy.loginfo("Keypoints of Blobs:{}".format(keypoints))
    # for keypoint in keypoints:
    #     rospy.loginfo("Pixel X of Keypoint {}:{}".format(keypoint, keypoint.pt[0]))
    #     rospy.loginfo("Pixel Y of Keypoint {}:{}".format(keypoint, keypoint.pt[1]))
    #     #rospy.loginfo("Size of Image {}:{}".format(keypoint, keypoint.size))
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

def quatToEuler(orientationValue):
    pass

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
    global myPose
    global depthImage
    global goal_dist
    global goal_ang 
    global follow_dist

    global bluenessImage
    global intenseImage

    global blueKeypoints

    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    blueImgPub = rospy.Publisher('/blueImage', Image, queue_size=10)
    redImgPub = rospy.Publisher('/redImage', Image, queue_size=10)
    greenImgPub = rospy.Publisher('/greenImage', Image, queue_size=10)
    intensifiedImgPub = rospy.Publisher('/intenseImage', Image, queue_size=10)

    rospy.init_node('lab4', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('camera/rgb/image_raw', Image, imageCallback)

    rospy.Subscriber('/camera/depth/image_raw', Image, IRCallback)
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

        #First, localize yer keypoints.
        seenBlues = localizeKeypoints(blueKeypoints, depthImage, "Enemy", myPose)

        for blueObj in seenBlues:
            rospy.loginfo("{} Spotted! \n Pos X: {} \n Pos Y: {}\n".format(blueObj.category, blueObj.position[0], blueObj.position[1]))
            mapEntity(blueObj)

        rate.sleep()

      
if __name__ == "__main__":
    main()    