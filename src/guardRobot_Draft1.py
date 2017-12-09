#!/usr/bin/env python
import rospy
import math
import cv2
import numpy
import scipy.cluster.hierarchy as hcluster
import time

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
myPose = [0,0,0,0]

global actualVelocityX
global actualAngularVelocityZ

global Max_angular_speed
global Max_linear_speed

Max_angular_speed = 0.1
Max_linear_speed = 0.1

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
global blurryImage
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
testEnemyList = []

guardList = []
waypointList = []

bridge = CvBridge()

global detectorParams 
detectorParams = cv2.SimpleBlobDetector_Params()
detectorParams.filterByInertia = False
detectorParams.filterByConvexity = False
detectorParams.filterByCircularity = False
detectorParams.filterByArea = True
detectorParams.filterByColor = True

detectorParams.minThreshold = 100
detectorParams.maxThreshold = 256

global blobFinder
blobFinder = cv2.SimpleBlobDetector(detectorParams)

global debugMode
debugMode = True


def debugPrint(debugString):
    global debugMode
    if(debugMode):
        rospy.loginfo(debugString)


def IRCallback(depthImg):
    #We'll have a collection of depth data at first. We can refine this to include color and such by processing
    # multiple images - a depth image and a color image, for instance. 
    global depthImage
    depthImage = depthImg
    depthImageData = numpy.array(depthImage.data)
   # rospy.loginfo('printing the image type: {}'.format((depthImageData.data)))
    #print 'Shape of depthImage : {}'.format((depthImage.data))
#Takes the image, rotates the robot until the blob's average location is near the center of the screen. (screenWidth / 2, rounded to nearest pixel, + margin)
# Then, it uses the depth camera to figure out how far it is in front of the robot, then calculates the world coordinates of that spot.
def localizeKeypoints(blob_keyPoints, depthImage, category, currentPose):

    seenEntities = []
    visionRange = 58.0

    degreesPerPixel = 1.0/11.0

    positions = []

    avgPositionOfKeyPoints
    for blob in blob_keyPoints:
        blobX=blob.pt[0]
        blobY=blob.pt[1]

        blobXPixel = int(round(blobX))
        blobYPixel = int(round(blobY))

        #rospy.loginfo("Depth Image X: {} \nDepth Image Y:{}\n".format(blobXPixel,blobYPixel))
        #rospy.loginfo(type(depthImage))



        #Data's in mm, so convert to meters.
        cv_image = bridge.imgmsg_to_cv2(depthImage, desired_encoding = "passthrough")
        depthMeasurement = cv_image[blobYPixel, blobXPixel] / 1000.0
        #rospy.loginfo("Depth at {},{}: {}".format(blobXPixel, blobYPixel,depthMeasurement))

        #depthMeasurement = depthImage.data[blobYPixel * blobXPixel] / 1000


        if(numpy.isnan(depthMeasurement) or depthMeasurement < 0.1):
            rospy.loginfo("Measurement is not a number, or invalid.")
            return seenEntities

        angleOfReading_deg = blobXPixel * degreesPerPixel
        angleOfReading_rad = numpy.pi/180.0 * angleOfReading_deg

        worldAngle = currentPose[2] + (((numpy.pi/180.0) * visionRange/2.0) - angleOfReading_rad)
        #angleOfReading_deg = angleOfReading_deg + 61
        #worldAngle = numpy.pi/180.0 * angleOfReading_deg
        #rospy.loginfo("Angle of Reading: {}".format(worldAngle))

        positionOfObj = [currentPose[0] + math.cos(worldAngle) * depthMeasurement, currentPose[1] + math.sin(worldAngle) * depthMeasurement]
        positions.append(positionOfObj)

        rospy.loginfo("Blob pose: {}".format(positionOfObj))
        seenEntities.append(Entity(category, positionOfObj))
    return seenEntities

#Since the blob finder often finds multiple keypoints, it is necessary to cluster them together into sensibly averaged "actual positions"
def clusterPositions(entityList):
    positionList = []
    for entity in entityList:
        np_position = numpy.array(entity.position)
        positionList.append(np_position)
    positionList = numpy.array(positionList)
    rospy.loginfo(positionList)
    clusterCenters = numpy.empty(0)
    numberOfCenters = 0



    if len(positionList) > 1:
        threshold = 0.7
        clusters = hcluster.fclusterdata(positionList, threshold, criterion="distance")

        for i in range(len(positionList)):
            clusterID = clusters[i]
            positionsInCluster = positionList[clusterID == clusters]
            rospy.loginfo("Cluster ID {}: {}".format(clusterID, positionsInCluster))

            clusterCenters = numpy.append(clusterCenters, numpy.mean(positionsInCluster, axis=1), axis=0)
        # clusterCenters.append([0.0,0.0])
        
        # for i in range (len(clusters)):
        #     if i!=0 and clusters[i-1]!= clusters[i]:
        #         clusterCenters[i] = numpy.mean(clusters[i])


            

        #clusterCenters[clusterNum] = numpy.mean(clusterCenters) 

        numberOfCenters = len(clusterCenters)
        print 'ClusterCenters : {}'.format(clusterCenters)
    elif len(positionList) ==1:
        numpy.append(clusterCenters, positionList[0])
    return clusterCenters, numberOfCenters



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
        matchesExisting = checkProbabilities(entity, guardList, probabilityThreshold)
        #rospy.loginfo("Matches existing: {}".format(matchesExisting))
        if(not matchesExisting):
            guardList = []
            guardList.append(entity)
    else:
        matchesExisting = checkProbabilities(entity, waypointList, probabilityThreshold)
        if(not matchesExisting):
            waypointList = []
            waypointList.append(entity)

# True if a match, false if not.
def checkProbabilities(newEntity, entityList, pThreshold):
    probList = []
    for entity in entityList:
        probList.append(1/numpy.hypot(newEntity.position[0] - entity.position[0], newEntity.position[1] - entity.position[1]))
    if (probList):
        maxProb = max(probList)
        minProb = min(probList)

    if (not probList or maxProb > pThreshold):
        return False

    else:
        return True



#This function's responsible for supplying a collection of blobs that fit the given criteria - for now, just "blueness". 
def imageCallback(img_rgb):
    t1 = time.clock()

    global bridge
    global blobFinder

    global bluenessImage
    global rednessImage
    global greennessImage
    global intenseImage

    global blueKeypoints
    global redKeypoints
    global greenKeypoints

    
    cv_image = bridge.imgmsg_to_cv2(img_rgb, desired_encoding = "bgr8")

    #Some preprocessing of the image to make it easier on the blob finder.
    '''
    maxIntensity = 255.0
    intensityRange = numpy.arange(maxIntensity)
    phi = 1
    theta = 1
    newImage_deShadowed = (maxIntensity/phi) * (cv_image/(maxIntensity/theta)) ** 0.5
    newImage_deShadowed = numpy.array(newImage_deShadowed, dtype='uint8')
    '''



    #Preprocessing done!
    cv_image = cv2.GaussianBlur(cv_image, (5,5), 25)


    #Convert to an HSV image.
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #rospy.loginfo("size of image:{}".format(img_hsv.shape))
    

    #These are the HSV ranges of the color Navy Blue. These will be the Enemy, as jeans are a horrid plague upon the world of fashion.
    #minBlueVals = numpy.array([110,80,30])
    #maxBlueVals = numpy.array([130,255,255])
    minBlueVals = numpy.array([110,150,50],dtype=numpy.uint8)
    maxBlueVals = numpy.array([130,255,255],dtype=numpy.uint8)


    minRedVals = numpy.array([150,150,50],dtype=numpy.uint8)
    maxRedVals = numpy.array([180,255,255],dtype=numpy.uint8)
    #rospy.loginfo ("minRedVals : {} maxRedVals :{}".format(minRedVals, maxRedVals))
    #These are the HSV ranges of the color Green. These will be the Waypoints: things to patrol between.

    minGreenVals = numpy.array([33,80,40],dtype=numpy.uint8)
    maxGreenVals = numpy.array([100,255,255],dtype=numpy.uint8)

    bluMask = cv2.inRange(img_hsv, minBlueVals, maxBlueVals)
    cv_bluenessImage = cv2.bitwise_and(cv_image, cv_image, mask = bluMask)

    redMask = cv2.inRange(img_hsv, minRedVals, maxRedVals)
    cv_rednessImage = cv2.bitwise_and(cv_image, cv_image, mask = redMask)

    greenMask = cv2.inRange(img_hsv, minGreenVals, maxGreenVals)
    cv_greennessImage = cv2.bitwise_and(cv_image, cv_image, mask = greenMask)

    #blobFinderParams = blobFinder.Params()


    blueKeypoints = blobFinder.detect(cv2.bitwise_not(cv_bluenessImage))
    redKeypoints = blobFinder.detect(cv2.bitwise_not(cv_rednessImage))
    greenKeypoints = blobFinder.detect(cv2.bitwise_not(cv_greennessImage))
    #rospy.loginfo("blueKeypoints :{} redKeypoint : {} greenKeypoint :{}".format(len(blueKeypoints), len(redKeypoints), len(greenKeypoints)))
    if len(blueKeypoints) != 0:
        for keypoint in blueKeypoints:
            pass
            #rospy.loginfo("blueKeypoint {}:{},{}".format(keypoint.pt[0], keypoint.pt[1],len(blueKeypoints)))
            cv2.drawKeypoints(image=cv_bluenessImage, keypoints=blueKeypoints, outImage=cv_bluenessImage, color = [0,0,255])
    elif len(greenKeypoints) != 0:
        for keypoint in greenKeypoints:
            pass
            #rospy.loginfo("greenKeypoint {}:{},{}".format(keypoint.pt[0], keypoint.pt[1],len(greenKeypoints)))
            cv2.drawKeypoints(image=cv_greennessImage, keypoints=greenKeypoints, outImage=cv_greennessImage, color = [0,255,0])

    elif len(redKeypoints) != 0:
        for keypoint in redKeypoints:
            pass
            #rospy.loginfo("redKeypoint {}:{},{}".format(keypoint.pt[0], keypoint.pt[1],len(redKeypoints)))
            cv2.drawKeypoints(image=cv_rednessImage, keypoints=redKeypoints, outImage=cv_rednessImage, color = [255,0,0])

    else:
        rospy.loginfo("nothing detected")
        #rospy.loginfo("Size of Image {}:{}".format(keypoint, keypoint.size))
    
    '''
    bgrImageBlue = cv2.cvtColor(cv_bluenessImage, cv2.COLOR_HSV2BGR)
    bgrImageRed = cv2.cvtColor(cv_rednessImage, cv2.COLOR_HSV2BGR)
    bgrImageGreen = cv2.cvtColor(cv_greennessImage, cv2.COLOR_HSV2BGR)
    '''
    bluenessImage = bridge.cv2_to_imgmsg(cv_bluenessImage, encoding="bgr8")
    rednessImage = bridge.cv2_to_imgmsg(cv_rednessImage, encoding="bgr8")
    greennessImage = bridge.cv2_to_imgmsg(cv_greennessImage, encoding="bgr8")
    t2 = time.clock()
    rospy.loginfo("Time taken in image processing:{}".format(t2-t1))


#Stores odometry data into our global variables.
def odomCallback(data):
    global myPose
    global actualVelocityX
    global actualAngularVelocityZ
    actualVelocityX = data.twist.twist.linear.x
    actualAngularVelocityZ = data.twist.twist.angular.z

    theta = quatToEuler(data.pose.pose.orientation.w, data.pose.pose.orientation.z )
    myPose = [data.pose.pose.position.x, data.pose.pose.position.y, theta ]
    #rospy.loginfo("Current Pose: {}".format(myPose))

#Given orientation w and z (quaternion form), converts 
def quatToEuler(orientationW, orientationZ):
    theta = math.atan2(2*orientationW * orientationZ, orientationW * orientationW - orientationZ*orientationZ)

    return theta

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

#SelfPose will have: [x,y, orientZ, orientW]
#EnemyPose will have: [x,y]
def enemyBehavior(selfPose, enemyPose):
    global angKp
    global linearKp

    global Max_linear_speed
    global Max_angular_speed

    dist_robot_goal = math.sqrt((enemyPose[0]-selfPose[0])**2+(enemyPose[1]-selfPose[1])**2)
    theta_desired = math.atan2(enemyPose[1]-selfPose[1], enemyPose[0]-selfPose[0])
    theta = selfPose[2]
    #theta = math.atan2(2*(orientation_x*orientation_y+orientation_w*orientation_z),orientation_w * orientation_w +orientation_x*orientation_x-orientation_y*orientation_y - orientation_z*orientation_z)
    gamma = theta_desired - theta

    theta_error = math.atan2(math.sin(gamma),math.cos(gamma))
    rospy.loginfo("Theta Error: {}".format(theta_error))

    vel=Twist()
    vel.linear.x=0
    vel.angular.z=0

    vel.angular.z = angKp * theta_error
    if vel.angular.z > Max_angular_speed:
        vel.angular.z = Max_angular_speed
    elif vel.angular.z < -Max_angular_speed:
        vel.angular.z = -Max_angular_speed

    vel.linear.x = linearKp * dist_robot_goal;
    if vel.linear.x > Max_linear_speed:
        vel.linear.x = Max_linear_speed
    elif vel.linear.x < -Max_linear_speed:
        vel.linear.x = -Max_linear_speed 

    if dist_robot_goal < 0.6:
        rospy.loginfo("Arrived!")
        vel.linear.x = 0
        vel.angular.z = 0

    return vel

#Given a list of seen entities, find the closest one to oneself.
def findClosestEntity(selfPose, entityList):
    if(len(entityList) == 0):
        rospy.loginfo("findClosestEntity: entityList is empty!\n")
        return None

    minDist = None
    minDistEntity = None
    for entity in entityList:
        distance = numpy.hypot(selfPose[0] - entity.position[0], selfPose[1] - entity.position[1])
        if(minDist is None or distance < minDist):
            minDist = distance
            minDistEntity = entity
    return minDistEntity



def main():
    global angKp
    global linearKp


    global actualVelocityX
    global actualAngularVelocityZ

    global myPose
    global depthImage
    global goal_dist
    global goal_ang 
    global follow_dist

    global bluenessImage
    global rednessImage
    global greennessImage
    global intenseImage

    global blueKeypoints
    global redKeypoints
    global greenKeypoints

    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    blueImgPub = rospy.Publisher('/blueImage', Image, queue_size=10)
    redImgPub = rospy.Publisher('/redImage', Image, queue_size=10)
    greenImgPub = rospy.Publisher('/greenImage', Image, queue_size=10)
    #intensifiedImgPub = rospy.Publisher('/intenseImage', Image, queue_size=10)

    rospy.init_node('lab4', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('camera/rgb/image_raw', Image, imageCallback)
    rospy.Subscriber('/camera/depth/image_raw', Image, IRCallback)
    rate = rospy.Rate(10)  # Main loop: 10Hz

    while not rospy.is_shutdown():
        #Do something: Probably try to approach the nearest thing to guard. That can come later, I guess.
        t1 = time.clock()
        if(bluenessImage is not None):
            blueImgPub.publish(bluenessImage)
        if(rednessImage is not None):
            redImgPub.publish(rednessImage)
        if(greennessImage is not None):
            greenImgPub.publish(greennessImage)
        #if(intenseImage is not None):
        #    intensifiedImgPub.publish(intenseImage)

        #First, localize yer keypoints.
        if(depthImage is not None):
            seenBlues = localizeKeypoints(blueKeypoints, depthImage, "Guard", myPose)
            seenReds = localizeKeypoints(redKeypoints, depthImage, "Enemy", myPose)
            seenGreens = localizeKeypoints(greenKeypoints, depthImage, "Waypoint", myPose)

            estimatedReds, numReds = clusterPositions(seenReds)

            rospy.loginfo("Cluster Positions Reds: {}".format(estimatedReds))

            closestEnemy = findClosestEntity(myPose, seenReds)

            if closestEnemy is not None:
                vel = enemyBehavior(myPose, closestEnemy.position)

                #rospy.loginfo("My Pose: {}".format(myPose))
                #rospy.loginfo("Enemy Pose: {}".format(closestEnemy.position))
                #rospy.loginfo("Linear Vel: {}".format(actualVelocityX))
                #rospy.loginfo("Angular Vel: {}".format(actualAngularVelocityZ))


                #vel_pub.publish(vel)
        t2 = time.clock()
        rospy.loginfo("time: {}".format(float(t1-t2)))



        '''for blueObj in seenBlues:
            rospy.loginfo("{} Spotted! \n Pos X: {} \n Pos Y: {}\n".format(blueObj.category, blueObj.position[0], blueObj.position[1]))
            #mapEntity(blueObj)

        for redObj in seenReds:
            rospy.loginfo("{} Spotted! \n Pos X: {} \n Pos Y: {}\n".format(redObj.category, redObj.position[0], redObj.position[1]))
            #mapEntity(redObj)

        for greenObj in seenGreens:
            rospy.loginfo("{} Spotted! \n Pos X: {} \n Pos Y: {}\n".format(greenObj.category, greenObj.position[0], greenObj.position[1]))
            #mapEntity(greenObj)
        '''
        rate.sleep()

    vel_pub.publish(Twist())
      
if __name__ == "__main__":
    main()    