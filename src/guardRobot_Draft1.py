#!/usr/bin/env python
import rospy
import math
import cv2
import numpy
import scipy.cluster.hierarchy as hcluster
import time
import pyaudio  
import wave  


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
global angKd
global linearKd

global prevError_ang
global prevError_linear

global depthImage



global myPose
myPose = [0,0,0,0]

global actualVelocityX
global actualAngularVelocityZ

global Max_angular_speed
global Max_linear_speed

Max_angular_speed = 1.0
Max_linear_speed = 0.5

linearKp = 0.3
linearKd = 0.1

angKp = 0.5
angKd = 0.2

prevError_ang = 0.0
prevError_linear = 0.0

depthImage = None


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
depthImage = None

blueKeypoints = []
redKeypoints = []
greenKeypoints = []

currentEnemy = None
testEnemyList = []

guardList = []
waypointList = []

#Initialize bridge object, for converting ROS images to openCV images and vice versa.
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


#
def IRCallback(depthImg):
    global depthImage
    depthImage = depthImg
    depthImageData = numpy.array(depthImage.data)
   # rospy.loginfo('printing the image type: {}'.format((depthImageData.data)))
    #print 'Shape of depthImage : {}'.format((depthImage.data))


#Takes the blob keypoints, figures out the angles of the blobs from the robot.
# Then, it uses the depth camera to figure out how far the blob is in front of the robot and its relative angle from the robot, then calculates the world coordinates of that spot from this information.
def localizeKeypoints(blob_keyPoints, depthImage, category, currentPose):

    seenEntities = []
    fieldOfView = 58.0

    degreesPerPixel = 1.0/11.0

    positions = []
    positionsAverage = 0.0

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
            #rospy.loginfo("Measurement is not a number, or invalid.")
            return seenEntities

        angleOfReading_deg = blobXPixel * degreesPerPixel
        angleOfReading_rad = numpy.pi/180.0 * angleOfReading_deg

        #Here, we take advantage of the fact that we know the field of view of the Kinect Sensor: 58 degrees.
        # We know that the angle of the reading will be at a certain theta for each group of 11 pixels in the image, from:
        # image_width / degrees_FOV

        worldAngle = currentPose[2] + (((numpy.pi/180.0) * fieldOfView/2.0) - angleOfReading_rad)

        #angleOfReading_deg = angleOfReading_deg + 61
        #worldAngle = numpy.pi/180.0 * angleOfReading_deg
        #rospy.loginfo("Angle of Reading: {}".format(worldAngle))

        positionOfObj = numpy.array([currentPose[0] + math.cos(worldAngle) * depthMeasurement, currentPose[1] + math.sin(worldAngle) * depthMeasurement])
        positions.append(positionOfObj)
        #rospy.loginfo("Positions:{}".format(positions))

        #rospy.loginfo("Blob pose: {}".format(positionOfObj))
        seenEntities.append(Entity(category, positionOfObj))

    positions = numpy.asarray(positions)
    if(numpy.any(positions)):
        positionsAverage = [numpy.mean(positions[:,0]), numpy.mean(positions[:,1])]


    return seenEntities


#The assumption is that the robot will only see one identifiable object at any given time. Otherwise, the average will probably be the midpoint between the two things.
def averagePositions(entityList):
    positionList = []
    for entity in entityList:
        positionList.append(entity.position)

    if(positionList):
        posXAvg = 0.0
        posYAvg = 0.0

        for position in positionList:
            posXAvg += position[0]
            posYAvg += position[1]

        posXAvg = posXAvg / len(positionList)
        posYAvg = posYAvg / len(positionList)

        averagePosition = [posXAvg, posYAvg]
        return [Entity(entityList[0].category, averagePosition)]
    else:
        return None

#Given an entity with a position and a category, it first figures out if the entity is likely to be the same entity as an already existing entity.
# The likilihood is inversely proportional to the distance between the new entity and an existing entity. 
# Then, if the probability of matching an existing object falls below a certain threshold, the entity is mapped as a new entity. 
def mapEntity(entity):
    global currentEnemy
    global guardList
    global waypointList
    distThreshold = 0.8

    if entity.category == "Enemy":
        currentEnemy = entity
        return True

    elif entity.category == "Guard":
        matchesExisting = checkDistances(entity, guardList, distThreshold)
        #rospy.loginfo("Matches existing: {}".format(matchesExisting))
        if(not matchesExisting):
            guardList.append(entity)
            return True
        else:
            return False

    else:
        matchesExisting = checkDistances(entity, waypointList, distThreshold)
        if(not matchesExisting):
            waypointList.append(entity)
            return True
        else:
            return False

# True if a match, false if not.
def checkDistances(newEntity, entityList, dThreshold):
    distList = []
    for entity in entityList:
        distList.append(numpy.hypot(newEntity.position[0] - entity.position[0], newEntity.position[1] - entity.position[1]))
    if (distList):
        maxDist = max(distList)

    if (not distList or maxDist > dThreshold):
        return False

    else:
        return True



#This function's responsible for supplying a collection of blobs that fit the given criteria - for now, just "blueness". 
def imageCallback(img_rgb):
    #t1 = time.clock()

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

    minGreenVals = numpy.array([33,150,40],dtype=numpy.uint8)
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
        pass
        #rospy.loginfo("nothing detected")
        #rospy.loginfo("Size of Image {}:{}".format(keypoint, keypoint.size))
    
    bluenessImage = bridge.cv2_to_imgmsg(cv_bluenessImage, encoding="bgr8")
    rednessImage = bridge.cv2_to_imgmsg(cv_rednessImage, encoding="bgr8")
    greennessImage = bridge.cv2_to_imgmsg(cv_greennessImage, encoding="bgr8")
    #t2 = time.clock()
    #ospy.loginfo("Time taken in image processing:{}".format(t2-t1))


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

#Given orientation w and z (quaternion form), converts into euler value for yaw.
def quatToEuler(orientationW, orientationZ):
    theta = math.atan2(2*orientationW * orientationZ, orientationW * orientationW - orientationZ*orientationZ)
    return theta

#Stores the occupancy grid created by the SLAM algorithm (third-party, perhaps later our own).
def mapCallback(data):
    global occGrid
    occGrid = data



#Given a leg of the journey, return the goal point that should be attained.
def explorationPlanner(leg, sizeOfSquare, startPose):
    goalPose = []
    if leg == 1:
        goalPose = [startPose[0] + sizeOfSquare, startPose[1]]
    elif leg == 2:
        goalPose = [startPose[0] + sizeOfSquare, startPose[1] + sizeOfSquare]
    elif leg == 3: 
        goalPose = [startPose[0], startPose[1] + sizeOfSquare]
    else:
        goalPose = startPose
    #rospy.loginfo("Goal Position:{}".format(goalPose))
    return goalPose

#Given a set of waypoints and a round number, determine which waypoint to go to.
def patrolPlanner(waypointList, currentWaypointIndex, round):
    if(round >= 2):
        return True

    return waypointList[currentWaypointIndex].position


#Given a center of the circle, supplies a point on a circle depending on what fraction of the circumference has been attained.
def guardPlanner(guardPose, circle_radius, indexCirclePoint):

    cir_x_goals = numpy.array([x/50.0 for x in range(0,628,1)])
    cir_y_goals = numpy.array([x/50.0 for x in range(0,628,1)])

    cir_goal_points_x = guardPose[0] + circle_radius * numpy.cos(cir_x_goals)
    cir_goal_points_y = guardPose[1] + circle_radius * numpy.sin(cir_y_goals)

    #If we've completed the trajectory, return True
    if indexCirclePoint == len(cir_goal_points_x) - 1:
        return True

    return [cir_goal_points_x[indexCirclePoint] , cir_goal_points_y[indexCirclePoint]]



def controller(goalPose, currentPose):
    global prevError_linear, prevError_ang
    global linearKp, linearKd
    global angKp, angKd
    global Max_linear_speed, Max_angular_speed

    goal_point_x = goalPose[0]
    goal_point_y = goalPose[1]

    x_pose = currentPose[0]
    y_pose = currentPose[1]

    theta_desired = math.atan2(goal_point_y-y_pose, goal_point_x-x_pose)
    theta = currentPose[2]
    theta_error = theta_desired - theta
    theta_error = math.atan2(math.sin(theta_error),math.cos(theta_error))

    dist_robot_goal = math.sqrt((goal_point_x-x_pose)**2+(goal_point_y-y_pose)**2)


    vel = Twist()
    vel.angular.z = angKp * theta_error + angKd * (theta_error - prevError_ang)



    if(dist_robot_goal < 0.1):
        rospy.loginfo("Reached goal point!")
        return True
    rospy.loginfo("dist,theta,theta_desired,theta_error:%.2f,%.2f,%.2f,%.2f:",dist_robot_goal,theta,theta_desired,theta_error)
    

    #Cap velocity values.
    if vel.angular.z > Max_angular_speed:
        vel.angular.z = Max_angular_speed;
    elif vel.angular.z < -Max_angular_speed:
        vel.angular.z = -Max_angular_speed;
    if theta_error < 0.1:
        vel.linear.x = linearKp * dist_robot_goal + linearKd * (dist_robot_goal - prevError_linear);
        if vel.linear.x > Max_linear_speed:
            vel.linear.x = Max_linear_speed
        elif vel.linear.x < -Max_linear_speed:
            vel.linear.x = -Max_linear_speed

    prevError_linear = dist_robot_goal
    prevError_ang = theta_error

    rospy.loginfo("Linear X Vel: {}\nAngular Z Vel:{}\n".format(vel.linear.x, vel.angular.z)) 
    rospy.loginfo("Goal Position:{}".format(goalPose))
    return vel

def playMusic(filename = "/home/robot6/catkin_ws/src/guard_robot/media/doggy.wav"):
    #music(bark):
    #define stream chunk   
    chunk = 1024  
    #open a wav format music  
    f = wave.open("/home/robot6/catkin_ws/src/guard_robot/media/doggy.wav","rb")  
    #instantiate PyAudio  
    p = pyaudio.PyAudio()  
    #open stream  
    stream = p.open(format = p.get_format_from_width(f.getsampwidth()),channels = f.getnchannels(),rate = f.getframerate(),output = True)  
    #read data  
    data = f.readframes(chunk)  
    #paly stream  
    while data:  
        stream.write(data)  
        data = f.readframes(chunk) 


def main():
    global angKp
    global linearKp
    global Max_linear_speed
    global Max_angular_speed

    global actualVelocityX
    global actualAngularVelocityZ

    global myPose
    global depthImage


    global bluenessImage
    global rednessImage
    global greennessImage
    global intenseImage

    global blueKeypoints
    global redKeypoints
    global greenKeypoints

    global currentEnemy
    global guardList
    global waypointList

    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)


    blueImgPub = rospy.Publisher('/blueImage', Image, queue_size=10)
    redImgPub = rospy.Publisher('/redImage', Image, queue_size=10)
    greenImgPub = rospy.Publisher('/greenImage', Image, queue_size=10)
    #intensifiedImgPub = rospy.Publisher('/intenseImage', Image, queue_size=10)

    rospy.init_node('lab4', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('camera/rgb/image_raw', Image, imageCallback)
    rospy.Subscriber('/camera/depth/image_raw', Image, IRCallback)
    rospy.Subscriber('/map', OccupancyGrid, mapCallback)

    rate = rospy.Rate(10)  # Main loop: 10Hz

    #Phase 1: Exploration - In which we map out the guard and the waypoint positions, storing them in memory.
    phase = 1

    numGreensSeen = 0
    maxGreens = 2
    green_flag = 0

    numBluesSeen = 0
    maxBlues = 1
    blue_flag = 0

    legOfExplorationRoute = 1

    explorationStartPose = [0,0,0]

    #Phase 2: Patrol
    currentWaypointIndex = 0
    currentGuardIndex = 0
    currentCircleIndex = 0

    patrolRound = 0
    guardMode = 0



    while not rospy.is_shutdown():
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

            #Assume that we only see one thing at a time.
            estimatedBlue = averagePositions(seenBlues)
            estimatedRed = averagePositions(seenReds)
            estimatedGreen = averagePositions(seenGreens)

            if(estimatedBlue is not None):
                pass
                rospy.loginfo("Average Positions Blue: {}".format(estimatedBlue[0].position))
            if(estimatedRed is not None):
                pass
                rospy.loginfo("Average Positions Red: {}".format(estimatedRed[0].position))
            if(estimatedGreen is not None):
                pass
                rospy.loginfo("Average Positions Green: {}".format(estimatedGreen[0].position))


            #The first phase, exploration.
            if(phase == 1):
                #Mapping Section: Chart the things we've seen in our movements.
                if(numGreensSeen < maxGreens and estimatedGreen is not None):
                    successfulMap = mapEntity(estimatedGreen[0])
                    if(successfulMap):
                        numGreensSeen +=1
                if(numBluesSeen < maxBlues and estimatedBlue is not None):
                    successfulMap = mapEntity(estimatedBlue[0])
                    if(successfulMap):
                        numBluesSeen +=1

                #Velocity/Trajectory Section: Create a velocity command, given a goal position. 

                rospy.loginfo("Position:{}".format(myPose))

                currentExplorationGoal = explorationPlanner(legOfExplorationRoute, 1, explorationStartPose)
                rospy.loginfo("Type of cur:{}".format(currentExplorationGoal))
                vel_cmd = controller(currentExplorationGoal, myPose)

                if(vel_cmd == True):
                    rospy.loginfo("Going to next leg: {}".format(legOfExplorationRoute + 1))
                    legOfExplorationRoute += 1
                else:
                    rospy.loginfo("Haven't reached goal yet.")
                    vel_pub.publish(vel_cmd)
            '''
            for guard in guardList:
                rospy.loginfo("Guard Mapped Position: {}".format(guard.position))
            for waypoint in waypointList:
                rospy.loginfo("Waypoint Mapped Position: {}".format(waypoint.position))            
            '''
            if (numGreensSeen == maxGreens) and (numBluesSeen == maxBlues):
                phase = 2
                rospy.loginfo("Got blue and green points!")

            if(guardList):
                rospy.loginfo("Guard Points:{}".format(guardList))
            if(waypointList):
                rospy.loginfo("Waypoints: {}".format(waypointList))
                
            if phase == 2:

                if estimatedRed:
                    playMusic() 
                    mapEntity(estimatedRed[0])
                    vel_cmd = controller(currentEnemy.position,myPose)
                    vel_pub.publish(vel_cmd)
                    rospy.loginfo("***BARK BARK BARK****")

                else:
                    if guardMode == 0:
                        goalPoint = patrolPlanner(waypointList, currentWaypointIndex, patrolRound)
                        #Check if trajectory finished
                        if(goalPoint == True):
                            rospy.loginfo("*************Patrol finished!***************")
                            guardMode = 1
                        
                        else:
                            vel_cmd = controller(goalPoint, myPose)
                            #Check if goal within trajectory reached.
                            if(vel_cmd == True):
                                rospy.loginfo("Waypoint {} reached".format(currentWaypointIndex))
                                currentWaypointIndex = (currentWaypointIndex + 1) % 2
                                patrolRound += 1
                            else:
                                rospy.loginfo("Following waypoint trajectory.")
                                vel_pub.publish(vel_cmd)
                    else:
                        goalPoint = guardPlanner(guardList[currentGuardIndex].position, 2, currentCircleIndex)
                        #Check if trajectory finished
                        if(goalPoint == True):
                            rospy.loginfo("*********Guard circling finished!*************")
                            guardMode = 0

                        else:
                            vel_cmd = controller(goalPoint, myPose)
                            #Check if goal within trajectory reached.
                            if(vel_cmd == True):
                                currentCircleIndex += 1
                            else:
                                rospy.loginfo("Following guard trajectory.")
                                vel_pub.publish(vel_cmd)




                


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

    #vel_pub.publish(Twist())
      
if __name__ == "__main__":
    main()    