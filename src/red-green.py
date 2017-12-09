#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


global x_goal
global y_goal
global Max_linear_speed
global Max_angular_speed
global get_inital_pose
global angKp
global linearKp
global x_pose
global y_pose
global orientation_z
global orientation_w
global circle_radius

global circle_center 
global green_pose_1
global green_pose_2

green_pose_1 = [2.0,2.0]#green point 1
green_pose_2 = [4.0,2.0]#green point 2
circle_radius = 0.5
circle_center = [1.0,0] #blue point

x_goal = [math.cos(x/50.0) * circle_radius + circle_center[0] for x in range(0+180,314+180,1)]
y_goal = [math.sin(x/50.0) * circle_radius + circle_center[1] for x in range(0+180,314+180,1)]



x_goal.append(green_pose_1[0])
y_goal.append(green_pose_1[1])
x_goal.append(green_pose_2[0])
y_goal.append(green_pose_2[1])

Max_linear_speed = 0.5
Max_angular_speed = 0.5 
get_inital_pose = False
angKp = 1
linearKp = 0.3
x_pose = 0
y_pose = 0
orientation_z = 0
orientation_w = 0
def odomCallback(data):  
    global x_pose
    global y_pose
    global orientation_z
    global orientation_w
    x_pose=data.pose.pose.position.x
    y_pose=data.pose.pose.position.y 
    orientation_z=data.pose.pose.orientation.z 
    orientation_w=data.pose.pose.orientation.w 
    #rospy.loginfo("position point:%.2f,%.2f:",x_pose,y_pose)


def main():
    global x_goal
    global y_goal
    global Max_linear_speed
    global Max_angular_speed
    global get_inital_pose
    global angKp
    global linearKp
    global x_pose
    global y_pose
    global orientation_z
    global orientation_w
    global circle_radius
    global circle_center
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.init_node('lab2_6', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rate = rospy.Rate(10)  # Main loop: 10Hz

    vel=Twist()
    vel.linear.x=0
    vel.angular.z=0
    green = 0


    for i in range(len(x_goal)):
        while not rospy.is_shutdown():
            if green == 0: 
                goal_point_x = x_goal[i]
                goal_point_y = y_goal[i]
            theta_desired = math.atan2(goal_point_y-y_pose,goal_point_x-x_pose)
            theta = math.atan2(2*orientation_w * orientation_z,orientation_w * orientation_w -orientation_z*orientation_z)
            theta_error = theta_desired - theta
            gamma = math.atan2(math.sin(theta_error),math.cos(theta_error))
            vel.angular.z = angKp * gamma
            dist_robot_goal = math.sqrt((goal_point_x-x_pose)**2+(goal_point_y-y_pose)**2)
            if vel.angular.z > Max_angular_speed:
                vel.angular.z = Max_angular_speed;
            elif vel.angular.z < -Max_angular_speed:
                vel.angular.z = -Max_angular_speed;
            if abs(gamma) < 0.1:
                dist_robot_goal = math.sqrt((goal_point_x-x_pose)**2+(goal_point_y-y_pose)**2)
                vel.linear.x = linearKp * dist_robot_goal;
            else:
                vel.linear.x = 0
            if vel.linear.x > Max_linear_speed:
                    vel.linear.x = Max_linear_speed
            elif vel.linear.x < -Max_linear_speed:
                    vel.linear.x = -Max_linear_speed 
            if dist_robot_goal < 0.2:
                if i < len(x_goal)-2:
                    rospy.loginfo("Arrived goal on circle")
                    break
                else:
                    rospy.loginfo("Move around the green points")
                    rospy.loginfo("next point:%.2f,%.2f",goal_point_x,goal_point_y)
                    green = 1
                    if goal_point_x == x_goal[-1]:
                        goal_point_x = x_goal[-2]
                        goal_point_y = y_goal[-2]
                    elif goal_point_x == x_goal[-2]:
                        goal_point_x = x_goal[-1]
                        goal_point_y = y_goal[-1]
                    
                    #rospy.loginfo("green point:%.2f,%.2f",y_goal[len(y_goal)-2],y_goal[len(y_goal)-1])
            rospy.loginfo("distance:%.2f:",math.sqrt((x_pose-circle_center[0])**2+(y_pose-circle_center[1])**2))
            rospy.loginfo("position point:%.2f,%.2f:",x_pose,y_pose)
            rospy.loginfo("dist,gamma:%.2f,%.2f:",dist_robot_goal,gamma)
            vel_pub.publish(vel)
            rate.sleep()

      
if __name__ == "__main__":
    main()       
