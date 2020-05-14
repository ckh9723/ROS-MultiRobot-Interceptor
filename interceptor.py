#! /usr/bin/env python

import rospy
import time
import math
import actionlib
import datetime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Create constants to indicate action status
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# Variables to be used globally
criminal_x = 0.0
criminal_y = 0.0
criminal_z = 0.0
criminal_w = 0.0
current_known_criminal_x = 0.0
current_known_criminal_y = 0.0
interceptor_x = 0.0
interceptor_y = 0.0
interceptor_z = 0.0
interceptor_w = 0.0
path_x = 0.0
path_y = 0.0
path_z = 0.0
path_w = 0.0
goal_x = 0.0
goal_y = 0.0
goal_z = 0.0
goal_w = 0.0
x_coor = 0.0
y_coor = 0.0
last_index = 0
count = 0
intercept_points = []
intercept_index = 2
point_id = 0
criminal_reached_goal = False
loop_count = 0

# Action client declared here to be used globally
client = actionlib.SimpleActionClient("tb3_0/move_base",MoveBaseAction)


def setup_operation():
    # Initialize the get main node
    rospy.init_node('criminal_interceptor')

    # Define the callback function and subscriber object to get odometry information of the criminal
    def amcl_callback(msg):
	global criminal_x
	global criminal_y
	global criminal_z
	global criminal_w

	criminal_x = msg.pose.pose.position.x
	criminal_y = msg.pose.pose.position.y
	criminal_z = msg.pose.pose.orientation.z
	criminal_w = msg.pose.pose.orientation.w

    criminal_amcl_sub = rospy.Subscriber('tb3_1/amcl_pose',PoseWithCovarianceStamped,amcl_callback)


    # Define the callback function and subscriber object to get odometry information of the interceptor
    def interceptor_callback(msg):
	global interceptor_x
	global interceptor_y
	global interceptor_z
	global interceptor_w

	interceptor_x = msg.pose.pose.position.x
	interceptor_y = msg.pose.pose.position.y
	interceptor_z = msg.pose.pose.orientation.z
	interceptor_w = msg.pose.pose.orientation.w

    interceptor_amcl_sub = rospy.Subscriber('tb3_0/amcl_pose',PoseWithCovarianceStamped,interceptor_callback)


    # Define the callback function and subscriber object to get path information of the criminal
    def path_callback(msg):
	global path_x
	global path_y
	global path_z
	global path_w
	global last_index
	global count
	global intercept_points

	if count == 0:
	    last_index = len(msg.poses)-1
	    current_index = 10
	    while current_index < last_index:
	        intercept_points.append(msg.poses[current_index])
	        current_index += 10 
	    
	    count+=1

	    #print(intercept_points)
	    start_operation()

    plan_sub = rospy.Subscriber('tb3_1/move_base/NavfnROS/plan',Path,path_callback)


    # Define the callback function and subscriber object to get criminal goal information
    def goal_callback(msg):
	global goal_x
	global goal_y
	global goal_z
	global goal_w
        global goal_seq

	goal_x = msg.pose.position.x
	goal_y = msg.pose.position.y
	goal_z = msg.pose.orientation.z
	goal_w = msg.pose.orientation.w

    criminal_goal_sub = rospy.Subscriber('tb3_1/move_base_simple/goal',PoseStamped,goal_callback)


    # Keep the program from exiting
    rospy.spin()



def start_operation():
    global index
    global client
    global criminal_x
    global criminal_y
    global interceptor_x
    global interceptor_y
    global x_coor
    global y_coor
    global intercept_index
    global point_id
    global criminal_reached_goal
    global loop_count
    
    send_goal(intercept_index,point_id)
    state = client.get_state()
    while state < DONE:
        distance_criminal_point = math.sqrt((criminal_x-x_coor)**2 + (criminal_y-y_coor)**2)
	if distance_criminal_point < 0.25:
	    if (intercept_index >= len(intercept_points)-2):
		distance_criminal_goal = math.sqrt((criminal_x-goal_x)**2 + (criminal_y-goal_y)**2)
		if distance_criminal_goal < 0.1:
	            print('Interception failed.')
		    criminal_reached_goal = True
		    break
	    else:
	        intercept_index+=2
	        send_goal(intercept_index,point_id)
    	state = client.get_state()
    
    if criminal_reached_goal == False:
        currently_known_criminal_x = criminal_x
        currently_known_criminal_y = criminal_y
        while True: 
	    if abs(currently_known_criminal_x - criminal_x) > 0.3 or abs(currently_known_criminal_y - criminal_y) > 0.3:
	        if (intercept_index >= len(intercept_points)-2):
		    distance_criminal_goal = math.sqrt((criminal_x-goal_x)**2 + (criminal_y-goal_y)**2)
		    if distance_criminal_goal < 0.1:
	                print('Interception failed.')
			criminal_reached_goal = True
			break
	        else:
	            intercept_index+=2
	            send_goal(intercept_index,point_id)

    	    loop_count+=1
	    if loop_count == 25000000:
		break

    if criminal_reached_goal == False:
	print("Interception is successful!")  
    
    rospy.is_shutdown()


def send_goal(index,seq):
    global client
    global x_coor
    global y_coor
    global point_id

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    x_coor = intercept_points[index].pose.position.x
    y_coor = intercept_points[index].pose.position.y
    goal.target_pose.pose.position.x = x_coor 
    goal.target_pose.pose.position.y = y_coor
    goal.target_pose.pose.orientation.w = 1.0 - criminal_z
    client.send_goal(goal)

    if seq == 0:
        print("First interception point sent.")    
    else:
	print("Criminal is too close, new interception point sent.")

    point_id+=1



# Main method to start the program
if __name__ == "__main__":
    try:
        setup_operation()
    except rospy.ROSInterruptException:
	rospy.loginfo("Operation interrupted, aborting operation...")

