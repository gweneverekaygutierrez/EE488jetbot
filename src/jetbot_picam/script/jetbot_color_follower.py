#!/usr/bin/python

# Old: !/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D

# Global variables ---------------------------------------------------
BALL_REACH = 0
GOAL_REACH = 0
SPIN_CHECK_BALL = 0
SPIN_CHECK_GOAL = 0

# States
STATE_SPIN_BALL = 1 
# Find ball ^
STATE_FIND_BALL = 2
# Go to the ball ^^
STATE_SPIN_GOAL = 3
STATE_FIND_GOAL = 4
STATE_KICK = 5

# First state
STATE = STATE_SPIN_BALL

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return

	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

def ball_detect_callback(msg): 
	# this is called only if rqt (ball channel) detected something 
	# have to make sure it is the "ball", as the camera is noisy
	global BALL_REACH, SPIN_CHECK_BALL
	SPIN_CHECK_BALL = 1
	# coordinate(x,y) and height of detector of ball/ goal
	if STATE == STATE_FIND_BALL:
		bbox_x = msg.center.x
		bbox_y = msg.center.y
		bbox_height = msg.size_y
                bbox_width = msg.size_x
		# parameter for 
		H_THR = 60

        # rospy.loginfo("WIDTH %d", bbox_width)

		# P-controller for x-position
		Kp = 1/1.3 # P control gain	
		speed = Kp* abs(bbox_x - 640/2) / (640/2) # P controller
		speed = min(speed, 0.50)		  # saturation of speed
		if speed <= 0.20:		# speed is too low, motor cannot move
			speed = 0.25

		rospy.loginfo("x:%d, y:%d, height:%d, speed: %f", bbox_x, bbox_y, bbox_height, speed)

		if bbox_x > 640/2 + 90:
			rospy.loginfo("left")
			set_speed(motor_left_ID, -speed)
			set_speed(motor_right_ID, speed)
		elif bbox_x < 640/2 - 90:
			rospy.loginfo("right")
			set_speed(motor_left_ID, speed)
			set_speed(motor_right_ID, -speed)
                elif (bbox_height + bbox_width)/2 < H_THR:
                        # P-controller for distance
    	                Kp_h = 1/1.3 # P control gain
                        speed_h = Kp_h* abs(bbox_height - H_THR) / (H_THR) # P controller
                        speed_h = min(speed_h, 0.50)                  # saturation of speed
                        if speed_h <= 0.2:               # speed is too low, motor cannot move
                            speed_h = 0.25

			rospy.loginfo("forward")
			set_speed(motor_left_ID, speed_h)
			set_speed(motor_right_ID, speed_h)
                        time.sleep(0.1)
                else:
			rospy.loginfo("stop")
			BALL_REACH = 1 # reach ball, set flag to spin to get goal in rqt_image_view
			set_speed(motor_left_ID,  0.0)
			set_speed(motor_right_ID, 0.0)
		

def goal_detect_callback(msg): 
	# this is called only if rqt (goal channel) detected something
	# have to make sure it is the "ball", as the camera is noisy
	global BALL_REACH, GOAL_REACH, SPIN_CHECK_GOAL
	SPIN_CHECK_GOAL = 1
	if STATE == STATE_FIND_GOAL:
		bbox_x = msg.center.x
		bbox_y = msg.center.y

		# P-controller for x-position
		Kp = 1/1.5 # P control gain	
		speed = Kp* abs(bbox_x - 640/2) / (640/2) # P controller
		speed = min(speed, 0.50)		  # saturation of speed

		if speed <= 0.15:	# speed is too low, motor cannot move
			speed = 0

		rospy.loginfo("x:%d, y:%d, speed: %f", bbox_x, bbox_y, speed)
		if BALL_REACH == 1:
			if bbox_x > 640/2 + 90:
				rospy.loginfo("goal left")
				set_speed(motor_left_ID, speed)
				set_speed(motor_right_ID, -speed)
			elif bbox_x < 640/2 - 90:
				rospy.loginfo("goal right")
				set_speed(motor_left_ID, speed)
				set_speed(motor_right_ID, -speed)
			else:
				rospy.loginfo("goal stop")
				GOAL_REACH = 1	# reach goal, set flag to move to kick STATE
				set_speed(motor_left_ID,  0.5)
				set_speed(motor_right_ID, 0.5)
        time.sleep(1)



# initialization
if __name__ == '__main__':
	global BALL_REACH, GOAL_REACH

	

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)
 
	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_color_follwer')
	# rospy.Subscriber('bounding_box', BoundingBox2D, detect_callback)
	rospy.Subscriber('ball/bounding_box', BoundingBox2D, ball_detect_callback)
	rospy.Subscriber('goalpost/bounding_box', BoundingBox2D, goal_detect_callback)


	# start running
	# rospy.spin()

	# STATE = STATE_SPIN

	rate = rospy.Rate(10) # 10 hz
	while not rospy.is_shutdown():
		if STATE == STATE_SPIN_BALL:
			# Set motor 	
			speed = .4
			rospy.loginfo("SPIN BALL %d", speed)
			set_speed(motor_left_ID, speed)
			set_speed(motor_right_ID, -speed)		
			if SPIN_CHECK_BALL == 1: # change STATE
				STATE = STATE_FIND_BALL

		elif STATE == STATE_FIND_BALL:
			rospy.loginfo("FIND BALL")

			if BALL_REACH == 1: # change STATE
				STATE = STATE_SPIN_GOAL

		elif STATE == STATE_SPIN_GOAL:
			rospy.loginfo("SPIN GOAL")
			if SPIN_CHECK_GOAL == 1: # change STATE
				STATE = STATE_FIND_GOAL

		elif STATE == STATE_FIND_GOAL:
			rospy.loginfo("find goal")
			if GOAL_REACH == 1:
				STATE = STATE_KICK

		elif STATE == STATE_KICK:
			rospy.loginfo("kick")
			time.sleep(0.7) # delay before kicking
			if GOAL_REACH == 1 and BALL_REACH == 1:
				# set motor to kick it to the goal
				time.sleep(1)
				rospy.loginfo("done kicking")
				GOAL_REACH = 0
				BALL_REACH = 0
				SPIN_CHECK_BALL = 0
				SPIN_CHECK_GOAL = 0
				STATE = STATE_SPIN_BALL
				time.sleep(3)
		
		rate.sleep()

	
	# stop motors before exiting
	all_stop()
