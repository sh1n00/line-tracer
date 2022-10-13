#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from pyrealsense2 import pyrealsense2 as rs
import cv2
import numpy as np

speed = Twist()

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
flag = count = 0
wait_time = 50
fast_x = -0.1
slow_x = -0.01
fast_z = 1.0
slow_z = 0.5

# stream initial
config = rs.config()
config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.z16, 30)

# stream start
pipe = rs.pipeline()
profile = pipe.start(config)

def line_trace(binary_image):
	global speed, flag, count, fast_x, slow_x, fast_z, slow_z
	x = z = cnt = 0
	pos_left = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)-250]
	pos_middle = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)]
	pos_right = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)+250]
	if (pos_left == 0 and pos_middle == 0 and pos_right == 0) or (pos_left == 255 and pos_middle == 0 and pos_right == 255) or (pos_left == 255 and pos_middle == 255 and pos_right == 255):
		print("Random")
		if flag == 0 and count < wait_time:
			print("Go to left")
			x = slow_x*2
			z = slow_z
			count += 1
			if count >= wait_time:
				flag = 1
				count = 0
		elif flag == 1 and count < wait_time:
			print("Go to right")
			x = slow_x*2
			z = -slow_z
			count += 1
			if count >= wait_time:
				flag = 0
				count = 0
		for i in range(IMAGE_HEIGHT-50):
			if binary_image[i][int(IMAGE_WIDTH/2)] == 255:
				cnt += 1
				if cnt >= 50:
					print("Find Line")
					x = fast_x
					z = 0					
	elif pos_left == 255 and pos_middle == 0 and pos_right == 0:
		print("Go to left")
		x = slow_x
		z = fast_z
	elif pos_left == 255 and pos_middle == 255 and pos_right == 0:
		print("Go to little left")
		x = fast_x
		z = slow_z
	elif pos_left == 0 and pos_middle == 0 and pos_right == 255:
		print("Go to right")
		x = slow_x
		z = -fast_z
	elif pos_left == 0 and pos_middle == 255 and pos_right == 255:
		print("Go to little right")
		x = fast_x
		z = -slow_z
	elif pos_left == 0 and pos_middle == 255 and pos_right == 0:
		print("Go to straight")
		x = fast_x
	speed.linear.x = x
	speed.angular.z = z

def realsense():
	global pipe
	#line_left = 0
	#line_right = 0
	frames = pipe.wait_for_frames()
	color_frame = frames.get_color_frame()
	depth_frame = frames.get_depth_frame()
	color_image = np.asanyarray(color_frame.get_data())
	depth_image = np.asanyarray(depth_frame.get_data())
	depth_color_frame = rs.colorizer().colorize(depth_frame)
	depth_image = np.asanyarray(depth_color_frame.get_data())
	rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
	# gray scale
	gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
	_, binary_image = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY_INV)
	"""for i in range(IMAGE_WIDTH):
		if binary_image[IMAGE_HEIGHT-1][i] == 255:
			if count == 0:
				line_left = i
				count += 1
			else:
				count += 1
		if binary_image[IMAGE_HEIGHT-1][i] == 0 and count != 0:
			if count > 10:
				line_right = i
				break 
			count = 0"""
	#laps_image = cv2. Laplacian(binary_image, cv2.CV_32F)
	cv2.imshow('show', binary_image)
	line_trace(binary_image)

def rover_go():
	global speed
	rospy.init_node('rover_gamepad', anonymous=True)
	pub = rospy.Publisher('rover_drive',Twist, queue_size=1)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		realsense()
		if cv2.waitKey(10) == 27:
			break
		pub.publish(speed)
		rate.sleep()

if __name__ == "__main__":
	rover_go()
