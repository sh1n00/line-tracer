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
d = 0.21		# [m]
D = 0.7		# [m]
color_code = "red"  #detect color
# put object on left side 
# realsense angle is not too big

# stream initial
config = rs.config()
config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.z16, 30)

# stream start
pipe = rs.pipeline()
profile = pipe.start(config)

def line_trace(binary_image, distance_red, distance_yellow):
	global speed, flag, count, fast_x, slow_x, fast_z, slow_z
	print(f"{distance_red:.2f} {distance_yellow:.2f}", end=" ")
	x = z = cnt = 0
	pos_left = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)-250]
	pos_middle = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)]
	pos_right = binary_image[IMAGE_HEIGHT-1][int(IMAGE_WIDTH/2)+250]
	if 0.0 < distance_red <= d:
		print("Stop")
		x = z = 0	
	elif pos_left == 0 and pos_middle == 255 and pos_right == 0:
		print("Go to straight")
		x = fast_x
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
	#if (pos_left == 0 and pos_middle == 0 and pos_right == 0) or (pos_left == 255 and pos_middle == 0 and pos_right == 255) or (pos_left == 255 and pos_middle == 255 and pos_right == 255):
	else:
		print("Random")
		if flag == 0 and count < wait_time:
			print("Search left")
			x = slow_x*2
			z = slow_z
			count += 1
			if count >= wait_time:
				flag = 1
				count = 0
		elif flag == 1 and count < wait_time:
			print("Search right")
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
	if d <= distance_yellow <= D:
		print("Slow Down")
		x *= 0.5
	speed.linear.x = x
	speed.angular.z = z

def create_mask_image(hsv_image: np.array, color: str) -> np.array:
	"""
		params:
			hsv_image: 
			color: object color. This param will be used to switch the move of the car.	
		return:
			create color mask image.
	"""
	if color == "red":
		mask1 = cv2.inRange(hsv_image, np.array([0, 127, 0]), np.array([7, 255, 255]))
		mask2 = cv2.inRange(hsv_image, np.array([238, 127, 0]), np.array([255, 255, 255]))
		mask_image = mask1 | mask2
	elif color == "yellow":
		mask_image = cv2.inRange(hsv_image, np.array([0, 42, 89]), np.array([58, 255, 137]))
	return mask_image


def object_detect(mask_image: np.array) -> bool:
	"""
		params:
			contours: the 2d-point of contours.
		return:
			contours, detect_flag: if the object is not detected, the flag shows "false"
	"""
	contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	return contours, len(contours) > 1

def calc_corner_pos(contours: np.array):
	"""
		return:
			calcuate corner position.	
	"""
	max_index = max_shape = 0
	area = 0
	for index, contour in enumerate(contours):
		if area < cv2.contourArea(contour, False):
			area = cv2.contourArea(contour, False)
			max_index = index
	if area <= 1000:
		return 0, 0, 0, 0
#	for index, contour in enumerate(ans):
#		if cv2.contourArea(contour, False) > max_area:
#			print(cv2.contourArea(contour, False))
#			max_area = cv2.contourArea(contour, False)
#			max_index = index	
	target_contours = contours[max_index].squeeze()
	min_y = min_x = max_y = max_x = 0
	if target_contours.ndim > 1:
		min_y, min_x, max_y, max_x = list(map(int, [*target_contours.max(axis=0), *target_contours.min(axis=0)]))
	return min_y, min_x, max_y, max_x

def realsense():
	global pipe
	frames = pipe.wait_for_frames()
	color_frame = frames.get_color_frame()
	depth_frame = frames.get_depth_frame()
#	if not depth_frame or not color_frame:
#		continue

	color_image = np.asanyarray(color_frame.get_data())
	depth_image = np.asanyarray(depth_frame.get_data())

	# depth_color_frame = rs.colorizer().colorize(depth_frame)
	# depth_image = np.asanyarray(depth_color_frame.get_data())

	rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
	hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV_FULL)
	gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

	_, binary_image = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY_INV)

	# red
	mask_image_red = create_mask_image(hsv_image, "red")
	contours_red, detect_flag_red = object_detect(mask_image_red)
	distance_red: float = 0.0
	min_y = min_x = max_y = max_x = 0
	if detect_flag_red:
		min_y, min_x, max_y, max_x = calc_corner_pos(contours_red)
		distance_red = depth_frame.get_distance((min_y+max_y)//2, (min_x+max_x)//2)
		cv2.rectangle(rgb_image, (min_y, min_x), (max_y, max_x), (255, 0, 0))	

	# yellow
	mask_image_yellow = create_mask_image(hsv_image, "yellow")
	contours_yellow, detect_flag_yellow = object_detect(mask_image_yellow)
	distance_yellow: float = 0.0
	min_y = min_x = max_y = max_x = 0
	if detect_flag_yellow:
		min_y, min_x, max_y, max_x = calc_corner_pos(contours_yellow)
		distance_yellow = depth_frame.get_distance((min_y+max_y)//2, (min_x+max_x)//2)
		cv2.rectangle(rgb_image, (min_y, min_x), (max_y, max_x), (255, 0, 0))

	cv2.imshow('show', rgb_image)
	line_trace(binary_image, distance_red, distance_yellow)


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
