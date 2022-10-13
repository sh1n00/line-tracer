#!/usr/bin/env python3

from pyrealsense2 import pyrealsense2 as rs
import cv2
import numpy as np

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360

# stream initial
config = rs.config()
config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.z16, 30)

# stream start
pipe = rs.pipeline()
profile = pipe.start(config)

try:
	while True:
		frames = pipe.wait_for_frames()
		color_frame = frames.get_color_frame()
		depth_frame = frames.get_depth_frame()
		color_image = np.asanyarray(color_frame.get_data())
		depth_image = np.asanyarray(depth_frame.get_data())
		depth_color_frame = rs.colorizer().colorize(depth_frame)
		depth_image = np.asanyarray(depth_color_frame.get_data())
		rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
		gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
		ret, binary_image = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY_INV)
		cv2.imshow('show', binary_image)
		cv2.waitKey(10)

finally:
	# stream stop
	pipe.stop()
	cv2.destroyAllWindows()
