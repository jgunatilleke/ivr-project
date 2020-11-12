#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    ########
    self.joints_pub = rospy.Publisher("/robot/joint_states",Float64MultiArray, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    self.t0 = rospy.get_time()
    self.cur_time = 0
    ########

  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the green circle
  def detect_green(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

# Detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate thered colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Calculate the conversion from pixel to meters
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3.5 / np.sqrt(dist)


  # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image)
    circle2Pos = a * self.detect_green(image)
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja4 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja2, ja3, ja4])

  # Detecting the centre of the orange circle
  def detect_orange_sphere(self,image):
      # Isolate the orange colour in the image as a binary image
      mask = cv2.inRange(image, (10, 100, 20), (25, 255, 255))
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)

      contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

      for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02* cv2.arcLength(cnt, True), True)
        if len(approx) < 5:
          cx = 0.0
          cy = 0.0
        else:
          # Obtain the moments of the binary image
          M = cv2.moments(cnt)
          # Calculate pixel coordinates for the centre of the blob
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])

      return np.array([cx, cy])

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    ########
    self.cur_time = np.array([rospy.get_time()]) - self.t0
    ########
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    a = self.detect_joint_angles(self.cv_image1)
    b = self.detect_orange_sphere(self.cv_image1)
    self.joints = Float64MultiArray()
    self.joints.data = a
    print(self.joints.data)
    print('####')
    print(b)
    print('####')
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    #im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)

    ########
    x_2 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 15)
    y_3 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 18)
    x_4 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 20)
    print(x_2, y_3, x_4)
    print('####')
    print('####')

    self.joint2=Float64()
    self.joint2.data= np.array(x_2)
    self.joint3=Float64()
    self.joint3.data= np.array(y_3)
    self.joint4=Float64()
    self.joint4.data= np.array(x_4)
    ########

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joints_pub.publish(self.joints)
      ########
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
      ########
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


