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
import message_filters


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)

    # initialize a subscriber to recieve messages from a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
    # initialize a subscriber to recieve messages from a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)

    # initialize a publisher to send the estimated joint values to a topic names joint_states
    # self.joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)

    # initialize spublishers to send the joint values to move the robot joints
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # access images from each of the two cameras
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2],1,1)
    self.ts.registerCallback(self.callback1)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

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
      if M['m00'] != 0:  # check circle has been adequately detected
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
      else:  # if circle has not been adequately detected do not calculate
          cx = ""
          cy = ""
      return np.array([cx, cy])


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
      if M['m00'] != 0:  # check circle has been adequately detected
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
      else:  # if circle has not been adequately detected do not calculate
          cx = ""
          cy = ""
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
      if M['m00'] != 0:  # check circle has been adequately detected
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
      else:  # if circle has not been adequately detected do not calculate
          cx = ""
          cy = ""
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
      if M['m00'] != 0:  # check circle has been adequately detected
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
      else:  # if circle has not been adequately detected do not calculate
          cx = ""
          cy = ""
      return np.array([cx, cy])

  # Calculate the conversion from pixel to meters
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3.5 / np.sqrt(dist)

  # detect end effector position
  def detect_end_effector(self, image):
      a = self.pixel2meter(image)
      endeffpos = a * (self.detect_yellow(image) - self.detect_red(image))
      return endeffpos


  # joint values to move robot to and test fk
  # not moving joint 1 as can't estimate that joint yet
  joint2_act = 0.5
  joint3_act = 0
  joint4_act = 0.5

  # calculate matrix tranfomation from DH values for each joint
  def transformation_matrix_dh(self, alpha, a, d, theta):
      matrix_values = np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta),a*np.cos(theta)],
                                [np.sin(theta),np.cos(alpha)*np.cos(theta),-np.sin(alpha)*np.cos(theta),a*np.sin(theta)],
                                [0,np.sin(alpha),np.cos(alpha),d],
                                [0,0,0,1]])
      return matrix_values


    # Calculate the relevant joint angles from the image in camera 1
  def detect_joint_angles_cam1(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image)
    circle2Pos = a * self.detect_green(image)
    circle3Pos = a * self.detect_red(image)
    # Calculate each of the joint values
    ja1 = np.arctan2(center[0]-circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja4 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja2, ja4])


    # Calculate the relevant joint angles from the image in camera 2
  def detect_joint_angles_cam2(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image)
    circle2Pos = a * self.detect_green(image)
    circle3Pos = a * self.detect_red(image)
    # Calculate each of the joint values
    ja1 = np.arctan2(center[0]-circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja4 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return ja3

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data, data1):

    # Receive the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)


    cv2.imshow('window1', self.cv_image1)
    cv2.imshow('window2', self.cv_image2)

    a = self.detect_joint_angles_cam1(self.cv_image1)  # estimate joints 2 and 4 using camera 1
    joint2 = a[0]
    joint4 = a[1]
    c = self.detect_joint_angles_cam2(self.cv_image2) # estimate orange sphere coordinates using camera 2
    joint3 = c

    tm0t1 = self.transformation_matrix_dh(0, 2.5, 0, 0)
    tm1t2 = self.transformation_matrix_dh(np.pi / 2, 0, 0, joint2)
    tm2t3 = self.transformation_matrix_dh(np.pi / 2, 3.5, 0, joint3)
    tm3t4 = self.transformation_matrix_dh(np.pi / 2, 3, 0, joint4)

    tm0t4 = tm0t1 * tm1t2 * tm2t3 * tm3t4

    print(tm0t4)

    cv2.waitKey(1)



    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

      #  sinusoidal signal values to move joints
      self.robot_joint2_pub.publish(self.joint2_act)
      self.robot_joint3_pub.publish(self.joint3_act)
      self.robot_joint4_pub.publish(self.joint4_act)

      ee_pos_cam1 = self.detect_end_effector(self.cv_image1)
      ee_pos_cam2 = self.detect_end_effector(self.cv_image2)
      print("\ncamera 1 estimate of end effector position:", ee_pos_cam1)
      print("camera 2 estimate of end effector position:", ee_pos_cam2)

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

