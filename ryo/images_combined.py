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
    self.joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)

    # initialize spublishers to send the joint values to move the robot joints
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # access images from each of the two cameras
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2],1,1)
    self.ts.registerCallback(self.callback1)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # get the current time at start
    self.t0 = rospy.get_time()
    self.cur_time = 0


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
    return np.array([ja2, ja3, ja4])

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
    return np.array([ja2, ja3, ja4])

  # Detecting the centre of the orange circle
  def detect_orange_sphere(self,image):
      # Isolate the orange colour in the image as a binary image
      mask = cv2.inRange(image, (10, 100, 20), (25, 255, 255))
      print('mask:', mask)
      # Applies a dilate that makes the binary region larger
      kernel = np.ones((5, 5), np.uint8)
      print('kernel:', kernel)
      mask = cv2.dilate(mask, kernel, iterations=3)
      print('mask2:', mask)

      # get contour information from the identified shapes
      contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      print('contours:', contours)


      for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02* cv2.arcLength(cnt, True), True) # calculate number of points
        print('approx:', approx)

        if len(approx) < 6: # if it is a square do not calculate coordinates
          c1 = 0.0
          cz = 0.0

        else: # if it is a circle calculate coordinates
          # Obtain the moments of the binary image
          M = cv2.moments(cnt)
          # Calculate pixel coordinates for the centre of the blob
          if M['m00'] != 0:  # check circle has been adequately detected
              c1 = int(M['m10'] / M['m00'])
              cz = int(M['m01'] / M['m00'])
          else:  # if circle has not been adequately detected do not calculate
              c1 = ""
              cz = ""

        print('c1, cz:', c1, cz)
        print('####')
        print()

      return np.array([c1, cz])


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data, data1):
    # calculate elapsed time since start
    self.cur_time = np.array([rospy.get_time()]) - self.t0

    # Receive the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    # calculate sinusoidal signals
    x_2 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 15)
    y_3 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 18)
    x_4 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 20)
    #print("actual joint positions: ", x_2, y_3, x_4)

    # pass calculated sinusoidal signals to relevant joints
    self.joint2_act = Float64()
    self.joint2_act.data = np.array(x_2)
    self.joint3_act = Float64()
    self.joint3_act.data = np.array(y_3)
    self.joint4_act = Float64()
    self.joint4_act.data = np.array(x_4)
    ########

    #cv2.imshow('window1', self.cv_image1)
    #cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    joint_est1 = self.detect_joint_angles_cam1(self.cv_image1)
    joint_est2 = self.detect_joint_angles_cam2(self.cv_image2)
    sphere_est1 = self.detect_orange_sphere(self.cv_image1)
    sphere_est2 = self.detect_orange_sphere(self.cv_image2)
    #print('joint estimated from camera1: ', joint_est1)
    #print('joint estimated from camera2: ', joint_est2)
    print('sphere estimated from cameras: ', sphere_est2[0], sphere_est1[0], sphere_est1[1])
    print('####')
    print('####')
    print()

    self.joints = Float64MultiArray()
    self.joints.data = joint_est2[0], joint_est1[1], joint_est2[2]

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joints_pub.publish(self.joints)  # estimated joint values

      #  sinusoidal signal values to move joints
      self.robot_joint2_pub.publish(self.joint2_act)
      self.robot_joint3_pub.publish(self.joint3_act)
      self.robot_joint4_pub.publish(self.joint4_act)

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


