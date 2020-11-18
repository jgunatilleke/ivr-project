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

    # Subscribe to trajectory positions published by target_move.py
    self.target_joint1_pub = rospy.Subscriber("/target/x_position_controller/command", Float64, self.callback2)
    self.target_joint2_pub = rospy.Subscriber("/target/y_position_controller/command", Float64, self.callback3)
    self.target_joint3_pub = rospy.Subscriber("/target/z_position_controller/command", Float64, self.callback4)

    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)




    # initialize a publisher to send the estimated joint values to a topic names joint_states
    # self.joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)

    # initialize publishers to send the joint values to move the robot joints
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # access images from each of the two cameras
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2],1,1)
    self.ts.registerCallback(self.callback1)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
    # initialize error and derivative of error for trajectory tracking
    self.error = np.array([0.0,0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')

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
      endeffpos = np.append(endeffpos, a * 300) #### TEMP HACK TO MAKE IT 3D
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

  # Get subscriber data
  def callback2(self,data):
      self.target_x = data.data

  # Get subscriber data
  def callback3(self, data):
      self.target_y = data.data

  # Get subscriber data
  def callback4(self, data):
      self.target_z = data.data

  # Define a circular trajectory
  def trajectory(self):
    x_d = self.target_x
    y_d = self.target_y
    z_d = self.target_z
    #z_d = self.target_joint3_pub
    return np.array([x_d, y_d, z_d])

  # Calculate the robot Jacobian
  def calculate_jacobian(self,image1, image2, ef_x, ef_y, ef_z):
    a = self.detect_joint_angles_cam1(image1)  # estimate joints 2 and 4 using camera 1
    joint2 = a[0]
    joint4 = a[1]
    c = self.detect_joint_angles_cam2(image2) # estimate orange sphere coordinates using camera 2
    joint3 = c
    #### NEED TO CALCULATE JOINT 1
    joint1 = 0.0

    print(ef_x, ef_y,ef_z, joint1, joint2, joint3, joint4)
    # Need to check if more mathematically accurate way of calculating derivatives
    jacobian = np.array([[ ef_x / joint1, ef_x / joint2 , ef_x / joint3, ef_x / joint4],
                         [ ef_y / joint1, ef_y / joint2 , ef_y / joint3, ef_y / joint4],
                         [ ef_z / joint1, ef_z / joint2 , ef_z / joint3, ef_z / joint4]])


    return jacobian

  def control_closed(self,image1, image2, ef_x, ef_y, ef_z):
    # P gain
    K_p = np.array([[10,0],[0,10]])
    # D gain
    K_d = np.array([[0.1,0],[0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.detect_end_effector(image1)  ### NEED TO MAKE 3D later
    print ("position",pos)
    print(pos.shape)
    # desired trajectory
    pos_d= self.trajectory()
    # estimate derivative of error
    print("trajectory:", pos_d)
    print(pos_d.shape)
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos

    #q = self.detect_joint_angles(image) # estimate initial value of joints'
    a = self.detect_joint_angles_cam1(image1)  # estimate joints 2 and 4 using camera 1
    joint2 = a[0]
    joint4 = a[1]
    c = self.detect_joint_angles_cam2(image2)  # estimate orange sphere coordinates using camera 2
    joint3 = c
    #### NEED TO CACULATE JOINT 1
    joint1 = 0.0
    q = [joint1, joint2, joint3, joint4]

    J_inv = np.linalg.pinv(self.calculate_jacobian(image1, image2,ef_x, ef_y, ef_z))  # calculating the psudeo inverse of Jacobian
    dq_d = np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d

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
    c = self.detect_joint_angles_cam2(self.cv_image2) # estimate joints 3 using camera 2
    joint3 = c
    joint1 = 0.0 ##### NEED TO CALCULATE joint1

    self.joints = [joint1, joint2, joint3, joint4]


    ##### DOES THE FOLLOWING NOT NEED joint 1?

    tm0t1 = self.transformation_matrix_dh(0, 2.5, 0, 0)
    tm1t2 = self.transformation_matrix_dh(np.pi / 2, 0, 0, joint2)
    tm2t3 = self.transformation_matrix_dh(np.pi / 2, 3.5, 0, joint3)
    tm3t4 = self.transformation_matrix_dh(np.pi / 2, 3, 0, joint4)

    tm0t4 = tm0t1 * tm1t2 * tm2t3 * tm3t4

    print(tm0t4)

    # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
    x_e = tm0t4 ##self.forward_kinematics(cv_image)
    x_e_image = self.detect_end_effector(self.cv_image1)
    self.end_effector=Float64MultiArray()
    self.end_effector.data= x_e_image

    # send control commands to joints (lab 3)
    q_d = self.control_closed(self.cv_image1, self.cv_image2, tm0t4[3][0], tm0t4[3][1], tm0t4[3][2])
    #q_d = self.control_open(cv_image)
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # Publishing the desired trajectory on a topic named trajectory(lab 3)
    x_d = self.trajectory()    # getting the desired trajectory
    self.trajectory_desired= Float64MultiArray()
    self.trajectory_desired.data=x_d


    cv2.waitKey(1)



    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

      #  sinusoidal signal values to move joints ### TE<P STOP
      #self.robot_joint2_pub.publish(self.joint2_act)
      #self.robot_joint3_pub.publish(self.joint3_act)
      #self.robot_joint4_pub.publish(self.joint4_act)

      ### NEW pubs
      self.joints_pub.publish(self.joints)
      self.end_effector_pub.publish(self.end_effector)
      self.trajectory_pub.publish(self.trajectory_desired)
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)

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


