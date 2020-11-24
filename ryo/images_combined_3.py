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
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)

        # initialize a subscriber to recieve messages from a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
        # initialize a subscriber to recieve messages from a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)

        # Subscribe to trajectory positions published by target_move.py
        self.target_joint1_pub = rospy.Subscriber("/target/x_position_controller/command", Float64, self.callback2)
        self.target_joint2_pub = rospy.Subscriber("/target/y_position_controller/command", Float64, self.callback3)
        self.target_joint3_pub = rospy.Subscriber("/target/z_position_controller/command", Float64, self.callback4)

        # initialize a publisher to send the estimated joint values to a topic names joint_states
        self.joints_pub = rospy.Publisher("/robot/joint_states", Float64MultiArray, queue_size=10)

        # initialize spublishers to send the joint values to move the robot joints
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # access images from each of the two cameras
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 1, 1)
        self.ts.registerCallback(self.callback1)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # get the current time at start
        self.t0 = rospy.get_time()
        self.cur_time = 0

    # Detecting the centre of the green circle
    def detect_green(self, image, image1):
        # Isolate the blue colour in the image as a binary image
        mask1 = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        mask2 = cv2.inRange(image1, (0, 100, 0), (0, 255, 0))
        # Applies a dilate that makes the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        # Obtain the moments of the binary image
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] != 0 and M2['m00'] != 0:  # check circle has been adequately detected
        # Calculate pixel coordinates for the centre of the blob from camera 1
            cam1_cx = int(M1['m10'] / M1['m00'])
            cam1_cy = int(M1['m01'] / M1['m00'])
        # Calculate pixel coordinates for the centre of the blob from camera 2
            cam2_cx = int(M2['m10'] / M2['m00'])
            cam2_cy = int(M2['m01'] / M2['m00'])
            return np.array([cam1_cx, cam2_cx, int((cam1_cy + cam2_cy) / 2)])

        else:
            return np.array([self.prev_green[0],self.prev_green[1], self.prev_green[2]])

    # Detecting the centre of the blue circle
    def detect_blue(self, image, image1):
        # Isolate the blue colour in the image as a binary image
        mask1 = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        mask2= cv2.inRange(image1, (100, 0, 0), (255, 0, 0))
        # Applies a dilate that makes the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        # Obtain the moments of the binary image
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] != 0 and M2['m00'] != 0:  # check circle has been adequately detected
        # Calculate pixel coordinates for the centre of the blob from camera 1
            cam1_cx = int(M1['m10'] / M1['m00'])
            cam1_cy = int(M1['m01'] / M1['m00'])
        # Calculate pixel coordinates for the centre of the blob from camera 2
            cam2_cx = int(M2['m10'] / M2['m00'])
            cam2_cy = int(M2['m01'] / M2['m00'])
            return np.array([cam1_cx, cam2_cx, int((cam1_cy + cam2_cy) / 2)])

        else:
            return np.array([self.prev_blue[0],self.prev_blue[1], self.prev_blue[2]])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image, image1):
        # Isolate the blue colour in the image as a binary image
        mask1 = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        mask2 = cv2.inRange(image1, (0, 100, 100), (0, 255, 255))
        # Applies a dilate that makes the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        # Obtain the moments of the binary image
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] != 0 and M2['m00'] != 0:  # check circle has been adequately detected
        # Calculate pixel coordinates for the centre of the blob from camera 1
            cam1_cx = int(M1['m10'] / M1['m00'])
            cam1_cy = int(M1['m01'] / M1['m00'])
        # Calculate pixel coordinates for the centre of the blob from camera 2
            cam2_cx = int(M2['m10'] / M2['m00'])
            cam2_cy = int(M2['m01'] / M2['m00'])
            return np.array([cam1_cx, cam2_cx, int((cam1_cy + cam2_cy) / 2)])

        else:
            return np.array([self.prev_yellow[0],self.prev_yellow[1], self.prev_yellow[2]])

    # Detecting the centre of the red circle
    def detect_red(self, image, image1):
        # Isolate thered colour in the image as a binary image
        mask1 = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        mask2 = cv2.inRange(image1, (0, 0, 100), (0, 0, 255))
        # Applies a dilate that makes the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask2 = cv2.dilate(mask2, kernel, iterations=3)
        # Obtain the moments of the binary image
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] != 0 and M2['m00'] != 0:  # check circle has been adequately detected
        # Calculate pixel coordinates for the centre of the blob from camera 1
            cam1_cx = int(M1['m10'] / M1['m00'])
            cam1_cy = int(M1['m01'] / M1['m00'])
        # Calculate pixel coordinates for the centre of the blob from camera 2
            cam2_cx = int(M2['m10'] / M2['m00'])
            cam2_cy = int(M2['m01'] / M2['m00'])
            return np.array([cam1_cx, cam2_cx, int((cam1_cy + cam2_cy) / 2)])

        else:
            return np.array([self.prev_red[0],self.prev_red[1], self.prev_red[2]])

    # Calculate the conversion from pixel to meters
    def pixel2meter(self, image, image1):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image, image1)
        circle2Pos = self.detect_green(image, image1)
        # find the distance between two circles
        #dist = np.sum((circle1Pos - circle2Pos) ** 2)

        dist = np.sqrt((circle1Pos[0] - circle2Pos[0])**2 + (circle1Pos[1] - circle2Pos[1])**2 + (circle1Pos[2] - circle2Pos[2])**2)
        return 3.5 / dist

    def distance_basetotarget(self, image, image1):

        a = self.pixel2meter(image, image1)

        base = self.detect_yellow(image, image1)
        target = self.detect_orange_sphere(image, image1)

        distance = a * np.sqrt((base[0] - target[0])**2 + (base[1] - target[1])**2 + (base[2] - target[2])**2)

        #distance = a * (base - target)

        return distance

    # Calculate the relevant joint angles from the image in camera 1
    def detect_joint_angles(self, image, image1):
        a = self.pixel2meter(image, image1)
        # Obtain the centre of each coloured blob
        center = a * self.detect_yellow(image, image1)
        circle1Pos = a * self.detect_blue(image, image1)
        circle2Pos = a * self.detect_green(image, image1)
        circle3Pos = a * self.detect_red(image, image1)
        # Calculate each of the joint values
        j1_x = np.arctan2(center[0] - circle1Pos[0], center[2] - circle1Pos[2])
        j1_y = np.arctan2(center[1] - circle1Pos[1], center[2] - circle1Pos[2])
        j2_x = np.arctan2(circle2Pos[0] - circle1Pos[0], circle1Pos[2] - circle2Pos[2]) - j1_x
        j2_y = np.arctan2(circle2Pos[1] - circle1Pos[1], circle1Pos[2] - circle2Pos[2]) - j1_y
        j3_x = np.arctan2(circle3Pos[0] - circle2Pos[0], circle2Pos[2] - circle3Pos[2]) - j1_x - j2_x
        j3_y = np.arctan2(circle3Pos[1] - circle2Pos[1], circle2Pos[2] - circle3Pos[2]) - j1_y - j2_y
        #print('arctan: ', j2_x, j2_y, j3_x)

        c1_x = np.arccos((center[0] - circle1Pos[0])/2.5)
        c1_y = np.arccos((center[1] - circle1Pos[1])/2.5)
        c2_x = np.arccos((circle1Pos[0] - circle2Pos[0])/3.5)
        c2_y = np.arccos((circle1Pos[1] - circle2Pos[1])/3.5)
        c3_x = np.arccos((circle2Pos[0] - circle3Pos[0])/3)
        c3_y = np.arccos((circle2Pos[1] - circle3Pos[1])/3)
        #print('arccos: ', c2_x, c2_y, c3_x)

        ja1 = np.arctan2(center[0], circle1Pos[0]) - np.arctan2(center[1],circle1Pos[1])
        #ja2 = np.arctan2(circle1Pos[1] - circle2Pos[1], circle1Pos[2] - circle2Pos[2]) - ja1
        ja2 = np.arctan2(circle1Pos[1],circle2Pos[1]) - np.arctan2(circle1Pos[2],circle2Pos[2]) - ja1
        ja3 = np.arctan2(circle1Pos[0], circle2Pos[0]) - np.arctan2(circle1Pos[2], circle2Pos[2]) - ja2
        ja4 = np.arctan2(circle2Pos[1], circle3Pos[1]) - np.arctan2(circle2Pos[2], circle3Pos[2]) - ja1
        #print('arctan2: ', ja2, ja3, ja4)


        ja1_ = 0
        ja2_ = np.arctan2(circle2Pos[1] - circle1Pos[1], circle1Pos[2] - circle2Pos[2])
        ja3_ = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[2] - circle2Pos[2]) - 0.1 # remove 0.2 bias
        ja4_ = np.arctan2(circle3Pos[1] - circle2Pos[1], circle2Pos[2] - circle3Pos[2]) - ja2_ + 0.3 # remove 0.3 bias
        #print('arctan3: ', ja2_, ja3_, ja4_)

        return np.array([j1_x, j2_x, j2_y, j3_x])
        #return np.array([ja1, ja2, ja3, ja4])

        # Calculate the relevant joint angles from the image in camera 2

    # detect end effector position
    def detect_end_effector(self, image, image1):
          a = self.pixel2meter(image, image1)
          endeffpos = a * (self.detect_yellow(image, image1) - self.detect_red(image, image1))
          return endeffpos

    # Calculate the forward kinematics
    def forward_kinematics(self, image, image1):
          joints = self.detect_joint_angles(image, image1)
          '''
          end_effector = np.array([3.5 * (np.cos(joints[0]) * np.sin(joints[2]) + np.sin(joints[0]) * np.sin(joints[1]) * np.cos(joints[2])) +
                                   3.0 * (np.sin(joints[0]) * np.cos(joints[1]) * np.sin(joints[3]) + (np.cos(joints[0]) * np.sin(joints[2]) + np.sin(joints[0]) * np.sin(joints[1]) * np.cos(joints[2])) * np.cos(joints[3])),
                                   3.5 * (np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2])) +
                                   3.0 * (- np.cos(joints[0]) * np.cos(joints[1]) * np.sin(joints[3]) + (np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2])) * np.cos(joints[3])),
                                   2.5 + 3.5 * np.cos(joints[1]) * np.cos(joints[2]) +
                                   3.0 * (- np.sin(joints[1]) * np.sin(joints[3]) + np.cos(joints[1]) * np.cos(joints[2]) * np.cos(joints[3]))
                                   ])
          return end_effector
          '''

          R1 = np.array([[np.cos(joints[0]), -np.sin(joints[0]), 0],
          [np.sin(joints[0]), np.cos(joints[0]), 0],
          [0, 0, 1]])
          R2 = np.array([[1, 0, 0],
          [0, np.cos(joints[1]), -np.sin(joints[1])],
          [0, np.sin(joints[1]), np.cos(joints[1])]])
          R3 = np.array([[np.cos(joints[2]), 0, np.sin(joints[2])],
          [0, 1, 0],
          [-np.sin(joints[2]),0 , np.cos(joints[2])]])
          R4 = np.array([[1, 0, 0],
          [0, np.cos(joints[3]), -np.sin(joints[3])],
          [0, np.sin(joints[3]), np.cos(joints[3])]])
          l1 = np.array([0, 0, 2.5])
          l2 = np.array([0, 0, 3.5])
          l3 = np.array([0, 0, 3])
          '''

          R2 = np.array([[np.cos(joints[1]), 0, np.sin(joints[1])],
          [0, 1, 0],
          [-np.sin(joints[1]),0 , np.cos(joints[1])]])
          R3 = np.array([[1, 0, 0],
          [0, np.cos(joints[2]), -np.sin(joints[2])],
          [0, np.sin(joints[2]), np.cos(joints[2])]])
          R4 = np.array([[np.cos(joints[3]), 0, np.sin(joints[3])],
          [0, 1, 0],
          [-np.sin(joints[3]),0 , np.cos(joints[3])]])
          '''
          print('joints: ', joints)


          #return R1 @ l1 + np.dot(np.dot(R1, R2), R3) @ l2 + np.dot(np.dot(np.dot(R1, R2), R3), R4) @ l3
          #return R1 @ l1 + np.dot(R3, np.dot(R2, R1)) @ l2 + np.dot(R4, np.dot(R3, np.dot(R1, R2))) @ l3
          return np.dot(R1, l1) + np.dot(np.dot(R3, np.dot(R2, R1)), l2) + np.dot(np.dot(R4, np.dot(R3, np.dot(R1, R2))), l3)


    # calculate matrix tranfomation from DH values for each joint
    def transformation_matrix_dh(self, alpha, a, d, theta):
        matrix_values = np.array(
            [[np.cos(theta), -np.cos(alpha) * np.sin(theta), np.sin(alpha) * np.sin(theta), a * np.cos(theta)],
             [np.sin(theta), np.cos(alpha) * np.cos(theta), -np.sin(alpha) * np.cos(theta), a * np.sin(theta)],
             [0, np.sin(alpha), np.cos(alpha), d],
             [0, 0, 0, 1]])
        return matrix_values

    # Detecting the centre of the orange circle
    def detect_orange_sphere(self, image, image1):
        # Isolate the orange colour in the image as a binary image
        mask1 = cv2.inRange(image, (10, 100, 20), (25, 255, 255))
        mask2 = cv2.inRange(image1, (10, 100, 20), (25, 255, 255))
        # Applies a dilate that makes the binary region larger
        kernel = np.ones((5, 5), np.uint8)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask2 = cv2.dilate(mask2, kernel, iterations=3)

        # get contour information from the identified shapes from camera 1
        contours, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)  # calculate number of points

            if len(approx) < 6:  # if it is a square do not calculate coordinates
                c1 = 0.0
                cz = 0.0

            else:  # if it is a circle calculate coordinates
                # Obtain the moments of the binary image
                M1 = cv2.moments(cnt)
                # Calculate pixel coordinates for the centre of the blob
                if M1['m00'] != 0:  # check circle has been adequately detected
                    cam1_c1 = int(M1['m10'] / M1['m00'])
                    cam1_c2 = int(M1['m01'] / M1['m00'])
                else:  # if circle has not been adequately detected do not calculate
                    cam1_c1 = self.prev_orange[0]
                    cam1_c2 = self.prev_orange[2]

        # get contour information from the identified shapes from camera 1
        contours, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)  # calculate number of points


            if len(approx) < 6:  # if it is a square do not calculate coordinates
                cam2_c1 = 0.0
                cam2_c2 = 0.0

            else:  # if it is a circle calculate coordinates
                # Obtain the moments of the binary image
                M2 = cv2.moments(cnt)
                # Calculate pixel coordinates for the centre of the blob
                if M2['m00'] != 0:  # check circle has been adequately detected
                    cam2_c1 = int(M2['m10'] / M2['m00'])
                    cam2_c2 = int(M2['m01'] / M2['m00'])
                else:  # if circle has not been adequately detected do not calculate
                    cam2_c1 = self.prev_orange[1]
                    cam2_c2 = self.prev_orange[2]

        return np.array([int(cam1_c1), int(cam2_c1),int((cam1_c2 + cam2_c2)/2)])

    # Get subscriber data
    def callback2(self, data):
        self.target_x = data.data

    # Get subscriber data
    def callback3(self, data):
        self.target_y = data.data

    # Get subscriber data
    def callback4(self, data):
        self.target_z = data.data

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data, data1):
        # calculate elapsed time since start
        self.cur_time = np.array([rospy.get_time()]) - self.t0

        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        cv2.imshow('window1', self.cv_image1)
        cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        joints = self.detect_joint_angles(self.cv_image1, self.cv_image2)

        tm0t1 = self.transformation_matrix_dh(0, 2.5, 0, 0)
        tm1t2 = self.transformation_matrix_dh(np.pi / 2, 0, 0, joints[1])
        tm2t3 = self.transformation_matrix_dh(np.pi / 2, 3.5, 0, joints[2])
        tm3t4 = self.transformation_matrix_dh(np.pi / 2, 3, 0, joints[3])
        #tm0t4 = tm0t1 * tm1t2 * tm2t3 * tm3t4
        tm0t4 = np.dot(np.dot(np.dot(tm0t1, tm1t2), tm2t3),  tm3t4)
        
        #print('joint: ', joints)
        print('estimated 1: ', tm0t4 @ joints)
        x_e = self.forward_kinematics(self.cv_image1, self.cv_image2)
        print('estimated 2: ', x_e)


        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            ee_pos_cam = self.detect_end_effector(self.cv_image1, self.cv_image1)
            print("camera estimate of end effector position:", ee_pos_cam)

        except CvBridgeError as e:
            print(e)

        print('####')
        print('####')
        #print()


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


