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


    # Get subscriber data
    def callback2(self, data):
        self.target_x = data.data


    # Get subscriber data
    def callback3(self, data):
        self.target_y = data.data


    # Get subscriber data
    def callback4(self, data):
        self.target_z = data.data

    # Detecting the centre - test using chamfer matching
    def detect_black_spheres_test(self, image, image1):

        ### Camera 1

        # Isolate the black colour in the image as a binary image
        mask1 = cv2.inRange(image, (0, 0, 0), (180, 255, 5))
        mask1 = cv2.threshold(mask1, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # fill the contours
        contours1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours1 = contours1[0] if len(contours1) == 2 else contours1[1]
        for cnt1 in contours1:
            cv2.drawContours(mask1, [cnt1], -1, (255, 255, 255), -1)

        # Morph open
        kernel_m1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        morph_open1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel_m1, iterations=3)

        template1 = cv2.imread('template4.png', 0)
        w, h = template1.shape[::-1]

        results1 = cv2.matchTemplate(morph_open1, template1, cv2.TM_CCOEFF_NORMED)
        threshold1 = 0.54
        location1 = np.where(results1 >= threshold1)

        for pt in zip(*location1[::-1]):
            cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), -1) ### test changed from 2 to -1 to fill

        template2 = cv2.imread('test1.png', 0)
        w, h = template2.shape[::-1]

        results2 = cv2.matchTemplate(morph_open1, template2, cv2.TM_CCOEFF_NORMED)
        threshold2 = 0.54
        location2 = np.where(results2 >= threshold2)

        for pt in zip(*location2[::-1]):
            cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), -1) ### test changed from 2 to -1 to fill
        #### test - count contours

        #imagec1 = cv2.inRange(image, (0, 0, 0), (180, 255, 5))
        imagec1 = cv2.inRange(image, (0, 0, 100), (0, 0, 255)) # filter red
        imagec1 = cv2.threshold(imagec1, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        i = 0
        cam1_c1 = list(range(20))
        cam1_c2 = list(range(20))

        contours1, _ = cv2.findContours(imagec1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt1 in contours1:

            M1 = cv2.moments(cnt1)
            # Calculate pixel coordinates for the centre of the blob
            if M1['m00'] != 0:  # check circle has been adequately detected
                cam1_c1[i] = int(M1['m10'] / M1['m00'])
                cam1_c2[i] = int(M1['m01'] / M1['m00'])
            else:  # if circle has not been adequately detected do not calculate
                cam1_c1[i] = 0  # self.prev_orange[0] ### NEED TO ERROR TRAP PROPERLY LATER
                cam1_c2[i] = 0  # self.prev_orange[2] ### NEED TO ERROR TRAP PROPERLY LATER
            i = i + 1

        #print ("Camera 1", cam1_c1[0], cam1_c2[0],cam1_c1[1], cam1_c2[1],cam1_c1[2], cam1_c2[2],cam1_c1[3], cam1_c2[3])

        ### Camera 2

        # Isolate the black colour in the image as a binary image
        mask2 = cv2.inRange(image1, (0, 0, 0), (180, 255, 5))
        mask2 = cv2.threshold(mask2, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # fill the contours
        contours2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours2 = contours2[0] if len(contours2) == 2 else contours2[1]
        for cnt2 in contours2:
            cv2.drawContours(mask2, [cnt2], -1, (255, 255, 255), -1)

        # Morph open
        kernel_m2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        morph_open2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel_m2, iterations=3)

        template1 = cv2.imread('template4.png', 0)
        w, h = template1.shape[::-1]

        results1 = cv2.matchTemplate(morph_open2, template1, cv2.TM_CCOEFF_NORMED)
        threshold1 = 0.54
        location1 = np.where(results1 >= threshold1)

        for pt in zip(*location1[::-1]):
            cv2.rectangle(image1, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), -1)  ### test changed from 2 to -1 to fill

        template2 = cv2.imread('test1.png', 0)
        w, h = template2.shape[::-1]

        results2 = cv2.matchTemplate(morph_open2, template2, cv2.TM_CCOEFF_NORMED)
        threshold2 = 0.54
        location2 = np.where(results2 >= threshold2)

        for pt in zip(*location2[::-1]):
            cv2.rectangle(image1, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), -1)  ### test changed from 2 to -1 to fill
        #### test - count contours

        # imagec1 = cv2.inRange(image, (0, 0, 0), (180, 255, 5))
        imagec2 = cv2.inRange(image1, (0, 0, 100), (0, 0, 255))  # filter red
        imagec2 = cv2.threshold(imagec2, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        j = 0
        cam2_c1 = list(range(20))
        cam2_c2 = list(range(20))

        contours2, _ = cv2.findContours(imagec2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt2 in contours2:

            M2 = cv2.moments(cnt2)
            # Calculate pixel coordinates for the centre of the blob
            if M1['m00'] != 0:  # check circle has been adequately detected
                cam2_c1[j] = int(M2['m10'] / M2['m00'])
                cam2_c2[j] = int(M2['m01'] / M2['m00'])
            else:  # if circle has not been adequately detected do not calculate
                cam2_c1[j] = 0  # self.prev_orange[0] ### NEED TO ERROR TRAP PROPERLY LATER
                cam2_c2[j] = 0  # self.prev_orange[2] ### NEED TO ERROR TRAP PROPERLY LATER
            j = j + 1

        #print("Camera 2", cam2_c1[0], cam2_c2[0], cam2_c1[1], cam2_c2[1], cam2_c1[2], cam2_c2[2], cam2_c1[3], cam2_c2[3])

        cv2.imshow("Camera1", image)
        cv2.imshow("Camera2", image1)
        #cv2.imwrite('templatecam2.png',morph_open2)

        cam12_c3 = list(range(4))

        for index in range(4):
            cam12_c3[index] = int((cam1_c2[index] + cam2_c2[index]) / 2)

        return(cam1_c1, cam2_c1, cam12_c3)

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data, data1):

        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
        except CvBridgeError as e:
            print(e)

        # get values for blob positions to use if blob cannot be seen later

        #self.prev_yellow = self.detect_yellow(self.cv_image1, self.cv_image2)
        #self.prev_blue = self.detect_blue(self.cv_image1, self.cv_image2)
        #self.prev_green = self.detect_green(self.cv_image1, self.cv_image2)
        #self.prev_red = self.detect_red(self.cv_image1, self.cv_image2)
        #self.prev_orange = self.detect_orange_sphere(self.cv_image1, self.cv_image2)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        #bottom_circle_positions_x, bottom_circle_positions_y, bottom_circle_positions_z = self.detect_black_spheres_initialYB(
        #    self.cv_image1,
        #    self.cv_image2)
        #print("YB-Circle 1 position: ", bottom_circle_positions_x[0], bottom_circle_positions_y[0],
        #      bottom_circle_positions_z[0])
        #print("YB-Circle 2 position: ", bottom_circle_positions_x[1], bottom_circle_positions_y[1],
        #      bottom_circle_positions_z[1])

        #initial_circle_positions_x, initial_circle_positions_y, initial_circle_positions_z = self.detect_black_spheres_initialGR(
        #    self.cv_image1,
        #    self.cv_image2)
        #print("GR-Circle 1 position: ", initial_circle_positions_x[0], initial_circle_positions_y[0],
        #      initial_circle_positions_z[0])
        #print("GR-Circle 2 position: ", initial_circle_positions_x[1], initial_circle_positions_y[1],
        #      initial_circle_positions_z[1])
        #print("GR-Circle 3 position: ", initial_circle_positions_x[2], initial_circle_positions_y[2],
        #      initial_circle_positions_z[2])


        # calculate elapsed time since start
        self.cur_time = np.array([rospy.get_time()]) - self.t0

        # calculate sinusoidal signals
        x_2 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 15)
        y_3 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 18)
        x_4 = (np.pi / 2) * np.sin(self.cur_time * np.pi / 20)
        # print("actual joint positions: ", x_2, y_3, x_4)

        # pass calculated sinusoidal signals to relevant joints
        self.joint2_act = Float64()
        self.joint2_act.data = np.array(x_2)
        self.joint3_act = Float64()
        self.joint3_act.data = np.array(y_3)
        self.joint4_act = Float64()
        self.joint4_act.data = np.array(x_4)
        ########

        #self.detect_black_spheres_test(self.cv_image1, self.cv_image2)

        #cv2.imshow('window1', self.cv_image1)
        #cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        #self.detect_circles(self.cv_image1)

        circle_positions_x, circle_positions_y, circle_positions_z = self.detect_black_spheres_test(self.cv_image1, self.cv_image2)
        print("Circle 1 position: ", circle_positions_x[0], circle_positions_y[0], circle_positions_z[0])
        print("Circle 2 position: ", circle_positions_x[1], circle_positions_y[1], circle_positions_z[1])
        print("Circle 3 position: ", circle_positions_x[2], circle_positions_y[2], circle_positions_z[2])
        print("Circle 4 position: ", circle_positions_x[3], circle_positions_y[3], circle_positions_z[3])

        #joints = self.detect_joint_angles(self.cv_image1,self.cv_image2)

        #print ("Estimated joint angles", joints[1], joints[2], joints[3], "\n")


        #joint_est1 = self.detect_joint_angles_cam1(self.cv_image1)
        #joint_est2 = self.detect_joint_angles_cam2(self.cv_image2)
        #sphere_est1 = self.detect_orange_sphere(self.cv_image1)
        #sphere_est2 = self.detect_orange_sphere(self.cv_image2)
        # print('joint estimated from camera1: ', joint_est1)
        # print('joint estimated from camera2: ', joint_est2)
        #print('sphere estimated from cameras: ', sphere_est2[0], sphere_est1[0], sphere_est1[1])

        #print('####')
        #print()

        #self.joints = Float64MultiArray()
        #self.joints.data = joint_est2[0], joint_est1[1], joint_est2[2]

        # Publish the results
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            #self.joints_pub.publish(self.joints)  # estimated joint values

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
