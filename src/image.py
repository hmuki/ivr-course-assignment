#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize publishers to send images from both cameras
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize publishers to send joints' angular positions to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    # initialize publishers to send desired trajectories
    self.joint1_trajectory_pub = rospy.Publisher("joint1_trajectory",Float64, queue_size=10)
    self.joint2_trajectory_pub = rospy.Publisher("joint2_trajectory",Float64, queue_size=10)
    self.joint3_trajectory_pub = rospy.Publisher("joint3_trajectory",Float64, queue_size=10)
    self.joint4_trajectory_pub = rospy.Publisher("joint4_trajectory",Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize subscribers to receive messages from topics
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    self.ts.registerCallback(self.callback)
    self.initial_time = rospy.get_time()
  
  def get_joint_trajectories(self):
    curr_time = np.array([rospy.get_time() - self.initial_time])
    #joint1 = float(0)
    joint1 = float((np.pi) * np.sin((np.pi/15)*curr_time))
    joint2 = float((np.pi/2) * np.sin((np.pi/15)*curr_time))
    joint3 = float((np.pi/2) * np.sin((np.pi/18)*curr_time))
    joint4 = float((np.pi/2) * np.sin((np.pi/20)*curr_time))
    return np.array([joint1, joint2, joint3, joint4])
      
  def callback(self,image1,image2):
    # get the desired trajectories
    trajectories = self.get_joint_trajectories()
    self.joint1_trajectory = Float64()
    self.joint1_trajectory.data = trajectories[0]
    self.joint2_trajectory = Float64()
    self.joint2_trajectory.data = trajectories[1]
    self.joint3_trajectory = Float64()
    self.joint3_trajectory.data = trajectories[2]
    self.joint4_trajectory = Float64()
    self.joint4_trajectory.data = trajectories[3]
    # send control commands to joints
    self.joint1 = Float64()
    self.joint1.data = trajectories[0] 
    self.joint2 = Float64()
    self.joint2.data = trajectories[1]
    self.joint3 = Float64()
    self.joint3.data = trajectories[2]
    self.joint4 = Float64()
    self.joint4.data = trajectories[3]
    try:
      self.joint1_trajectory_pub.publish(self.joint1_trajectory)
      self.joint2_trajectory_pub.publish(self.joint2_trajectory)
      self.joint3_trajectory_pub.publish(self.joint3_trajectory)
      self.joint4_trajectory_pub.publish(self.joint4_trajectory)
      self.robot_joint1_pub.publish(self.joint1_trajectory)
      self.robot_joint2_pub.publish(self.joint2_trajectory)
      self.robot_joint3_pub.publish(self.joint3_trajectory)
      self.robot_joint4_pub.publish(self.joint4_trajectory)
    except CvBridgeError as e:
      print(e)
    # Receive the images
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e)
    # publish robot joint angles
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)
    try:
       self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
             
  def detect_red(self,image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (0, 70, 50)
    upper = (10, 255, 255)
    mask = cv2.inRange(image, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return np.array([center[0], center[1]])
    
  def detect_green(self,image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (36, 25, 25)
    upper = (70, 255, 255)
    mask = cv2.inRange(image, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    #canvas = image.copy()
    #cv2.circle(canvas, center, 2, (0, 0, 255), -1)
    #cv2.imshow('image', canvas)
    #cv2.waitKey(5000)
    return np.array([center[0], center[1]])

  def detect_blue(self,image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (100, 150, 0)
    upper = (140, 255, 255)
    mask = cv2.inRange(image, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return np.array([center[0], center[1]])
    
  def detect_yellow(self,image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (20, 100, 100)
    upper = (30, 255, 255)
    mask = cv2.inRange(image, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return np.array([center[0], center[1]])
    
  def detect_sphere(self,image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (5, 50, 50)
    upper = (15, 255, 255)
    mask = cv2.inRange(image, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    shapes = np.zeros((len(contours),3))
    i = 0
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        l = len(approx)
        M = cv2.moments(cnt)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #cv2.circle(img, center, 2, (255, 0, 0), -1)
        shapes[i] = np.array([l, center[0], center[1]])
        i += 1
    r = np.argmax(shapes[:, 0])
    center = (int(shapes[r][1]), int(shapes[r][2]))
    return np.array([center[0], center[1]])
    
  def detect_baseframe(self,image):
    canvas = image.copy()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (0, 0, 180)
    upper = (0, 0, 255)
    mask = cv2.inRange(image, lower, upper)
    #cv2.imshow('mask', mask)
    #cv2.waitKey(3000)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    #cv2.circle(canvas, center, 2, (255, 0, 0), -1)
    #cv2.imshow('image', canvas)
    #cv2.waitKey(3000)
    return np.array([center[0], center[1]])
    
  def get_3d_coords(self,list1,list2):
    y = list1[0]
    x = list2[0]
    z = (list1[1] + list2[1])/2
    return np.array([x, y, z])

  def pixel2meter(self,image1,image2):
    yellowPos = self.get_3d_coords(self.detect_yellow(image1), self.detect_yellow(image2))
    bluePos = self.get_3d_coords(self.detect_blue(image1), self.detect_blue(image2))
    greenPos = self.get_3d_coords(self.detect_green(image1), self.detect_green(image2))
    redPos = self.get_3d_coords(self.detect_red(image1), self.detect_red(image2))
    a1 = 2.5/np.sqrt(np.sum((yellowPos - bluePos)**2))
    a2 = 3.5/np.sqrt(np.sum((bluePos - greenPos)**2))
    a3 = 3/np.sqrt(np.sum((greenPos - redPos)**2))
    return a1, a2, a3
  
  def get_2d_blob_coords(self,image1,image2,num):
    if num == 1:
        img = image1
    elif num == 2:
        img = image2
    a1, a2, a3 = self.pixel2meter(image1, image2)
    center = self.detect_yellow(img)
    bluePos = self.detect_blue(img)
    greenPos = self.detect_green(img)
    redPos = self.detect_red(img)
    blueVec = a1 * np.array([bluePos[0]-center[0], center[1]-bluePos[1]])
    diff = a2 * np.array([greenPos[0] - bluePos[0], bluePos[1] - greenPos[1]])
    greenVec = diff + blueVec
    diff = a3 * np.array([redPos[0] - greenPos[0], greenPos[1] - redPos[1]])
    redVec = diff + greenVec
    return blueVec, greenVec, redVec

  def get_joint_1(self,image1,image2):
    blue1, _, _ = self.get_2d_blob_coords(image1, image2, 1)
    blue2, _, _ = self.get_2d_blob_coords(image1, image2, 2)
    x, y, z = self.get_3d_coords(blue1, blue2)
    adj = np.sqrt(2.5**2 - z**2)
    theta = np.arctan2(z, adj)
    return theta
  
  def get_joint_2(self,image1,image2):
    blue1, green1, _ = self.get_2d_blob_coords(image1, image2, 1)
    blue2, green2, _ = self.get_2d_blob_coords(image1, image2, 2)
    Xb, Yb, Zb = self.get_3d_coords(blue1, blue2)
    Xg, Yg, Zg = self.get_3d_coords(green1, green2)
    greenVec = np.array([Xg-Xb, Yg-Yb, Zg-Zb])
    greenMod = np.linalg.norm(greenVec, 2)
    blueVec = np.array([Xb, Yb, Zb])
    blueMod = np.linalg.norm(blueVec, 2)
    dot_p = np.inner(greenVec, blueVec)
    theta = np.arccos((dot_p / (greenMod * blueMod)))
    return theta

  def get_joint_4(self,image1,image2):
    blue1, green1, red1 = self.get_2d_blob_coords(image1, image2, 1)
    blue2, green2, red2 = self.get_2d_blob_coords(image1, image2, 2)
    Xb,Yb,Zb = self.get_3d_coords(blue1, blue2)
    Xg,Yg,Zg = self.get_3d_coords(green1, green2)
    Xr,Yr,Zr = self.get_3d_coords(red1, red2)
    greenVec = np.array([Xg-Xb, Yg-Yb, Zg-Zb])
    greenMod = np.linalg.norm(greenVec,2)
    redVec = np.array([Xr-Xg, Yr-Yg, Zr-Zg])
    redMod = np.linalg.norm(redVec, 2)
    dot_p = np.inner(greenVec, redVec)
    theta = np.arccos((dot_p/(greenMod * redMod)))
    return theta

  def detect_joint_angles(self,image1,image2):
    joint1 = self.get_joint_1(image1,image2)
    joint2 = self.get_joint_2(image1,image2)
    joint3 = 0
    joint4 = self.get_joint_4(image1,image2)
    return np.array([joint1, joint2, joint3, joint4])
        
  def pixel2meter2(self,image1,image2):
    bluePos = self.get_3d_coords(self.detect_blue(image1), self.detect_blue(image2))
    greenPos = self.get_3d_coords(self.detect_green(image1), self.detect_green(image2))
    dist = np.sqrt(np.sum((bluePos - greenPos)**2))
    return 3.5/dist

  def get_sphere_coords(self,image1,image2):
    a = self.pixel2meter2(image1, image2)
    basePos = self.get_3d_coords(self.detect_baseframe(image1), self.detect_baseframe(image2))
    spherePos = self.get_3d_coords(self.detect_sphere(image1), self.detect_sphere(image2))
    X = spherePos[0] - basePos[0]
    Y = spherePos[1] - basePos[1]
    Z = basePos[2] - spherePos[2]
    return a * np.array([X,Y,Z])
    	    
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

