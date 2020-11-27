#!/usr/bin/env python3

import os
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
    # initialize subscribers to receive images from cameras
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    self.ts.registerCallback(self.callback2)
    self.initial_time = rospy.get_time()
  
  def get_joint_trajectories(self):
    curr_time = np.array([rospy.get_time() - self.initial_time])
    joint1 = 0
    joint2 = float((np.pi/2) * np.sin((np.pi/15)*curr_time))
    joint3 = float((np.pi/2) * np.sin((np.pi/18)*curr_time))
    joint4 = float((np.pi/2) * np.sin((np.pi/20)*curr_time))
    return np.array([joint1, joint2, joint3, joint4])
  
  # publishes joint angles
  def callback(self,image1,image2):
    # Receive the images
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e) 
    self.link1 = cv2.imread('link1.png',0)
    self.link2 = cv2.imread('link2.png',0)
    self.link3 = cv2.imread('link3.png',0)  
    self.cv_image1 = cv2.cvtColor(self.cv_image1, cv2.COLOR_BGR2GRAY) # convert to grayscale
    self.cv_image2 = cv2.cvtColor(self.cv_image2, cv2.COLOR_BGR2GRAY) # convert to grayscale
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)
    try:
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
      
  # publishes joint states as joint angles follow various trajectories    
  def callback2(self,image1,image2):
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
    self.link1 = cv2.imread('link1.png',0)
    self.link2 = cv2.imread('link2.png',0)
    self.link3 = cv2.imread('link3.png',0)  
    self.cv_image1 = cv2.cvtColor(self.cv_image1, cv2.COLOR_BGR2GRAY) # convert to grayscale
    self.cv_image2 = cv2.cvtColor(self.cv_image2, cv2.COLOR_BGR2GRAY) # convert to grayscale  
    # publish robot joint angles
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)
    try:
       self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
      
  def rotate_image(self,image,angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], borderValue=(255,255,255))
    return result    

  def detect_link(self,image,num):
    if num == 1:
      w, h = self.link1.shape[::-1]
    elif num == 2:
      w, h = self.link2.shape[::-1]
    elif num == 3:
      w, h = self.link3.shape[::-1]
    img_mask = cv2.inRange(image, (0), (10))
    	
    min_elts = []
    res_elts = []
    min_locs = []

    angles = np.arange(-90,91,5) # 5 degree increments
    for angle in angles:
      if num == 1:
        rotatedTemplate = cv2.inRange(self.rotate_image(self.link1, angle), (0), (10))
      elif num == 2:
        rotatedTemplate = cv2.inRange(self.rotate_image(self.link2, angle), (0), (10))
      elif num == 3:
        rotatedTemplate = cv2.inRange(self.rotate_image(self.link3, angle), (0), (10))
      # Apply template Matching
      res = cv2.matchTemplate(img_mask, rotatedTemplate, cv2.TM_SQDIFF_NORMED)
      min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
      min_elts.append(min_val)
      min_locs.append(min_loc)
      res_elts.append(res)

    # get minimum element
    min_elts = np.array(min_elts)
    min_val = min_elts.min()
    res = res_elts[np.argmin(min_elts)]
    min_loc = min_locs[np.argmin(min_elts)]
    # method is TM_SQDIFF_NORMED, so take minimum
    top_left = min_loc
    center = (min_loc[0], min_loc[1])
    return np.array([center[0], center[1]])
    
  def detect_link1(self,image):
    return self.detect_link(image, 1)  
  
  def detect_link2(self,image):
    return self.detect_link(image, 2)
	  
  def detect_link3(self,image):
    return self.detect_link(image, 3)	  

  def get_3d_coords(self,list1,list2):
    if np.array_equal(np.zeros((1,2)), list1):
      return np.array([list2[0], 0, list2[1]]) # (x,0,z)
    elif np.array_equal(np.zeros((1,2)), list2):
      return np.array([0, list1[0], list1[1]]) # (0,y,z)
    else:
      y = list1[0]
      x = list2[0]
      z = (list1[1] + list2[1])/2
      return np.array([x, y, z])
  
  def pixel2meter(self):
    bottomPos = self.get_3d_coords(self.L11, self.L12)
    middlePos = self.get_3d_coords(self.L21, self.L22)
    dist = np.sqrt(np.sum((bottomPos - middlePos)**2))
    return 3.5/dist
   
  # get 2d coordinates relative to sphere 1
  def get_coords(self):
    a = self.pixel2meter()
    bottomPos = self.get_3d_coords(self.L11, self.L12)
    middlePos = self.get_3d_coords(self.L21, self.L22)
    topPos = self.get_3d_coords(self.L31, self.L32)
    Xm = middlePos[0] - bottomPos[0]   
    Ym = middlePos[1] - bottomPos[1]
    Zm = bottomPos[2] - middlePos[2]
    Xt = topPos[0] - bottomPos[0]   
    Yt = topPos[1] - bottomPos[1]
    Zt = bottomPos[2] - topPos[2]
    middleVec = a * np.array([Xm,Ym,Zm])
    topVec = a * np.array([Xt,Yt,Zt])
    return middleVec, topVec 
  
  def get_joint_2(self):
    middle, _ = self.get_coords()
    _, Y, Z = middle
    return np.arctan2(-Y, Z)
  
  def get_joint_3(self):
    middle, _ = self.get_coords()
    X, _, _ = middle 
    return np.arcsin((X/3.5))
    
  def get_joint_4(self):
    beta = self.get_joint_2()
    gamma = self.get_joint_3()
    a1 = -3*np.sin(beta)*np.cos(gamma)
    a2 = -3*np.cos(beta)
    b1 = 3*np.cos(beta)*np.cos(gamma)
    b2 = -3*np.sin(beta)
    _, top = self.get_coords()
    _, Yt, Zt = top
    c1 = Yt + 3.5*np.sin(beta)*np.cos(gamma)
    c2 = Zt - 3.5*np.cos(beta)*np.cos(gamma)
    A = np.array([[a1,a2],[b1,b2]])
    b = np.array([c1,c2])
    if np.linalg.det(A) == 0:
      return 0
    x = (np.linalg.inv(A)).dot(b)
    return np.arctan2(x[1],x[0])

  def detect_joint_angles(self,image1,image2):
    self.L11 = self.detect_link1(image1)
    self.L12 = self.detect_link1(image2)
    self.L21 = self.detect_link2(image1)
    self.L22 = self.detect_link2(image2)
    self.L31 = self.detect_link3(image1)
    self.L32 = self.detect_link3(image2)
    joint1 = 0
    joint2 = self.get_joint_2()
    joint3 = self.get_joint_3()
    joint4 = self.get_joint_4()
    return np.array([joint1, joint2, joint3, joint4])
    	  
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

  
  
