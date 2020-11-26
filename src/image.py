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
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector",Float64MultiArray, queue_size=10)
    # initialize a publisher to send sphere position
    self.sphere_pub = rospy.Publisher("sphere_pos",Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize subscribers to receive images from cameras
    self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw",Image)
    self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
    self.ts.registerCallback(self.callback5)
    self.initial_time = rospy.get_time()
    # initialize error
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
  
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
    # publish robot joint angles
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)
    try:
       self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
  
  # publishes sphere position
  def callback3(self,image1,image2):
    # Receive the images
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e)
    sphere_coordinates = self.get_sphere_coords(self.cv_image1, self.cv_image2)
    self.sphere = Float64MultiArray()
    self.sphere.data = sphere_coordinates
    try:
      self.sphere_pub.publish(self.sphere)
    except CvBridgeError as e:
      print(e)
      
  # publishes the robot end-effector position upon rotation of robot joint angles
  def callback4(self,image1,image2):
    a = np.pi/180.0 # conversion factor
    angles = a * np.array([130,65,65,65])
    self.joint1 = Float64()
    self.joint1.data = angles[0] 
    self.joint2 = Float64()
    self.joint2.data = angles[1]
    self.joint3 = Float64()
    self.joint3.data = angles[2]
    self.joint4 = Float64()
    self.joint4.data = angles[3]
    # issue commands to robot
    try:
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)  
    # Receive the images
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.end_effector = Float64MultiArray()
    self.end_effector.data = self.detect_end_effector(self.cv_image1,self.cv_image2)
    try:
      self.end_effector_pub.publish(self.end_effector)
    except CvBridgeError as e:
      print(e)
  
  # uses inverse kinematics to move robot towards target
  def callback5(self,image1,image2):
    # Receive the images
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(image1, "bgr8")
      self.cv_image2 = self.bridge.imgmsg_to_cv2(image2, "bgr8")
    except CvBridgeError as e:
      print(e)
    # send control commands to joints
    q_d = self.control_closed(self.cv_image1,self.cv_image2)
    self.joint1 = Float64()
    self.joint1.data = q_d[0]
    self.joint2 = Float64()
    self.joint2.data = q_d[1]
    self.joint3 = Float64()
    self.joint3.data = q_d[2] 
    self.joint4 = Float64()
    self.joint4.data = q_d[3]
    # publish the trajectories
    x_d = self.get_sphere_coords(self.cv_image1,self.cv_image2)
    self.trajectory_desired = Float64MultiArray()
    self.trajectory_desired.data = x_d
    x_e = self.detect_end_effector(self.cv_image1,self.cv_image2)
    self.end_effector = Float64MultiArray()
    self.end_effector.data = x_e
    try:
      self.end_effector_pub.publish(self.end_effector)
      self.sphere_pub.publish(self.trajectory_desired)
    except CvBridgeError as e:
      print(e)
    
  def forward_kinematics(self,image1,image2):
    pass
             
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
        if M["m00"] == 0:
          return np.array([0,0]) # sphere can't be detected
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
  
  def pixel2meter2(self,image1,image2):
    bluePos = self.get_3d_coords(self.detect_blue(image1), self.detect_blue(image2))
    greenPos = self.get_3d_coords(self.detect_green(image1), self.detect_green(image2))
    dist = np.sqrt(np.sum((bluePos - greenPos)**2))
    return 3.5/dist
  
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
    pass

  def get_joint_2(self,image1,image2):
    blue1, green1, _ = self.get_2d_blob_coords(image1, image2, 1)
    blue2, green2, _ = self.get_2d_blob_coords(image1, image2, 2)
    _, Yb, Zb = self.get_3d_coords(blue1, blue2)
    _, Yg, Zg = self.get_3d_coords(green1, green2)
    Y, Z = Yg-Yb, Zg-Zb
    return np.arctan2(-Y, Z)
  
  def get_joint_3(self,image1,image2):
    blue1, green1, _ = self.get_2d_blob_coords(image1, image2, 1)
    blue2, green2, _ = self.get_2d_blob_coords(image1, image2, 2)
    Xb, _, _ = self.get_3d_coords(blue1, blue2)
    Xg, _, _ = self.get_3d_coords(green1, green2)
    X = Xg-Xb
    return np.arcsin((X/3.5))
    
  def get_joint_4(self,image1,image2):
    beta = self.get_joint_2(image1,image2)
    gamma = self.get_joint_3(image1,image2)
    a1 = -3*np.sin(beta)*np.cos(gamma)
    a2 = -3*np.cos(beta)
    b1 = 3*np.cos(beta)*np.cos(gamma)
    b2 = -3*np.sin(beta)
    blue1, _, red1 = self.get_2d_blob_coords(image1, image2, 1)
    blue2, _, red2 = self.get_2d_blob_coords(image1, image2, 2)
    _, Yb, Zb = self.get_3d_coords(blue1, blue2)
    _, Yr, Zr = self.get_3d_coords(red1, red2)
    c1 = (Yr-Yb) + 3.5*np.sin(beta)*np.cos(gamma)
    c2 = (Zr-Zb) - 3.5*np.cos(beta)*np.cos(gamma)
    A = np.array([[a1,a2],[b1,b2]])
    b = np.array([c1,c2])
    if np.linalg.det(A) == 0:
      return 0
    x = (np.linalg.inv(A)).dot(b)
    return np.arctan2(x[1],x[0])

  def detect_joint_angles(self,image1,image2):
    joint1 = 0
    joint2 = self.get_joint_2(image1,image2)
    joint3 = self.get_joint_3(image1,image2)
    joint4 = self.get_joint_4(image1,image2)
    return np.array([joint1, joint2, joint3, joint4])
  
  def detect_end_effector(self,image1,image2):
    _, _, red1 = self.get_2d_blob_coords(image1, image2, 1)
    _, _, red2 = self.get_2d_blob_coords(image1, image2, 2)
    return self.get_3d_coords(red1, red2)
  
  # get sphere coordinates relative to robot base frame
  def get_sphere_coords_RBF(self,image1,image2):
    a = self.pixel2meter2(image1, image2)
    basePos = self.get_3d_coords(self.detect_baseframe(image1), self.detect_baseframe(image2))
    spherePos = self.get_3d_coords(self.detect_sphere(image1), self.detect_sphere(image2))
    X = spherePos[0] - basePos[0]
    Y = spherePos[1] - basePos[1]
    Z = basePos[2] - spherePos[2]
    return a * np.array([X,Y,Z])
  
  # get sphere coordinates relative to yellow sphere
  def get_sphere_coords(self,image1,image2):
    a = self.pixel2meter2(image1,image2)
    yellowPos = self.get_3d_coords(self.detect_yellow(image1), self.detect_yellow(image2))
    if np.array_equal(np.array([0,0]), self.detect_sphere(image1)) or np.array_equal(np.array([0,0]), self.detect_sphere(image2)): # if sphere can't be detected in one of the images
      return np.array([0,0,0])
    spherePos = self.get_3d_coords(self.detect_sphere(image1), self.detect_sphere(image2))
    X = spherePos[0] - yellowPos[0]
    Y = spherePos[1] - yellowPos[1]
    Z = yellowPos[2] - spherePos[2]
    return a * np.array([X,Y,Z])
  
  def calculate_jacobian(self,image1,image2):
    a,b,c,d = self.detect_joint_angles(image1,image2)
    jacobian = np.zeros((3,4))
    
    jacobian[0,0] = (-3*np.sin(a)*np.sin(c) + 3*np.sin(b)*np.cos(a)*np.cos(c))*np.cos(d) - 3.5*np.sin(a)*np.sin(c) + 3.5*np.sin(b)*np.cos(a)*np.cos(c) + 3*np.sin(d)*np.cos(a)*np.cos(b)
    jacobian[0,1] =  -3*np.sin(a)*np.sin(b)*np.sin(d) + 3*np.sin(a)*np.cos(b)*np.cos(c)*np.cos(d) + 3.5*np.sin(a)*np.cos(b)*np.cos(c)
    jacobian[0,2] =  (-3*np.sin(a)*np.sin(b)*np.sin(c) + 3*np.cos(a)*np.cos(c))*np.cos(d) - 3.5*np.sin(a)*np.sin(b)*np.sin(c) + 3.5*np.cos(a)*np.cos(c)
    jacobian[0,3] =  -(3*np.sin(a)*np.sin(b)*np.cos(c) + 3*np.sin(c)*np.cos(a))*np.sin(d) + 3*np.sin(a)*np.cos(b)*np.cos(d)
      
    jacobian[1,0] = (3*np.sin(a)*np.sin(b)*np.cos(c) + 3*np.sin(c)*np.cos(a))*np.cos(d) + 3.5*np.sin(a)*np.sin(b)*np.cos(c) + 3*np.sin(a)*np.sin(d)*np.cos(b) + 3.5*np.sin(c)*np.cos(a)  
    jacobian[1,1] = 3*np.sin(b)*np.sin(d)*np.cos(a) - 3*np.cos(a)*np.cos(b)*np.cos(c)*np.cos(d) - 3.5*np.cos(a)*np.cos(b)*np.cos(c)
    jacobian[1,2] =  (3*np.sin(a)*np.cos(c) + 3*np.sin(b)*np.sin(c)*np.cos(a))*np.cos(d) + 3.5*np.sin(a)*np.cos(c) + 3.5*np.sin(b)*np.sin(c)*np.cos(a)
    jacobian[1,3] =  -(3*np.sin(a)*np.sin(c) - 3*np.sin(b)*np.cos(a)*np.cos(c))*np.sin(d) - 3*np.cos(a)*np.cos(b)*np.cos(d)  
  
    jacobian[2,0] = 0
    jacobian[2,1] =  -3*np.sin(b)*np.cos(c)*np.cos(d) - 3.5*np.sin(b)*np.cos(c) - 3*np.sin(d)*np.cos(b)
    jacobian[2,2] =  -3*np.sin(c)*np.cos(b)*np.cos(d) - 3.5*np.sin(c)*np.cos(b)
    jacobian[2,3] =  -3*np.sin(b)*np.cos(d) - 3*np.sin(d)*np.cos(b)*np.cos(c)
    
    return jacobian
        
  def control_closed(self,image1,image2):
    # P gain
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    curr_time = np.array([rospy.get_time()])
    dt = curr_time - self.time_previous_step
    self.time_previous_step = curr_time
    # robot end-effector position
    pos = self.detect_end_effector(image1,image2)
    # desired trajectory
    pos_d = self.get_sphere_coords(image1,image2)
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.detect_joint_angles(image1,image2) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image1,image2))  # calculating the psudeo inverse of Jacobian
    dq_d = np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d
     	    
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

