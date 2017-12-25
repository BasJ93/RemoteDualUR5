#!/usr/bin/env python


# -*- coding: utf-8 -*-
#"""
#Created on Tue Dec 19 11:18:44 2017
#
#@author: bas
#"""

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import rospkg
import tf
import socket

def main(argv):
  
  #This should become dynamic, e.g. rosparam
  offset = [-0.450, -0.450, 0.0]
  UDP_IP = "127.0.0.1"
  UDP_PORT = 9000

  rospy.init_node("Collision_adder")
  transform = tf.TransformListener()
  rospack = rospkg.RosPack()
  
  #while not t.canTransform("shoulder_link", "base_link", rospy.Time.now()):
  #    print "."
  
  mesh_path = rospack.get_path("ur_description") + "/meshes/ur5/collision/"
  
  scene = PlanningSceneInterface()
  robot = RobotCommander()
  
  rospy.sleep(2)
  
  sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
  sock.bind((UDP_IP, UDP_PORT))
  
  #I assume that the base can not move, e.g. is mounted in a fixed location.
  quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
      
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = offset[0]
  p.pose.position.y = offset[1]
  p.pose.position.z = offset[2]
  p.pose.orientation.x = quaternion[0]
  p.pose.orientation.y = quaternion[1]
  p.pose.orientation.z = quaternion[2]
  p.pose.orientation.w = quaternion[3]
  scene.add_mesh("base", p, mesh_path + "/base.stl")  
  
  #scene.is_diff = True
  while not rospy.is_shutdown():
      data, addr = sock.recvfrom(2024) # buffer size is 1024 bytes
      print "received message:", data
      jointstates = data.split(",")    
      
      #t = transform.getLatestCommonTime("/shoulder_link", "/base_link")
      #shoulder_pose, shoulder_quaternion = transform.lookupTransform("base_link", "shoulder_link", t)      
      
      shoulderstate = jointstates[0].split(";")
      
      shoulder = PoseStamped()
      shoulder.header.frame_id = robot.get_planning_frame()
      shoulder.pose.position.x = offset[0] + float(shoulderstate[1])
      shoulder.pose.position.y = offset[1] + float(shoulderstate[2])
      shoulder.pose.position.z = offset[2] + float(shoulderstate[3])
      shoulder.pose.orientation.x = float(shoulderstate[4])
      shoulder.pose.orientation.y = float(shoulderstate[5])
      shoulder.pose.orientation.z = float(shoulderstate[6])
      shoulder.pose.orientation.w = float(shoulderstate[7])
      scene.add_mesh(shoulderstate[0], shoulder, mesh_path + shoulderstate[0] + ".stl")
      
      #t = transform.getLatestCommonTime("/upper_arm_link", "/base_link")
      #upper_arm_pose, upper_arm_quaternion = transform.lookupTransform("base_link", "upper_arm_link", t)
      
      upperarmstate = jointstates[1].split(";")
      
      upper_arm = PoseStamped()
      upper_arm.header.frame_id = robot.get_planning_frame()
      upper_arm.pose.position.x = offset[0] + float(upperarmstate[1])
      upper_arm.pose.position.y = offset[1] + float(upperarmstate[2])
      upper_arm.pose.position.z = offset[2] + float(upperarmstate[3])
      upper_arm.pose.orientation.x = float(upperarmstate[4])
      upper_arm.pose.orientation.y = float(upperarmstate[5])
      upper_arm.pose.orientation.z = float(upperarmstate[6])
      upper_arm.pose.orientation.w = float(upperarmstate[7])
      scene.add_mesh(upperarmstate[0], upper_arm, mesh_path + upperarmstate[0] + ".stl")
      
      #t = transform.getLatestCommonTime("/forearm_link", "/base_link")
      #fore_arm_pose, fore_arm_quaternion = transform.lookupTransform("base_link", "forearm_link", t)
      
      forearmstate = jointstates[2].split(";")      
      
      fore_arm = PoseStamped()
      fore_arm.header.frame_id = robot.get_planning_frame()
      fore_arm.pose.position.x = offset[0] + float(forearmstate[1])
      fore_arm.pose.position.y = offset[1] + float(forearmstate[2])
      fore_arm.pose.position.z = offset[2] + float(forearmstate[3])
      fore_arm.pose.orientation.x = float(forearmstate[4])
      fore_arm.pose.orientation.y = float(forearmstate[5])
      fore_arm.pose.orientation.z = float(forearmstate[6])
      fore_arm.pose.orientation.w = float(forearmstate[7])
      scene.add_mesh(forearmstate[0], fore_arm, mesh_path + forearmstate[0] + ".stl")
      
      #t = transform.getLatestCommonTime("/wrist_1_link", "/base_link")
      #wrist1_pose, wrist1_quaternion = transform.lookupTransform("base_link", "wrist_1_link", t)
      
      wrist1state = jointstates[3].split(";") 
      
      wrist1 = PoseStamped()
      wrist1.header.frame_id = robot.get_planning_frame()
      wrist1.pose.position.x = offset[0] + float(wrist1state[1])
      wrist1.pose.position.y = offset[1] + float(wrist1state[2])
      wrist1.pose.position.z = offset[2] + float(wrist1state[3])
      wrist1.pose.orientation.x = float(wrist1state[4])
      wrist1.pose.orientation.y = float(wrist1state[5])
      wrist1.pose.orientation.z = float(wrist1state[6])
      wrist1.pose.orientation.w = float(wrist1state[7])
      scene.add_mesh(wrist1state[0], wrist1, mesh_path + wrist1state[0] + ".stl")
      
      #t = transform.getLatestCommonTime("/wrist_2_link", "/base_link")
      #wrist2_pose, wrist2_quaternion = transform.lookupTransform("base_link", "wrist_2_link", t)
      wrist2state = jointstates[4].split(";")
      wrist2 = PoseStamped()
      wrist2.header.frame_id = robot.get_planning_frame()
      wrist2.pose.position.x = offset[0] + float(wrist2state[1])
      wrist2.pose.position.y = offset[1] + float(wrist2state[2])
      wrist2.pose.position.z = offset[2] + float(wrist2state[3])
      wrist2.pose.orientation.x = float(wrist2state[4])
      wrist2.pose.orientation.y = float(wrist2state[5])
      wrist2.pose.orientation.z = float(wrist2state[6])
      wrist2.pose.orientation.w = float(wrist2state[7])
      scene.add_mesh(wrist2state[0], wrist2, mesh_path + wrist2state[0] + ".stl")
      
      #t = transform.getLatestCommonTime("/wrist_3_link", "/base_link")
      #wrist3_pose, wrist3_quaternion = transform.lookupTransform("base_link", "wrist_3_link", t)
      
      wrist3state = jointstates[5].split(";")
      
      wrist3 = PoseStamped()
      wrist3.header.frame_id = robot.get_planning_frame()
      wrist3.pose.position.x = offset[0] + float(wrist3state[1])
      wrist3.pose.position.y = offset[1] + float(wrist3state[2])
      wrist3.pose.position.z = offset[2] + float(wrist3state[3])
      wrist3.pose.orientation.x = float(wrist3state[4])
      wrist3.pose.orientation.y = float(wrist3state[5])
      wrist3.pose.orientation.z = float(wrist3state[6])
      wrist3.pose.orientation.w = float(wrist3state[7])
      scene.add_mesh(wrist3state[0], wrist3, mesh_path + wrist3state[0] + ".stl")
      
      
if __name__ == "__main__":
  try:
    main(sys.argv)
  except rospy.ROSInterruptException:
    pass