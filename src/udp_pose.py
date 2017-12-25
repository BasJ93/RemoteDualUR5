#!/usr/bin/env python

# -*- coding: utf-8 -*-
#"""
#Created on Sun Dec 24 13:20:16 2017

#@author: bas
#"""

import sys
import rospy
import tf
import socket

def main(argv):
    rospy.init_node("udp_pose")
    transform = tf.TransformListener()
    UDP_IP = "127.0.0.1" #debug
    UDP_PORT = 9000

    #It appears that the receiving code is very slow, only able to keep up with a 1hz update rate.
    rospy.sleep(2)

    r = rospy.Rate(1.5) # 10hz
    count = 0
    while not rospy.is_shutdown():
        #Currently fixed to the names of the UR5, should be made dynamic
        t = transform.getLatestCommonTime("/shoulder_link", "/base_link")
        shoulder_pose, shoulder_quaternion = transform.lookupTransform("base_link", "shoulder_link", t)
        t = transform.getLatestCommonTime("/upper_arm_link", "/base_link")
        upper_arm_pose, upper_arm_quaternion = transform.lookupTransform("base_link", "upper_arm_link", t)
        t = transform.getLatestCommonTime("/forearm_link", "/base_link")
        fore_arm_pose, fore_arm_quaternion = transform.lookupTransform("base_link", "forearm_link", t)
        t = transform.getLatestCommonTime("/wrist_1_link", "/base_link")
        wrist1_pose, wrist1_quaternion = transform.lookupTransform("base_link", "wrist_1_link", t)
        t = transform.getLatestCommonTime("/wrist_2_link", "/base_link")
        wrist2_pose, wrist2_quaternion = transform.lookupTransform("base_link", "wrist_2_link", t)
        t = transform.getLatestCommonTime("/wrist_3_link", "/base_link")
        wrist3_pose, wrist3_quaternion = transform.lookupTransform("base_link", "wrist_3_link", t)
    
        #Define how to serialize the data. "joint;x;y;z;qx;qy;qz;qw;"
        #Do this for all the joints at once.
        Message = "%s;%g;%g;%g;%g;%g;%g;%g,"%("shoulder", shoulder_pose[0], shoulder_pose[1], shoulder_pose[2], shoulder_quaternion[0], shoulder_quaternion[1], shoulder_quaternion[2], shoulder_quaternion[3])            
        Message = Message + "%s;%g;%g;%g;%g;%g;%g;%g,"%("upperarm", upper_arm_pose[0], upper_arm_pose[1], upper_arm_pose[2], upper_arm_quaternion[0], upper_arm_quaternion[1], upper_arm_quaternion[2], upper_arm_quaternion[3])            
        Message = Message + "%s;%g;%g;%g;%g;%g;%g;%g,"%("forearm", fore_arm_pose[0], fore_arm_pose[1], fore_arm_pose[2], fore_arm_quaternion[0], fore_arm_quaternion[1], fore_arm_quaternion[2], fore_arm_quaternion[3])            
        Message = Message + "%s;%g;%g;%g;%g;%g;%g;%g,"%("wrist1", wrist1_pose[0], wrist1_pose[1], wrist1_pose[2], wrist1_quaternion[0], wrist1_quaternion[1], wrist1_quaternion[2], wrist1_quaternion[3])
        Message = Message + "%s;%g;%g;%g;%g;%g;%g;%g,"%("wrist2", wrist2_pose[0], wrist2_pose[1], wrist2_pose[2], wrist2_quaternion[0], wrist2_quaternion[1], wrist2_quaternion[2], wrist2_quaternion[3])
        Message = Message + "%s;%g;%g;%g;%g;%g;%g;%g,"%("wrist3", wrist3_pose[0], wrist3_pose[1], wrist3_pose[2], wrist3_quaternion[0], wrist3_quaternion[1], wrist3_quaternion[2], wrist3_quaternion[3])
        Message = Message + "%d"%(count)
    
    
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(Message, (UDP_IP, UDP_PORT))
    
        rospy.loginfo(Message)
        count = count + 1
        r.sleep()
        #pub.publish(Message)
    
    
if __name__ == "__main__":
  try:
    main(sys.argv)
  except rospy.ROSInterruptException:
    pass