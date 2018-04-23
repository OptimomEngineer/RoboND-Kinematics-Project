#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya/Divya Patel

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#here I define the four symbols needed for forward kinematics translations.
	#first symbol is the link-offsets which is in the user defined z-axis from 		#the links i-1 to i
	d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	#second symbol is the link lengths in user defined x-axis from links i-1 to i
	a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	#third symbol is twist angle 
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	#fourth symbol is theta values for rotational transformations
	q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
	
	# Create Modified DH parameters: using the symbols above to help us define transformation matrix with each transformation using two 		rotations and two translations in the two different axes
	#the calculations is shown in pictures in README with the first three pages of illustrations. 
	DH_Table = {d1: 0.75, d2: 0, d3: 0, d4: 1.5, d5: 0, d6: 0, d7: 0.303,
		    a0: 0, a1: 0.35, a2: 1.25, a3: -0.054, a4: 0, a5: 0, a6: 0,
		    alpha0: 0, alpha1: -pi/2, alpha2: 0, alpha3: -pi/2, alpha4: pi/2, alpha5: -pi/2, alpha6: 0,
		    q1: q1, q2: q2 - pi/2, q3: q3, q4: q4, q5: q5, q6: q6, q7: 0}
	
	# Define Modified DH Transformation matrix (This matrix was shown multiple times in the lesson to give two translations and two 		rotations per coordinate frame change)
	def TM(alpha, a, d, q):
		Mod_DH = Matrix([[cos(q), -sin(q), 0, a], 
					[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
					[sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d], 
					[0, 0, 0, 1]])
		return Mod_DH

	# Create individual transformation matrices using the DH Table, copy the matrix multiple times
	
	
	T0_1 = TM(alpha0, a0, d1, q1).subs(DH_Table)
	T1_2 = TM(alpha1, a1, d2, q2).subs(DH_Table)
	T2_3 = TM(alpha2, a2, d3, q3).subs(DH_Table)
	T3_4 = TM(alpha3, a3, d4, q4).subs(DH_Table)
	T4_5 = TM(alpha4, a4, d5, q5).subs(DH_Table)
	T5_6 = TM(alpha5, a5, d6, q6).subs(DH_Table)
	T6_EE = TM(alpha6, a6, d7, q7).subs(DH_Table)
	
	# finally multiply them all together and you get the T0_EE which is the end effector coordinates in the baselink 
	#coordinate frame. 
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	#
	# Extract rotation matrices from the transformation matrices
	ROT0_1 = T0_1[0:3,0:3]
	ROT1_2 = T1_2[0:3,0:3]
	ROT2_3 = T2_3[0:3,0:3]
	ROT3_4 = T3_4[0:3,0:3]
	ROT4_5 = T04_5[0:3,0:3]
	ROT5_6 = T5_6[0:3,0:3]
	ROT6_EE = T6_EE[0:3,0:3]
	
	

	#
        #

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
   	    px = req.poses[x].position.x
  	    py = req.poses[x].position.y
  	    pz = req.poses[x].position.z

  	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])         
### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #I'm assuming this is the correction between the URDF and the DH parameter axis for the gripper 
	    #We want to rotate about the z-xis 180 degrees and then around the y-axis by -90 degrees. 
	    #this graphic is shown in README
	    #due to this rotation being extrinsic, we would reverse the intrinsic operation normally done on an airplane which would be roll, 		    pitch and yaw. So to reverse x-y-z, we will find the rotation of the end effector and its correction using the opposite operations 		    of yaw, pitch and then roll (z,y,x) and then times the correction between urdf and our analysis.
            r,p,y = symbols('r p y')
            ROT_x = Matrix([[1,0,0],[0, cos(r), -sin(r)], [0, sin(r), cos(r)]]) #roll    
   	    ROT_z = Matrix([[cos(y), -sin(y), 0],[sin(y), cos(y), 0],[0,0,1]]) #yaw
   	    ROT_y = Matrix([[cos(p), 0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]]) #pitch
            R_corr_gripper = ROT_z.subs({y: pi}) * ROT_y.subs({p: -pi/2})
    
   	    ROT_EE = ROT_z * ROT_y * ROT_x * R_corr_gripper
    	    #now sub in the given values using a dictionary
    
   	    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})      
    
       
	    # Calculate joint angles using Geometric IK method
	    #First find the wrist center using the rotation matrix of the end effector and subtracting d7 from the end effector position to 	        give us WC position and orientation (which, the orientation of the WC is the same at the end effector in the z-axis direction)
    	    EE = Matrix([[px],[py],[pz]])
    	    WC = EE - .303 * ROT_EE[:,2]
	    #d = .303
	    #ROT_EE[0:2]
	    #now that we know WC, we do trigonmetric analysis to find theta1, theta2, theta3
	    # to find theta1 we know that WCx and WCy are adjacent and opposite. since joint1 rotates about the z-axis, its movement is in the 		    #xy-plane. So, we project the movement on xy-place and using SOHCAHTOA => tan^-1(O/A).  using atan2 since my mentor said it was better. I googled it and found out atan2 actually helps determine quadrants. Very helpful since I was getting wild paths and my errors were not low enough
            theta1 = atan2(WC[1],WC[0])
            #to find theta2 we need to set theta1 to zero and draw out the robot in the xz-plane. See figure in write up.
	    #we use law of cosines to find triangle angle and side values
    	    side_A = sqrt((.054)**2.0 + (1.5)**2.0)
    	    side_B = sqrt((sqrt(WC[0]**2 + WC[1]**2)-.350)**2 + (WC[2]-.75)**2)	
    	    side_C = 1.25	

	    #supporting angles to find theta2 and theta3 using law of cosines
   	    angle_a = acos((side_B**2 + side_C**2 - side_A**2)/(2*side_B*side_C))
   	    angle_b = acos((side_A**2 + side_C**2 - side_B**2)/(2*side_A*side_C))
   	    angle_c = acos((side_A**2 + side_B**2 - side_C**2)/(2*side_A*side_B))
        
        #supporting angle for finding theta2 (see write up picture)
   	    angle_aprime = atan2((WC[2]-.75),(sqrt(WC[0]**2+WC[1]**2)-.35))

 

            theta2 = pi/2 - angle_a - angle_aprime
	    #to find theta3 we set theta2 and theta1 equal to zero and analyze over xz-plane
   	    theta3 = pi/2 - angle_b - atan2(.054,1.5)
	    
	    #now find theta4,theta5,theta6
	    #we know that the Rotation matrix for 3/6 = the inverse rotation matrix of joints 0/3 times the rotation matrix for joints 0/6
	    #See write up for proof. also needed to understand the LU inversion as talked about in the course: https://docs.sympy.org/0.7.0/modules/matrices.html
            #T0_3 = T0_1 * T1_2 * T2_3
            #ROT0_3 = T0_3[:3,:3]
            ROT0_3 = ROT0_1 * ROT1_2 * ROT2_3

            ROT0_3 = ROT0_3.evalf(subs={q1:theta1, q2: theta2, q3: theta3})

    
            #T3_EE = T3_4*T4_5*T5_6*T6_EE

            
            ROT3_EE = ROT0_3.transpose() * ROT_EE
   
            
   	    	    
    	    
            #OK now we have our rotation matrix from joints 4-6
	    #For these angles we can think about that each roll, pitch and yaw is done on a seperate axis which is described
	    #by a column of the rotation matrix. From the lecture "Euler angles from a rotation matrix" We find:
	    #theta4 is represented by the roll and gamma = arctan(r32, r33)
            #theta 5 is represented by pitch or beta
            #theta6 is represented by yaw of the end effector or alpha
       
    	    
   	    theta4 = atan2(ROT3_EE[2,2], -ROT3_EE[0,2])
            theta5  = atan2(sqrt(ROT3_EE[0,2]**2 + ROT3_EE[2,2]**2), ROT3_EE[1,2])
            theta6 = atan2(-ROT3_EE[1,1], ROT3_EE[1,0])
    
    
    
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
