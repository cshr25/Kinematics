#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import math


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

	# count number of processed poses	
	n = 0 

        for x in xrange(0, len(req.poses)):
		joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
	   	d1, d2, d3, d4, d5, d6 = symbols('d1:7')
           	a0, a1, a2, a3, a4, a5 = symbols('a0:6')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6')
            
            # Joint angle symbols
		q1, q2, q3, q4, q5, q6 = symbols('q1:7')
      
       	     # Modified DH params
		a12=0.35
		a23=1.25
		a34=-0.054
		d01=0.75
		d34=1.5
		d6g=0.4

            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
		px = req.poses[x].position.x
      		py = req.poses[x].position.y
      		pz = req.poses[x].position.z

       		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                	[req.poses[x].orientation.x, req.poses[x].orientation.y,
                    	req.poses[x].orientation.z, req.poses[x].orientation.w])
            # Calculate joint angles using Geometric IK method

	#compute Rrpy transformation matrix
		xq = roll
		yq = pitch
		zq1 = yaw

		R_x = Matrix([[ 1,              0,      0],
              [ 0,        cos(xq), -sin(xq)],
             [ 0,        sin(xq),  cos(xq)]])
	
		R_y = Matrix([[ cos(yq),        0,  sin(yq)],
             [       0,        1,        0],
             [-sin(yq),        0,  cos(yq)]])
	
		R_z = Matrix([[ cos(zq1), -sin(zq1),        0],
             [ sin(zq1),  cos(zq1),        0],
              [ 0,              0,        1]])
		
		Rrpy = R_z*R_y*R_x



	# calculate wrist center position
		xc = px - d6g * cos(yaw)
		yc = py - d6g * sin(yaw)
		zc = pz - d6g * sin(pitch)

	# calculate first joint angles
		q1r = atan2 (yc, xc)
	
	# set some auxilary variables
		bottom = sqrt(xc**2 + yc**2) - a12
		height = zc - d01
	
	#calculate theta2 by dividing it to two angles
		theta21 = atan2(height, bottom)
		theta31 = atan2(bottom, height)

	# use Law of cosines to find joint angle 2 and angle 3
		costheta22 = (a23**2 + (bottom**2+height**2) - d34**2)/(2*a23*sqrt(bottom**2+height**2))
		theta22t = max(min(costheta22, 1), -1)
		theta22 = acos(theta22t)

		costheta32 = (d34**2 + (bottom**2+height**2) - a23**2)/(2*d34*sqrt(bottom**2+height**2))
		theta32t = max(min(costheta32, 1), -1)
		theta32 = acos(theta32t)


		q2rt = theta21 + theta22
		q3rt = theta31 + theta32
		
	#convert triangle angle to joint angle
		q3r =  - (pi/2 - q2rt + pi/2 - q3rt)
		q2r = pi/2 - q2rt

	#set mutable Modified DH Transformation matrix
		sl = {alpha0: 0,      a0:   0, 	 d1: d01,
		alpha1: -pi/2,  a1: a12,    d2: 0,  
		alpha2: 0,      a2: a23,    q3: 0,
   	   	alpha3: -pi/2,  a3: a34,    d4: d34,
	    	alpha4: pi/2,   a4:   0,	 d5: 0,
	    	alpha5: -pi/2,  a5:   0,    d6: 0,
		q1:q1r, q2:-pi/2+q2r, q3:q3r}

	#set each transformation matrix
		T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
		T0_1 = T0_1.subs(sl)

		T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
		T1_2 = T1_2.subs(sl)

		T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
		T2_3 = T2_3.subs(sl)

		T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
		T3_4 = T3_4.subs(sl)

		T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
		T4_5 = T4_5.subs(sl)

		T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
		T5_6 = T5_6.subs(sl)
		
	#parameters to rotate final reference frame to base one
		xq = pi
		yq = -pi/2
		R_x2 = Matrix([[ 1,              0,      0],
              			[0,        cos(xq), -sin(xq)],
             			[0,        sin(xq),  cos(xq)]])
	
		R_y2 = Matrix([[ cos(yq),        0,  sin(yq)],
             			[       0,        1,        0],
             			[-sin(yq),        0,  cos(yq)]])
		
	#compute last three joint angles
		T0_3 = simplify(T0_1 * T1_2 * T2_3 )
		R0_3=T0_3[0:3,0:3]
		rhs = transpose(R0_3)*Rrpy*R_y2*R_x2

	#use specific element to calculate joint angles as candicates
		q4r_o = atan(-rhs[2,2]/rhs[0,2])
		q6r_o = atan(-rhs[1,1]/rhs[1,0])
		q5r_o = acos(rhs[1,2])

	#processing number +=1
		n+=1

	#choice the angle that satisfy final rotation requirements

		#q4 and q6 are obtained by atan, thus their only other solution are qi + pi
		q4_cand = [q4r_o, q4r_o + pi] 
		q6_cand = [q6r_o, q6r_o + pi]

		#q5 is solved using acos, the other possible solution is -q5
		q5_cand = [q5r_o, - q5r_o]

		#flag shows whether a solid siolution can be found
		ok_flag = 0

		#initialize final solutions incase none is found
		q4r = q4r_o
		q5r = q5r_o
		q6r = q6r_o

		#compute T3_6
		lhs = simplify(T3_4*T4_5*T5_6)
		lhsr = lhs [0:3,0:3]

		#try each combination of q4,q5 and q6
		for i in range(0,2):
			for j in range(0,2):
				for k in range(0,2):
					if (ok_flag!=1): #do this only when no solid solution has been found yet
						candlist = {q4:q4_cand[i], q5:q5_cand[j],q6:q6_cand[k]}
						lhs_cand = lhsr.subs(candlist)
						# difference between T3_6 and transpose(R0_3)*Rrpy
						diffa = lhs_cand - rhs
						if (sum(diffa**2) < 1): #threshold for solid solution
							q4r = q4_cand[i]
							q5r = q5_cand[j]
							q6r = q6_cand[k]
							ok_flag = 1

		print('ok_flag=',ok_flag)	# are we lucky?		
		print('progressing...',n,"/",len(req.poses))
	
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
		joint_trajectory_point.positions = [q1r, q2r, q3r, q4r, q5r, q6r]
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
