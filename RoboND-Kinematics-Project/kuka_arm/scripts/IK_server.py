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

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols

	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
	DH = {  alpha0:	 0	, a0: 0		, d1: 0.75	, q1: q1,
		alpha1: -pi/2	, a1: 0.35	, d2: 0		, q2: -pi/2 + q2,
		alpha2:  0	, a2: 1.25	, d3: 0		, q2: q3,
		alpha3: -pi/2	, a3: -0.054	, d4: 1.5	, q3: q4,
		alpha4:  pi/2	, a4: 0		, d5: 0		, q4: q5,
		alpha5: -pi/2	, a5: 0		, d6: 0		, q5: q6,
		alpha6:  0	, a6: 0		, d7: 0.303	, q7: 0
		}

	# Define Modified DH Transformation matrix
	def T_Matrix(alpha, a, d, q):
		TF = Matrix([[	cos(q),		-sin(q),	0,	a],
			[sin(q)*cos(alpha),	cos(q)*cos(alpha),	-sin(alpha),	-sin(alpha)*d],
			[sin(q)*sin(alpha),	cos(q)*sin(alpha),	cos(alpha),	cos(alpha)*d],
			[	0,	0,	0,	1]])
		return TF	
	# Create individual transformation matrices
	T0_1 = T_Matrix(alpha0, a0, d1, q1)
	T0_1 = T0_1.subs(DH)
	T1_2 = T_Matrix(alpha1, a1, d2, q2)
	T1_2 = T1_2.subs(DH)
	T2_3 = T_Matrix(alpha2, a2, d3, q3)
	T2_3 = T2_3.subs(DH)
	T3_4 = T_Matrix(alpha3, a3, d4, q4)
	T3_4 = T3_4.subs(DH)
	T4_5 = T_Matrix(alpha4, a4, d5, q5)
	T4_5 = T4_5.subs(DH)
	T5_6 = T_Matrix(alpha5, a5, d6, q6)
	T5_6 = T5_6.subs(DH)
	T6_7 = T_Matrix(alpha6, a6, d7, q7)
	T6_7 = T6_7.subs(DH)
	
	T0_7 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

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
	    r,p,y = symbol('r p y')

	    ROT_x = Matrix([[1,		0,	0,
			     0,		cos(r),		-sin(r),
			     0,		sin(r),		cos(r)]])
	    ROT_y = Matrix([[cos(p),	0,	sin(p),
			     0,		1,	0,
			     -sin(p),	0,	cos(p)]])
	    ROT_z = Matrix([[cos(y),	-sin(y),	0,
			     sin(y),	cos(y),		0,
			     0,		0,	1]])
	    ROT_EE = ROT_z * ROT_y * ROT_x
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    Rot_corr = ROT_z.subs({y: np.pi})*ROT_y.subs({p: -np.pi/2})
	    
	    # Calculate joint angles using Geometric IK method
	    ROT_EE = ROT_EE * Rot_corr
	    ROT_EE = ROT_EE.subs({r: roll, p: pitch, y: yaw})
		
	    EE = Matrix([[px],
			[py],
			[pz]])
	    
	    WC = EE - (0.303) * ROT_EE*Matrix([[0,0,1]])
	    #
            ###
	    _theta1 = atan2(WC[1],WC[0])
	    #alway have two results of (theta1,theta2,theta3) maybe right or not, if the first result can be satisfy, we will continue with theta4,5,6 
	    theta1 = []
	    theta1.append(_theta1)
	    theta1.append(_theta1 - np.pi)
	    
	    for p1 in theta1:
	     ## calculate position of link 2 in base Frame
	     link_2 = ROT_z.subs({y:p1})*Matrix([[0.35],[0],[0.75]])
	     
	     ## calculate distance between link 2 and link WC
	     distance_2_5 = float(sqrt(pow(link_2[0]-WC[0],2)+pow(link_2[1]-WC[1],2)+pow(link_2[2]-WC[2],2)))
	     distance_3_5 = float(sqrt((0.54+0.96)**2 + 0.054**2))
	     signed = 1
	     if abs(WC[0]) <= abs(link_2[0,0]) or WC[1]*link_2[1,0] < 0:
	      signed = -1
	     cos_link_3 = float((-distance_2_5**2 + 1.25**2 + distance_3_5**2))/(2*1.25*distance_3_5)
	     if abs(cos_link_3) > 1:
	      continue
	     v25 = Matrix([[signed*sqrt((WC[0]-link_2[0])**2 + (WC[1]-link_2[1])**2)],[WC[2]-link_2[2]]])
	     angle_z_25 = atan2(v25[0,0],v25[1,0])
	     cos_325 = (1.25**2 + distance_2_5**2 - distance_3_5**2)/(2*distance_2_5*1.25)
	     angle_325 = acos(cos_325)
	     theta2 = []
	     theta3 = []
	     theta2.append(-angle_325 + angle_z_25)
	     theta2.append(+angle_325 + angle_z_25)
	     angle_235 = acos(cos_link_3)
	     theta3.append(atan2(1.5,0.054) - angle_235)
	     theta3.append(atan2(1.5,0.054) - (angle_235 + 2*(np.pi - angle_235)))
             #Check satify limit of theta2 and theta3:
	     #If first result satify we will take it and break
	     if theta2[0]>= -0.79 and theta2[0] <=1.48 and theta3[0]>=-3.67 and theta3[0] <=1.13: 
	      theta2 = [theta2[0]]
	      theta3 = [theta3[0]]
	      theta1 = p1
	      break
	     if theta2[1]>= -0.79 and theta2[1] <=1.48 and theta3[1]>=-3.67 and theta3[1] <=1.13:
	      theta2 = [theta2[1]]
	      theta3 = [theta3[1]]
	      theta1 = p1
	      break
	     #If don't have any results satify
	     theta2 = []
	     theta3 = []
	    if len(theta2) == 0 or len(theta3) == 0:
	     joint_trajectory_point.positions = [0, 0, 0, 0, 0, 0]
	     joint_trajectory_list.append(joint_trajectory_point)
	     rospy.loginfo("Error: out of boundary")
             return CalculateIKResponse(joint_trajectory_list)
	    theta2 = theta2[0]
	    theta3 = theta3[0]
	    ## Calculate T0_3 with theta1,theta2,theta3
	    T0_1 = T_Matrix(alpha0, a0, d1, theta1)
    	    T0_1 = T0_1.subs(DH)
	    T1_2 = T_Matrix(alpha1, a1, d2, theta2-pi/2)
	    T1_2 = T1_2.subs(DH)
	    T2_3 = T_Matrix(alpha2, a2, d3, theta3)
	    T2_3 = T2_3.subs(DH)
	    
	    T0_3 = T0_1*T1_2*T2_3
	    
	    ## Calculate R3_EE 
	    T3_4 = T_Matrix(alpha3, a3, d4, q4)
	    T3_4 = T3_4.subs(DH)
	    T4_5 = T_Matrix(alpha4, a4, d5, q5)
	    T4_5 = T4_5.subs(DH)
	    T5_6 = T_Matrix(alpha5, a5, d6, q6)
	    T5_6 = T5_6.subs(DH)
	    T6_7 = T_Matrix(alpha6, a6, d7, q7)
	    T6_7 = T6_7.subs(DH)

	    R0_3 = T0_3[0:3,0:3]	    ## ROT_EE = R0_3*R3_EE
	    R3_EE = R0_3.inv('LU')*ROT_EE
	    ## theta 6
	    theta6 = atan2(-R3_EE[1,1],R3_EE[1,0])
	    cos5 = R3_EE[1,2]
	    ## check sin == 0 with each angle.
            if sin(theta6) <= 0.0001:
	     sin5 = R3_EE[1,0]/cos(theta6)
	    if sin(theta6) > 0.0001:
	     sin5 = -R3_EE[1,1]/sin(theta6)
	    #theta 5
	    theta5 = atan2(sin5,cos5)
	    #theta 4
	    if sin(theta5) <= 0.0001:
	     if sin(theta6) <= 0.0001:
	      theta4 = atan2(-R3_EE[2,0],-R3_EE[2,1])
	     if sin(theta6) > 0.0001:
	      sq4_cq4 = Matrix([[-cos(theta6),-sin(theta6)],[sin(theta6),-cos(theta6)]]).inv('LU')*Matrix([[R3_EE[2,0]],[R3_EE[2,1]]])
	      sin4 = sq4_cq4[0,0]
	      cos4 = sq4_cq4[1,0]
	      theta4 = atan2(sin4,cos4)
	    if sin(theta5) > 0.0001:
	     cos4 = R3_EE[0,2]/(-sin(theta5))
	     sin4 = R3_EE[2,2]/sin(theta5)
	     theta4 = atan2(sin4,cos4)
	   
	  
	    #R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
	    #R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})

	    #R3_6 = R0_3.inv('LU') * ROT_EE
	    #theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
	    #theta6 = atan2(-R3_6[1,1], R3_6[1,0])
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
