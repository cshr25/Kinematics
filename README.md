# Robotic arm - Pick & Place project

This is the second robot project I received from udacity robotic and machine learning course.
In this project, a simlation involving a robotic arm built in GAZEBO along with ROS is performed. We were asked to solve for a series of inverse-kinematics problem so that the robotic arm could finish the pick & place task.

The project report is attached below:



[//]: # (Image References)
[image1]: ./DH_info.PNG
[image2]: ./transform.PNG
[image3]: ./Scheme.jpg	
[image4]: ./Scheme2.jpeg	

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

In this part, the Kuka KR210 DH parameters were obtained by measurment in the Rviz demo and validated approximated by the given part center position. 

At first, I was trying to search for length and bias information in kr210's urdf file, which only offered inertial and visual origins. Therefore, the distance between joints as required in DH table were measured by the measurement tool in Rviz and compared to the 'link position' values listed in the link inforamtion. The later information is shown below:

![alt text][image1]

For example, the difference between the base link and link 2 on z axis is the value for d1 in DH.
Accordinglly, the full DH table was obtained and shown below:

i | alpha_i-1 | a_i-1 | d_i | theta_i
--- | --- | --- | --- | ---
1 | 0 | 0 | d1 | Theta_1
2 | -pi/2 | a1 | 0 | Theta_2 - pi/2
3 | 0 | a2 | 0 | Theta_3
4 | -pi/2 | a3 | d4 | Theta_4
5 | pi/2 | 0 | 0 | Theta_5
6 | -pi/2 | 0 | 0 | Theta_6
7 | 0 | 0 | d_G |0

And the parameter values are (units are meter)
    a12=0.35
		a23=1.25
		a34=-0.054
		d01=0.75
		d34=1.5
		d6g=0.4

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The modified DH transformation matric has the form of 
[             cos(qi),            -sin(qi),            0,              a_i-1],
               [ sin(qi)*cos(alpha_i-1), cos(qi)*cos(alpha_i-1), -sin(alpha_i-1), -sin(alpha_i-1)*di],
               [ sin(qi)*sin(alpha_i-1), cos(qi)*sin(alpha_i-1),  cos(alpha_i-1),  cos(alpha_i-1)*di],
               [                   0,                   0,            0,               1]
Each individual transormation matrices between joints could be easily calculated. 

The homogeneous transform between base_link and gripper_link should have a form as shown below__

![alt text][image2]

Where R is the rotation matrix calculated by doing a rotation composition such as 

R= Rz * Ry * Rx

where Rz, Ry and Rx are rotaion matrix along z,y,and x axis:

R_x=


1 | 0 | 0 
--- | --- | --- 
0 | cos(xq) | -sin(xq) 
0 | sin(xq) | cos(xq) 


	
R_y = 


1 | 0 | 0 
--- | --- | --- 
0 | cos(yq) | -sin(yq) 
0 | sin(yq) | cos(yq) 
	
R_z =


1 | 0 | 0 
--- | --- | --- 
0 | cos(zq) | -sin(zq) 
0 | sin(zq) | cos(zq) 
              
The angles (xq, yq, zq) substituted in are yaw (z axis), pitch (y axis) and roll (x axis) angles.
And the T vector is the origin shift between end-effector and base link.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Position Kinematics
Since the end-effector yaw-pitch-roll angles are given, we can compute the end-effector center position by

xc = px - d6g * cos(yaw)


yc = py - d6g * sin(yaw)


zc = pz - d6g * sin(pitch)

Then we can compute the first joint angles that could reach this center position. The scheme of the robot arm is shown below:

![alt text][image3]

As the figure shows, the first joint angle could be easily calculated by


q1r = atan2 (yc, xc)


But the other two joint angles have to be computed by solving the equations listed below:


s2*sin(Theta_3) +s1*sin(Theta_2) = s4


s2*cos(Theta_3) + s1*cos(Theta_2) = s0


s2 = d34

s1 = a23

s4 = zc - d01

s0 = sqrt(xc^2 + xy^2) - a12


The equations is solvable but sonsume a lot of computation time. Therefore, in my code the problem is solve by adding an auxiliary line so that Theta_2 and Theta_3 could be solvd in such a mannar:

![alt text][image4]


Since all three sides of the triangle O'-K-P_c are known, we can solve the angles Theta_22 and Theta_32 using the law of cosine:

c^2 = a^2 + b^2 - 2*a*b *cos(C)

And angle Theta_21 and Theta_31 could be computed by atan functions. Thus, we obtained the first three joint angles.

##### Inverse Orientation Kinematics

In this part I just followed the procedure taught class to compare between guiven rotation transformation matrix (calculated using yaw-pitch-roll angles) and obtained transformation matrix to solve for joint angle.

The desired rotation transformation is:

Rrpy = R_z(yaw)*R_y(pitch)*R_x(roll)

where R are rotation matrix on corresponding axis. But since the last reference frame has a different orientation compared to the base one, we need to multiply it with two additional rotation matrix to put them aligned. Thus, the final Rrpy matrix is

Rrpy = R_rpy * R_y2(-pi/2)*R_x2(pi)

Ideally, this rotation matrix should be equal to the overall combination of all joint rotations matrics:

R0_6 = Rrpy

Since we now have the first three joint angles and Inv(R0_3)=Transpose(R0_3), we can compute R3_6 and R0_3 that:


R3_6 = inv(R0_3) * Rrpy = RHS


Thus, we can obtain a matrix equation:


RHS = LHS =

cos(q4)*cos(q5)*cos(q6)-sin(q4)*sin(q6)	|-cos(q6)*sin(q4)-cos(q4)*cos(q5)*sin(q6)	|-cos(q4)*sin(q5)
|---|---|---
cos(q6)*sin(q5)	|-sin(q5)*sin(q6)	|cos(q5)
-cos(q5)*cos(q6)*sin(q4)-cos(q4)*sin(q6)|	cos(q5)*sin(q4)*sin(q6)-cos(q4)*cos(q6)|	sin(q4)*sin(q5)	

as the matrix indicates, we can compute the last joint angles by using RHS elements so that:

q4r = atan(-rhs[2,2]/rhs[0,2])


q6r = atan(-rhs[1,1]/rhs[1,0])


q5r = acos(rhs[1,2])

But because these angles were computed using anti-triangle functions, the solution might not be the exact angle. To deal with this, I put a simple test loop in the coding to test all possible solution given the solved angles above.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The coding part is very much alike the process discussed above. The only thing needs to be shown here is the solution test part. The code go as follows:


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


This code will compute the exact RHS and LHS and show whether a solid solution is found. 

Also, a counter is added in to demonstrate the progress.

In the simulation, this code could accomplish the desired task almost every time. But some times the sover gave out some tidious rotation (such as one 180 degree followed by -180 degree) near the dropping point. 
