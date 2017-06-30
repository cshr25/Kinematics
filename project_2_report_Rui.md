

[//]: # (Image References)
[image1]: ./DH_info.PNG
[image2]: ./transform.PNG
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

In this part, the Kuka KR210 DH parameters were obtained by measurment in the Rviz demo and validated approximated by the given part center position. 

At first, I was trying to search for length and bias information in kr210's urdf file, which only offered inertial and visual origins. Therefore, the distance between joints as required in DH table were measured by the measurement tool in Rviz and compared to the 'link position' values listed in the link inforamtion. The later information is shown below:

![alt text][image1]

And the difference between the base link and link 2 on z axis is the value for d1 in DH.
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

R_x = __
				| 1 |            0 |      0  |
				|---| ---					 |---			 |	
				| 0	|       cos(xq)| -sin(xq)|	
 				| 0	|       sin(xq)|  cos(xq)|
	
R_y = [ cos(yq),        0,  sin(yq)]
[       0,        1,        0]
[-sin(yq),        0,  cos(yq)]
	
R_z =[ cos(zq), -sin(zq),        0]__
		[ sin(zq),  cos(zq),        0]__
		[ 0,              0,        1]
              
And the angles substituted in are yaw (z axis), pitch (y axis) and roll (x axis) angles.
And the T vector is the origin shift between end-effector and the base link.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


