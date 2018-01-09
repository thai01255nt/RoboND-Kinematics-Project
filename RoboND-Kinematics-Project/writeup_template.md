## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

In here, you can use Rviz or kr210.urdf.xacro file to take the relative position between current joint and its parent joint. The choice of x-y-z axis for each frame follows the rules of minimizing the number of non-zero parameters. Or you can customize if it make you have more convinient (thuan tien hon). But becareful, No có thể gây ra sai số nhiều hơn.

*** NOTE: nếu bạn dùng Rviz, framework của các link sẽ được init cùng orientation với base-link

For example, the d value of joint 1 can be chosen to be 0.33 or other arbitrary number. But it is better to choose x1 axis to be intersected with next joint, which can reduce the number of non-zero parameters. As a result, D is chosen to be 0.75 for joint 1.
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0
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

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 
Đầu tiên, extract end-effector position and orientation from request.
Từ EE orientation tính ra roll,pitch,yaw

Sau đó, theo phương pháp instric rotation (em k nhớ rõ tên) chuyển từ framework EE về framework base-link (chuyển hệ tọa độ từ EE về gốc) theo thứ tự quay của các axis là x->y->z tương ứng với các góc roll,pitch,yaw.

Do đó Rotation equation để Rotate EE frame về base-link sẽ là ROT_EE = ROT_z(yaw) * ROT_y(pitch) * ROT_x(roll)

Tuy nhiên, do sự khác nhau giữa DH parameters và Gazebo + Rviz. Nên trước tiên cần Rotate EE frame trong DH parameters về trùng với EE frame trong Rviz.

==> Rot_corr = ROT_z(pi) * ROT_y(-pi/2)
==> ROT_EE = ROT_z(yaw) * ROT_y(pitch) * ROT_x(roll) * Rot_corr

Do link 6 là cố định chúng ta có thể tính link5 position trong base_link framework theo công thức sau

    Vector 0_5(base_link framework) = Vector 0_EE(base_link framework) - Vector5_EE(base_link framework)
    Vector5_EE(base_link framework) = ROT_EE * Vector5_EE(EE framework)
    Vector 0_EE(base_link framework) = EE position
    link 5 position = Vector 0_5(base_link framework)
Than we have postion link 5. Calculate the angles of first 3 joints by the Wrist Center using cosine laws
With each EE position and orientation, we can find many results of angles. So if any result of joint angles satify firstly all of condition and equation, we will take it because joint angles still make right EE position and orientation.

With these 3 angles - 1,2,3, calculate the R0_3, then get R3_EE **ROT_EE = R0_3*R3_EE

The remaining angles can be calculated by the matrix element.

    theta6 = atan2(-R3_EE[1,1],R3_EE[1,0])

    cos5 = R3_EE[1,2]
    sin5 = -R3_EE[1,1]/sin(theta6)
    theta5 = atan2(sin5,cos5)

    cos4 = R3_EE[0,2]/(-sin(theta5))
    sin4 = R3_EE[2,2]/sin(theta5)
    theta4 = atan2(sin4,cos4)
Becarefull if sin(theta6) or sin(theta5) = 0 , check it!

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Tôi vẫn chưa hiểu rõ về việc calculate roll,pitch,yaw from EE orientations.

Tôi gặp vấn đề về việc load models when i launch this project, so i changed path of meshes in kr210.urdf.xacro from "package://" to "file:///home/..." and it worked

When i calculate joint angles 1,2,3. tôi nhận ra sẽ có nhiều nghiệm. Do đó t quyết định vẫn xét lần lượt các nghiệm cho đến khi tìm ra được bộ nghiệm thỏa mãn đầu tiên cho angles 1,2,3. Điều đó gây ra việc sai angles, tuy nhiên vẫn cho kết quả đúng với EE position and orientation.

Bạn có thể sẽ confuse khi xem cách tính theta 2,3 của tôi. từ theta1 calculate link 2 position. Do biết position of link 5 và link 2. Tôi có thể tính khoảng cách giữa link2 và link5, link 3 và link 5(it's fixed) and link 2 and link3(it's fixed). Sau đó Tôi sử dụng công thức cosin để tính 3 góc của tam giác 235 (3 đỉnh là link 2, link3, link5). Từ đó tìm ra theta 2 and theta 3



