# RoboND-Kinematics-Project (April 2018)
## Project: Pick and Place project for RoboND Term 1
### Divya Patel (OptimomEngineer)
---

[//]: # (Image References)

[image1]: ./misc/Kuka_arm_image_1.png
[image2]: ./misc/Kuka_wire_model.png
[image3]: ./misc/Kuka_arm.png 
[image4]: ./misc/Rotation Matrices_xyz.png
[image5]: ./misc/Rotation_Matrix_forTheta456.png
[image6]: ./misc/gripper_frame_DHparametersvsURDF.png
[image7]: ./misc/IK_debug_results_image.png
[image8]: ./misc/rviz_screen_image.png

This is the Udacity Kinematics Project for Term 1 of the Sofware Robotics Nanodegree Program. 

**The goals / steps of this project are the following:**
1) **Forward Kinematics Model** The first purpose of the project was to calculate the forward kinematic model

2) **Inverse Kinematic model** The IK model allowed the robot to identify location of an object on a shelf, approach and pick up object, calculate a trajectory path to a destination (a bin in this case) and follow the path to drop object in bin. The success metric was 8/10 on object identification, pick up and place in bin. The object appeared in at each attempt randomly at 9 different shelf locations.

![Robot Model][image1]

Above you can see the environment the kuka arm was located in as well as the shelf, the cylindrical object that the Kuka Arm is currently dropping into the bin.


