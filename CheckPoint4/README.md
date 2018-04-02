# ECE_470_Final_Project
# CheckPoint4

1.	The code for this CheckPoint is mainly based on the code from Checkpoint2.

2.	Initialize the 50 sets of joint angles in a linear motion.
	
3. 	Initialize the volume of robot, the volume of object, and the position of object. And the initialize c to 50 zeros, to represent whether there is a collision or not. Zero means there is no collision.

4. 	For each set of joint angles, we first test if the robot is in self collision, by checking if the joint angles are out of bounds. If the robot is in self collision, we set c(idx) to 1.

5. 	If the robot is not in self collision, we start to derive the forward kinematics for each joint.

6.	Using the points we just derived, we derive more points between joints to cover the whole robot. In the video we represents the volume of robot using yellow Dummy points.

7.	Using the method told in class, we test if the robot is in collision with other object. If the robot is in collision with other object, we set c(idx) to 2.

8.	After walking through all sets of angles, we move the red Dummy point to represent the object. And we start to move the robot through the linear motion.

9.	The collision we see from the video fits what we get from displaying c in matlab.

Here is the Link to the Youtube Video:
https://youtu.be/Z0bdfDHXgyg


