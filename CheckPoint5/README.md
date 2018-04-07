# ECE_470_Final_Project
# CheckPoint5

1.	Obtain the Homogeneous transfor function from world frame to base frame.

2.	Initialize the obstacles' positions and size and put yellowDummy Points to represent these obstacles.
	
3. 	Initialize the start and goal configurations. 

4. 	Use forward kinematics to derive the goal position of the goal configuration and use a red Dummy Point to represent it.

5. 	Use the path planning method we told in class to derive a desired path which contains several line segments.

6.	Note that I used DH frames to obtain the positions of Jaco joints. And I used more points to represent the Jaco volume, just as last Check Point.

7.  In order to make the Jaco robot move smoothly, I divided each line segment into 50 configurations.

8.	The result fits what we expected. The Jaco robot reaches the goal positions without hitting the obstacles.

9. Note that in order to decrease the run time, the video only shows some simple configurations. I have tested more complecated cases, and they also works.

Here is the Link to the Youtube Video:
https://youtu.be/wcz2E2z5ubE


