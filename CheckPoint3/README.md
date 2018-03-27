# ECE_470_Final_Project
# CheckPoint3

1.	Get a set of positions randomly. We picked three reachable points and one not reachable point.

2.	Get the starting postion and orientation of the end-effector and get the starting T.
	
3. 	Get all six screw axes for all the six joints using V_REP.

4. 	Use four dummy points to represent the four points we are going to solve in inverse kinematics.

5. 	Use the method we used in homework: S is the six screw axes, M is the starting T, and T_1in0 in the point's position and orientation.

6.	After getting the angels, we limit them to the range from -180 degree to 180 degree.

7.	Give the Jaco robot these angles and start to simulate.

8.	Do step 5,6,7 for all 4 points.

9.	The three reachable points fit what we expected.

10. The not reachable point showed us an error, after it has spent a long time searching for a solution.

Here is the Link to the Youtube Video:
https://youtu.be/S9E25wkMWDY


