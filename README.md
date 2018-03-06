# ECE_470_Final_Project
1.	Download V-REP PRO EDU from the Coppelia Robotics website:
	Here is the [Link!](http://www.coppeliarobotics.com)

2.	From the folder run "./vrep.app/Contents/MacOS/vrep"
	![image of terminal](https://github.com/YueXxxXuuu/ECE_470_Final_Project/blob/master/image_1.png)

3.	Drag a robot into the GUI, and remove associated scripts.
	![image of simulator](https://github.com/YueXxxXuuu/ECE_470_Final_Project/blob/master/image_2.png)

4.	Save the scene "File->Save Scene As..."

5. 	Install python. Conda is highly recommended (Products -> Download -> Python 3.6 version)
	Here is the [Link!](https://www.anaconda.com)

6.	create a workspace for your robot: create a new "my_workspace" folder.

7.	copy these files into "my_workspace" folder:
	vrep/programming/remoteApiBindings/python/python/vrep.py
	vrep/programming/remoteApiBindings/python/python/vrepConst.py
	vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
	
8.	copy this python test code (vrep/programming/remoteApiBindings/python/python/simpleTest.py) into "my_workspace"

9.	modify this code like the code we provide, rename it "test.py"

10.	from terminal run "python test.py"

You should see your robot move, and in the terminal, you should see the joint angle values. 

More information can be found in the documentation (e.g., look at "Writing code in and around V-REP"):
	Here is the [Link!](http://www.coppeliarobotics.com/helpFiles/index.html)
	
Commands available to you in the python remote API:
	Here is the [Link!](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
