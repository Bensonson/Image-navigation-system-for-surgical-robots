# 7MRI0070-Assignment
This is a course assignment project.

Step1:

1. Open 3D Slicer

2. Import your entries, targets, vessel, and ventricle

3. In Python terminal run:
exec(open(your_path/PathPlanning.py').read())

best_path, max_d = findpath('entries_name', 'targets_name', 'vessels_name', 'ventricles_name', max_length=infinity, scale_rate=1)
(You can set max_length to any values and scale_rate to any integers.   Note: larger scale_rate brings significant performance loss.)

NOTE: findpath() will freeze your computer for a while, minutes or hours. This is normal, do not force quit 3D Slicer. The result from whole dataset is in folder "EntryTarget".

gettransform('entry', 'target',patient_position=[0,0,0])
(You can set a different patient_position for batter path)

4. Open OpenIGTLinkIF and build connection with Ubuntu machine with ROS.


Step2:

1. Build a ROS workspace and put folder "MyBot" into the space

2. Open a terminal, and run:

source your_workspace_path/devel/setup.bash

roslaunch MyBot demo.launch

3. Open another terminal, and run:

source your_workspace_path/devel/setup.bash

rosrun MyBot PoseGoal0.2.py

4. Follow the instructions in the terminal opened in step2.3. You should be able to see the movement in RViz, the window you open in step 2.2.
