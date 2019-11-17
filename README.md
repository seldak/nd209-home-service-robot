# nd209-home-service-robot
This is a ROS project, part of the Udacity Robotics Nanodegree program. This was developed under ROS Kinetic.
This project simulates a home service robot, where the turtlebot is used to model the robot. At the start of the simulation,
the robot is given a pose estimate similar to the "2D Pose Estimate" function in RViz, through `roscpp`.

Two ROS nodes were developed.
   1. `pick_objects`: This node sends pose goals to the robot to pick-up a cube object and subsequently drop it at a different goal. 
       The act of sending the goal poses is simalar to the "2D Nav Goal" function in RViz, but through `roscpp`. 
   2. `add_markers`: This simulates the object to be transferred in RViz.

# Build
Go to your catkin workspace:
```
cd /path/to/catkin_ws
```
Clone the repository and run
```
git submodule init
git submodule update
catkin_make
source devel/setup.bash  # or whatever your environment script is
```

# Run
A script is written to launch everything in `xterm` instances. Gazebo, AMCL and RViz standard packages are launched.
The two ROS nodes developed in this project are also launched.
```
./src/home_service_robot/scripts/home_service.sh
```

It is important to execute the script as shown above, as the yaml, map and world files are referenced relatively from the catkin workspace folder.
