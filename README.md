# table-tennis-robot
A robot that can play table tennis, our final project for Berkeley EECS 106A. Please check out [our wiki](https://github.com/AVSurfer123/table-tennis-robot/wiki) for an in-depth report and discussion of what we did.

## Installation

Clone the project then move inside it. Then install required lib

    sudo apt install ros-noetic-desktop-full
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-rviz ros-noetic-tf2-ros
    pip install scipy numpy

Then do

    rosdep install --from-paths src --ignore-src -r -y

to install all necessary dependencies. Finally, do `catkin_make` from the root workspace directory. 

Now to run the robot in Gazebo with MoveIt, do

    roslaunch kuka_kr5_gazebo moveit_control.launch

Then to begin the vision pipeline, do

    roslaunch ball_detection vision_pipeline.launch

Then to begin the robot arm controller, do

    rosrun planning ball_controller.py

Then to spawn in some ping pong balls, do 

    rosrun kuka_kr5_gazebo spawn_ball.py

