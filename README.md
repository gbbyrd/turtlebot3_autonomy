# Description
This package is meant to provide examples for various autonomy algorithms for the Turtlebot3 for ROS 1 Noetic. The code for each of these examples is broken down into several python files for real world and simulation implementations. Each task is broken down into its own python file to test individually. `final_real.py` and `final_gazebo.py` files combine all of the tasks into one file that can be ran in the simulation and real world environments to show an example autonomy stack.

# How to Install
This package is meant for ROS Noetic on Ubuntu 20.04. Ensure that you have Ubuntu 20.04 installed before continuing.

First, complete the Overview, Features, and Quick Start Guide sections located here: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview. For the Quick Start Guide portion, ensure that you have chosen "Noetic" at the top of the screen.

*If you only want to run the gazebo simulations, you only need to complete up to 3.1.5 "Set TurtleBot3 Model Name" in the Quick Start Guide.*

PyTorch with CUDA will also need to be installed as the AI models for object detection are implemented in PyTorch. Refer to the installation guide here: https://pytorch.org/get-started/locally/.

Once you have completed the initial setup, follow the steps below to setup your directory and install the necessary packages and dependencies.

1. Create a base directory and name it whatever you like. Navigate into that directory.
2. Create a catkin workspace and src subdirectory by running the below command.

```
mkdir catkin_ws && cd $_ && mkdir src && cd $_
```

3. Download this package as well as the necessary packages by running the below command.

```
git clone https://github.com/gbbyrd/turtlebot3_autonomy.git & \
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git & \
git clone -b feature-raspicam https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
```

4. Build the catkin workspace by navigating to your catkin_ws directory and running the below command.

```
catkin_make
```

5. Each time you restart a terminal, you will need to run the following command in your catkin workspace to set up your paths for the roslaunch commands.

```
source devel/setup.bash
```

6. Once this is done, copy the course and traffic_stop folders in the `catkin_ws/src/turtlebot3_autonomy/models` directory and place them in the `catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models`.

You should now have a built catkin workspace complete will all of the launch files and python files necessary for running these autonomy algrithms in the gazebo simulator as well as the real world!

# World Overview
The simulation world is broken up into 4 different sections each requiring different algorithms to navigate. These algorithms will rely on camera and LiDAR sensors to navigate. These 4 navigation tasks are some of the most common tasks for real world autonomy, so real world environments could be easily built to test out the real world algorithms.

![alt text](https://github.com/gbbyrd/turtlebot3_autonomy/blob/master/catkin_ws/src/turtlebot3_autonomy/ref/sim_world_overview.png?raw=true)

# Algorithms Explained

Before running any of the following python files, launch your simulation environment with the following command (make sure to `source devel/setup.bash` in your catkin_ws first).

```
roslaunch turtlebot3_autonomy turtlebot3_autonomy_final.launch
```

## Wall Following and Obstacle Avoidance

The wall following and obstacle avoidance functionality of the robot use the same algorithm. This algorithm follows the following steps:

1. Divide the lidar into two left and right sections, each of 60 degrees.
2. The LiDAR outputs values between 0 and 3.5 with anything greater than 3.5 corresponding to infinity. Normalize the lidar data to be between 0 and 1 using the sigmoid function and some basic math with numbers closer to 1 corresponding to closer objects and numbers closer to zero representing objects further away.
3. Take a sum of the normalized lidar values for the left and right sections. Find the difference between them. This difference becomes the error for the PID controller. Input this error into the PID controller to get angular velocity for the turtlebot.

If the turtlebot detects more LiDAR points closer to it in one section, it will turn away from that section.

![alt text](https://github.com/gbbyrd/turtlebot3_autonomy/blob/master/catkin_ws/src/turtlebot3_autonomy/ref/obstacle_avoidance.png?raw=true)

To perform the wall following and obstacle avoidance, launch your gazebo world and navigate to the src directory of the turtlebot3_autonomy package. Run `python3 obstacle_avoidance_gazebo.py`.

![alt text](https://github.com/gbbyrd/turtlebot3_autonomy/blob/master/catkin_ws/src/turtlebot3_autonomy/src/ref/obstacle_avoidance.gif)

## Line Following

The line following algorithm uses blob detection and masking. First, the hsv parameters must be tuned to get a binary image of the camera feed where the line is represented in white (1s) and the rest of the image is represented in black (0s). To get the min and max hsv values, take a picture of whatever line you are trying to follow and change the file location to that picture in the hsv_tuner.py file. Run the file to tune your picture. Once the hsv parameters are tuned, a binary image of the camera feed can be created. The binary image is then masked by cropping out all but the very bottom of the image (the part of the line closest to the turtlebot). The centroid of the binary image is then calculated, which corresponds to the center of the line. The error for the PID controller is the distance between this centroid and the center of the image.

![alt text](https://github.com/gbbyrd/turtlebot3_autonomy/blob/master/catkin_ws/src/turtlebot3_autonomy/ref/binary_image.png?raw=true)

To perform the wall following and obstacle avoidance, launch your gazebo world and navigate to the src directory of the turtlebot3_autonomy package. You can teleop the turtlebot to the line following portion of the world using `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`. Run `python3 line_follower_gazebo.py` to follow the line.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/wall_following.gif)

## Stop Sign Detection

The stop sign detection uses a PyTorch implementation of Yolov5. For more information on Yolov5 and this specific implementation, look here: https://pytorch.org/hub/ultralytics_yolov5/.

To perform the stop sign detection, launch your gazebo world and navigate to the src directory of the turtlebot3_autonomy package. You can teleop the turtlebot to the stop sign using `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`. Run `python3 detect_sign_gazebo.py` to begin detecting stop signs. You can telop your Turtlebot around and see that it will detect the stop signs.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/stop_sign_detection.gif)

## Full Gazebo Stack
To run the full stack that can autnomously navigate the gazebo world, launch your gazebo world, navigate to the src directory of the turtlebot3_autonomy package and run the following command.

```
python3 final_gazebo.py
```

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/full_vid.gif)

## Real World
The algorithmns for the real world implementation are the same as those used in the simulation with some minor differences in the topics that are used for the subscribers. These algorithms are meant to be modified and explored to understand the difficulties associated with going from simulation to the real world. Here is an example of the wall following and obstacle avoidance working on a real Turtlebot.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/real_world_obstacle_avoidance.gif)

Maintainers:

* Grayson Byrd