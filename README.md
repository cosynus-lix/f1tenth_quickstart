This repository is for providing a quick start on using the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) to launch a F1tenth virtual compeition.

# Quick demonstration by using example code
## Preparation:
```
Install Docker see https://docs.docker.com/engine/install/
$ mkdir -p ~/f110_ws/src
$ cd ~/f110_ws/src/
$ git clone https://github.com/cosynus-lix/f1tenth_quickstart.git
$ git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

## Single vehicle mode:
### Step 1 - Launch the simulator:
```
$ cd ~/f110_ws/src/f1tenth_gym_ros/
$ git checkout origin/master
$ sudo ./build_docker.sh (it takes several minutes)
$ sudo ./docker.sh
```
In case that fail to download docker image (take too much time) or not correctly compile, try to launch the simulator alternatively:
```
$ cd ~/f110_ws/src/f1tenth_gym_ros/
$ ./start.sh
```


### Step 2 - Launch the controller
```
$ cd ~/f110_ws/
$ catkin_make
$ roslaunch f1tenth_controller_example wall_following_agent_node.launch
```

## Head-to-head mode, ego + opp:
### Step 1 - Launch the controller
```
$ cd ~/f110_ws/src/f1tenth_gym_ros
$ git checkout origin/multi_node
$ sudo ./build_docker.sh (it takes several minutes)
$ sudo ./docker.sh
```

### Step 2 - Launch the controller
```
edit the class Agent() in file wall_following_agent_node.py: comment the part about “single vehicle racing” and uncomment the part about “head-to-head racing”
$ cd ~/f110_ws/
$ catkin_make
$ roslaunch f1tenth_controller_example wall_following_agent_node.launch
```

# Write the controller code by yourself!
## What to change?
1. in `src/`: modify the file `wall_following_agent_node.py` or write an independent file with similiar structure; you can also write the controller in C++, refer to [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials). 
2. the main point is to get environment information from the topic `/scan`(Message type - [`sensor_msgs/LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html), [more details](http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData)) and(or) `/odom`(Message type - [`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html), [more details](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)), calculate the controll command and send to the topic `/drive` (Message type - [`ackermann_msgs/AckermannDrive`](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html), [more details](http://wiki.ros.org/Ackermann%20Group)).
2. in `Launch/`: in case that you write the controller in a new file, write a launch file similiar to `wall_following_agent_node.launch`.
3. file `package.xml`: in case that you use new dependancies (such as `ackermann_msgs` etc), add them in this file.
4. file `CMakeLists.txt`: in case that you write the controller in C++, you need to modify this file.

## What is missing in the example code?
1. in the example code, we only use the information of Lidar via the topic `/scan` but not `/odom`. You can use `/odom` to obtain the ego agent's odometry for your own algorithm!
2. we have not use the information from the topic `/race_info`. You can subscript to it for getting both agents' elapsed runtimes, lap count, and the collsion info.
3. for head-to-head mode, we suppose that the ego and opp vehicle use the same controller and put their controller inside a same file. Of course you can use different controllers for each! In practice, it is better to write their controllers in different files. 
