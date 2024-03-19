[![ROS 2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)

# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots.

The documentation is available here: https://imrclab.github.io/crazyswarm2/.

# Introduction 
This Github Repository uses an existing ROS 2 based stack called Crazyswarm 2 to achieve object transportation using Multiple Crazyflie drones, Crazyswarm 2 is a library for controlling a swarm of crazyflies. For all the experiments the Crazyflie 2.1 was used along with the Flowdeck V2 for local positioning, the aim of the experiment was to lift a simple flexible object to a desired height with 2 quadrotor robots with one of them being the leader and the other following the leader's movements balancing the object while lifting, the image of the crazyflie 2.1 and the Flowdeck V2 is attached below

![crazyflie_2 1](https://github.com/Eccelric/Crazyflie_Object_Transport/assets/87143120/824c121a-addf-4d90-b602-2491684b5901)

![image](https://github.com/Eccelric/Crazyflie_Object_Transport/assets/87143120/5e3ee283-0a9d-4140-9912-575965e212d8)

The python scripts for the leader robot node and the follower robot node are in crazyflie->scripts, The leader_cf.py is the ROS2 node controlling the leader robot and the follower_cf.py is the ROS2 node controlling the follower robot, the follower robot uses the odometry of the leader robot to track it's position and balance the object while lifting.

# Results

The graphs of the follower and leader crazyflies are attached below showing the effectiveness of tracking and hence the successful lifting of the object by the 2 crazyflies.

![Crazyflie_Graphs](https://github.com/Eccelric/Crazyflie_Object_Transport/assets/87143120/6f9e7f56-5c90-4580-a5c4-897620bb7276)

As can be seen from the above graphs the follower very closely tracks the leader with very little time delay, this illustrates the successful formation tracking of the quadrotors. Below is a video of the crazyflies lifting the object 

[![Watch the video](https://github.com/Eccelric/Crazyflie_Object_Transport/assets/87143120/4eee9740-e1f8-4f92-a834-a86581d65a59)](https://youtu.be/w7L2u_7muU4)
