Neural Control and Online Learning for Speed Adaptation (NCOLA)

## Neural Control and Online Learning for Speed Adaptation of Unmanned Aerial Vehicles 

## Introduction
This project provides a neural proactive control with fast online learning for speed adaptation to deal with the different maximum flying speeds of UAVs. The control approach is developed based on a three-neuron network while the online learning algorithm is derived from input correlation-based (ICO) learning with the predictive and reflexive input information. This proactive neural control technique here relies on one simple input signal from a LiDAR sensor which can be converted to the predictive and reflexive sensory information for the learning algorithm. Such speed adaptation is a fundamental function that can be used as a part of other complex control functions, like obstacle avoidance. This technique is implemented and evaluated on both simulations and real UAV systems. As a result, the UAV can quickly learn within 3-4 trials to proactively adapt its flying speed to brake at a safe distance from the obstacle. This speed adaptation is robust against wind perturbation. We also demonstrate a combination of speed adaptation and obstacle avoidance for automatic navigation, which is an important intelligent function toward inspection and exploration.

<div align="center">
   <img width="50%" height="50%" src="/Fig.git_R2.png">
</div>

***Fig 1:** (a) An overview of the neural control and online learning system. In this scenario, the UAV flies at a given speed toward an obstacle. When it gets close to the obstacle, the learning system will let it learn to gradually adapt its speed and brake at a safe distance from the obstacle. The proposed control and learning system is not restricted to a certain flight path. It can be used to adapt speed to the horizontal plane (x and y directions). (b) The model-free speed adaptation algorithm is based on neural control with ICO learning mechanism and online learning capability (or considered as neural proactive control, insert circle). The reflexive path is the path for generating reactive behavior while the predictive path is the path for performing online learning and generating adaptive behavior. The algorithm acts as a primitive control function for obstacle avoidance. It allows the UAV to fly autonomously (see from right to left) and proactively adapt its flying speed and brake at a safe distance before avoiding an obstacle.*

## Framework
The project is organized by three sub-folders including **Gazebo_Simulation**, **Simulated_Drone**, and **Real_Drone**.
- **Gazebo_Simulation** contains interfacing detail between the gazebo simulation drone platform and software in the loop (SITL) flight simulator.
- **Simulated_Drone** contains code of the neural control and Mavros interface with the software in the loop (SITL) flight simulator.
- **Real_Drone** contains code of the neural control, sensor interface, and Mavros interface with the pixhawk flight controller of the UAV.

If you have any questions. Feel free to contact me at vatsanai.j_s18@vistec.ac.th
