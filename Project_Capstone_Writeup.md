# Udacity Self-Driving Car - Capstone Project

This is the project resources the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## 0) Issues Caused by the tl_detector
* I've had many issues implementing the tl_detector node. After 3-4 days, I've decided to submit the project with tl_detector working faulty.
* I've search the Peer Chat and also asked to a mentor for the issue. You may find the issue recorded in 'traffic_light_stateDelay' video. The Rostopic publishing the light.states having a delay after the simulator is shut down while other topics (example: vehicle_pos) does not create the same error.
* I'll describe the System Architecture as it is needed to be but implementation part will not consider the tl_detector node.

## 1) Introduction

* This is the final capstone project in Udacity’s Self-Driving Car (SDC) Nanodegree.
* I've completed this project as a single Udacity SDC Nanodegree student.
* In this project, I have written a code using the ROS backbone for every module. I've tested the developed code on the simulated car.
* Due to Traffic Light Detection node working sketchy, I've tested the code on both online workspace and the local machine setup with the Udacity's provided VM image and the simulator.
* The capstone project brings together several modules covered by the Nanodegree Program: Perception, Motion Planning, Control, etc.

## 2) System Architecture

* The system consists of three key modules: `Perception`, `Planning`, and `Control`.
* Each module consists the ROS nodes to publish the planner and control inputs while subscribing to the waypoints and the traffic light states.
* (Ideally) 'Perception module' brings the info to the 'Planning module'
* 'Planning module' is responsible from setting the waypoints and target velocities while using the info coming from the 'Perception module'
* `The Control module` takes the waypoints and executes the target velocities on the car’s controller.  A PID controller, a similar controller was also covered during the course, is utilized to control the vehicle velocity.
* The control outputs are passed to the drive-by-wire (DBW) module. DBW publishes the ROS messages to the vehicle in the simulation environment.
* Some of the described functions in the System Architecture, such as the PID controller, is coming from the Udacity provided code.
* After all, the vehicle controls itself within the simulation environment autonomously.


## 3) Implementation

### 1. Perception module

* In this project, the Perception module has two ROS nodes: Obstacle Detector and Traffic Light Detector.
* The Obstacle Detector is not required by the project, the environment in the simulation has no obstacles.
* The discussion below is for the Traffic Light Detector node.

#### 1-1. Traffic light detector

This project is a single member effort and due to struggles in simulation environment, I've finally switched to getting the traffic_light.state (which was intented as a ground truth for NN training or image based applications)

- subscribed topics:
	- `/base_waypoints`: provides a complete list of waypoints on the course
	- `/current_pose` : provides the car’s current position on the course
	- `/vehicle/traffic_lights`: provides the Traffic Light Info.

- published topics:
	- `/traffic_waypoint`: 

- node files:
	- `tl_detector/tl_detector.py`: node file
	
#### 1-2. Obstacle Detector

- It is not implemented.

### 2. Planning module

* This project implements the `Waypoint Updater` node. 
* The purpose of the Waypoint Updater node is to update the target velocities of final waypoints, based on the info coming from the perception module.
* Intented functionality of the planning module: if closest traffic light state = 0(for red light), apply brakes and stop. if the light states message is coming 2, it is for green light and throttle is applied.

#### 2-1. Waypoint Updater

- subscribed topics:
	- `/base_waypoints` : containing all the waypoints in front of, and behind, the vehicle
	- `/obstacle_waypoints` : the waypoints that are obstacles (not used). This information comes from the Perception module (commented out in the submitted code)
	- `/traffic_waypoint` : the waypoint of the traffic light and the state of the traffic light
	- `/current_pose` : the current position of the vehicle, as obtained by the simulator.
	
- published topics:
	- `/final_waypoints` : this is a list of waypoints ahead of the car and the target velocities for those waypoints.
	
- node files:
	- `waypoint_updater/waypoint_updater.py`: node file

### 3. Control module

* The capstone project implements the `Drive-by-Wire (DBW)` node.
* The `DBW` node commands the car or the simulator by providing it with appropriate commands to control the `throttle`/`brake` (Longitudinal controls) and `steering angle`(Lateral Controls).

* The DBW node instantiates two controllers (longitudinal contoller and lateral contoller)
* Longitudinal contoller 
	* takes as input the target speed (as described in the /twist_cmd), the current speed and time delta.
	* control throttle and brake using PID contol algorithm.	
* Lateral contoller
	* takes a target yaw rate, the current speed, current position, the following waypoints.
	* calculates the required steering angle also considering the maximum limit for the steering.
	
#### 3-1. Drive-by-Wire (DBW)

- Parameters acquired:
Vehicle_mass, fuel_capacity,brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle

- subscribed topics:
	- /vehicle/dbw_enabled : a flag to show if the car is under DBW control or Driver control
	- /twist_cmd : contains commands for proposed linear and angular velocities
	- /current_velocity : contains current velocity of the car

- published topics:
	- `/vehicle/throttle_cmd` : the throttle value
	- `/vehicle/steer_cmd` : the steering angle value
	- `/vehicle/brake_cmd` : the brake value

- node files:
	- `twist_controller/dbw_node.py`

## 4) Project Simulation Result

The project simulation results is included in the "Capstone_Project.mp4" file submitted with the project workspace.
In both files I've submitted, you may see the traffic light state messages getting delayed.

## 5) Conclusion:

I am feeling sorry that I was not able to properly test the tl_detector node. I've also spent too much time on that node. Also, due to the COVID-19 situations, I am getting hard time to concentrate...

Other than that, I am really happy with the progress that I've made during this SDC programme.

Thanks for your understanding.
