# Nonlinear Motion Control of a Multirotor Slung Load System:  Experimental Results


This repo contains the simulation code for Nonlinear Motion Control of a Multirotor Slung Load System:  Experimental Results in ACC 2024. The corresponding video is at: https://www.youtube.com/watch?v=wyE0HNX4Rf8&t=1s.

The code runs real-world experiments with a multirotor UAV running the [PX4 autopilot firmware](https://px4.io/). The vehicle is available through [Holybro](https://holybro.com/). 

## Important Folders and Files

1. Folder containing the module which implements the quasi-static controller on ROS : [ROS src folder](./src)

2. The corresponding Matlab and Maple file: [MatlabMapleCode](./MatlabMapleCode)

## Usage

1. Using QgroundControl to take off.

2. Enagle quasi-static controller:

```
rosrun offb_control get_states_node
rosrun offb_control offb_control_node 
```

## Debug/Customize Controller

If you run into any problems using the code, please open an [issue](https://help.github.com/en/github/managing-your-work-on-github/creating-an-issue), or you can email one of us:

* Zifei Jiang <zifei3@ualberta.ca>
* Alan F.Lynch <alan.lynch@ualberta.ca>

## Citing

## Acknowledgement
Thanks to the [PX4 team](https://px4.io/) and [PX4 Gazebo Plugin](https://github.com/PX4/PX4-SITL_gazebo) for their open-source autopilot on which this code is based. 
