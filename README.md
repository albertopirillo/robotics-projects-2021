This repository contains the lab projects for the Robotics course of AA. 2020-2021 at Politecnico di Milano.

Both the projects are implemented in ROS, an open-source robotics middleware suite.

# First project
## SCOUT 2.0: Odometry with ROS
Estimate linear and angular velocity from motors speed and then compute odometry, using the C++ language.

**1.** Compute odometry using skid steering (approx) kinematics
   - using Euler and Runge-Kutta integration
   - ROS parameter specifies initial pose
    
**2.** Use dynamic reconfigure to select between integration method

**3.** Write 2 services to reset the odometry to (0,0,0) or to a pose (x,y,θ)

**4.** Publish a custom message with odometry value and type of integration

# Second project
## Mapping and localization with ROS
Calibrate sensors, create a map and perform localization

**1.** Calibrate the static TF between the odometry and the laser sensor

**2.** Create the map with GMapping, using the provided odometry 

**3.** IMU Tools can be used to preprocess IMU data

**4.** Perform localization with AMCL and Robot Localizaiton using the map

## Software
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic 1.0](http://wiki.ros.org/melodic)
- [Visual Studio Code](https://code.visualstudio.com/)
- [CLion 2020.3.4](https://www.jetbrains.com/clion/)

## License & Copyright
Licensed under [MIT License](LICENSE)

## Sensors
- [SICK LMS100 LIDAR](https://www.sick.com/it/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms100-10000/p/p109841)
- [OptiTrack Markers](https://optitrack.com/applications/robotics/)
- [Intel T265 Odom Camera](https://www.intelrealsense.com/tracking-camera-t265/)
- [Pixhawk mini IMU](http://www.holybro.com/product/pixhawk-mini/) 

## About the robot
- [Website](https://www.agilex.ai/index/product/id/2?lang=en-us)
- [User manual](https://www.generationrobots.com/media/agilex/SCOUT2.0_UserManual_v2.0_EN.pdf)

![alt text](https://www.agilex.ai/upload/blocks/6_imgen.png?s=0)
