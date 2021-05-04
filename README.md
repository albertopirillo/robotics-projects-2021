# First Robotics project
## SCOUT 2.0: Odometry with ROS, AA. 2020-2021
Estimate linear and angular velocity from motors speed and then compute odometry, using C/C++ language.

**1.** Compute odometry using skid steering (approx) kinematics
   - using Euler and Runge-Kutta integration
   - ROS parameter specifies initial pose
    
**2.** Use dynamic reconfigure to select between integration method

**3.** Write 2 services to reset the odometry to (0,0,0) or to a pose (x,y,θ)

**4.** Publish a custom message with odometry value and type of integration

## Software
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic 1.0](http://wiki.ros.org/melodic)
- [Visual Studio Code](https://code.visualstudio.com/)

## About the robot
- [Website](https://www.agilex.ai/index/product/id/2?lang=en-us)
- [User manual](https://www.generationrobots.com/media/agilex/SCOUT2.0_UserManual_v2.0_EN.pdf)

![alt text](https://www.agilex.ai/upload/blocks/6_imgen.png?s=0)
