Goals:

  I. compute odometry using skid steering (approx) kinematics
    - using Euler and Runge-Kutta integration
    - ROS parameter specifies initial pose
    
  II. use dynamic reconfigure to select between integration method
  
  III. write 2 services to reset the odometry to (0,0) or to a pose(x,y,θ)
  
  IV. publish a custom message with odometry value and type of integration
