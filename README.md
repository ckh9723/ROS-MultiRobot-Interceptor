# ROS-MultiRobot-Interceptor  
A ROS program to simulate two robots(interceptor & criminal) doing chase-and-run through Gazebo and RViZ.   
  
  
## Files  
turtlebot3_world_multi.launch: Launch Gazebo simulation with two robots  
turtlebot3_navigation_multi.launch: Launch the map server, amcl & move base of interceptor & criminal and RViZ  
turtlebot3_navigation_multi.rviz: RViZ file to integrate two robots in RViZ  
  
  
amcl_tb3_0:  launch amcl with specified configs (interceptor)  
amcl_tb3_1: launch amcl with specified configs (criminal)  
move_base_tb3_0: launch move base with specified configs (interceptor)  
move_base_tb3_1: launch move base with specified configs (criminal)  
  
  
interceptor.py: Python code to start the chase-and-run program. When the criminal is moving towards its goal, the interceptor will attempt to intercept it.
  
