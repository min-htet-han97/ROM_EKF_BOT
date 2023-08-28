

Step_1
	roslaunch rom_ekf_robot full_slam.launch

Step_2
	roslaunch rom_ekf_robot costmap.launch
Step_3
	rosrum rom_ekf_robot path_planner
Step_4 
	rosrun rom_ekf_robot go_to_goal
	
	
Reference 
1)	https://nu-msr.github.io/navigation_site/
2)	Modern Robotics: Mechanics, Planning and Control by Frank Chongwoo Park and Kevin M. Lynch
3)	Introduction to the Simultaneous Localization and Mapping Problem (SLAM) by Cyrill Stachniss

