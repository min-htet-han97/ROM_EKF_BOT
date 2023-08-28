
/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
 * ROS Simulation Online Course
 * Min_Htet_Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/
// Check Section (1.2) EKF for more detials
#include <ros/ros.h>
#include "rom_ekf_robot/ekf_slam.h"
#include "rom_ekf_robot/transform2_3.h"
#include "rom_ekf_robot/differential_drive.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf_slam_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	double wheel_radius = 0.033;
	double wheel_separation = 0.17;

	//double wheel_radius = 0.085;
	//double wheel_separation = 0.406;
	int num_landmarks = 12;
	std::string map_frame_ = "map";
	std::string odom_frame_ = "odom";
	std::string base_frame_ = "base_link";
	std::string left_wheel_joint_ = "wheel_left_joint";
	std::string right_wheel_joint_ = "wheel_right_joint";
	//std::string left_wheel_joint_ = "left_wheel_joint";
	//std::string right_wheel_joint_= "right_wheel_joint";

	Ekf_Slam ekf_slam(nh, num_landmarks, wheel_radius, wheel_separation, map_frame_, odom_frame_, base_frame_, right_wheel_joint_, left_wheel_joint_);
	ros::spin();
	return 0;
}