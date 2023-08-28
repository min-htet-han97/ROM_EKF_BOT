
/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
 * 
 * Min Htet Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/
//  Check Section 1.1 ( Equ 5, 6, 7) for more details
#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rom_ekf_robot/transform2_3.h"
#include "ros/ros.h"
#include <array>
#include <iostream>

namespace tf2d{

class Kinematics
{
public:

	double prev_encoder_ticks_right_wheel;
	double prev_encoder_ticks_left_wheel;

	double left_wheel_vel;
	double right_wheel_vel;	

	tf2d::Pose2D pose_;

Kinematics()
{
	pose_.x = 0;
	pose_.y = 0;
	pose_.th = 0;

	tf2d::Pose2D T_l(0, -wheel_separation_/2, 0);
	tf2d::Pose2D T_r(0, wheel_separation_/2, 0);

	setTransform();

}

Kinematics(tf2d::Pose2D pose, double wheel_separation, double wheel_radius)
{
	pose_ = pose;
	wheel_separation_ = wheel_separation;
	wheel_radius_ = wheel_radius;

  tf2d::Pose2D T_l(0, -wheel_separation_/2, 0);
	tf2d::Pose2D T_r(0, wheel_separation_/2, 0);

	setTransform();

}


std::array<double,2> wheels_to_twist(double right_w_vel, double left_w_vel)
{
	double d = 1 / wheel_separation_;
	std::array<double,2> twist;
	twist[0] = wheel_radius_ * (d * right_w_vel - d * left_w_vel); // angular vel
	twist[1] = wheel_radius_ * (0.5 * right_w_vel + 0.5 * left_w_vel); // linear vel along x 

	return twist;

	
}


std::array<double,3> twist_to_wheels(double vx, double vy, double wz)
{


}




std::array<double,2> updateOdom(double left_e_ticks, double right_e_ticks)
{
	std::array<double,2> current_e_ticks;
	std::array<double,2> base_vel;

	current_e_ticks[0] = left_e_ticks - prev_encoder_ticks_left_wheel;   
	current_e_ticks[1] = right_e_ticks - prev_encoder_ticks_right_wheel;

	double right_w_vel = current_e_ticks[1];
	double left_w_vel  = current_e_ticks[0];

	prev_encoder_ticks_right_wheel = right_e_ticks;
	prev_encoder_ticks_left_wheel = left_e_ticks;

	base_vel = wheels_to_twist(right_w_vel, left_w_vel);

	tf2d::Twist2D cmd;
	cmd.vx = base_vel[1];
	cmd.vy = 0.0;
	cmd.wz = base_vel[0];

	T_wheel_base = T_wheel_base.integrateTwist(cmd);

	pose_ = T_wheel_base.displacementRad();

	pose_.th = tf2d::normalize_angle(pose_.th);

	return current_e_ticks;

}

void setTransform()
  {
  	tf2d::Transform2D robot_base(pos);
  	T_wheel_base = robot_base;
  }

 double  getLeft_e_ticks() const
 {
 	return prev_encoder_ticks_left_wheel;
 }

 double  getRight_e_ticks() const
 {
 	return prev_encoder_ticks_right_wheel;
 }


tf2d::Pose2D pose() const;

private:

	tf2d::Pose2D pos;



	tf2d::Transform2D T_wheel_base;

	//void setTransform();

	
	double wheel_radius_;
	double wheel_separation_;
	
};

tf2d::Pose2D Kinematics::pose() const 
{
	return pose_;
}


}


#endif