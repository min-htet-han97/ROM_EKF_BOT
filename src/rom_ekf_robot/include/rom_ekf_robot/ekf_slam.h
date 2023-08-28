/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
 * 
 * Min Htet Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/
#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <geometry_msgs/Point.h>
#include "rom_ekf_robot/MapData.h"
#include "rom_ekf_robot/transform2_3.h"
#include "rom_ekf_robot/ekf_slam.h"
#include "rom_ekf_robot/differential_drive.h"

class Ekf_Slam
{
public:
	Ekf_Slam(const ros::NodeHandle &, int num_landmarks, double radius, double separation, const std::string map_frame_, const std::string odom_frame_, const std::string base_frame_, const std::string right_wheel_joint_, const std::string left_wheel_joint_);


	void PredictionStep(tf2d::Twist2D tw);
	void Correction_w_MeasurementModel(rom_ekf_robot::MapData map_data);

	std::vector<double> getRobotState();

	std::vector<geometry_msgs::Point> getLandmarkStates();




private:
	void landmark_Callback(const rom_ekf_robot::MapData::ConstPtr &);
	void joint_Callback(const sensor_msgs::JointState::ConstPtr &);
	void initpose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

    int findJointIndex(std::vector<std::string> joints, std::string target);



	Eigen::VectorXd getStateNoise();

	Eigen::VectorXd getMeasurementNoise();

	Eigen::Vector2d sensorModel(double x, double y, Eigen::VectorXd noise);
	Eigen::Vector2d sensorModel_new(double x, double y);

	Eigen::MatrixXd getHMatrix(int id);

	int data_association(double x, double y);

	double mahalonbis_distance(double data_x, double data_y, int id);

	double sampleNormalDistribution();

	// UKF implementation

	//std::vector<Eigen::VectorXd> generateSigmaPoints(Eigen::VectorXd state, Eigen::MatrixXd cov);
	//std::vector<double> generateSigmaWeights(unsigned int numStates);




	ros::NodeHandle nh_;
	ros::Subscriber landmark_sub_;
	ros::Subscriber joint_sub_;
	ros::Subscriber initpose_sub_;
	ros::Publisher slam_path_pub;
	ros::Publisher odom_path_pub;
	ros::Publisher slam_landmark_data_pub;

	nav_msgs::Path slam_path_;
	nav_msgs::Path odom_path_;
	tf2_ros::TransformBroadcaster T_map_odom_broadcaster;


	double wheel_radius_;
	double wheel_separation_;
	std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string map_frame_id_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    double frequency_;
    int num_landmarks_;

    int total_landmarks_ = 0;
    int created_landmarks_ = 0;

    Eigen::MatrixXd landmark_history_;

    int state_space_mu_ = 0;  // size of state vector
    double deadband_min_ = 100; 
    double deadband_max_ = 500;

    double PI = 3.14159;
    double robot_pose_threshold_ = 0.1;
    double time_threshold_ = 15.0;

    Eigen::MatrixXd sigma, sigma_bar; // Covariance Matrix
    Eigen::VectorXd prev_state_;  // State Vector

    Eigen::Matrix3d motion_model_noise;
    //Eigen::Matrix2d motion_model_noise;

    Eigen::Matrix2d sensor_model_noise;


	sensor_msgs::JointState current_joint;

    //tf2d::DiffDrive bot;
    //tf2d::DiffDrive ekf_bot;

    tf2d::Kinematics bot;
    tf2d::Kinematics ekf_bot;
    rom_ekf_robot::MapData current_landmark;


    // Create the diff drive object
	tf2d::Pose2D pos;
	tf2d::Twist2D tw;
	//tf2d::WheelVelocities cmd;
    
	std::array<double,2> cmd;
	std::array<double,2> twist;

	double cur_right_e_ticks;
	double cur_left_e_ticks;
	//tf2d::DiffDrive bott;
 


	int got_odom_data = 0;
	int got_landmark_data = 0;
	int got_initpose_data = 0;

	tf2::Quaternion q;
	geometry_msgs::Quaternion q_geometry;


	std::vector<double> slam_pose;
	std::vector<double> get_pose;
	tf2d::Pose2D slam_pose2d;


	geometry_msgs::PoseStamped odom_point;
	std::vector<geometry_msgs::PoseStamped> odom_points;


	geometry_msgs::PoseStamped slam_point;
	std::vector<geometry_msgs::PoseStamped> slam_points;

   	rom_ekf_robot::MapData estimate_landmarks;
	std::vector<double> radii;

	double ekf_cmd_cur_r_e_ticks;
    double ekf_cmd_cur_l_e_ticks;

   	tf2d::Twist2D ekf_tw;


   	// UKF implementation

   	//int n_x;
   	//int n_w;
   	//int n_aug;
   	//Eigen::VectorXd x_aug;
   	//Eigen::MatrixXd P_aug;
   	Eigen::VectorXd state_;

   	double init_pose_x;
   	double init_pose_y;
   	double init_a;
   	double init_b;
   	double init_c;
   	double init_d;
   	double init_yaw;

   




};

#endif