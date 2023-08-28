/**
 * ROS Simulation Online Course
 * Min Htet Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/
// Check Section (1.2) for more details
// Publishe:
	//	odom_path (nav_msg/Path)
	//	slam_path (nav_msg/Path)
	//  slam_landmark_data
// Subscribe:
	// joint_states ( angle of the left and right wheels )
	// landmark_data ( landmarks position )


#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>
#include <algorithm>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>

#include "rom_ekf_robot/ukf_slam.h"
#include "rom_ekf_robot/MapData.h"
#include "rom_ekf_robot/transform2_3.h"
#include "rom_ekf_robot/differential_drive.h"

// constexpr double ACCEL_STD = 1.0;
// constexpr double GYRO_STD = 0.01/180.0 * M_PI;
// constexpr double INIT_VEL_STD = 10.0;
// constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
// constexpr double GPS_POS_STD = 1.0;
// constexpr double LIDAR_RANGE_STD = 100.0;
// constexpr double LIDAR_THETA_STD = 2.0;

constexpr double ACCEL_STD = 0.001;
constexpr double GYRO_STD = 1e-6;//0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 0.0020;
constexpr double INIT_PSI_STD = 1e-8;//45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 1e-8;
constexpr double LIDAR_RANGE_STD = 1e6;//100.0;
constexpr double LIDAR_THETA_STD = 1e6;//2.0;





Ukf_Slam::Ukf_Slam(const ros::NodeHandle &nh, int num_landmarks, double radius, double separation,const std::string map_frame_,const std::string odom_frame_,const std::string base_frame_,const std::string right_wheel_joint_, const std::string left_wheel_joint_): 
																				 nh_(nh),
																				 num_landmarks_(num_landmarks),
																				 wheel_radius_(radius),
																				 wheel_separation_(separation),
																				 odom_frame_id_(odom_frame_),
																				 base_frame_id_(base_frame_),
																				 map_frame_id_(map_frame_),
																				 left_wheel_joint_(left_wheel_joint_),
																				 right_wheel_joint_(right_wheel_joint_),
																				 frequency_(100)
																				 

{
	landmark_sub_ = nh_.subscribe("landmark_data", 1, &Ukf_Slam::landmark_Callback, this);

	joint_sub_ = nh_.subscribe("joint_states", 1, &Ukf_Slam::joint_Callback, this);

  nav_sub_ = nh_.subscribe("noisy_odom", 1, &Ukf_Slam::nav_Callback, this);



	int findJointIndex(std::vector<std::string> joints, std::string target);
	void PredictionStep(tf2d::Twist2D tw);
	Eigen::VectorXd getStateNoise();

	double sampleNormalDistribution();

	void Cordata_associationrection_w_MeasurementModel(rom_ekf_robot::MapData map_data);

	int data_association(double x, double y);

  double mahalonbis_distance(double data_x, double data_y, int id);

  Eigen::MatrixXd getHMatrix(double x, double y,int id);

	Eigen::Vector2d sensorModel(double x, double y, Eigen::VectorXd noise);
  Eigen::Vector2d sensorModel_new(double x, double y);

	Eigen::VectorXd getMeasurementNoise();

  Eigen::VectorXd vehicleProcessModel(Eigen::VectorXd aug_state, double wz, double vx, double dt);


	//std::vector<double> getRobotState();
  //std::vector<double> setRobotState(const Eigen::VectorXd& state);

  Eigen::VectorXd getRobotState();
  void setRobotState(const Eigen::VectorXd& state);

  Eigen::MatrixXd getCo();
  void setCo(const Eigen::MatrixXd& cov);

	std::vector<geometry_msgs::Point> getLandmarkStates();


  //  ****************************** UKF Step *****************************************

  //std::vector<Eigen::VectorXd> generateSigmaPoints(Eigen::VectorXd state, Eigen::MatrixXd cov);
  //std::vector<double> generateSigmaWeights(unsigned int numStates);





	slam_path_pub = nh_.advertise<nav_msgs::Path>("slam_path", 10);
	odom_path_pub = nh_.advertise<nav_msgs::Path>("odom_path", 10);
	slam_landmark_data_pub = nh_.advertise<rom_ekf_robot::MapData>("slam_landmark_data", 10);


	 //tf2d::DiffDrive bott(pos, wheel_separation_, wheel_radius_);
  tf2d::Kinematics bott(pos, wheel_separation_, wheel_radius_);
	 bot = bott;




	ros::Rate r(frequency_);

	nav_msgs::Odometry odom;

	geometry_msgs::TransformStamped T_odom_to_base;


	tf2_ros::TransformBroadcaster T_mobile_odom;

	// ***************************  SLAM initialization ********************************************

	// Motion model noise Rt(3x3) matrix
	motion_model_noise << 1e-5, 0,
						            0, 1e-5;

    // Sensor model noise Qt(2x2) matrix
	sensor_model_noise << 1e-3, 0,
						          0, 1e-3;

  // motion_model_noise << 1.0,  0,
  //                         0,1.0;

  // sensor_model_noise << 9.0, 0,
  //                       0, 0.004;

	//ROS_INFO_STREAM(" motions is " << motion_model_noise(0,0));

	state_space_mu_ = 4;  // x, y, theta, velocity, 

	total_landmarks_ = num_landmarks_;

	prev_state_.resize(state_space_mu_);  // 
	prev_state_.setZero();

  state_.resize(3);
  state_.setZero();

	landmark_history_.resize(28, 5); // (x + 2n, 5)
	landmark_history_.setZero();

	sigma_bar.resize(state_space_mu_, state_space_mu_); 
	sigma_bar.setZero();

	sigma.resize(state_space_mu_, state_space_mu_);
  sigma(0,0) = GPS_POS_STD*GPS_POS_STD;
  sigma(1,1) = GPS_POS_STD*GPS_POS_STD;
  sigma(2.2) = INIT_PSI_STD*INIT_PSI_STD;
  sigma(3,3) = INIT_VEL_STD*INIT_PSI_STD;



	// sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8; // Uncertainty of diff drive 
	// sigma.topRightCorner(3, 2 * num_landmarks_).setZero();  // Uncertainty between landmarks and robot
	// sigma.bottomLeftCorner(2*num_landmarks_, 3).setZero();  // Uncertainty between robot and landmarks
	// sigma.bottomRightCorner(2*num_landmarks_, 2*num_landmarks_) = Eigen::MatrixXd::Identity(2*num_landmarks_, 2*num_landmarks_) * 1e6; // Uncertainty  of landmarks

	// ***************************** End ***********************************************************

	//tf2d::DiffDrive init_robot(tf2d::Pose2D(0, 0, 0), wheel_separation_, wheel_radius_);
	tf2d::Kinematics init_robot(tf2d::Pose2D(0, 0, 0), wheel_separation_, wheel_radius_);

  ekf_bot = init_robot;
  //bot = init_robot;

	//tf2d::WheelVelocities ekf_cmd;


  std::array<double,2> ekf_cmd;
  std::array<double,2> ekf_twist;

	
	slam_pose2d.th = 0;
	slam_pose2d.x = 0;
	slam_pose2d.y = 0;

	radii.assign(radii.size(), 0.01);


}

// ************************************************* Implement UKF *******************************************************************************************

Eigen::VectorXd Ukf_Slam::getRobotState()
{
  //return {prev_state_(0), prev_state_(1), prev_state_(2)};
  return prev_state_;
}

void Ukf_Slam::setRobotState(const Eigen::VectorXd& state)
{
  prev_state_ = state;
}

Eigen::MatrixXd Ukf_Slam::getCo()
{
  return sigma;
}

void Ukf_Slam::setCo(const Eigen::MatrixXd& cov)
{
  sigma = cov;
}


  void setCo(const Eigen::MatrixXd& cov);

double wrapAngle(double angle)
{
  angle = fmod(angle, (2.0*M_PI));
  if (angle <= -M_PI){angle += (2.0*M_PI);}
  else if (angle > M_PI){angle -= (2.0*M_PI);}
  return angle;
}


Eigen::VectorXd normaliseState(Eigen::VectorXd state)
{
  state(2) = wrapAngle(state(2));
  //state[0](0) = wrapAngle(state[0](0));

  return state;
}

Eigen::VectorXd normaliseLidarMeasurement(Eigen::VectorXd meas)
{
  meas(1) = wrapAngle(meas(1));
  return meas;
}




std::vector<Eigen::VectorXd> generateSigmaPoints(Eigen::VectorXd state, Eigen::MatrixXd cov)  //x_aug, P_aug 
{

  std::vector<Eigen::VectorXd> sigmaPoints;

  int numStates = state.size(); 
  double lambda = 3 - numStates;
  Eigen::MatrixXd sqrtCov = cov.llt().matrixL();
  sigmaPoints.push_back(state);
  
  for(int iState = 0; iState < numStates; iState++)
  {
    sigmaPoints.push_back(state + sqrt(lambda + numStates) * sqrtCov.col(iState));
    sigmaPoints.push_back(state - sqrt(lambda + numStates) * sqrtCov.col(iState));
  }
  return sigmaPoints;

}

std::vector<double> generateSigmaWeights(unsigned int numStates)
{
  std::vector<double> weights;

  double lambda = 3.0 - numStates;
  double w0 = lambda / (lambda + numStates);
  double wi = 0.5 / (numStates + lambda);
  weights.push_back(w0);
  for(int i = 0; i < 2*numStates; ++i)
  {
    weights.push_back(wi);
  }

  return weights;


}

Eigen::VectorXd Ukf_Slam::vehicleProcessModel(Eigen::VectorXd aug_state, double wz, double vx, double dt)
{
  Eigen::VectorXd new_state = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd getState = getRobotState();
  double th = prev_state_(2);  //getState(2);

  double x = aug_state(0);
  double y   = aug_state(1);
  double psi   = aug_state(2);
  double V = aug_state(3);
  double psi_dot_noise = aug_state(4);
  double accel_noise = aug_state(5);

  double x_new_1 = x + dt * vx * std::cos(th);
  double y_new_1 = y + dt * vx * std::sin(th);
  double psi_new_1 = th + dt * (wz * psi_dot_noise);
  double V_new_1 = V + dt * accel_noise;
  psi_new_1 = tf2d::normalize_angle(psi_new_1);

  //new_state << x_new_1, y_new_1, psi_new_1, V_new_1;

  //return new_state;

  
  //double th = prev_state_(2);
  double psi_new;
  double x_new;
  double y_new;
  double V_new;
  if(tf2d::almost_equal(wz, 0.0, 1e-5))
  {
    // update(0) = 0;
    // update(1) = tw.vx * std::cos(th);
    // update(2) = tw.vx * std::sin(th);
    psi_new = th;
    x_new = x +  vx * std::cos(th);
    y_new = y +  vx * std::sin(th);
    V_new = vx + accel_noise;
  }
  else
  {
    double vel_ratio = static_cast<double> (vx)/wz;

    // update(0) = tw.wz;
    // update(1) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
    // update(2) = vel_ratio * std::cos(th) - vel_ratio * std::cos(th + tw.wz);
    psi_new = th + (wz);//psi + dt * (wz * psi_dot_noise);//* psi_dot_noise;
    x_new = x + (-vel_ratio * std::sin(th) + vel_ratio * std::sin(th + wz));
    y_new = y + (vel_ratio * std::cos(th) - vel_ratio * std::cos(th + wz));
    V_new =vx + accel_noise;
  }

  //prev_state_(0) = x + x_new;
  //prev_state_(1) = y + y_new;
  //prev_state_(2) = psi + psi_new;
  psi_new = tf2d::normalize_angle(psi_new);

  new_state << x_new, y_new, psi_new, vx;


  return new_state;

}

Eigen::VectorXd lidarMeasurementModel(Eigen::VectorXd aug_state, double lidar_x, double lidar_y)
{
  Eigen::VectorXd z_hat = Eigen::VectorXd::Zero(2);

  double x = aug_state(0);
  double y = aug_state(1);
  double psi = aug_state(2);
  double range_noise = aug_state(4);
  double theta_noise = aug_state(5);

  double delta_x = lidar_x - x;
  double delta_y = lidar_y - y;
  double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y) + range_noise;
  double zhat_theta = atan2(delta_y, delta_x) - aug_state[2] + theta_noise;
  //zhat_theta = tf2d::normalize_angle(zhat_theta);

  //z_hat << zhat_range, zhat_theta;
  z_hat(0) = zhat_range;
  z_hat(1) = zhat_theta;

  return z_hat;
}


// ************************************************** END ****************************************************************************************************














    // Check if the estimated feature is near an existing feature
	  // if not, add it to the map
    // (xa−xb)TΣ−1(xa−xb)




static std::mt19937 & get_random()
  {
      static std::random_device rd{};
      static std::mt19937 mt{rd()};

      return mt;
  }


double Ukf_Slam::sampleNormalDistribution()
{
	std::normal_distribution<> d(0, 1);
	return d(get_random());
}


Eigen::Vector2d Ukf_Slam::sensorModel_new(double x, double y)
{
  Eigen::Vector2d output;
  
    // del_x and dely are the distances between robot and landmark (position of robot and landmarks)
    double del_x = x - prev_state_(0);
    double del_y = y - prev_state_(1);

    // z_expected << output(0), output(1);

    //output = cart2polar(del_x, del_y);  
    output(0) = std::sqrt(del_x*del_x + del_y*del_y);
    output(1) = std::atan2(del_y,del_x);
    // output(1) = (the angle between the coordinate system where the robot currently is to the landmarks) - (current heading angle)
    //output += noise;
    output(1) -= prev_state_(2);  

    output(1) = tf2d::normalize_angle(output(1));

    if (output(1) > tf2d::PI || output(1) < -tf2d::PI)
    {
      std::cout << "SENSOR MODEL FAILED TO PROPERLY NORMALIZE ANGLE \n\n";
    }

    return output;

}

Eigen::MatrixXd Ukf_Slam::getHMatrix(double x, double y, int id)
{
  Eigen::MatrixXd Hi = Eigen::MatrixXd::Zero(2, state_space_mu_);
  //Eigen::MatrixXd<double, 2, 5> low_H;

      double delta_x = x - prev_state_(0);
    double delta_y   = y - prev_state_(1);

        double q = std::pow(x - prev_state_(0), 2)  + std::pow(y - prev_state_(1), 2);
    Hi << -delta_x/q,-delta_y/q,0,0,delta_y/q/q,-delta_x/q/q,-1,0;

    //Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, state_space_mu_);

    return Hi;



}


// This is the basic idea for unknow data association

// for observation vector in observation matrix z do
// 		Calculate Mahalanobis distance to all known landmarks
// 				if Mahalanobis distance bigger than Mahalanobis distance threshold then
// 							Compute landmark position using Xˆt and observation vector
// 							Add landmark coordinates to Xˆt and Pˆt
// end

// Continue the Correction Step





// *********************************************** EKF Prediction *******************************************************************************
// void Ukf_Slam::PredictionStep(tf2d::Twist2D tw)
// {
// 	//Eigen::Vector3d noise = Ukf_Slam::getStateNoise();
// 	Eigen::Vector3d update;

// 	double th = prev_state_(0);

// 	if(tf2d::almost_equal(tw.wz, 0.0, 1e-5))
// 	{
// 		update(0) = 0;
// 		update(1) = tw.vx * std::cos(th);
// 		update(2) = tw.vx * std::sin(th);
// 	}
// 	else
// 	{
// 		double vel_ratio = static_cast<double>(tw.vx)/tw.wz;

// 		update(0) = tw.wz;
// 		update(1) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
// 		update(2) = vel_ratio * std::cos(th) - vel_ratio * std::cos(th + tw.wz);
// 	}

// 	// Update Prediction mu

// 	prev_state_(0) += update(0);
// 	prev_state_(1) += update(1);
// 	prev_state_(2) += update(2);

// 	prev_state_(0) = tf2d::normalize_angle(prev_state_(0));


// 	  // set Gt to (27x27) matrix, changes are only on topleft 3x3 sub matrix for x, y and theta based on vx and wx
//     // Jacobian of the motion w.r.t theta, x, y of the robot state
//     Eigen::MatrixXd Gt = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_); 


//     if (tf2d::almost_equal(tw.wz,0.0, 1e-5))
//     {
//       Gt(0, 0) = 0;                                 
//       Gt(1, 0) = -tw.vx * std::sin(prev_state_(0)); 
//       Gt(2, 0) = tw.vx * std::cos(prev_state_(0));  
//     }
//     else
//     {
//       Gt(0, 0) = 0;
//       Gt(1, 0) = -(tw.vx / tw.wz) * std::cos(prev_state_(0)) + (tw.vx / tw.wz) * std::cos(prev_state_(0) + tw.wz);
//       Gt(2, 0) = -(tw.vx / tw.wz) * std::sin(prev_state_(0)) + (tw.vx / tw.wz) * std::sin(prev_state_(0) + tw.wz);
//     }


//     // Update for new x , y and theta and then assign all (2n x 2n) sub matrix to identity
//     Gt += Eigen::MatrixXd::Identity(state_space_mu_, state_space_mu_);

//     Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_);


//     Rt.block(0, 0, 3, 3) = motion_model_noise;

//     // Qbar(R) is the additional uncertainty through motion command
//     sigma_bar.setZero();
//     ROS_INFO("I m predict");

//     sigma_bar = Gt * sigma * Gt.transpose() + Rt; 



// }
// *********************************************************** END *************************************************************************************

// ************************************************* UKF Prediction ************************************************************************************


// *************************************************  END   ********************************************************************************************





int Ukf_Slam::findJointIndex(std::vector<std::string> joints, std::string target)
{
	std::vector<std::string>::iterator find_joint;
	find_joint = std::find(joints.begin(), joints.end(), target);

	return std::distance(joints.begin(), find_joint);
}

void Ukf_Slam::landmark_Callback(const rom_ekf_robot::MapData::ConstPtr &landmark_msg)
{
	current_landmark = *landmark_msg;

  got_landmark_data = 1;
 
}


void Ukf_Slam::nav_Callback(const nav_msgs::Odometry::ConstPtr &nav_msg)
{
 // double x = nav_msg->pose.pose.position.x;
 // double y = nav_msg->pose.pose.position.y;
  // Eigen::VectorXd state = getRobotState();
  // Eigen::MatrixXd cov   = getCo();
  // state(2) = tf2d::normalize_angle(state(2));
  const geometry_msgs::Pose& pose = nav_msg->pose.pose;

  const geometry_msgs::Point& position = pose.position;

  // double a=position.x;
  // double b=position.y;
  a = position.x;
  b = position.y;

  nav_status = true;

  // ROS_INFO_STREAM("x is "<<a);
  // ROS_INFO_STREAM("y is "<<b);



  // Eigen::VectorXd z = Eigen::Vector2d::Zero();
  // Eigen::MatrixXd H = Eigen::MatrixXd(2,4);
  // Eigen::MatrixXd R = Eigen::Matrix2d::Zero();
  // z(0) = a;
  // z(1) = b;
  // H << 1, 0, 0, 0, 
  //      0, 1, 0, 0; 

  // R(0,0) = GPS_POS_STD*GPS_POS_STD;
  // R(1,1) = GPS_POS_STD*GPS_POS_STD;

  // // ROS_INFO_STREAM("state_0"<<prev_state_(0));
  // // ROS_INFO_STREAM("state_1"<<prev_state_(1));
  // // ROS_INFO_STREAM("state_2"<<prev_state_(2));
  // // ROS_INFO_STREAM("state_3"<<prev_state_(3));


  // Eigen::VectorXd z_hat_nav = H * state;
  // ROS_INFO_STREAM("z_hat_nav_0 "<<z_hat_nav(0));
  // ROS_INFO_STREAM("z_hat_nav_1 "<<z_hat_nav(1));


  // Eigen::VectorXd y_nav = z - z_hat_nav;
  // ROS_INFO_STREAM("y_nav_0 "<<y_nav(0));
  // ROS_INFO_STREAM("y_nav_1 "<<y_nav(1));

  // Eigen::MatrixXd S = H * cov * H.transpose() + R;
  // Eigen::MatrixXd K = cov * H.transpose()* S.inverse();

  // state = state + K * y_nav;
  // cov = (Eigen::Matrix4d::Identity() - K*H) * cov;

  // state(2) = tf2d::normalize_angle(state(2));

  // setRobotState(state);
  // setCo(cov);

  // ROS_INFO_STREAM("result x "<<state(0));
  // ROS_INFO_STREAM("result y "<<state(1));
  // ROS_INFO_STREAM("result z "<<state(2));





}


void Ukf_Slam::joint_Callback(const sensor_msgs::JointState::ConstPtr &joint_msg)
{
	current_joint = *joint_msg;
	got_odom_data = 1;

	std::vector<std::string> joints;
	int left_wheel, right_wheel;



	if(got_odom_data == 1)
	{  setCo(sigma);
    setRobotState(prev_state_);
		joints = current_joint.name;
		left_wheel = findJointIndex(joints, left_wheel_joint_);
		right_wheel = findJointIndex(joints, right_wheel_joint_);

   // return the e_ticks difference 
    cmd =(bot.updateOdom(current_joint.position[left_wheel], current_joint.position[right_wheel]));

    cur_right_e_ticks = cmd[1];
    cur_left_e_ticks  = cmd[0];


    twist = bot.wheels_to_twist(cur_right_e_ticks, cur_left_e_ticks);
    tw.vx = twist[1];
    tw.vy = 0.0;
    tw.wz = twist[0];

    
		pos = bot.pose_;  // pose()
 		q.setRPY(0, 0, pos.th);
		q_geometry = tf2::toMsg(q);

		odom_point.header.frame_id = map_frame_id_;
		odom_point.header.stamp = ros::Time::now();

		odom_point.pose.position.x = pos.x;
		odom_point.pose.position.y = pos.y;
		odom_point.pose.position.z = 0;
		odom_point.pose.orientation = q_geometry;

		odom_points.push_back(odom_point);
    if (status == true)
    {
    prev_state_(0)= pos.x;
    prev_state_(1)= pos.y;
    prev_state_(2)= pos.th;
    setRobotState(prev_state_);
    status = false;


   }

		odom_path_.header.frame_id = map_frame_id_;
		odom_path_.header.stamp = ros::Time::now();

		odom_path_.poses = odom_points;
		odom_path_pub.publish(odom_path_);
    ROS_INFO("Publishing Odom Path");



		if(got_landmark_data == 1)
		{
      prev_state_ = getRobotState();
      sigma = getCo();
      std::array<double,2> ekf_cmd = ekf_bot.updateOdom(current_joint.position[left_wheel], current_joint.position[right_wheel]);

      ekf_cmd_cur_r_e_ticks = ekf_cmd[1];
      ekf_cmd_cur_l_e_ticks = ekf_cmd[0];
      std::array<double,2> ekf_twist = ekf_bot.wheels_to_twist(ekf_cmd_cur_r_e_ticks, ekf_cmd_cur_l_e_ticks);

      ekf_tw.wz = ekf_twist[0];
      ekf_tw.vx = ekf_twist[1];
      ekf_tw.vy = 0.0;


      //ROS_INFO("Before predict %d and %d", ekf_tw.wz, ekf_tw.vx);

			// **************************************  Prediction Step for EKF ***********************************************************************
			//Ukf_Slam::PredictionStep(ekf_tw);

      // Generate Q Matrix
        Eigen::MatrixXd Q = Eigen::Matrix2d::Zero();
        Q(0,0) = GYRO_STD*GYRO_STD;
        Q(1,1) = ACCEL_STD*ACCEL_STD;

        // Augment the State Vector with Noise States
        int n_x = prev_state_.size();  // 4
        int n_w = 2;
        int n_aug = n_x + n_w; // 6
        Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug); // 6
        Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug, n_aug); //6x6
        x_aug.head(n_x) = prev_state_;
        P_aug.topLeftCorner(n_x,n_x) = sigma;
        P_aug.bottomRightCorner(n_w,n_w) = Q;

        // Generate Augmented Sigma Points
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug); // 6, 6x6
        std::vector<double> sigma_weights = generateSigmaWeights(n_aug);

        // Predict Augmented Sigma Points
        double wz = ekf_tw.wz;
        double vx = ekf_tw.vx;
        double dt = 1.0;
        std::vector<Eigen::VectorXd> sigma_points_predict;
        for (const auto& sigma_point : sigma_points)
        {
            sigma_points_predict.push_back(vehicleProcessModel(sigma_point, wz, vx, dt));
        }

        // Calculate Mean
        prev_state_ = Eigen::VectorXd::Zero(n_x);
        for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            prev_state_ += sigma_weights[i] * sigma_points_predict[i];
        }
        //prev_state_ = normaliseState(prev_state_);
        prev_state_(0) = tf2d::normalize_angle(prev_state_(0));

        // Calculate Covariance
        sigma = Eigen::MatrixXd::Zero(n_x,n_x);
        for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
        {
            Eigen::VectorXd diff = normaliseState(sigma_points_predict[i] - prev_state_);
            sigma += sigma_weights[i] * diff * diff.transpose();
        }

        setRobotState(prev_state_);
        setCo(sigma);
         ROS_INFO_STREAM("state_0"<<prev_state_(0));
          ROS_INFO_STREAM("state_1"<<prev_state_(1));
        ROS_INFO_STREAM("state_2"<<prev_state_(2));
        ROS_INFO_STREAM("state_3"<<prev_state_(3));


      // ************************************** END ********************************************************************************************

      // ******************************************* UKF Prediction Step ***********************************************************************
 
      

      // ****************************** UKF Update **************************************************************************************************
          if(nav_status == true)
           {
               Eigen::VectorXd state = getRobotState();
               Eigen::MatrixXd cov   = getCo();
                      Eigen::VectorXd z = Eigen::Vector2d::Zero();
                      Eigen::MatrixXd H = Eigen::MatrixXd(2,4);
                      Eigen::MatrixXd R = Eigen::Matrix2d::Zero();
                      z(0) = a;
                      z(1) = b;
                      H << 1, 0, 0, 0, 
                           0, 1, 0, 0; 

                      R(0,0) = GPS_POS_STD*GPS_POS_STD;
                      R(1,1) = GPS_POS_STD*GPS_POS_STD;

                      // ROS_INFO_STREAM("state_0"<<prev_state_(0));
                      // ROS_INFO_STREAM("state_1"<<prev_state_(1));
                      // ROS_INFO_STREAM("state_2"<<prev_state_(2));
                      // ROS_INFO_STREAM("state_3"<<prev_state_(3));


                      Eigen::VectorXd z_hat_nav = H * state;
                      ROS_INFO_STREAM("z_hat_nav_0 "<<z_hat_nav(0));
                      ROS_INFO_STREAM("z_hat_nav_1 "<<z_hat_nav(1));


                      Eigen::VectorXd y_nav = z - z_hat_nav;
                      ROS_INFO_STREAM("y_nav_0 "<<y_nav(0));
                      ROS_INFO_STREAM("y_nav_1 "<<y_nav(1));

                      Eigen::MatrixXd S = H * cov * H.transpose() + R;
                      Eigen::MatrixXd K = cov * H.transpose()* S.inverse();

                      state = state + K * y_nav;
                      cov = (Eigen::Matrix4d::Identity() - K*H) * cov;

                      state(2) = tf2d::normalize_angle(state(2));

                      setRobotState(state);
                      setCo(cov);

                      ROS_INFO_STREAM("result x "<<state(0));
                      ROS_INFO_STREAM("result y "<<state(1));
                      ROS_INFO_STREAM("result z "<<state(2));
                      nav_status = false;
           }

                int data_size = current_landmark.centers.size();  
      
    landmark_history_.col(4).setZero(); 


  double current_x = 0, current_y = 0;
  auto landmark_index = 0;

  landmark_history_.col(4).setZero();
for (int i =  0; i < data_size; i++){
  prev_state_ = getRobotState();
      sigma = getCo();
      current_x = current_landmark.centers.at(i).x;
      current_y = current_landmark.centers.at(i).y;
      landmark_index = -1;
      

      std::cout << "goint to data association" << "\n";
      //landmark_index = data_association(current_x, current_y);
      std::cout << "continue" << "\n";

      //if(landmark_index >= 0){



  // Generate Measurement Vector
            Eigen::VectorXd z = Eigen::Vector2d::Zero();
            //z << meas.range, meas.theta;
            z(0) = std::sqrt(current_x * current_x + current_y * current_y);
            z(1) = std::atan2(current_y, current_x);

            // Generate Measurement Model Noise Covariance Matrix
            Eigen::MatrixXd R = Eigen::Matrix2d::Zero();
            R(0,0) = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
            R(1,1) = LIDAR_THETA_STD*LIDAR_THETA_STD;

            // Augment the State Vector with Noise States
            int n_x = prev_state_.size();
            int n_v = 2;
            int n_z = 2;
            int n_aug = n_x + n_v;
            Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug);
            Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug, n_aug);
            x_aug.head(n_x) = prev_state_;
            P_aug.topLeftCorner(n_x,n_x) = sigma;
            P_aug.bottomRightCorner(n_v,n_v) = R;

            // Generate Augmented Sigma Points
            std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
            std::vector<double> sigma_weights  = generateSigmaWeights(n_aug);

            // Measurement Model Augmented Sigma Points
            std::vector<Eigen::VectorXd> z_sig;
            for (const auto& sigma_point : sigma_points){z_sig.push_back(lidarMeasurementModel(sigma_point, current_x, current_y));}

            // Calculate Measurement Mean
            Eigen::VectorXd z_mean = Eigen::VectorXd::Zero(n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i){z_mean += sigma_weights[i] * z_sig[i];}

            // Calculate Innovation Covariance
            Eigen::MatrixXd Py = Eigen::MatrixXd::Zero(n_z,n_z);
            for(unsigned int i = 0; i < z_sig.size(); ++i)
            {
                Eigen::VectorXd diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Py += sigma_weights[i] * diff * diff.transpose();
            }

            // Calculate Cross Covariance
            Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(n_x, n_z);
            for(unsigned int i = 0; i < 2*n_x + 1; ++i)
            {
                Eigen::VectorXd x_diff = normaliseState(sigma_points[i].head(n_x) - prev_state_);
                Eigen::VectorXd z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
                Pxy += sigma_weights[i] * x_diff * z_diff.transpose();
            }

            Eigen::MatrixXd K = Pxy*Py.inverse();
            Eigen::VectorXd y = normaliseLidarMeasurement(z - z_mean);
            prev_state_ = prev_state_ + K*y;
            prev_state_(2) = tf2d::normalize_angle(prev_state_(2));
            sigma = sigma - K * Py * K.transpose(); 

            setRobotState(prev_state_);  
           setCo(sigma);




         // }
        }

      slam_pose = Ukf_Slam::getRobotState();
      ROS_INFO("Getting robot state");

      slam_pose2d.x = slam_pose[0];
      slam_pose2d.y = slam_pose[1];
      slam_pose2d.th = slam_pose[2];

      slam_point.header.frame_id = map_frame_id_;
      slam_point.header.stamp = ros::Time::now();

      //slam_point.pose.position.x = slam_pose.at(0);
      //slam_point.pose.position.y = slam_pose.at(1);
      slam_point.pose.position.x = slam_pose[0];
      slam_point.pose.position.y = slam_pose[1];
      slam_point.pose.position.z = 0;

      q.setRPY(0, 0, slam_pose[2]);
      q_geometry = tf2::toMsg(q);

      slam_point.pose.orientation = q_geometry;
      slam_points.push_back(slam_point);

      slam_path_.header.stamp = ros::Time::now();
      slam_path_.header.frame_id = map_frame_id_;
      slam_path_.poses = slam_points;
      slam_path_pub.publish(slam_path_);


			// Correction Step
			//Ukf_Slam::Correction_w_MeasurementModel(current_landmark);
     

			// get robot pose in (th, x, y) 
			// slam_pose = Ukf_Slam::getRobotState();
      // ROS_INFO("Getting robot state");

			// slam_pose2d.x = slam_pose[0];
			// slam_pose2d.y = slam_pose[1];
			// slam_pose2d.th = slam_pose[2];

			// slam_point.header.frame_id = map_frame_id_;
			// slam_point.header.stamp = ros::Time::now();

			// //slam_point.pose.position.x = slam_pose.at(0);
			// //slam_point.pose.position.y = slam_pose.at(1);
      // slam_point.pose.position.x = slam_pose[0];
      // slam_point.pose.position.y = slam_pose[1];
			// slam_point.pose.position.z = 0;

			// q.setRPY(0, 0, slam_pose[2]);
			// q_geometry = tf2::toMsg(q);

			// slam_point.pose.orientation = q_geometry;
			// slam_points.push_back(slam_point);

			// slam_path_.header.stamp = ros::Time::now();
			// slam_path_.header.frame_id = map_frame_id_;
			// slam_path_.poses = slam_points;
			// slam_path_pub.publish(slam_path_);

			//estimate_landmarks.centers = Ukf_Slam::getLandmarkStates();
			estimate_landmarks.radii = std::vector<double>(radii.size(), 0.01);

			//slam_landmark_data_pub.publish(estimate_landmarks);

			//got_landmark_data = 0;


		}

		tf2d::Transform2D T_odom_robot(pos);
		tf2d::Transform2D T_map_robot(slam_pose2d);
		tf2d::Transform2D T_map_odom = T_map_robot * T_odom_robot.inv();

		tf2::Quaternion q;
		q.setRPY(0, 0, T_map_odom.displacementRad().th);


		geometry_msgs::TransformStamped T_map_odom_;

        T_map_odom_.header.stamp = ros::Time::now();
        T_map_odom_.header.frame_id = map_frame_id_;

        T_map_odom_.child_frame_id = odom_frame_id_;

        T_map_odom_.transform.translation.x = T_map_odom.displacement().x;
        T_map_odom_.transform.translation.y = T_map_odom.displacement().y;
        T_map_odom_.transform.translation.z = 0.0;

        T_map_odom_.transform.rotation.x = q.x();
        T_map_odom_.transform.rotation.y = q.y();
        T_map_odom_.transform.rotation.z = q.z();
        T_map_odom_.transform.rotation.w = q.w();

        T_map_odom_broadcaster.sendTransform(T_map_odom_);
        ROS_INFO("Broadcasting TF");
        got_odom_data = 0;
        got_landmark_data = 0;


	}




}





																					
