/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rom_ekf_robot/ekf_slam.h"
#include "rom_ekf_robot/MapData.h"
#include "rom_ekf_robot/transform2_3.h"
#include "rom_ekf_robot/differential_drive.h"





Ekf_Slam::Ekf_Slam(const ros::NodeHandle &nh, int num_landmarks, double radius, double separation,const std::string map_frame_,const std::string odom_frame_,const std::string base_frame_,const std::string right_wheel_joint_, const std::string left_wheel_joint_): 
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
	landmark_sub_ = nh_.subscribe("landmark_data", 1, &Ekf_Slam::landmark_Callback, this);

	joint_sub_ = nh_.subscribe("joint_states", 1, &Ekf_Slam::joint_Callback, this);

  initpose_sub_ = nh_.subscribe("initialpose", 1, &Ekf_Slam::initpose_Callback, this);

	int findJointIndex(std::vector<std::string> joints, std::string target);
	void PredictionStep(tf2d::Twist2D tw);
	Eigen::VectorXd getStateNoise();

	double sampleNormalDistribution();

	void Correction_w_MeasurementModel(rom_ekf_robot::MapData map_data);

	int data_association(double x, double y);

    double mahalonbis_distance(double data_x, double data_y, int id);

   	Eigen::MatrixXd getHMatrix(int id);

	Eigen::Vector2d sensorModel(double x, double y, Eigen::VectorXd noise);
  Eigen::Vector2d sensorModel_new(double x, double y);

	Eigen::VectorXd getMeasurementNoise();

	std::vector<double> getRobotState();

	std::vector<geometry_msgs::Point> getLandmarkStates();

  //  ****************************** UKF Step *****************************************


	slam_path_pub = nh_.advertise<nav_msgs::Path>("slam_path", 10);
	odom_path_pub = nh_.advertise<nav_msgs::Path>("odom_path", 10);


	 //tf2d::DiffDrive bott(pos, wheel_separation_, wheel_radius_);
  tf2d::Kinematics bott(pos, wheel_separation_, wheel_radius_);
	 bot = bott;




	ros::Rate r(frequency_);

	nav_msgs::Odometry odom;

	geometry_msgs::TransformStamped T_odom_to_base;


	tf2_ros::TransformBroadcaster T_mobile_odom;

	// ***************************  SLAM initialization ********************************************

	// Motion model noise Rt(3x3) matrix
	motion_model_noise << 1e-5, 0, 0,
						  0, 1e-5, 0,
						  0, 0, 1e-5;

    // Sensor model noise Qt(2x2) matrix
	sensor_model_noise << 1e-3, 0,
						  0, 1e-3;


	state_space_mu_ = 3 + 2 * num_landmarks_;

	total_landmarks_ = num_landmarks_;

	prev_state_.resize(state_space_mu_);  // 3 + 2n vector
	prev_state_.setZero();

  state_.resize(3);
  state_.setZero();

	landmark_history_.resize(state_space_mu_, 5);
	landmark_history_.setZero();

	sigma_bar.resize(state_space_mu_, state_space_mu_); 
	sigma_bar.setZero();

	sigma.resize(state_space_mu_, state_space_mu_);
	sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8; // Uncertainty of diff drive 
	sigma.topRightCorner(3, 2 * num_landmarks_).setZero();  // Uncertainty between landmarks and robot
	sigma.bottomLeftCorner(2*num_landmarks_, 3).setZero();  // Uncertainty between robot and landmarks
	sigma.bottomRightCorner(2*num_landmarks_, 2*num_landmarks_) = Eigen::MatrixXd::Identity(2*num_landmarks_, 2*num_landmarks_) * 1e6; // Uncertainty  of landmarks

	// ***************************** End ***********************************************************

	//tf2d::DiffDrive init_robot(tf2d::Pose2D(0, 0, 0), wheel_separation_, wheel_radius_);
	tf2d::Kinematics init_robot(tf2d::Pose2D(0, 0, 0), wheel_separation_, wheel_radius_);

  ekf_bot = init_robot;
  

  std::array<double,2> ekf_cmd;
  std::array<double,2> ekf_twist;

	
	slam_pose2d.th = 0;
	slam_pose2d.x = 0;
	slam_pose2d.y = 0;

	radii.assign(radii.size(), 0.01);

  got_initpose_data = 0;


}

// ************************************************* Implement UKF *******************************************************************************************

std::vector<double> Ekf_Slam::getRobotState()
{
  return {prev_state_(0), prev_state_(1), prev_state_(2)};
}

double wrapAngle(double angle)
{
  angle = fmod(angle, (2.0*M_PI));
  if (angle <= -M_PI){angle += (2.0*M_PI);}
  else if (angle > M_PI){angle -= (2.0*M_PI);}
  return angle;
}


Eigen::VectorXd normaliseState(Eigen::VectorXd state)
{
  state(0) = wrapAngle(state(0));

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

  int numStates = state.size();  // 29
  double lambda = 15.0 - numStates;
  Eigen::MatrixXd sqrtCov = cov.llt().matrixL();
  sigmaPoints.push_back(state);
  int iState = 0;
  for(iState = 0; iState < numStates; iState++)
  {
    sigmaPoints.push_back(state + sqrt(lambda + numStates) * sqrtCov.col(iState));
    sigmaPoints.push_back(state - sqrt(lambda + numStates) * sqrtCov.col(iState));
  }
  return sigmaPoints;

}

std::vector<double> generateSigmaWeights(unsigned int numStates)
{
  std::vector<double> weights;

  double lambda = 15.0 - numStates;
  double w0 = lambda / (lambda + numStates);
  double wi = 0.5 / (numStates + lambda);
  weights.push_back(w0);
  for(int i = 0; i < 2*numStates; ++i)
  {
    weights.push_back(wi);
  }

  return weights;


}

Eigen::VectorXd vehicleProcessModel(Eigen::VectorXd aug_state, double wz, double vx, double dt)
{
  Eigen::VectorXd new_state = Eigen::VectorXd::Zero(3);

  double psi = aug_state(0);
  double x   = aug_state(1);
  double y   = aug_state(2);
  double psi_dot_noise = aug_state(27);
  double x_noise = aug_state(28);

  double psi_new;
  double x_new;
  double y_new;
  if(tf2d::almost_equal(wz, 0.0, 1e-5))
  {

    psi_new = 0;
    x_new = x + dt * vx * std::cos(psi);
    y_new = y + dt * vx * std::sin(psi);
  }
  else
  {
    double vel_ratio = static_cast<double> (vx)/wz;

    // update(0) = tw.wz;
    // update(1) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
    // update(2) = vel_ratio * std::cos(th) - vel_ratio * std::cos(th + tw.wz);
    psi_new = (wz + psi) * psi_dot_noise;
    x_new = x + (-vel_ratio * std::sin(psi) + vel_ratio * std::sin(psi + wz));
    y_new = y + (vel_ratio * std::cos(psi) - vel_ratio * std::cos(psi + wz));
  }

  new_state << psi_new, x_new, y_new;


  return new_state;

}

Eigen::VectorXd lidarMeasurementModel(Eigen::VectorXd aug_state, double lidar_x, double lidar_y)
{
  Eigen::VectorXd z_hat = Eigen::VectorXd::Zero(2);

  double psi = aug_state(0);
  double x = aug_state(1);
  double y = aug_state(2);
  double range_noise = aug_state(27);
  double theta_noise = aug_state(28);

  double delta_x = lidar_x - x;
  double delta_y = lidar_y - y;
  double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y) + range_noise;
  double zhat_theta = atan2(delta_y, delta_x) - aug_state[0] + theta_noise;

  z_hat << zhat_range, zhat_theta;

  return z_hat;
}


// ************************************************** END ****************************************************************************************************


Eigen::Vector2d Ekf_Slam::sensorModel(double x, double y, Eigen::VectorXd noise)
{
	Eigen::Vector2d output;
  
    // del_x and dely are the distances between robot and landmark (position of robot and landmarks)
    double del_x = x - prev_state_(1);
    double del_y = y - prev_state_(2);

    // z_expected << output(0), output(1);

    //output = cart2polar(del_x, del_y);  
    output(0) = std::sqrt(del_x*del_x + del_y*del_y);
    output(1) = std::atan2(del_y,del_x);
    // output(1) = (the angle between the coordinate system where the robot currently is to the landmarks) - (current heading angle)
    output += noise;
    output(1) -= prev_state_(0);  

    output(1) = tf2d::normalize_angle(output(1));

    if (output(1) > tf2d::PI || output(1) < -tf2d::PI)
    {
      std::cout << "SENSOR MODEL FAILED TO PROPERLY NORMALIZE ANGLE \n\n";
    }

    return output;

}


Eigen::Vector2d Ekf_Slam::sensorModel_new(double x, double y)
{
  Eigen::Vector2d output;
  
    // del_x and dely are the distances between robot and landmark (position of robot and landmarks)
    double del_x = x - prev_state_(1);
    double del_y = y - prev_state_(2);

    // z_expected << output(0), output(1);

    //output = cart2polar(del_x, del_y);  
    output(0) = std::sqrt(del_x*del_x + del_y*del_y);
    output(1) = std::atan2(del_y,del_x);
    // output(1) = (the angle between the coordinate system where the robot currently is to the landmarks) - (current heading angle)
    //output += noise;
    output(1) -= prev_state_(0);  

    output(1) = tf2d::normalize_angle(output(1));

    if (output(1) > tf2d::PI || output(1) < -tf2d::PI)
    {
      std::cout << "SENSOR MODEL FAILED TO PROPERLY NORMALIZE ANGLE \n\n";
    }

    return output;

}





Eigen::MatrixXd Ekf_Slam::getHMatrix(int id)
{
	Eigen::MatrixXd Hi = Eigen::MatrixXd::Zero(2,state_space_mu_);  
    Eigen::Matrix<double, 2, 5> low_H;  // Jacobian for the Observation  Section ( 1.3 )


    double delta_x = prev_state_(id) - prev_state_(1);
    double delta_y = prev_state_(id + 1) - prev_state_(2);
    double q = std::pow(prev_state_(id) - prev_state_(1), 2)  + std::pow(prev_state_(id + 1) - prev_state_(2), 2);

    low_H << 0, -std::sqrt(q)*delta_x, -std::sqrt(q)*delta_y, std::sqrt(q)*delta_x, std::sqrt(q)*delta_y,  // Only this dimension matter
                -q, delta_y, -delta_x, -delta_y, delta_x;
    low_H = low_H / q;


    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, state_space_mu_);
    F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    F(3, id) = 1;
    F(4, id + 1) = 1;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state_space_mu_);
    H = low_H * F;

    return H;

}

    // Check if the estimated feature is near an existing feature
	  // if not, add it to the map
    // (xa−xb)TΣ−1(xa−xb)

double Ekf_Slam::mahalonbis_distance(double data_x, double data_y, int id)
{
	// get H matrix
    auto Hi = getHMatrix(id);  

    // calculate covarience
    Eigen::MatrixXd psi = Hi * sigma_bar * Hi.transpose() + sensor_model_noise;

    // calculate expected measurement
    Eigen::Vector2d noise;
    noise.setZero();
    Eigen::Vector2d z_hat = sensorModel_new(prev_state_(id), prev_state_(id + 1));

    // get actual measurement
    Eigen::Vector2d z;

    z(0) = std::sqrt(data_x * data_x + data_y * data_y);
    z(1) = std::atan2(data_y, data_x);

    Eigen::Vector2d z_diff = z - z_hat;

    double dist = z_diff.transpose() * psi.inverse() * z_diff;

    return dist;
}


static std::mt19937 & get_random()
  {
      static std::random_device rd{};
      static std::mt19937 mt{rd()};

      return mt;
  }


double Ekf_Slam::sampleNormalDistribution()
{
	std::normal_distribution<> d(0, 1);
	return d(get_random());
}



// This is the basic idea for unknow data association

// for observation vector in observation matrix z do
// 		Calculate Mahalanobis distance to all known landmarks
// 				if Mahalanobis distance bigger than Mahalanobis distance threshold then
// 							Compute landmark position using Xˆt and observation vector
// 							Add landmark coordinates to Xˆt and Pˆt
// end

// Continue the Correction Step



int Ekf_Slam::data_association(double x, double y)
{
	int landmark_index = -1;
    int output_index = -1;


     // Check data point against all created landmarks
    for(int i = 0; i < created_landmarks_; i++)
    {
      landmark_index = 3 + 2*i;

      double dist = mahalonbis_distance(x, y, landmark_index);


      if(dist < deadband_min_) // if less than the deadband, consider this a match, already seen

  
      {
          output_index = landmark_index;

          // Update history info
          landmark_history_(output_index, 1) = prev_state_(1);
          landmark_history_(output_index, 2) = prev_state_(2);
          landmark_history_(output_index, 3) = ros::Time::now().toSec();
          landmark_history_(output_index, 4) = 1;
          //break;
        //}
      }
      else if(dist > deadband_max_) 
      {
        output_index += 0;
      }
    }


    // If the landmark was unmatched, outside the deadband, and if there is room
    // left in the state vector, add it.
    if(output_index == -1 && created_landmarks_ < total_landmarks_)
    {
      // Find next open index
      int j = 0;
      while(output_index < 0 && j < total_landmarks_)
      {
        if(landmark_history_(3 + (2*j), 0) == 0) output_index = 3 + 2*j;  // landmark_history(23,5) ,(4)column is zero
        j++;
      }

      prev_state_(output_index) = x + prev_state_(1);
      prev_state_(output_index+1) = y + prev_state_(2);

      // Update history info
      landmark_history_(output_index, 0) = 1;  // output index start from 3
      landmark_history_(output_index, 1) = prev_state_(1);
      landmark_history_(output_index, 2) = prev_state_(2);
      landmark_history_(output_index, 3) = ros::Time::now().toSec();
      landmark_history_(output_index, 4) = 1;
      created_landmarks_++;
      //ROS_INFO("Adding new landmarks !!!");
    }

    return output_index;



}
// *********************************************** EKF Prediction *******************************************************************************
void Ekf_Slam::PredictionStep(tf2d::Twist2D tw)
{
	Eigen::Vector3d update;

	double th = prev_state_(0);

	if(tf2d::almost_equal(tw.wz, 0.0, 1e-5))
	{
		update(0) = 0;
		update(1) = tw.vx * std::cos(th);
		update(2) = tw.vx * std::sin(th);
	}
	else
	{
		double vel_ratio = static_cast<double>(tw.vx)/tw.wz;

		update(0) = tw.wz;
		update(1) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
		update(2) = vel_ratio * std::cos(th) - vel_ratio * std::cos(th + tw.wz);
	}

	// Update Prediction mu

	prev_state_(0) += update(0);
	prev_state_(1) += update(1);
	prev_state_(2) += update(2);

	prev_state_(0) = tf2d::normalize_angle(prev_state_(0));


	  // set Gt to (27x27) matrix, changes are only on topleft 3x3 sub matrix for x, y and theta based on vx and wx
    // Jacobian of the motion w.r.t theta, x, y of the robot state
    Eigen::MatrixXd Gt = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_); 


    if (tf2d::almost_equal(tw.wz,0.0, 1e-5))
    {
      Gt(0, 0) = 0;                                 
      Gt(1, 0) = -tw.vx * std::sin(prev_state_(0)); 
      Gt(2, 0) = tw.vx * std::cos(prev_state_(0));  
    }
    else
    {
      Gt(0, 0) = 0;
      Gt(1, 0) = -(tw.vx / tw.wz) * std::cos(prev_state_(0)) + (tw.vx / tw.wz) * std::cos(prev_state_(0) + tw.wz);
      Gt(2, 0) = -(tw.vx / tw.wz) * std::sin(prev_state_(0)) + (tw.vx / tw.wz) * std::sin(prev_state_(0) + tw.wz);
    }


    // Update for new x , y and theta and then assign all (2n x 2n) sub matrix to identity
    Gt += Eigen::MatrixXd::Identity(state_space_mu_, state_space_mu_);

    Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_);


    Rt.block(0, 0, 3, 3) = motion_model_noise;

    // Qbar(R) is the additional uncertainty through motion command
    sigma_bar.setZero();
    //("I m predict");

    sigma_bar = Gt * sigma * Gt.transpose() + Rt; 



}
// *********************************************************** END *************************************************************************************

// ************************************************* UKF Prediction ************************************************************************************

// void Ekf_Slam::PredictionStep(tf2d::Twist2D tw)
// {
//       sigma_bar.setZero();

//       int n_x = prev_state_.size();
//       int n_w = 2;
//       int n_aug = n_x + n_w;  // 29
//       Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug); // 29 
//       Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug, n_aug); // 29x29

//       int rows = P_aug.rows();
//       int cols = P_aug.cols();

//       ROS_INFO("%d and %d and %d", rows, cols, n_aug);


//       //Eigen::VectorXd state_space_mu_eigen = Eigen::VectorXd::Map(state_space_mu_.data(), state_space_mu_.size());
//       //assert(state_space_mu_eigen.size() == n_x);
//       // Eigen::VectorXd state_space_mu_eigen(state_space_mu_.size());
//       // for (std::size_t i = 0; i < state_space_mu_.size(); ++i) 
//       // {
//       //       state_space_mu_eigen(i) = state_space_mu_[i];
//       // }

//       // Copies the values from prev_state_ vecto to x_aug vector
//       x_aug.head(n_x) = prev_state_;
//       //x_aug.head(n_x) = state_space_mu_;
      

//       P_aug.topLeftCorner(n_x, n_x) = sigma;  // sigma (27x27)
//       P_aug.bottomRightCorner(n_w, n_w) = motion_model_noise;  //sensor_model_noise; 2x2

//       // Generate Sigma Points
//       std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
//       std::vector<double> sigma_weights = generateSigmaWeights(n_aug); 





//       // Predict Sigma Points
//       std::vector<Eigen::VectorXd> sigma_points_predict;
//       //Eigen::VectorXd sigma_points_predict;
//       double dt = 1.0;
//       double wz = tw.wz;
//       double vx = tw.vx;
//       for(const auto& sigma_point : sigma_points)
//       {
//         sigma_points_predict.push_back(vehicleProcessModel(sigma_point, wz, vx, dt));

//       }

//       // Calculate Mean
//       //state_.resize(prev_state_);

//       for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
//       {
//         state_ += sigma_weights[i] * sigma_points_predict[i];
//       }

//       //prev_state_(0,0) = state_(0,0);
//       //prev_state_(1,0) = state_(1,0);
//       //prev_state_(2,0) = state_(2,0);
//       prev_state_.segment<3>(0) = state_;
//       state_(0) = tf2d::normalize_angle(state_(0));

//       prev_state_(0) = tf2d::normalize_angle(prev_state_(0));
//       //state_space_mu_ = normalizeState(state_space_mu_);
                              

      
//       // Calculate Covariance 
//       //sigma = Eigen::MatrixXd::Zero(3,3);
//       //sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8; // Uncertainty of diff drive 
//       //sigma_bar.setZero();

//       for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
//       {
//         Eigen::VectorXd diff = normaliseState(sigma_points_predict[i] - state_);
//         sigma_bar.topLeftCorner(3,3) = sigma.topLeftCorner(3,3) + ( sigma_weights[i] * diff * diff.transpose() );
//       }



//       //sigma_bar = sigma;
//        ROS_INFO("I m predict");
// }


// *************************************************  END   ********************************************************************************************


void Ekf_Slam::Correction_w_MeasurementModel(rom_ekf_robot::MapData map_data)
{
	  Eigen::Vector2d z_expected = Eigen::Vector2d::Zero();
    Eigen::Vector2d z_actual   = Eigen::Vector2d::Zero();

    Eigen::MatrixXd Ki_kalman_gain = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_);
    Eigen::MatrixXd Ri = Eigen::MatrixXd::Zero(state_space_mu_, state_space_mu_);
    Eigen::MatrixXd Hi_observed_Jacobian = Eigen::MatrixXd::Zero(2, state_space_mu_);

    Eigen::Vector2d noise = Eigen::Vector2d::Zero();

    double current_x = 0, current_y = 0;
    auto landmark_index = 0;

    int data_size = map_data.centers.size();  

    landmark_history_.col(4).setZero(); 

    for(int i = 0; i < data_size; i++)
    {
    	current_x = map_data.centers.at(i).x;
    	current_y = map_data.centers.at(i).y;

    	landmark_index = -1;

    	landmark_index = data_association(current_x, current_y);

    	      // if the data correlates to a landmark process it
      if(landmark_index >= 0) 
      {
        // Compute actual measurment
        z_actual(0) = std::sqrt(current_x * current_x + current_y * current_y);
        z_actual(1) = std::atan2(current_y, current_x);

        // Compute the expected measurment
        //noise = Ekf_Slam::getMeasurementNoise();

        // z_expected << output(0), output(1);  what should measure
        //z_expected = sensorModel(prev_state_(landmark_index), prev_state_(landmark_index + 1), noise);
        z_expected = sensorModel_new(prev_state_(landmark_index), prev_state_(landmark_index + 1));

        // Compute error between actual measurement and expected measurement
        Eigen::Vector2d z_diff = (z_actual - z_expected);
        z_diff(1) = tf2d::normalize_angle(z_diff(1));

        // Assemble H Matrix
        Hi_observed_Jacobian = getHMatrix(landmark_index);

        // Compute the Kalman Gain  
        Ki_kalman_gain = sigma_bar * Hi_observed_Jacobian.transpose() * ((Hi_observed_Jacobian * sigma_bar * Hi_observed_Jacobian.transpose() + sensor_model_noise).inverse());

        // Update the Posterior
        prev_state_ += Ki_kalman_gain * z_diff;
        prev_state_(0) = tf2d::normalize_angle(prev_state_(0));

        // Update the Covarience
        sigma_bar = (Eigen::MatrixXd::Identity(state_space_mu_, state_space_mu_) - Ki_kalman_gain * Hi_observed_Jacobian) * sigma_bar;
        //ROS_INFO("Im here for update");

      }

      // ************************************************** UKF Update **************************************************************************
      // if(landmark_index >= 0) 
      // {
      //   // Compute actual measurment
      //   z_actual(0) = std::sqrt(current_x * current_x + current_y * current_y);
      //   z_actual(1) = std::atan2(current_y, current_x);

      //   // Compute the expected measurment
      //   //noise = Ekf_Slam::getMeasurementNoise();

      //   //int n_x = state_space_mu_;
      //   int n_x = prev_state_.size();
      //   int n_v = 2;
      //   int n_z = 2;
      //   int n_aug = n_x + n_v;
      //   Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug);
      //   Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug, n_aug); // 29 x 29
      //   x_aug.head(n_x) = prev_state_;
      //   P_aug.topLeftCorner(n_x, n_x) = sigma_bar;  
      //   P_aug.bottomRightCorner(n_v, n_v) = sensor_model_noise;

      //   // Generate Augmented Sigma Points
      //   std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
      //   std::vector<double> sigma_weights = generateSigmaWeights(n_aug);

      //   // Measurement Model Augmented Sigma Points
      //   std::vector<Eigen::VectorXd> z_sig;
      //   for (const auto& sigma_point : sigma_points)
      //     {
      //       z_sig.push_back(lidarMeasurementModel(sigma_point, current_x, current_y));
      //     }


      //   // Calculate Measurement Mean
      //   Eigen::VectorXd z_mean = Eigen::VectorXd::Zero(n_z);
      //   for(unsigned int i = 0; i < z_sig.size(); ++i)
      //   {
      //     z_mean += sigma_weights[i] * z_sig[i];
      //   }

      //   // Calculate Innovation Covariance
      //   Eigen::MatrixXd Py = Eigen::MatrixXd::Zero(n_z,n_z);
      //   for(unsigned int i = 0; i < z_sig.size(); ++i)
      //   {
      //           Eigen::VectorXd diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
      //           Py += sigma_weights[i] * diff * diff.transpose();
      //   }


      //   // Calculate Cross Covariance
      //   Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(n_x, n_z);
      //   for(unsigned int i = 0; i < 2*n_x + 1; ++i)
      //   {
      //           Eigen::VectorXd x_diff = normaliseState(sigma_points[i].head(n_x) - prev_state_);
      //           Eigen::VectorXd z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean);
      //           Pxy += sigma_weights[i] * x_diff * z_diff.transpose();
      //   }




      //   Eigen::MatrixXd K = Pxy * Py.inverse();
      //   Eigen::VectorXd y = normaliseLidarMeasurement(z_actual - z_mean);

      //   prev_state_ += K * y;
      //   prev_state_(0) = tf2d::normalize_angle(prev_state_(0));

      //   state_ = prev_state_.head<3>();
      //   get_pose = Ekf_Slam::getRobotState();
      //  get_pose.at(0) = tf2d::normalize_angle(get_pose.at(0));
      //  for (int i = 0; i < 3; ++i)
      //  {
      //    state_(i) = get_pose.at(i);
      //  }



      //  ROS_INFO("Im here for update");



      //   sigma_bar = sigma_bar - K * Py  * K.transpose();
      // }
    }

    sigma = sigma_bar;

    // *************************************** UKF Update END ***********************************************************************************

}



int Ekf_Slam::findJointIndex(std::vector<std::string> joints, std::string target)
{
	std::vector<std::string>::iterator find_joint;
	find_joint = std::find(joints.begin(), joints.end(), target);

	return std::distance(joints.begin(), find_joint);
}

void Ekf_Slam::landmark_Callback(const rom_ekf_robot::MapData::ConstPtr &landmark_msg)
{
	current_landmark = *landmark_msg;
	got_landmark_data = 1;



}

void Ekf_Slam::initpose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initpose_msg)
{
  
  init_pose_x = initpose_msg->pose.pose.position.x;
  init_pose_y = initpose_msg->pose.pose.position.y;

  init_a = initpose_msg->pose.pose.orientation.x;
  init_b = initpose_msg->pose.pose.orientation.y;
  init_c = initpose_msg->pose.pose.orientation.z;
  init_d = initpose_msg->pose.pose.orientation.w;

  tf2::Quaternion quat;
  tf2::fromMsg(initpose_msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);


  init_yaw = yaw;
  //init_yaw = tf2d::normalize_angle(init_yaw);


  ROS_INFO("Receiving !!!");
  //slam_pose2d.x = init_pose_x;
  //slam_pose2d.y = init_pose_y;
  //slam_pose2d.th = init_yaw;

  prev_state_(0) = init_yaw;
  prev_state_(1) = init_pose_x;
  prev_state_(2) = init_pose_y;

  //tf2d::Kinematics init_robot(tf2d::Pose2D(init_yaw, init_pose_x, init_pose_y), wheel_separation_, wheel_radius_);

  //ekf_bot = init_robot;

  //ROS_INFO_STREAM("X is "<<init_pose_x);
  //ROS_INFO_STREAM("Y is "<<init_pose_y);
  got_initpose_data = 1;


}

void Ekf_Slam::joint_Callback(const sensor_msgs::JointState::ConstPtr &joint_msg)
{
	current_joint = *joint_msg;
	got_odom_data = 1;

	std::vector<std::string> joints;
	int left_wheel, right_wheel;



	if(got_odom_data == 1)
	{

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

    // if (got_initpose_data == 1 || got_odom_data == 1)
    // {
    //   bot.pose_.x = init_pose_x;
    //   bot.pose_.y = init_pose_y;
    //   bot.pose_.th= init_yaw;

    //   //pos = bot.pose_;

    //   //pos.x = init_pose_x;
    //   //pos.y = init_pose_y;

    //   ROS_INFO_STREAM("x is "<<bot.pose_.x);
    //   ROS_INFO_STREAM("y is "<<bot.pose_.y);
    //   ROS_INFO_STREAM("theta is "<<bot.pose_.th);

    //   //q.setRPY(0, 0, pos.th);
    //   //q_geometry = tf2::toMsg(q);
    //   //got_initpose_data = 0;
    // }
    // else if(got_initpose_data == 0)
    // {
    //   pos.x = pos.x;
    //   pos.y = pos.y;
    //   q_geometry = q_geometry;
    //   got_initpose_data = 0;
    // }



    pos = bot.pose_;  // pose()

    //ROS_INFO_STREAM("pos_x is "<<pos.x);
    //ROS_INFO_STREAM("pos_y is "<<pos.y);
    //ROS_INFO_STREAM("pos_th is "<<pos.th);
    q.setRPY(0, 0, pos.th);
    q_geometry = tf2::toMsg(q);


		odom_point.header.frame_id = map_frame_id_;
		odom_point.header.stamp = ros::Time::now();


		odom_point.pose.position.x = pos.x;
		odom_point.pose.position.y = pos.y;
		odom_point.pose.position.z = 0;
		odom_point.pose.orientation = q_geometry;

		odom_points.push_back(odom_point);

		odom_path_.header.frame_id = map_frame_id_;
		odom_path_.header.stamp = ros::Time::now();

		odom_path_.poses = odom_points;
		odom_path_pub.publish(odom_path_);
    //ROS_INFO("Publishing Odom Path");

		if(got_landmark_data == 1)
		{
      std::array<double,2> ekf_cmd = ekf_bot.updateOdom(current_joint.position[left_wheel], current_joint.position[right_wheel]);

      ekf_cmd_cur_r_e_ticks = ekf_cmd[1];
      ekf_cmd_cur_l_e_ticks = ekf_cmd[0];
      std::array<double,2> ekf_twist = ekf_bot.wheels_to_twist(ekf_cmd_cur_r_e_ticks, ekf_cmd_cur_l_e_ticks);

      ekf_tw.wz = ekf_twist[0];
      ekf_tw.vx = ekf_twist[1];
      ekf_tw.vy = 0.0;


			// **************************************  Prediction Step for EKF ***********************************************************************
			Ekf_Slam::PredictionStep(ekf_tw);

      // ************************************** END ********************************************************************************************

      // ******************************************* UKF Prediction Step ***********************************************************************
 
      // int n_x = prev_state_.size();
      // int n_w = 2;
      // int n_aug = n_x + n_w;  // 29
      // Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug); // 27 + 2 
      // Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug, n_aug); // 29x29

      // int rows = P_aug.rows();
      // int cols = P_aug.cols();

      // ROS_INFO("%d and %d and %d", rows, cols, n_aug);


      // //Eigen::VectorXd state_space_mu_eigen = Eigen::VectorXd::Map(state_space_mu_.data(), state_space_mu_.size());
      // //assert(state_space_mu_eigen.size() == n_x);
      // // Eigen::VectorXd state_space_mu_eigen(state_space_mu_.size());
      // // for (std::size_t i = 0; i < state_space_mu_.size(); ++i) 
      // // {
      // //       state_space_mu_eigen(i) = state_space_mu_[i];
      // // }

      // // Copies the values from prev_state_ vecto to x_aug vector
      // x_aug.head(n_x) = prev_state_;
      // //x_aug.head(n_x) = state_space_mu_;
      

      // P_aug.topLeftCorner(n_x, n_x) = sigma;
      // P_aug.bottomRightCorner(n_w, n_w) = motion_model_noise;  //sensor_model_noise;

      // // Generate Sigma Points
      // std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_aug, P_aug);
      // std::vector<double> sigma_weights = generateSigmaWeights(n_aug);





      // // Predict Sigma Points
      // std::vector<Eigen::VectorXd> sigma_points_predict;
      // //Eigen::VectorXd sigma_points_predict;
      // double dt = 1.0;
      // double wz = ekf_twist[0];
      // double vx = ekf_twist[1];
      // for(const auto& sigma_point : sigma_points)
      // {
      //   sigma_points_predict.push_back(vehicleProcessModel(sigma_point, wz, vx, dt));

      // }

      // // Calculate Mean
      // //state_.resize(prev_state_);
      // for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
      // {
      //   state_ += sigma_weights[i] * sigma_points_predict[i];
      // }

      // prev_state_(0,0) = state_(0,0);
      // prev_state_(1,0) = state_(1,0);
      // prev_state_(2,0) = state_(2,0);
      // state_(0) = tf2d::normalize_angle(state_(0));

      // prev_state_(0) = tf2d::normalize_angle(prev_state_(0));
      // //state_space_mu_ = normalizeState(state_space_mu_);
                              

      
      // // Calculate Covariance 
      // //sigma = Eigen::MatrixXd::Zero(3,3);
      // //sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8; // Uncertainty of diff drive 
      // //sigma_bar.setZero();

      // for(unsigned int i = 0; i < sigma_points_predict.size(); ++i)
      // {
      //   Eigen::VectorXd diff = normaliseState(sigma_points_predict[i] - state_);
      //   sigma_bar.topLeftCorner(3,3) = sigma_bar.topLeftCorner(3,3) +  sigma_weights[i] * diff * diff.transpose();
      // }



      // //sigma_bar = sigma;
      //  ROS_INFO("I m predict");

      // ****************************** UKF Update **************************************************************************************************

			// Correction Step
			Ekf_Slam::Correction_w_MeasurementModel(current_landmark);

			// get robot pose in (th, x, y) 
			slam_pose = Ekf_Slam::getRobotState();
      //ROS_INFO("Getting robot state");

			slam_pose2d.x = slam_pose.at(1);
			slam_pose2d.y = slam_pose.at(2);
			slam_pose2d.th = slam_pose.at(0);

			slam_point.header.frame_id = map_frame_id_;
			slam_point.header.stamp = ros::Time::now();

			slam_point.pose.position.x = slam_pose.at(1);
			slam_point.pose.position.y = slam_pose.at(2);
			slam_point.pose.position.z = 0;

			q.setRPY(0, 0, slam_pose.at(0));
			q_geometry = tf2::toMsg(q);

			slam_point.pose.orientation = q_geometry;
			slam_points.push_back(slam_point);

			slam_path_.header.stamp = ros::Time::now();
			slam_path_.header.frame_id = map_frame_id_;
			slam_path_.poses = slam_points;
			slam_path_pub.publish(slam_path_);

			got_landmark_data = 0;



		}

    // if(got_initpose_data == 1)
    // {
    //   slam_pose2d.x = init_pose_x;
    //   slam_pose2d.y = init_pose_y;
    //   slam_pose2d.th = init_yaw;

    //   got_initpose_data = 0;
    // }


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
        //ROS_INFO("Broadcasting TF");
        got_odom_data = 0;
        got_landmark_data = 0;


	}




}





																					
