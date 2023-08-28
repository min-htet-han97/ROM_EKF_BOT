
/**
 * ROS Simulation Online Course
 * Min Htet Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/

#include <iostream>
#include "transform2d.h"
#include "differential_drive.h"
#include <cmath>
#include <vector>


using namespace std;

int main()
{
    tf2d::Vector2D vec;
    cout << vec.GetX() << "\n";
    cout << vec.GetY() << "\n";

    cout.precision(5);
    cout.setf(ios::showpoint);

    cin  >> vec;
    cout << vec;

    cout << "cout is : " << vec.GetX() << "\n";
    double xco= 10.0;
    double yco = 10.0;
    tf2d::Vector2D vec_a(xco ,yco);



    tf2d::Transform2D tf(vec_a);
    //cin >> tf;
    //cout << tf;
    tf2d::Pose2D pose = tf.displacement();

    cout << "call xco : " << tf.displacement().x_pose << "\n";
    cout << "call yco : " << tf.displacement().y_pose << "\n";

    tf2d::Vector2D v1(1, 1);
    tf2d::Vector2D v2(2, 2);

    v1 += v2;
    cout << "Vector sum in x dir : " << v1.GetX();
    cout << "Vector sum in y dir : " << v1.GetY() << "\n";

//    tf2d::Transform2D tf;
//    cin >> tf;
//    cout << tf;

  tf2d::Vector2D vector1(2,3);
  tf2d::Vector2D vector2(5,5);

  tf2d::Vector2D result;
  result = vector1  + vector2;


  cout << "Result x is : " << result.x_;
  cout << "Result y is : " << result.y_ << "\n";

  double dist = vector1.distance(vector2);

  cout << "Distance is : " << dist << "\n";

  tf2d::Transform2D transform1(vec_a);
  tf2d::Vector2D inputVector(1.0, 1.0);
  tf2d::Vector2D transformVector = transform1(inputVector);

  double a = transformVector.x_;
  double b = transformVector.y_;

  cout << "a : " << a << ", " << "b : " << b << "\n";

  tf2d::Vector2D v4(1.0, 2.0);
  tf2d::Vector2D v5(3.0, 4.0);

  double radians_1 = 50.0;//0.5;
  double radians_2 = 50.0;//1.2;

  double about_axis = 1;
  tf2d::Transform2D tf1(v4, radians_1);
  tf2d::Transform2D tf2(v5, radians_2);
  tf2d::Transform2D result_;

  //result_ = tf1 * tf2;

  //tf1 *= tf2;
  result_ = tf1 * tf2;



  //tf1 *= tf2;

  cout << "Result X is : " << result_.GetTransform2DX() << "\n";
  cout << "Result Y is : " << result_.GetTransform2DY() << "\n";
  cout << "Result Theta is : " << result_.GetTransform2D_theta() << "\n";

  cout << "X is : " << tf1.GetTransform2DX() << "\n";
  cout << "Y is : " <<  tf1.GetTransform2DY() << "\n";
  cout << "Theta is : " << tf1.GetTransform2D_theta() << "\n";

  double radians_0 = 50;
  std::vector<std::vector<double>> matrix1 =
  {
      {std::cos(radians_0), -std::sin(radians_0), 0.0, 1.0},
      {std::sin(radians_0), std::cos(radians_0), 0.0, 2.0},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
  };

    std::vector<std::vector<double>> matrix2 =
  {
      {std::cos(radians_0), -std::sin(radians_0), 0.0, 3.0},
      {std::sin(radians_0), std::cos(radians_0), 0.0,  4.0},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
  };
  std::vector<std::vector<double>>  result_001 = tf2d::matrixMultiplication_3d(matrix1, matrix2);
  std::cout << "x is " << result_001[0][3] << "\n";
  std::cout << "y is " << result_001[1][3] << "\n";

  std::vector<double> values = tf2d::extractAngles(result_001);
  std::cout << "Theta is : " << values[2] << "\n";



  //tf2d::Transform2D tf3(radians);
  //cout << "Theta is : " << tf3.GetTransform2D_theta() << "\n";


  // Testing !!!!!!!!!!!!!!!!!!!!!!1
    tf2d::Pose2D pos;
    tf2d::Kinematics bot;
    tf2d::Pose2D pos2d(1.0, 8, 6);
    tf2d::Kinematics init_robot_(pos2d, 13, 17);

    pos = init_robot_.pose();

    std::cout << "x is : " << pos.x_pose;
    std::cout << "y is : " << pos.y_pose;
    std::cout << "theta is :" << pose.th << "\n";


    std::array<double,2> result_odom;

    result_odom = init_robot_.updateOdom(6.0, 6.0);

    pos = init_robot_.pose();

    std::cout << "x is : " << pos.x_pose;
    std::cout << "y is : " << pos.y_pose;
    std::cout << "theta is :" << pose.th << "\n";





return 0;

}
