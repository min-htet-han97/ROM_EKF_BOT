/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
 * ROS Simulation Online Course
 * Min Htet Han (ROM Robotics) 
 * 
 * 
 **/
// Check Section 1.0 for more details
#include "rom_ekf_robot/transform2_3.h"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>


namespace tf2d
{
    // ****************** Vector2D ******************************************************

   Vector2D::Vector2D() : x(0), y(0){}  // initialize to zero

   Vector2D::Vector2D(double input_x, double input_y) : x(input_x), y(input_y) {}
//   Vector2D::Vector2D(double xcom, double ycom)
//   {
//       x_ = xcom;
//       y_ = ycom;
//   }

   // ***********************************************************************************

   // ***************** Twist2D **********************************************************
   Twist2D::Twist2D() : wz(0), vx(0), vy(0) {}

   Twist2D::Twist2D(double angular_v, double linear_x, double linear_y) : wz(angular_v), vx(linear_x), vy(linear_y){}

   Twist2D Twist2D::scaleTwist(double dt)
   {
       Twist2D buf(this->wz*dt, this->vx*dt, this->vy*dt);
   }


   std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
  {
    os << "2D Twist, [" << tw.wz << ", " << tw.vx << ", " << tw.vy << "]\n";
    return os;
  }
  std::istream & operator>>(std::istream & is, Twist2D & tw)
  {
    std::cout << "Enter wz value of twist: ";
    is >> tw.wz;
    std::cout << "Enter vx value of twist: ";
    is >> tw.vx;
    std::cout << "Enter vy value of twist: ";
    is >> tw.vy;
    return is;
  }

   // ************************************************************************************

   // **************** Transform2D *******************************************************

   Transform2D::Transform2D()
   {
       x = 0.0;
       y = 0.0;
       theta = 0.0;
       ctheta = 1;
       stheta = 0;
   }

   Transform2D::Transform2D(const Vector2D& trans)
   {
       ctheta = 1;
       stheta = 0;
       theta = 0;
       x = trans.x;
       y = trans.y;
   }

   Transform2D::Transform2D(double radians)
   {
       theta = radians;
       ctheta = std::cos(radians);
       stheta = std::sin(radians);
       x = 0.0;
       y = 0.0;
   }

   Transform2D::Transform2D(const Vector2D& trans, double radians)
   {
       theta = radians;
       ctheta = std::cos(radians);
       stheta = std::sin(radians);
       x = trans.x;
       y = trans.y;
   }

   Transform2D::Transform2D(const Vector2D& trans, double radians, bool y_axis)
   {
       theta = radians;
       ctheta = std::cos(radians);
       stheta = std::sin(radians);
       x = trans.x;
       y = trans.y;
   }

   Transform2D::Transform2D(const Pose2D pos)
   {

       theta = pos.th;
       ctheta = std::cos(pos.th);
       stheta = std::sin(pos.th);
       x = pos.x;
       y = pos.y;
   }
   // ****************************************************************************************************

   Pose2D::Pose2D()
   {
       th = 0.0;
       x = 0.0;
       y = 0.0;
   }

   Pose2D::Pose2D(double ang, double xpos, double ypos)
   {
       th = ang;
       x = xpos;
       y = ypos;
   }
  // return x, y and theta (x, y and theta are the private variable of Transform2D class)
   Pose2D Transform2D::displacement() const
{
    return {rad2deg(theta), x, y};
}

 Pose2D Transform2D::displacementRad() const
 {
     return {theta, x, y};
 }

   // ****************** For Vector2D ***************************************

   std::ostream& operator<<(std::ostream& os, const Vector2D &v)
   {
       os << "2D  Vector : " << v.x << ", " << v.y << "\n";
       return os;
   }

   std::istream& operator>>(std::istream& is, Vector2D &v)
   {
       std::cout << "Enter x : ";
       is >> v.x;
       std::cout << "Enter y : ";
       is >> v.y;
       return is;
   }

   double Vector2D::distance(Vector2D vec) const
   {

       double x_different, y_different;
       x_different = std::fabs(vec.x - x);
       y_different = std::fabs(vec.y - y);

       return pow(x_different*x_different  + y_different*y_different, 0.5);
   }

   double Vector2D::dot(Vector2D vec)const
   {
       return (x*vec.x + y*vec.y);
   }

   double Vector2D::angle()const
   {
       return rad2deg(std::atan(y/x));
   }

   double Vector2D::length()const
   {
       return pow(x*x + y*y, 0.5);
   }

   Vector2D Vector2D::normalize()const
   {
       float magnitude;
       Vector2D unit_vector;

       magnitude = this->length();

       unit_vector.x = x / magnitude;
       unit_vector.y = y / magnitude;

       return unit_vector;
   }

   Vector2D& Vector2D::operator+=(const Vector2D& rhs)
   {
       x +=rhs.x;
       y +=rhs.y;

       return *this;
   }

   Vector2D& Vector2D::operator-=(const Vector2D& rhs)
   {
       x -= rhs.x;
       y -= rhs.y;

       return *this;
   }

   Vector2D& Vector2D::operator*=(const Vector2D& rhs)
   {
       x *= rhs.x;
       y *= rhs.y;

       return *this;
   }

   Vector2D operator+(Vector2D lho, const Vector2D& rho)
   {
       lho += rho;
       return lho;
   }

   Vector2D operator-(Vector2D lho, const Vector2D& rho)
   {
       lho -= rho;
       return lho;
   }

   Vector2D operator*(Vector2D lho, const Vector2D& rho)
   {
       lho *= rho;
       return lho;
   }

   Vector2D operator*(const double& d, Vector2D lho)
   {
       lho.x *= d;
       lho.y *= d;

       return lho;
   }

   Vector2D operator*(Vector2D lho, const double& d)
   {
       lho.x *= d;
       lho.y *= d;

       return lho;
   }

   Vector2D operator/(const double& d, Vector2D lho)
   {
       lho.x /= d;
       lho.y /= d;

       return lho;
   }

   Vector2D operator/(Vector2D lho, const double& d)
   {
       lho.x /= d;
       lho.y /= d;

       return lho;
   }
   // ***********************************************************************

   // ****************** For Transform2D ************************************
   std::ostream& operator<<(std::ostream& os, const Transform2D &tf)
   {
       os << "Transform2D : " << tf.theta << ", " << tf.x << ", " << tf.y << "\n";
       return os;
   }

   std::istream& operator>>(std::istream& is, Transform2D &tf)
   {
       std::cout << "Enter x_tf : ";
       is >> tf.x;
       std::cout << "Enter y_tf : ";
       is >> tf.y;
       std::cout << "Enter theta : ";
       is >> tf.theta;
       return is;
   }


 // Apply a transformation to a Vector2D
 // Note: following equations for v_prime.x and y are only for rotation along z axis ( Equ - 2 )
   Vector2D Transform2D::operator()(Vector2D v)const
   {
       Vector2D v_prime;

       v_prime.x = ctheta * v.x - stheta * v.y + x;
       v_prime.y = stheta * v.x + ctheta * v.y + y;

       return v_prime;
   }

   // Twist with respect to another new frame , ( Equ 3 )
   Twist2D Transform2D::operator()(Twist2D tw) const
   {
       Twist2D tw_prime;

       tw_prime.wz = tw.wz;
       tw_prime.vx = tw.vx * ctheta - tw.vy * stheta + tw.wz*y;
       tw_prime.vy = tw.vy * stheta + tw.vy * ctheta - tw.wz*x;

       return tw_prime;
   }

   Transform2D Transform2D::inv() const
   {
       Transform2D inv_trans(theta, ctheta, -stheta, -x * ctheta - y * stheta, x * stheta - y * ctheta);
       return inv_trans;
   }

   // This multiplication only allow for the rotation about Z axis
   // Rz = | cos(theta) -sin(theta) 0 |
   //      | sin(theta)  cos(theta) 0 |
   //      | 0           0          1 |

   // Ry = | cos(theta)  0  sin(theta) |
   //      | 0           1           0 |
   //      | -sin(theta) 0  cos(theta) |
   Transform2D & Transform2D::operator*=(const Transform2D & rho)
   {
       double x_buf, y_buf, cth_buf, sth_buf;

       cth_buf = ctheta*rho.ctheta - stheta*rho.stheta;
       sth_buf = stheta*rho.ctheta + ctheta*rho.stheta;

       x_buf = ctheta*rho.x - stheta*rho.y + x;
       y_buf = stheta*rho.x + ctheta*rho.y + y;
       theta = std::atan2(sth_buf, cth_buf);


       x = x_buf;
       y = y_buf;
       ctheta = cth_buf;
       stheta = sth_buf;

       return *this;
   }

   Transform2D operator*(Transform2D lho, const Transform2D & rho)
  {
    lho *= rho ;
    return lho;
  }

  // Private
  Transform2D::Transform2D(double theta, double ctheta, double stheta, double x, double y)
  {
    this->theta = theta;
    this->ctheta = ctheta;
    this->stheta = stheta;
    this->x = x;
    this->y = y;
  }

   //
   // ***********************************************************************
   double Transform2D::GetTransform2DX()const
   {
       return x;
   }
   double Transform2D::GetTransform2DY()const
   {
       return y;
   }
   double Transform2D::GetTransform2D_theta()const
   {
       return theta;
   }

double Vector2D::GetX() const
    {

    return x;
    }

double Vector2D::GetY() const
    {
    return y;
    }

// void Vector2D::SetX(const double x)
//     {
//     x = x;
//     }

// void Vector2D::SetY(const double y)
//     {
//     y = y;
//     }

Transform2D transformFromTwist(Twist2D tw)
  {
    Pose2D pose(tw.wz, tw.vx, tw.vy);
    Transform2D T_bbp;

    if(almost_equal(pose.th, 0))
    {
      Vector2D vec_bp(pose.x, pose.y);
      Transform2D buffer(vec_bp);

      T_bbp = buffer;
    }

    else
    {
      Vector2D vec_s( pose.y/pose.th, -pose.x/pose.th);

      // Create the transform of the current transform relative to the point of rotation.
      Transform2D T_sb(vec_s);

      // Rotate at the point of rotation
      Transform2D T_ssp(pose.th);

      // Get the Transform  of the new frame after applying the twist relative to the current frame
      // Note: T_sb == T_spbp

      T_bbp = T_sb.inv() * T_ssp * T_sb;
    }

    return T_bbp;
  }

    Transform2D Transform2D::integrateTwist(const Twist2D tw) const
  {
    return *this * transformFromTwist(tw);
  }

   

}  // namespace
