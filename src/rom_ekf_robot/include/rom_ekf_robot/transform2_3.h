/**
 * 
 * Source -> https://nu-msr.github.io/navigation_site/
 * 
 * Min Htet Han (ROM Robotics) 
 * minhtethan94@gmail.com
 * 
 **/
// Check Section 1.0 for more details

#ifndef TF2D_INCLUDE_GUARD_HPP
#define TF2D_INCLUDE_GUARD_HPP

 
 //**************************/

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>
#include <vector>


namespace tf2d
{
    constexpr double PI=3.14159;

    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
      return std::fabs(d1 - d2) < epsilon ? true : false;
    }

    constexpr double deg2rad(double deg)
    {
      return deg * (PI/180.0);
    }

    constexpr double rad2deg(double rad)
    {
        return rad * (180.0/PI);
    }

    constexpr double normalize_angle(double rad)
    {
      double ang = 0;
      double i = 0;

      i = std::floor((rad + PI)/(2.0 * PI));
      ang = rad + PI - (i * 2.0 * PI);
      if(ang < 0)
      {
        ang += 2.0*PI;
      }
      return ang - PI;
    }

    /// \brief Linear Interpolation function
    /// \param x value to interpolate from xlims to ylims
    /// \param xlims limits for the input range
    /// \param ylims limits for the output range
    /// \return angle mapped to the range [-pi, pi)
    constexpr double linInterp(double x, const double xlims[], const double ylims[])
    {
      double y = 0;
      y = ylims[0] + (x - xlims[0]) * ((ylims[1] - ylims[0])/(xlims[1] - xlims[0]));
      return y;
    }



// ***************************** Vector2D ************************************************************************

    struct Vector2D
    {
        double x;
        double y;

        Vector2D();

        Vector2D(double xcom, double ycom);


        double GetX() const;
        double GetY() const;

        void SetX(const double a);
        void SetY(const double b);

        Vector2D normalize() const;

        double length() const;

        double angle() const;

        // dot product
        double dot(Vector2D vec) const;

        // distance between two vectors
        double distance(Vector2D vec) const;

        Vector2D &operator+=(const Vector2D &rhs);

        Vector2D &operator-=(const Vector2D &rhs);

        Vector2D &operator*=(const Vector2D &rhs);

        bool operator==(const Vector2D& rhs);
        bool operator!=(const Vector2D& rhs);


    };

    Vector2D operator+(Vector2D lho, const Vector2D& rho);

    Vector2D operator-(Vector2D lho, const Vector2D& rho);

    Vector2D operator*(Vector2D lho, const Vector2D& rho);

    // Multiply a vector and a double
    // double value d as the first argument and a'Vector2D' object lho as the second argument
    // usage -> result = 2.0 * vector;  // d = 2.0
    Vector2D operator*(const double& d, Vector2D lho);

    // double value d as the second argument and a 'VectorD' object lho as the first argument
    // usage -> result = vector * 2.0;  // d = 2.0
    Vector2D operator*(Vector2D lho, const double& d);


    // usage -> result = vector / 2.0; // d = 2.0
    Vector2D operator/(Vector2D lho, const double& d);

    // usage -> result = 2.0 / vector; // d = 2.0
    Vector2D operator/(const double& d, Vector2D lho);

    std::ostream &operator<<(std::ostream& os, const Vector2D &v);

    std::istream &operator>>(std::istream& is, Vector2D &v);

// *************************************************************************************************************

// ********************** Pose2D *******************************************************************************

    struct Pose2D
    {
    public:
        double th = 0.0;
        double x = 0.0;
        double y = 0.0;

        Pose2D();

        Pose2D(double ang, double xpos, double ypos);
    };

// *************************************************************************************************************

// ******************** Twist2D ********************************************************************************

    struct Twist2D
    {
    public:
        double wz = 0.0; // angular velocity
        double vx = 0.0; // x  velocity component
        double vy = 0.0; // y  velocity component

        Twist2D(); // Default constructor

        Twist2D(double ang, double linx, double liny); // liny will always be zero for diff drive model

        Twist2D scaleTwist(double dt);
    };

    std::ostream& operator<<(std::ostream& os, const Twist2D& t);
    std::istream& operator>>(std::istream& is, Twist2D& t);
// ************************************************************************************************************


// *********************** TF 3D ******************************************************************************

 std::vector<std::vector<double>> matrixMultiplication_3d(const std::vector<std::vector<double>>& matrix1, const std::vector<std::vector<double>>& matrix2);
 void printMatrix(const std::vector<double>& matrix);
 std::vector<double> extractAngles(const std::vector<std::vector<double>>& result_matrix);


// ************************************************************************************************************



// *********************** Transform2D ************************************************************************

    class Transform2D
    {
    public:
        Transform2D();

        explicit Transform2D(const Vector2D& trans);

        explicit Transform2D(double radians);

        Transform2D(const Vector2D& trans, double radians);

        Transform2D(const Vector2D& trans, double radians, bool y_axis);


        // transform with translation and rotation in 2dimensional
        Transform2D(const Pose2D);

        Vector2D operator()(Vector2D v) const;

        Twist2D operator()(Twist2D tw)const;

        Transform2D inv() const;

        Pose2D displacement() const;
        Pose2D displacementRad() const;

        Transform2D integrateTwist(const Twist2D tw)const;

        Transform2D& operator*=(const Transform2D& rho);

        double GetTransform2DX()const;
        double GetTransform2DY()const;
        double GetTransform2D_theta()const;

        friend std::ostream& operator<<(std::ostream& os, const Transform2D& tf);
        friend std::istream& operator>>(std::istream& is, Transform2D& tf);

    private:
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        double theta, ctheta, stheta, x, y;
        bool y_axis = false;



    };
    Transform2D transformFromTwist(Twist2D tw);


    //Transform2D transformFromTwist(Twist2D tw);

    Transform2D operator*(Transform2D lho, const Transform2D& rho);

    std::ostream& operator<<(std::ostream& os, const Transform2D& tf);
    std::istream& operator>>(std::istream& is, Transform2D& tf);

    // *********************************************************************************************************



}





#endif // TRANSFORM2D_H_INCLUDED


