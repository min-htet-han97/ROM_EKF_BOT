#ifndef LANDMARK_INCLUDE_GUARD_HPP
#define LANDMARK_INCLUDE_GUARD_HPP
/// \file
/// \brief function to fit circles.

#include <vector>
#include "rom_ekf_robot/transform2_3.h"

namespace cylinder
{

  /// \brief function to fit a circle to a cluster of points
  /// \param cluster: a vector of x,y points that are physically near each other
  /// \return circle_params: a vector where the first two number are the cenroid
  ///         coordinates and the third is the radius
  std::vector<double> fit_circles(std::vector<tf2d::Vector2D> cluster);

}
#endif
