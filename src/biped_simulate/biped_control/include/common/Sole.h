#include "../../robotwrapper/MyWrapper.hpp"


struct Sole
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d leftAnkleOffset =Eigen::Vector2d::Zero(); /**< Offset from center to ankle frames in the left sole frame */
  double friction = 0.7; /**< Friction coefficient */
  double halfLength = 0.112; /**< Half-length of the sole in [m] */
  double halfWidth = 0.065; /**< Half-width of the sole in [m]*/
};