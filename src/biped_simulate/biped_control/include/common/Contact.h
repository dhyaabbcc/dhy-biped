#pragma once

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <string>
#include <algorithm>
#include <iostream>
#include "Sole.h"
#include "FootstepPlanData.hpp"


using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;

enum class ContactState
{
  DoubleSupport,
  LeftFoot,
  RightFoot
};

class Contact
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact() = default;

  Contact(const pinocchio::SE3 & pose) : pose(pose) {}

  Eigen::Vector3d sagittal() const
  {
    return pose.rotation().row(0);
  }

  Eigen::Vector3d lateral() const
  {
    return pose.rotation().row(1);
  }

  Eigen::Vector3d normal() const
  {
    return pose.rotation().row(2);
  }

  const Eigen::Vector3d & position() const
  {
    return pose.translation();
  }

  const Eigen::Vector3d & p() const
  {
    return position();
  }

  Eigen::Vector3d anklePos(const Sole & sole) const
  {
    if (surfaceName == "lcontactpoint")
    {
      return p() + sole.leftAnkleOffset.x() * sagittal() + sole.leftAnkleOffset.y() * lateral();
    }
    else if (surfaceName == "rcontactpoint")
    {
      return p() + sole.leftAnkleOffset.x() * sagittal() - sole.leftAnkleOffset.y() * lateral();
    }
    else
    {
      std::cerr << "[ERROR] Unknown surface name: " << surfaceName << std::endl;
      return p();
    }
  }

  pinocchio::SE3 anklePose(const Sole & sole) const
  {
    return pinocchio::SE3(pose.rotation(), anklePos(sole));
  }

  double x() const { return position()(0); }
  double y() const { return position()(1); }
  double z() const { return position()(2); }

  Eigen::Vector3d vertex0() const { return position() + halfLength * sagittal() + halfWidth * lateral(); }
  Eigen::Vector3d vertex1() const { return position() + halfLength * sagittal() - halfWidth * lateral(); }
  Eigen::Vector3d vertex2() const { return position() - halfLength * sagittal() - halfWidth * lateral(); }
  Eigen::Vector3d vertex3() const { return position() - halfLength * sagittal() + halfWidth * lateral(); }

  template<int i>
  double minCoord() const
  {
    return std::min({vertex0()(i), vertex1()(i), vertex2()(i), vertex3()(i)});
  }

  template<int i>
  double maxCoord() const
  {
    return std::max({vertex0()(i), vertex1()(i), vertex2()(i), vertex3()(i)});
  }

  double xmin() const { return minCoord<0>(); }
  double xmax() const { return maxCoord<0>(); }
  double ymin() const { return minCoord<1>(); }
  double ymax() const { return maxCoord<1>(); }
  double zmin() const { return minCoord<2>(); }
  double zmax() const { return maxCoord<2>(); }

  HrepXd hrep() const
  {
    Eigen::Matrix<double, 4, 2> localHrepMat, worldHrepMat;
    Eigen::Matrix<double, 4, 1> localHrepVec, worldHrepVec;
    // clang-format off
    localHrepMat <<
      +1, 0,
      -1, 0,
      0, +1,
      0, -1;
    localHrepVec <<
      halfLength,
      halfLength,
      halfWidth,
      halfWidth;
    // clang-format on
    if((normal() - Eigen::Vector3d(0,0,1.0)).norm() > 1e-3)
    {
      std::cout<<"Contact is not horizontal"<<std::endl;
    }
    const pinocchio::SE3 & X_0_c = pose;
    worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
    worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
    return HrepXd(worldHrepMat, worldHrepVec);
  }

  Eigen::Vector3d refVel = Eigen::Vector3d::Zero();
  double halfLength = 0.;
  double halfWidth = 0.;

  std::string surfaceName;
  pinocchio::SE3 pose;
  SwingPlan swing;
  unsigned id = 0;
};

inline Contact operator*(const pinocchio::SE3 & X, const Contact & contact)
{
  Contact result = contact;
  result.pose = X.act(contact.pose);
  return result;
}
