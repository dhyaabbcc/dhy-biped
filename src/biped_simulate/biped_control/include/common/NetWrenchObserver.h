#pragma once

#include <pinocchio/spatial/force.hpp>
#include <Eigen/Core>
#include "Contact.h"
#include "../messages/LowlevelState.h"


class NetWrenchObserver
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NetWrenchObserver();

  NetWrenchObserver(const std::vector<std::string> & sensorNames);

  void update(const LowlevelState & state, const Contact & contact);

  const pinocchio::Force & wrench() const { return netWrench_; }

  const Eigen::Vector3d & zmp() const { return netZMP_; }

private:
  void updateNetWrench(const LowlevelState & state);
  void updateNetZMP(const Contact & contact);

private:
  Eigen::Vector3d netZMP_;
  std::vector<std::string> sensorNames_;
  pinocchio::Force netWrench_;
};

