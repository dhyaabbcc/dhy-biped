#pragma once

#include <Eigen/Core>
#include "Contact.h" 

/** State of the inverted pendulum model.
 *
 */
class Pendulum
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Pendulum(const Eigen::Vector3d & com = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  void completeIPM(const Contact & plane);
  void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);
  void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);
  void reset(const Eigen::Vector3d & com,
             const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());
  void resetCoMHeight(double height, const Contact & contact);

  const Eigen::Vector3d & com() const { return com_; }
  const Eigen::Vector3d & comd() const { return comd_; }
  const Eigen::Vector3d & comdd() const { return comdd_; }

  Eigen::Vector3d dcm() const { return com_ + comd_ / omega_; }

  double omega() const { return omega_; }

  const Eigen::Vector3d & zmp() const { return zmp_; }
  const Eigen::Vector3d & zmpd() const { return zmpd_; }

protected:
  Eigen::Vector3d com_;
  Eigen::Vector3d comd_;
  Eigen::Vector3d comdd_;
  Eigen::Vector3d comddd_;
  Eigen::Vector3d zmp_;
  Eigen::Vector3d zmpd_;
  double omega_;
  Eigen::Vector3d gravity_=Eigen::Vector3d(0, 0, -9.81);
};

