#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>

#include "Contact.h"
#include "Preview.h"

// qpOASES
#include <qpOASES.hpp>

using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;

struct CostTerm
{
  double weight = 1.0;
  Eigen::MatrixXd Q;
  Eigen::VectorXd c;
};

struct ConstraintTerm
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  bool isInequality = false;
  double weight = 1.0;
};


class ModelPredictiveControl
{
public:
  ModelPredictiveControl();

  void solve();

  void phaseDurations(double initSupportDuration,
                      double doubleSupportDuration,
                      double targetSupportDuration);

  void comHeight(double height)
  {
    double zeta = height / 9.81;
    double omegaInv = std::sqrt(zeta);
    // clang-format off
    dcmFromState_ <<
      1, 0, omegaInv, 0, 0, 0,
      0, 1, 0, omegaInv, 0, 0;
    zmpFromState_ <<
      1, 0, 0, 0, -zeta, 0,
      0, 1, 0, 0, 0, -zeta;
    // clang-format on
  }

  void contacts(Contact initContact, Contact targetContact, Contact nextContact)
  {
    initContact_ = initContact;
    nextContact_ = nextContact;
    targetContact_ = targetContact;
  }

  void initState(const Pendulum &pendulum)
  {
      initState_ = Eigen::VectorXd(STATE_SIZE);
      initState_ << pendulum.com().head<2>(), pendulum.comd().head<2>(), pendulum.comdd().head<2>();
  }

  unsigned indexToHrep(unsigned i) const
  {
      return indexToHrep_[i];
  }

  const Contact &initContact() const
  {
      return initContact_;
  }

  unsigned nbInitSupportSteps() const
  {
      return nbInitSupportSteps_;
  }

  unsigned nbDoubleSupportSteps() const
  {
      return nbDoubleSupportSteps_;
  }

  const Contact &nextContact() const
  {
      return nextContact_;
  }

  void sole(const Sole &sole)
  {
      sole_ = sole;
  }

  const std::shared_ptr<Preview> solution() const
  {
      return solution_;
  }

  const Contact &targetContact() const
  {
      return targetContact_;
  }

private:
  void initializeSystemMatrices();
  void buildQP();

  void computeZMPRef();
  void updateTerminalConstraint();
  void updateZMPConstraint();
  void updateJerkCost();
  void updateVelCost();
  void updateZMPCost();

  Eigen::VectorXd buildTrajectoryFromSolution(const Eigen::VectorXd &sol);
  Eigen::VectorXd buildControlFromSolution(const Eigen::VectorXd &sol);

private:
  static constexpr double SAMPLING_PERIOD = 0.1; // 例子值，请根据实际修改
  static constexpr unsigned INPUT_SIZE = 2;
  static constexpr unsigned STATE_SIZE = 6;
  static constexpr unsigned NB_STEPS = 16;

  double jerkWeight_=1.;
  double velWeights_=10.;
  double zmpWeight_=1000.;

  unsigned N_;
  double T_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::VectorXd initState_;

  Eigen::MatrixXd S_;
  Eigen::MatrixXd Tmat_;

  Eigen::MatrixXd H_;
  Eigen::VectorXd g_;

  Eigen::MatrixXd Aeq_;
  Eigen::VectorXd beq_;

  Eigen::MatrixXd Aineq_;
  Eigen::VectorXd bineq_;

  Eigen::VectorXd zmpRef_;
  Eigen::VectorXd velRef_;

  Eigen::Matrix<double, 2 * (NB_STEPS + 1), STATE_SIZE *(NB_STEPS + 1)> velCostMat_;

  unsigned nbInitSupportSteps_ = 0;
  unsigned nbDoubleSupportSteps_ = 0;
  unsigned nbTargetSupportSteps_ = 0;
  unsigned nbNextDoubleSupportSteps_ = 0;

  HrepXd hreps_[4];
  unsigned indexToHrep_[NB_STEPS + 1];

  Contact initContact_;
  Contact targetContact_;
  Contact nextContact_;
  Sole sole_; // 左右脚标识

  CostTerm jerkCost_;
  CostTerm velCost_;
  CostTerm zmpCost_;

  ConstraintTerm termDCMCons_;
  ConstraintTerm termZMPCons_;
  ConstraintTerm zmpCons_;

  Eigen::MatrixXd dcmFromState_ = Eigen::Matrix<double, 2, STATE_SIZE>::Zero();
  Eigen::MatrixXd zmpFromState_ = Eigen::Matrix<double, 2, STATE_SIZE>::Zero();

  Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> A_qp;
  Eigen::VectorXd lbA_qp, ubA_qp;

  std::shared_ptr<Preview> solution_ = nullptr;

  // 工具函数
  double clamp(double x, double lower, double upper)
  {
    return std::min(std::max(x, lower), upper);
  }

  Eigen::Matrix3d slerp(const Eigen::Matrix3d &R_1, const Eigen::Matrix3d &R_2, double w)
  {
      // 将旋转矩阵转换为四元数
      Eigen::Quaterniond q1(R_1);
      Eigen::Quaterniond q2(R_2);

      // 保证最短路径插值
      if (q1.dot(q2) < 0.0)
      {
          q2.coeffs() *= -1.0;
      }

      // 进行球面线性插值
      Eigen::Quaterniond q_interp = q1.slerp(w, q2);

      // 转换回旋转矩阵
      return q_interp.toRotationMatrix();
  }
};
