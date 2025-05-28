#include "../../include/common/FloatingBaseObserver.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>

FloatingBaseObserver::FloatingBaseObserver(
    LowlevelState* state,
    std::shared_ptr<StateEstimatorContainer> estimator,
    const pinocchio::Model& controlModel,
    pinocchio::Data& controlData)
  : state_(state),
    estimator_(estimator),
    controlModel_(controlModel),
    controlData_(controlData),
    estimator_ori_(nullptr),
    estimator_pos_(nullptr)
{
  // 获取两个子估计器的指针
  estimator_ori_ = estimator_->findEstimator<CheaterOrientationEstimator>();
  estimator_pos_ = estimator_->findEstimator<CheaterPositionVelocityEstimator>();
  if (!estimator_ori_) {
    std::cerr << "[FloatingBaseObserver] Error: CheaterOrientationEstimator not found!" << std::endl;
  }
  if (!estimator_pos_) {
    std::cerr << "[FloatingBaseObserver] Error: CheaterPositionVelocityEstimator not found!" << std::endl;
  }
}

void FloatingBaseObserver::reset(const pinocchio::SE3& X_0_fb)
{
  orientation_ = X_0_fb.rotation();
  position_ = X_0_fb.translation();
}

void FloatingBaseObserver::run()
{
  estimateOrientation();
  estimatePosition();
}

void FloatingBaseObserver::estimateOrientation()
{
  if (!estimator_ori_) return;

  const auto& rpy_measured = estimator_ori_->_stateEstimatorData.result->rpy;

  Eigen::Matrix3d R_control = controlModel_.frames[controlModel_.getFrameId("trunk")].placement.rotation();
  Eigen::Vector3d rpy_control = R_control.eulerAngles(0, 1, 2);  // 控制器 yaw

  orientation_ =
      Eigen::AngleAxisd(rpy_control(2), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy_measured(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy_measured(0), Eigen::Vector3d::UnitX());
}

void FloatingBaseObserver::estimatePosition()
{
  if (!estimator_pos_) return;

  std::string support = estimator_pos_->supportContact();
  Eigen::Vector3d anchorPos_real;

  if (support == "LEFT") {
    anchorPos_real << state_->feetpose[7], state_->feetpose[8], state_->feetpose[9];
    leftFootRatio_ = 1.0;
  } else if (support == "RIGHT") {
    anchorPos_real << state_->feetpose[0], state_->feetpose[1], state_->feetpose[2];
    leftFootRatio_ = 0.0;
  } else {
    Eigen::Vector3d left(state_->feetpose[7], state_->feetpose[8], state_->feetpose[9]);
    Eigen::Vector3d right(state_->feetpose[0], state_->feetpose[1], state_->feetpose[2]);
    anchorPos_real = leftFootRatio_ * left + (1.0 - leftFootRatio_) * right;
  }

  // 控制模型中的 anchor 位姿
  pinocchio::SE3 X_0_c = getAnchorFrameInWorld(controlModel_, controlData_);
  Eigen::Vector3d r_c_0 = X_0_c.translation();

  // anchor 相对 base 的向量（实测）
  Eigen::Vector3d r_s_real = orientation_.transpose() * (anchorPos_real - position_);

  // 估算基座位置
  position_ = r_c_0 - orientation_.transpose() * r_s_real;
}

pinocchio::SE3 FloatingBaseObserver::getAnchorFrameInWorld(const pinocchio::Model& model, const pinocchio::Data& data)
{
  auto left_id = model.getFrameId("lcontactpoint");
  auto right_id = model.getFrameId("rcontactpoint");
  pinocchio::SE3 X_0_l=data.oMf[left_id];
  pinocchio::SE3 X_0_r=data.oMf[right_id];

  return MyWrapper::interpolate(X_0_r, X_0_l, leftFootRatio_);
}

// void FloatingBaseObserver::updateState()
// {
//   state_->base_quat = Eigen::Quaterniond(orientation_);
//   state_->base_pos = position_;
// }
