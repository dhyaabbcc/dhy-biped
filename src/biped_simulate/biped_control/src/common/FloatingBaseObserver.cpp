#include "../../include/common/FloatingBaseObserver.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>

FloatingBaseObserver::FloatingBaseObserver(
    LowlevelState *state,
    std::shared_ptr<StateEstimatorContainer> estimator,
    Biped &robot)
    : robot_(robot),
      state_(state),
      estimator_(estimator),
      controlModel_(robot._Dyptr->model()),
      estimator_ori_(nullptr),
      estimator_pos_(nullptr)
{
  // 获取两个子估计器的指针
  estimator_ori_ = estimator_->findEstimator<CheaterOrientationEstimator>();
  estimator_pos_ = estimator_->findEstimator<CheaterPositionVelocityEstimator>();
  if (!estimator_ori_)
  {
    std::cerr << "[FloatingBaseObserver] Error: CheaterOrientationEstimator not found!" << std::endl;
  }
  if (!estimator_pos_)
  {
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
  //1.构造初始化时，设置qv为默认值弯曲腿(在main中实现)
  //2.pino模型应该是实时更新的,从biped中获取
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


//有错误，anchorPos_real应该时plan规划出来的
//X_0_c应该是用base下，根据姿态qv求解处footpose
void FloatingBaseObserver::estimatePosition()
{
  if (!estimator_pos_) return;

  std::string support = estimator_pos_->supportContact();
  Eigen::Vector3d anchorPos_real;


  //部署时改为编码器关节角度+上一时刻的质心位姿估计（此处为了方便，直接用足底位姿）
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

  // 控制模型中的 anchor 位姿(稳定器+微分逆动力学解算完关节角度，导入model),使用观测前先导入角度，在更新模型
  pinocchio::SE3 X_0_c = getAnchorFrameInWorld(controlModel_, *robot_._Dataptr);
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



//参考！！！！！
  // // 1. 选择支撑脚
  // bool leftFootInContact = leftFootForce.wrench(2) > contactForceThreshold_;
  // bool rightFootInContact = rightFootForce.wrench(2) > contactForceThreshold_;

  // std::string supportFoot;
  // if(leftFootInContact && !rightFootInContact)
  //   supportFoot = leftFoot_;
  // else if(rightFootInContact && !leftFootInContact)
  //   supportFoot = rightFoot_;
  // else
  //   supportFoot = leftFootForce.wrench(2) > rightFootForce.wrench(2) ? leftFoot_ : rightFoot_;

  // // 2. 获得控制模型中 base -> foot 的 transform
  // pinocchio::SE3 baseToFoot = pinocchio::updateFramePlacement(model_, controlData,
  //                                         model_.getFrameId(supportFoot));
  // pinocchio::SE3 footToBase = baseToFoot.inverse();

  // // 3. 构造 base 姿态：IMU 提供 roll/pitch，使用估计的 yaw
  // Eigen::Matrix3d R_imu = imu.orientation;
  // Eigen::AngleAxisd yawRot(yawEstimate, Eigen::Vector3d::UnitZ());
  // Eigen::Matrix3d R_base = yawRot.toRotationMatrix() * R_imu;

  // // 4. 假设支撑脚在世界系中的位置（锚定），如 (0,0,0)
  // pinocchio::SE3 footWorld(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // // 5. base 在世界中的估计：X_0_base = X_0_foot * X_foot_base
  // basePoseWorld = footWorld * footToBase;
  // basePoseWorld.rotation() = R_base;