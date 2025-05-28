#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>
#include <memory>
#include "StateEstimatorContainer.h"
#include "PositionVelocityEstimator.h"
#include "OrientationEstimator.h"
#include "../messages/LowlevelState.h"

class FloatingBaseObserver {
public:
  FloatingBaseObserver(LowlevelState* state, std::shared_ptr<StateEstimatorContainer> estimator,
                       const pinocchio::Model& controlModel, pinocchio::Data& controlData);

  void reset(const pinocchio::SE3& X_0_fb);
  void run();

  Eigen::Matrix3d orientation() const { return orientation_; }
  Eigen::Vector3d position() const { return position_; }

private:
  void estimateOrientation();
  void estimatePosition();
  pinocchio::SE3 getAnchorFrameInWorld(const pinocchio::Model& model, const pinocchio::Data& data);

  LowlevelState* state_;
  std::shared_ptr<StateEstimatorContainer> estimator_;
  CheaterOrientationEstimator* estimator_ori_;
  CheaterPositionVelocityEstimator* estimator_pos_;

  Eigen::Matrix3d orientation_;
  Eigen::Vector3d position_;
  double leftFootRatio_ = 0.5;

  const pinocchio::Model& controlModel_;
  pinocchio::Data& controlData_;
};
