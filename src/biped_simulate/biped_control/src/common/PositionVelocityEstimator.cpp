#include "../../include/common/PositionVelocityEstimator.h"

void CheaterPositionVelocityEstimator::run() {
 // std::cout << "run StateEstimator" << std::endl;
  for(int i = 0; i < 3; i++){
    this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
    this->_stateEstimatorData.result->aWorld[i]=this->_stateEstimatorData.lowState->imu.accelerometer[i];
  }

  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
}

//检测当前支撑情况
std::string CheaterPositionVelocityEstimator::supportContact() {
  bool leftContact = _stateEstimatorData.lowState->feettwist[2];
  bool rightContact = _stateEstimatorData.lowState->feettwist[8];

  if (leftContact && rightContact) {
    return "DOUBLE";
  } else if (leftContact) {
    return "LEFT";
  } else if (rightContact) {
    return "RIGHT";
  } else {
    return "NONE";  // 两脚都不接触地面（如腾空）
  }
}