#pragma once

#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp> 
#include <Eigen/Core>
#include <array>
#include <string>
#include <unordered_map>

#include "../messages/LowLevelState.h"
#include "StateEstimatorContainer.h"

class Estimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Estimator() = default;

    void update();

    Eigen::Vector3d com() const;
    Eigen::Vector3d comd() const;

    std::array<Eigen::Matrix<double, 6, 1>, 2> footforce;

    bool leftFootRatioJumped_ = false;
    double leftFootRatio = 0.5;
    bool pauseWalking = false;
    bool pauseWalkingRequested = false;
    int nbMPCFailures = 0;

    LowlevelState* lowState = nullptr;
    StateEstimatorContainer* container = nullptr;

    std::string supportContact() const;

    void setSensorForce(const std::string &sensorName, const Eigen::Matrix<double, 6, 1> &force);

    Eigen::Vector3d realCoM;
    Eigen::Vector3d realCoMd;
};
