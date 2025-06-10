#pragma once

#include <memory>
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <qpOASES.hpp>
#include <chrono>

#include "Biped.h"
#include "Pendulum.h"
#include "Contact.h"
#include "../messages/LowlevelState.h"
#include "./Utilities/ExponentialMovingAverage.h"
#include "./Utilities/LeakyIntegrator.h"
#include "./Utilities/StationaryOffsetFilter.h"


struct Stabilizer{

public:

  // 常量定义
  static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  static constexpr double MAX_COM_ADMITTANCE = 20; /**< Maximum admittance for CoM admittance control */
  static constexpr double MAX_COP_ADMITTANCE = 0.1; /**< Maximum CoP admittance for foot damping control */
  static constexpr double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  static constexpr double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  static constexpr double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  static constexpr double MAX_DFZ_ADMITTANCE =
      5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  static constexpr double MAX_DFZ_DAMPING =
      10.; /**< Maximum normalized damping in [Hz] for foot force difference control */

  //这些参数需要在姿态任务轨迹定义是给出上下限
  static constexpr double MAX_FDC_RX_VEL =
      0.2; /**< Maximum x-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RY_VEL =
      0.2; /**< Maximum y-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RZ_VEL =
      0.2; /**< Maximum z-axis angular velocity in [rad] / [s] for foot damping control. */

  static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; /**< Maximum CoM offset due to admittance control in [m] */
  static constexpr double MIN_DSP_FZ = 15.; /**< Minimum normal contact force in [N] for DSP, used to avoid low-force
                                               targets when close to contact switches. */

  Stabilizer(Biped &controlRobot, Pendulum &pendulum, double dt, LowlevelState &state);

  void addTasks(tsid::InverseDynamicsFormulationAccForce & solver);

  void disable();

  Eigen::Vector3d computeZMP(const pinocchio::Force & wrench) const;

  void reconfigure();

  bool detectTouchdown(const Contact & contact, const pinocchio::SE3 & footPoseMeasured, const Eigen::Vector3d & footForceWorld);

  void removeTasks(tsid::InverseDynamicsFormulationAccForce & solver);

  void reset(MyWrapper & robot);

  void run();

  void seekTouchdown(std::shared_ptr<tsid::tasks::TaskSE3Equality> footTask, tsid::trajectories::TrajectorySample & ref, const Eigen::Vector3d & footForceWorld);
  
  void setContact(std::shared_ptr<tsid::contacts::Contact6d> &contactTask, const Contact & contact, const std::string & contactFrameName);

  void setSwingFoot(std::shared_ptr<tsid::tasks::TaskSE3Equality> footTask);

  ContactState contactState()
  {
    return contactState_;
  }

  void contactState(ContactState contactState)
  {
    contactState_ = contactState;
  }

  void sole(const Sole &sole)
  {
      sole_ = sole;
  }

  void updateState(const Eigen::Vector3d &com,
                   const Eigen::Vector3d &comd,
                   const pinocchio::Force &wrench,
                   double leftFootRatio);

  void wrenchFaceMatrix(const Sole &sole)
  {
      double X = sole.halfLength;
      double Y = sole.halfWidth;
      double mu = sole.friction;
      // clang-format off
        wrenchFaceMatrix_ <<
        // fx   fy   fz   tx   ty   tz
        -1. ,  0. , -mu ,  0. ,  0. ,  0. ,  // Fx ≥ -mu * Fz
        +1. ,  0. , -mu ,  0. ,  0. ,  0. ,
        0. , -1. , -mu ,  0. ,  0. ,  0. ,  // Fy ≥ -mu * Fz
        0. , +1. , -mu ,  0. ,  0. ,  0. ,
        0. ,  0. ,  -Y , -1. ,  0. ,  0. ,  // τx ≥ -Y * Fz
        0. ,  0. ,  -Y , +1. ,  0. ,  0. ,
        0. ,  0. ,  -X ,  0. , -1. ,  0. ,  // τy ≥ -X * Fz
        0. ,  0. ,  -X ,  0. , +1. ,  0. ,
        -Y , -X , -(X + Y)*mu , +mu , +mu , -1. , // Coupled torque/friction cone approx
        -Y , +X , -(X + Y)*mu , +mu , -mu , -1. ,
        +Y , -X , -(X + Y)*mu , -mu , +mu , -1. ,
        +Y , +X , -(X + Y)*mu , -mu , -mu , -1. ,
        +Y , +X , -(X + Y)*mu , +mu , +mu , +1. ,
        +Y , -X , -(X + Y)*mu , +mu , -mu , +1. ,
        -Y , +X , -(X + Y)*mu , -mu , +mu , +1. ,
        -Y , -X , -(X + Y)*mu , -mu , -mu , +1. ;
      // clang-format on
  }


  Eigen::Vector3d zmp() const
  {
    return computeZMP(distribWrench_);
  }

private:

  struct FDQPWeights
  {
    void configure()
    {
      double ankleTorqueWeight = 100.0;
      double forceRatioWeight = 1.0;
      double netWrenchWeight = 10000.0;
      ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
      forceRatioSqrt = std::sqrt(forceRatioWeight);
      netWrenchSqrt = std::sqrt(netWrenchWeight);
    }
  public:
    double ankleTorqueSqrt;
    double forceRatioSqrt;
    double netWrenchSqrt;
  };

  void checkInTheAir();

  void checkGains();

  pinocchio::Force computeDesiredWrench();

  void distributeWrench(const pinocchio::Force & desiredWrench);

  void saturateWrench(const pinocchio::Force &desiredWrench, const Contact & contact, std::shared_ptr<tsid::contacts::Contact6d> & contactTask);

  void setSupportFootGains();

  void updateCoMTaskZMPCC();

  void updateZMPFrame();

public:
  Contact leftFootContact; 
  Contact rightFootContact; 
  std::shared_ptr<tsid::tasks::TaskComEquality> comTask;
  std::shared_ptr<tsid::tasks::TaskSE3Equality> leftFootTask;
  std::shared_ptr<tsid::tasks::TaskSE3Equality> rightFootTask;
  std::shared_ptr<tsid::contacts::Contact6d> contactL;
  std::shared_ptr<tsid::contacts::Contact6d> contactR;

  bool leftFootInContact = true;
  bool rightFootInContact = true;

private:
  ContactState contactState_ = ContactState::DoubleSupport;
  Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_;
  Eigen::Matrix<double, 3, 4> contactPoints;
  double frictionCoeff_;
  double minNormalForce_ = 0;
  double maxNormalForce_ = 2000;

  Eigen::Vector2d comAdmittance_ = Eigen::Vector2d::Zero(); //无用
  Eigen::Vector2d copAdmittance_ = Eigen::Vector2d::Zero(); //无用
  Eigen::Vector3d comStiffness_ = {1000., 1000., 100.};

  Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmVelError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMAccel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMOffset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMVel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccError_ = Eigen::Vector3d::Zero();

  pinocchio::Force distribWrench_ = pinocchio::Force::Zero();
  pinocchio::Force measuredWrench_ = pinocchio::Force::Zero();
  pinocchio::Motion contactDamping_ = pinocchio::Motion::Zero();
  pinocchio::Motion contactStiffness_ = pinocchio::Motion::Zero();
  pinocchio::SE3 zmpFrame_ = pinocchio::SE3::Identity();

  ExponentialMovingAverage dcmIntegrator_;
  FDQPWeights fdqpWeights_;
  LeakyIntegrator zmpccIntegrator_;
  StationaryOffsetFilter dcmDerivator_;
  LowlevelState state_;

  const Pendulum & pendulum_;
  const Biped & controlRobot_;
  Sole sole_;
  double mass_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);

  bool inTheAir_ = false;
  bool zmpccOnlyDS_ = true;

  double comWeight_ = 1000.;
  double dcmDerivGain_ = 0.;
  double dcmIntegralGain_ = 5.;
  double dcmPropGain_ = 1.;
  double dfzAdmittance_ = 1e-4;
  double dfzDamping_ = 0.;
  double dfzForceError_ = 0.;
  double dfzHeightError_ = 0.;
  double swingFootStiffness_ = 2000.;
  double swingFootWeight_ = 500.;
  double footTaskPriority_ = 0.0;
  double vdcStiffness_ = 1000.;

  double dt_ = 0.005;  //时间需要同步和外部控制器
  double leftFootRatio_ = 0.5;
  double runTime_ = 0.;

};