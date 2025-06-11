#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <memory>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-qpData.hpp>
#include <tsid/solvers/solver-HQP-output.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "../messages/LowLevelCmd.h"
#include "../interface/IOInterface.h"

#include "./Utilities/LowPassVelocityFilter.h"

#include "DesiredCommand.h"
#include "LegController.h"
#include "Biped.h"
#include "FootstepPlanData.hpp"
#include "FootstepPlan.h"
#include "FloatingBaseObserver.h"
#include "NetWrenchObserver.h"
#include "Stabilizer.h"
#include "ModelPredictiveControl.h"


struct ControlFSMData {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControlFSMData(Biped &biped,
                 tsid::InverseDynamicsFormulationAccForce &tsid,
                 std::shared_ptr<StateEstimatorContainer> &estimatorcontainer,
                 FootstepPlan &plan,
                 Pendulum &pendulum,
                 FloatingBaseObserver &baseObs,
                 NetWrenchObserver &wrenchObs,
                 Stabilizer &stabilizer,
                 LowlevelState &lowState,
                 ModelPredictiveControl &mpc,
                 double dt,
                 double leftFootRatio = 0.5)
      : _biped(biped), tsid_(tsid), _stateEstimator(estimatorcontainer), plan_(plan), pendulum_(pendulum),
        baseObs_(baseObs), wrenchObs_(wrenchObs), stabilizer_(stabilizer), _lowState(lowState), 
        mpc_(mpc), leftFootRatio_(leftFootRatio), timeStep(dt),comVelFilter_(dt,/* cutoff period = */ 0.01), 
        robot_(biped._Dyptr), data_(biped._Dataptr)
  {
    pinocchio::SE3 X_0_lfc = data_->oMf[robot_->model().getFrameId("lcontactpoint")];
    pinocchio::SE3 X_0_rfc = data_->oMf[robot_->model().getFrameId("rcontactpoint")];
    pinocchio::SE3 X_0_lf = data_->oMf[robot_->model().getFrameId("l_foot")];

    // 左足踝相对于足底中心（lfc）的位置
    pinocchio::SE3 X_lfc_lf = X_0_lfc.inverse() * X_0_lf;

    // 右足底中心相对于左足底中心的位置（y方向是步宽）
    pinocchio::SE3 X_rfc_lfc = X_0_lfc.inverse() * X_0_rfc;
    double stepWidth = X_rfc_lfc.translation().y();

    // 设置左脚踝在局部坐标系下的位置偏移（xy）
    sole_.leftAnkleOffset = X_lfc_lf.translation().head<2>();

    //设置
    plan_.load();
    mpc_.sole(sole_);
    stabilizer_.reset(*robot_);
    stabilizer_.sole(sole_);
    stabilizer_.wrenchFaceMatrix(sole_);
  }

  double doubleSupportDuration()
  {
    double duration;
    if(doubleSupportDurationOverride_ > 0.)
    {
      duration = doubleSupportDurationOverride_;
      doubleSupportDurationOverride_ = -1.;
    }
    else
    {
      duration = plan_.doubleSupportDuration();
    }
    return duration;
  }

  bool isLastDSP()
  {
    return (supportContact().id > targetContact().id);
  }

  bool isLastSSP()
  {
    return (targetContact().id > nextContact().id);
  }

  double leftFootRatio()
  {
    return leftFootRatio_;
  }

  double measuredLeftFootRatio()
  {
    double leftFootPressure = _lowState.feettwist[8];
    double rightFootPressure = _lowState.feettwist[2];
    leftFootPressure = std::max(0., leftFootPressure);
    rightFootPressure = std::max(0., rightFootPressure);
    return leftFootPressure / (leftFootPressure + rightFootPressure);
  }

  ModelPredictiveControl & mpc()
  {
    return mpc_;
  }

  const Contact & nextContact() const
  {
    return plan_.nextContact();
  }

  void nextDoubleSupportDuration(double duration)
  {
    doubleSupportDurationOverride_ = duration;
  }

  Pendulum & pendulum()
  {
    return pendulum_;
  }

  const Contact & prevContact() const
  {
    return plan_.prevContact();
  }

  double singleSupportDuration()
  {
    return plan_.singleSupportDuration();
  }

  const Sole & sole() const
  {
    return sole_;
  }

  Stabilizer & stabilizer()
  {
    return stabilizer_;
  }

  const Contact & supportContact()
  {
    return plan_.supportContact();
  }

  const Contact & targetContact()
  {
    return plan_.targetContact();
  }

  tsid::InverseDynamicsFormulationAccForce& solver(){
    return tsid_;
  }



  void sendRecv()
  {
    _interface->sendRecv(&_lowCmd,& _lowState);
  }

  void internalReset();
  void loadFootstepPlan(FootstepPlan &plan);
  void updateRealFromKinematics();
  void leftFootRatio(double ratio);
  void warnIfRobotIsInTheAir();
  bool updatePreview();
  void pauseWalkingCallback(bool verbose);
  void tsidsolve();


  Biped _biped;
  std::shared_ptr<StateEstimatorContainer> _stateEstimator;
  std::shared_ptr<LegController> _legController;
  DesiredStateCommand *_desiredStateCommand = nullptr;
  std::shared_ptr<IOInterface> _interface;

  LowlevelCmd _lowCmd;
  LowlevelState _lowState;
  std::shared_ptr<MyWrapper> robot_;
  std::shared_ptr<pinocchio::Data> data_;

  tsid::InverseDynamicsFormulationAccForce tsid_;
  std::shared_ptr<tsid::tasks::TaskJointPosture> postureTask_;

  std::shared_ptr<Preview> preview_;

  FootstepPlan plan_;
  Pendulum pendulum_;
  LowPassVelocityFilter<Eigen::Vector3d> comVelFilter_;
  FloatingBaseObserver baseObs_;
  NetWrenchObserver wrenchObs_;
  Stabilizer stabilizer_;
  ModelPredictiveControl mpc_;
  Sole sole_;

  double leftFootRatio_;
  Eigen::Vector3d realComd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d realCom_ = Eigen::Vector3d::Zero();
  unsigned int nbMPCFailures_ = 0;
  Eigen::VectorXd dv;

  bool leftFootRatioJumped_ = true;
  bool pauseWalking = false;
  bool pauseWalkingRequested = false;
  double doubleSupportDurationOverride_ = -1.;
  double timeStep = 0.005; // 需要正确定义 0.005s参考
};


#endif  // CONTROLFSMDATA_H