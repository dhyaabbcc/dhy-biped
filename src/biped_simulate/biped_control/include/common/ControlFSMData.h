#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "DesiredCommand.h"
#include "LegController.h"
#include "Biped.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "../interface/IOInterface.h"
#include "StateEstimatorContainer.h"
#include <memory>
#include "FootstepPlanData.hpp"
//#include "../robotwrapper/robotwrapper.h"

struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Biped* _biped;
  
  std::shared_ptr<StateEstimatorContainer> _stateEstimator;

  std::shared_ptr<LegController> _legController;


  DesiredStateCommand* _desiredStateCommand;

  std::shared_ptr<IOInterface> _interface;

  LowlevelCmd* _lowCmd;

  LowlevelState* _lowState;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
  }

  void loadFootstepPlan(FootstepPlanner & plan);
  void updateRealFromKinematics();
  void resetController(MyWrapper &robot,
                       tsid::InverseDynamicsFormulationAccForce &tsid,
                       pinocchio::Model &model,
                       pinocchio::Data &data,
                       StateEstimator &estimator,
                       StateEstimatorContainer &estimatorcontainer,
                       FootstepPlanner &planner,
                       PendulumModel &pendulum,
                       CoMVelocityFilter &comVelFilter,
                       FloatingBaseObserver &baseObs,
                       NetWrenchObserver &wrenchObs,
                       Stabilizer &stabilizer,
                       LowlevelState &lowState,
                       ModelPredictiveControl &mpc,
                       Preview* preview,
                       double leftFootRatio = 0.5)
      : robot_(robot), tsid_(tsid), model_(model), data_(data), _stateEstimator(estimatorcontainer), estimator_(estimator), planner_(planner), pendulum_(pendulum),
        comVelFilter_(comVelFilter), baseObs_(baseObs), wrenchObs_(wrenchObs), stabilizer_(stabilizer), _lowState(lowState), mpc_(mpc),preview_(preview),leftFootRatio_(leftFootRatio)
  {
  }

    bool leftFootRatioJumped_;
    bool pauseWalking;
    bool pauseWalkingRequested;
    int nbMPCFailures_;
    void leftFootRatio(double ratio);
    void warnIfRobotIsInTheAir();



    MyWrapper* robot_;
    tsid::InverseDynamicsFormulationAccForce* tsid_;
    pinocchio::Model* model_;
    pinocchio::Data* data_;
    StateEstimator* estimator_;
    FootstepPlanner* planner_;
    PendulumModel* pendulum_;
    CoMVelocityFilter* comVelFilter_;
    FloatingBaseObserver* baseObs_;
    NetWrenchObserver* wrenchObs_;
    Stabilizer* stabilizer_;
    ModelPredictiveControl* mpc_;
    Preview* preview_;
    double leftFootRatio_;
    Eigen::Vector3d realComd_=Eigen::Vector3d::Zero();
    Eigen::Vector3d realCom_=Eigen::Vector3d::Zero();
     unsigned nbMPCFailures_ = 0;
};


#endif  // CONTROLFSM_H