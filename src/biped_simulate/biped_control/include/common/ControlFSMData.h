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
};


#endif  // CONTROLFSM_H