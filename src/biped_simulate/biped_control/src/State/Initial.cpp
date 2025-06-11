#include "../../include/State/Initial.h"
#include "../../include/State/FSM.h"

void Initial::start()
{
  auto &ctl = controller();
  std::cout << "ctl_set" << std::endl;
  ctl._interface->zeroCmdPanel();
  std::cout << "_interface" << std::endl;
  ctl._stateEstimator->run();
  std::cout << "_stateEstimator" << std::endl;
  ctl._legController->zeroCommand();
  std::cout << "_legController" << std::endl;


  postureTaskIsActive_ = false;
  postureTaskWasActive_ = false;
  startStandingButton_ = false;
  startStanding_ = false;

  stateName_ = "Initial";

  ctl.internalReset();
  std::cout << "internalReset_end" << std::endl;

  // runState(); // don't wait till next cycle to update reference and tasks
}

void Initial::teardown()
{
}

void Initial::runState()
{
  auto &ctl = controller();
  ctl._legController->updateData(&ctl._lowState);
  ctl._stateEstimator->run();

  postureTaskIsActive_ = (ctl.postureTask_->velocity().norm() > 1e-2);
  if (postureTaskIsActive_)
  {
    postureTaskWasActive_ = true;
  }
  else if (postureTaskWasActive_)
  {
    ctl.internalReset();
    postureTaskWasActive_ = false;
  }
  else
  {
    startStanding_ = true;
  }
  ctl._legController->updateCommand(&ctl._lowCmd);
  std::cout << "ctl._legController->updateCommand_end" << std::endl;
  std::cout << startStanding_ << std::endl;
  std::cout << postureTaskIsActive_ << std::endl;
}

bool Initial::checkTransitions()
{
  nextStateName_ = "Initial";
  if (startStanding_ && !postureTaskIsActive_)
  {
    nextStateName_ = "Standing";
    return true;
  }
  return false;
}
