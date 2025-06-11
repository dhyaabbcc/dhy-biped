#include "State.h"
#include "Initial.h"
#include "Standing.h"

class FSM
{
public:
  FSM(ControlFSMData *controller) : controller_(controller)
  {
    // 构造状态对象，并绑定 controller 指针
    std::cout << "Initializing FSM..." << std::endl;
    Initial_ = std::make_shared<Initial>();
    Standing_ = std::make_shared<Standing>();
    // doubleSupportState_ = std::make_shared<DoubleSupportState>();
    // singleSupportState_ = std::make_shared<SingleSupportState>();

    std::cout << "Constructed state objects" << std::endl;
    Initial_->bindController(this, controller_);
    Standing_->bindController(this, controller_);
    // doubleSupportState_->bindController(this);
    // singleSupportState_->bindController(this);
    std::cout << "Bound controllers" << std::endl;
    currentState_ = Initial_;
    currentState_->start();
    nextState_ = currentState_;
    mode_ = FSMMode::NORMAL;
  }

  void run()
  {
    controller_->sendRecv();
    if (mode_ == FSMMode::NORMAL)
    {
      currentState_->run();
      currentState_->checkTransitions();
      nextStateName_ = currentState_->nextStateName_;
      if (nextStateName_ != currentState_->stateName_)
      {
        mode_ = FSMMode::CHANGE;
        nextState_ = getNextState(nextStateName_);
      }
      std::cout << "FSMMode::NORMAL" << std::endl;
    }
    else if (mode_ == FSMMode::CHANGE)
    {
      std::cout << "change state" << std::endl;
      currentState_->teardown();
      currentState_ = nextState_;
      currentState_->start();
      mode_ = FSMMode::NORMAL;
      currentState_->run();
      std::cout << "FSMMode::CHANGE" << std::endl;
    }
  }

  std::shared_ptr<BaseState> getNextState(const std::string &stateName)
  {
    if (stateName == "Initial")
    {
      return Initial_;
    }
    else if (stateName == "Standing")
    {
      return Standing_;
    }
    // else if (stateName == "singleSupportState") {
    //     return singleSupportState_;
    // }
    // else if (stateName == "doubleSupportState") {
    //     return doubleSupportState_;
    // }
    else
    {
      return Initial_; // 默认返回 Initial 状态
    }
  }

  double dt() const { return 0.002; }

private:
  ControlFSMData *controller_;

private:
  std::shared_ptr<BaseState> currentState_;
  std::shared_ptr<BaseState> nextState_;
  // std::shared_ptr<DoubleSupportState> doubleSupportState_;
  // std::shared_ptr<SingleSupportState> singleSupportState_;
  std::shared_ptr<Initial> Initial_;
  std::shared_ptr<Standing> Standing_;
  FSMMode mode_;
  std::string stateName_;
  std::string nextStateName_;
};
