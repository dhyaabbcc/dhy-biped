#pragma once

#include <memory>
#include <string>
#include "../common/ControlFSMData.h"
#include "../common/enumClass.h"


class FSM; // 前向声明

class BaseState
{
public:
  virtual ~BaseState() = default;

  void bindController(FSM *fsm_main_controller, ControlFSMData *controller)
  {
    fsm_ctrl = fsm_main_controller;
    controller_ = controller;
  }

  virtual void start() = 0;
  virtual void runState() = 0;
  virtual bool checkTransitions() = 0;
  virtual void teardown() = 0;

  // 主调度函数：控制器中调用
  bool run()
  {
    // fsm.h中已经做了切换检查和转换
    //  if(checkTransitions())
    //  {
    //    return true; // 请求状态切换
    //  }
    runState();
    return false;
  }

  Pendulum &pendulum()
  {
    return controller_->pendulum();
  }

  FootstepPlan &plan()
  {
    return controller_->plan_;
  }

  Stabilizer &stabilizer()
  {
    return controller_->stabilizer();
  }

  ControlFSMData &controller()
  {
    return *controller_;
  }

  std::string nextStateName_;
  std::string stateName_;
  double PREVIEW_UPDATE_PERIOD = ModelPredictiveControl::SAMPLING_PERIOD;

protected:
  FSM *fsm_ctrl = nullptr;
  ControlFSMData *controller_ = nullptr;
};
