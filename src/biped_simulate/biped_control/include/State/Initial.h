#pragma once

#include "State.h"

struct Initial : BaseState
{

  void start() override;

  void teardown() override;

  bool checkTransitions() override;

  void runState() override;

  void showStartStandingButton();

  void hideStartStandingButton();

private:
  bool postureTaskIsActive_;
  bool postureTaskWasActive_;
  bool startStandingButton_;
  bool startStanding_;
};
