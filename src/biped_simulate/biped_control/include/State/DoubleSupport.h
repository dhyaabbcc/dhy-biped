
#pragma once

#include "State.h"

struct DoubleSupport : BaseState
{
  /** Start state.
   *
   */
  void start() override;

  /** Teardown state.
   *
   */
  void teardown() override;

  /** Check transitions at beginning of control cycle.
   *
   */
  bool checkTransitions() override;

  /** Main state function, called if no transition at this cycle.
   *
   */
  void runState() override;

  /** Update horizontal MPC preview.
   *
   */
  void updatePreview();

private:
  bool stopDuringThisDSP_; /**< Stop walking during this DSP */
  double duration_; /**< Total duration of the DSP in [s] */
  double initLeftFootRatio_; /**< Left foot ratio at the beginning of DSP */
  double remTime_; /**< Time remaining until the end of the phase */
  double stateTime_; /**< Time since the beginning of the DSP */
  double targetLeftFootRatio_; /**< Left foot ratio at the end of DSP */
  double timeSinceLastPreviewUpdate_; /**< Time count used to schedule MPC updates, in [s] */
};



