#pragma once

#include "Pendulum.h"

struct Preview
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preview();

  Preview(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & inputTraj);

  void integrate(Pendulum & state, double dt);

  void integratePlayback(Pendulum & state, double dt);

  void integratePostPlayback(Pendulum & state, double dt);

  unsigned playbackStep() const
  {
    return playbackStep_;
  }

  double playbackTime() const
  {
    return playbackTime_;
  }

private:
  Eigen::VectorXd inputTraj_; /**< Stacked vector of input trajectory */
  Eigen::VectorXd stateTraj_; /**< Stacked vector of state trajectory */
  double playbackTime_ = 0.; /**< Current time in the preview window */
  unsigned playbackStep_ = 0; /**< Current step index in the preview window */
};

