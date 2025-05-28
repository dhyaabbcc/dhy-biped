#include <iomanip>
#include "../../include/common/ModelPredictiveControl.h"
#include "../../include/common/Preview.h"

namespace
{

constexpr double SAMPLING_PERIOD = ModelPredictiveControl::SAMPLING_PERIOD;
constexpr unsigned INPUT_SIZE = ModelPredictiveControl::INPUT_SIZE;
constexpr unsigned NB_STEPS = ModelPredictiveControl::NB_STEPS;
constexpr unsigned STATE_SIZE = ModelPredictiveControl::STATE_SIZE;

} // namespace

Preview::Preview()
{
  inputTraj_ = Eigen::VectorXd::Zero(NB_STEPS * INPUT_SIZE);
  stateTraj_ = Eigen::VectorXd::Zero((NB_STEPS + 1) * STATE_SIZE);
}

Preview::Preview(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & inputTraj)
{
  if(stateTraj.size() / STATE_SIZE != 1 + inputTraj.size() / INPUT_SIZE)
  {
    std::cout<<"Invalid state/input sizes, respectively " << stateTraj.size() << " and " << inputTraj.size()<<std::endl;
  }
  stateTraj_ = stateTraj;
  inputTraj_ = inputTraj;
}

void Preview::integrate(Pendulum & pendulum, double dt)
{
  if(playbackStep_ < NB_STEPS)
  {
    integratePlayback(pendulum, dt);
  }
  else // (playbackStep_ >= NB_STEPS)
  {
    integratePostPlayback(pendulum, dt);
  }
}

void Preview::integratePlayback(Pendulum & pendulum, double dt)
{
  Eigen::Vector3d comddd;
  comddd.head(INPUT_SIZE) = inputTraj_.segment(INPUT_SIZE * playbackStep_, INPUT_SIZE);
  comddd.z() = 0.;
  playbackTime_ += dt;
  if(playbackTime_ >= (playbackStep_ + 1) * SAMPLING_PERIOD)
  {
    playbackStep_++;
  }
  pendulum.integrateCoMJerk(comddd, dt);
}

void Preview::integratePostPlayback(Pendulum & pendulum, double dt)
{
  Eigen::Vector3d comddd;
  Eigen::VectorXd lastState = stateTraj_.segment(INPUT_SIZE * playbackStep_, INPUT_SIZE);
  Eigen::Vector2d comd_f = lastState.segment<2>(2);
  Eigen::Vector2d comdd_f = lastState.segment<2>(4);
  if(std::abs(comd_f.x() * comdd_f.y() - comd_f.y() * comdd_f.x()) > 1e-4)
  {
    std::cout<<"MPC terminal condition is not properly fulfilled"<<std::endl;
  }
  double omega_f = -comd_f.dot(comdd_f) / comd_f.dot(comd_f);
  double lambda_f = std::pow(omega_f, 2);
  comddd = -omega_f * pendulum.comdd() - lambda_f * pendulum.comd();
  comddd.z() = 0.;
  pendulum.integrateCoMJerk(comddd, dt);
}
