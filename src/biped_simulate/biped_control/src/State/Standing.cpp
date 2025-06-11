#include "../../include/State/Standing.h"
#include "../../include/State/FSM.h"

constexpr double COM_STIFFNESS = 5.;


void Standing::start()
{
  std::cout << "Standing::start_begin "<< std::endl;
  auto &ctl = controller();

  ctl._interface->zeroCmdPanel();
  ctl._stateEstimator->run();
  ctl._legController->zeroCommand();

  stateName_ = "Standing";

  auto &supportContact = ctl.supportContact();
  auto &targetContact = ctl.targetContact();

  isMakingFootContact_ = false;
  leftFootRatio_ = ctl.leftFootRatio();
  startWalking_ = false;
  if (supportContact.surfaceName == "rcontactpoint")  
  {
    leftFootContact_ = targetContact;
    rightFootContact_ = supportContact;
  }
  else if (supportContact.surfaceName == "lcontactpoint")
  {
    leftFootContact_ = supportContact;
    rightFootContact_ = targetContact;
  }
  else
  {
    std::cout << "Unknown surface name: " << supportContact.surfaceName << std::endl;
  }

  if (ctl.isLastDSP())
  {
    ctl.loadFootstepPlan(ctl.plan_);
  }

  stabilizer().contactState(ContactState::DoubleSupport);
  stabilizer().setContact(stabilizer().contactL, leftFootContact_, "lcontactpoint");
  stabilizer().setContact(stabilizer().contactR, rightFootContact_, "rcontactpoint");
  stabilizer().leftFootInContact = true;
  stabilizer().rightFootInContact = true;
  stabilizer().addTasks(ctl.solver());

  updateTarget(leftFootRatio_);

  // runState(); // don't wait till next cycle to update reference and tasks
}

void Standing::teardown()
{
  auto &ctl = controller();
  stabilizer().removeTasks(ctl.solver());
}

// pendulum和stabilizer是否绑定了？
void Standing::runState()
{
  std::cout << "Standing::runState_begin "<< std::endl;
  auto &ctl = controller();
  ctl._legController->updateData(&ctl._lowState);
  ctl._stateEstimator->run();
  auto &pendulum = ctl.pendulum();

  Eigen::Vector3d comTarget = copTarget_ + Eigen::Vector3d{0., 0., ctl.plan_.comHeight()};
  const Eigen::Vector3d &com_i = pendulum.com();
  const Eigen::Vector3d &comd_i = pendulum.comd();
  const Eigen::Vector3d &cop_f = copTarget_;

  double K = COM_STIFFNESS;
  double D = 2 * std::sqrt(K);
  Eigen::Vector3d comdd = K * (comTarget - com_i) - D * comd_i;
  Eigen::Vector3d n = ctl.supportContact().normal();
  double lambda = n.dot(comdd - gravity_) / n.dot(com_i - cop_f);
  Eigen::Vector3d zmp = com_i + (gravity_ - comdd) / lambda;

  pendulum.integrateIPM(zmp, lambda, ctl.timeStep);
  ctl.leftFootRatio(leftFootRatio_);
  ctl.stabilizer().run();
  ctl.tsidsolve();

  ctl._legController->updateCommand(&ctl._lowCmd);
}

void Standing::updateTarget(double leftFootRatio)
{
  auto &sole = controller().sole();
  if (controller().stabilizer().contactState() != ContactState::DoubleSupport)
  {
    std::cout << "Cannot update CoM target while in single support" << std::endl;
    return;
  }
  leftFootRatio = MyWrapper::clamp(leftFootRatio, 0., 1.);
  pinocchio::SE3 X_0_lfr = MyWrapper::interpolate(rightFootContact_.anklePose(sole), leftFootContact_.anklePose(sole), leftFootRatio);
  copTarget_ = X_0_lfr.translation();
  leftFootRatio_ = leftFootRatio;
}

bool Standing::checkTransitions()
{
  auto &ctl = controller();
  nextStateName_ = "Standing";

  if (isMakingFootContact_ || !startWalking_)
  {
    return false;
  }

  ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
  ctl.mpc().phaseDurations(0., ctl.plan_.initDSPDuration(), ctl.singleSupportDuration());
  if (ctl.updatePreview())
  {
    ctl.nextDoubleSupportDuration(ctl.plan_.initDSPDuration());
    //nextStateName_ = "DoubleSupport";
    nextStateName_ = "Standing";
    return true;
  }
  return false;
}

void Standing::startWalking()
{
  startWalking_ = true;
}
