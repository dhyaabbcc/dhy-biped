#include "../../include/common/NetWrenchObserver.h"
#include <iostream>


NetWrenchObserver::NetWrenchObserver()
: sensorNames_({"RightFootForceSensor", "LeftFootForceSensor"}) {}

NetWrenchObserver::NetWrenchObserver(const std::vector<std::string> & sensorNames)
: sensorNames_(sensorNames) {}

void NetWrenchObserver::update(const LowlevelState & state, const Contact & contact)
{
  updateNetWrench(state);
  updateNetZMP(contact);
}

void NetWrenchObserver::updateNetWrench(const LowlevelState & state)
{
  netWrench_ = pinocchio::Force::Zero();

  for (const auto & name : sensorNames_)
  {
    if (name == "RightFootForceSensor")
    {
      Eigen::Vector3d f(state.feettwist[0], state.feettwist[1], state.feettwist[2]);
      Eigen::Vector3d tau(state.feettwist[3], state.feettwist[4], state.feettwist[5]);
      if (f.norm() > 1.)
      {
        netWrench_ += pinocchio::Force(f, tau);
      }
    }
    else if (name == "LeftFootForceSensor")
    {
      Eigen::Vector3d f(state.feettwist[6], state.feettwist[7], state.feettwist[8]);
      Eigen::Vector3d tau(state.feettwist[9], state.feettwist[10], state.feettwist[11]);
      if (f.norm() > 1.)
      {
        netWrench_ += pinocchio::Force(f, tau);
      }
    }
  }
}

//根据合力、和力矩、接触点位置，计算zmp
void NetWrenchObserver::updateNetZMP(const Contact & contact)
{
  const Eigen::Vector3d & force = netWrench_.linear();
  const Eigen::Vector3d & moment_0 = netWrench_.angular();
  Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);

  if (force.squaredNorm() > 42.) 
  {
    netZMP_ = contact.p() + contact.normal().cross(moment_p) / contact.normal().dot(force);
  }
}

