#include "../../include/common/FootstepPlan.h"

pinocchio::SE3 makeHorizontal(pinocchio::SE3 pose)
{
  const Eigen::Matrix3d R = pose.rotation();
  const Eigen::Vector3d p = pose.translation();
  Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R); //从R = RotZ(yaw) * RotY(pitch) * RotX(roll)，中获得RPY
  Eigen::Matrix3d yawRot = pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(0.0, 0.0, rpy[2]));
  return pinocchio::SE3(yawRot, Eigen::Vector3d(p.x(), p.y(), 0.0));
}

void FootstepPlan::load()
{
    std::vector<ContactPlan> contactPlans = loadSimpleFootstepPlan();
    contacts_.clear();
    for (const auto &plan : contactPlans)
    {
        Contact contact(plan.pose);
        contact.surfaceName = plan.surface;
        contact.swing = plan.swing;
        contacts_.emplace_back(contact);
    }

  comHeight_ = 0.8; // [m]
  doubleSupportDuration_ = 0.2; // [s]
  finalDSPDuration_ = 0.6; // [s]
  initDSPDuration_ = 0.6; // [s]
  landingDuration_ = 0.15; // [s]
  landingPitch_ = 0.;
  singleSupportDuration_ = 0.8; // [s]
  swingHeight_ = 0.04; // [m]
  takeoffDuration_ = 0.05; // [s]
  takeoffPitch_ = 0.;
}

void FootstepPlan::complete(const Sole & sole)
{
  for(unsigned i = 0; i < contacts_.size(); i++)
  {
    auto & contact = contacts_[i];
    contact.id = i;
    if(contact.halfLength < 1e-4)
    {
      contact.halfLength = sole.halfLength;
    }
    if(contact.halfWidth < 1e-4)
    {
      contact.halfWidth = sole.halfWidth;
    }
    if(contact.surfaceName.length() < 1)
    {
      std::cout<<"Footstep plan has no surface name for contact " << i<<std::endl;
    }
  }
}

void FootstepPlan::reset(unsigned startIndex)
{
  nextFootstep_ = startIndex + 1;
  supportContact_ = contacts_[startIndex > 0 ? startIndex - 1 : 0];
  targetContact_ = contacts_[startIndex];
  goToNextFootstep();
}

void FootstepPlan::goToNextFootstep()
{
  prevContact_ = supportContact_;
  supportContact_ = targetContact_;
  unsigned targetFootstep = nextFootstep_++;
  targetContact_ = (targetFootstep < contacts_.size()) ? contacts_[targetFootstep] : prevContact_;
  nextContact_ = (nextFootstep_ < contacts_.size()) ? contacts_[nextFootstep_] : supportContact_;
}

void FootstepPlan::goToNextFootstep(const pinocchio::SE3 & actualTargetPose)
{
  assert(nextFootstep_ >= 1);
  pinocchio::SE3 targetInv = targetContact_.pose.inverse();
  pinocchio::SE3 poseDrift = actualTargetPose * targetInv;

  const Eigen::Vector3d & posDrift = poseDrift.translation();
  pinocchio::SE3 xyDrift(
    Eigen::Matrix3d::Identity(), 
    Eigen::Vector3d(posDrift.x(), posDrift.y(), 0.0) 
  );
  for(unsigned i = nextFootstep_ - 1; i < contacts_.size(); i++)
  {
    contacts_[i] = xyDrift * contacts_[i];
  }
  targetContact_.pose = xyDrift * targetContact_.pose;
  goToNextFootstep();
}

void FootstepPlan::restorePreviousFootstep()
{
  nextContact_ = targetContact_;
  targetContact_ = supportContact_;
  supportContact_ = prevContact_;
  nextFootstep_--;
  if(nextFootstep_ >= contacts_.size())
  {
    // at goToNextFootstep(), targetContact_ will copy prevContact_
    prevContact_ = nextContact_;
  }
}

void FootstepPlan::updateInitialTransform(const pinocchio::SE3 & X_0_lf,
                                          const pinocchio::SE3 & X_0_rf,
                                          double initHeight)
{
  pinocchio::SE3 X_0_mid = MyWrapper::interpolate(X_0_lf, X_0_rf, 0.5);
  pinocchio::SE3 X_0_old = MyWrapper::interpolate(contacts_[0].pose, contacts_[1].pose, 0.5);
  pinocchio::SE3 X_delta = makeHorizontal(X_0_old.inverse() * X_0_mid);

  // BUG here: https://github.com/jrl-umi3218/lipm_walking_controller/issues/37
  // This function should but does not update the reference velocity (refVel)
  // for each contact. Thanks to @Saeed-Mansouri for pointing out this bug. The
  // velocity used by \ref ModelPredictiveControl::updateVelCost() is then in
  // the wrong frame.

  for(unsigned i = 2; i < contacts_.size(); i++)
  {
    // X_0_nc = X_old_c X_0_new = X_0_c X_old_0 X_0_new
    const pinocchio::SE3 & X_0_c = contacts_[i].pose;
    contacts_[i].pose = X_0_c * X_delta;
  }
  if(contacts_[0].surfaceName == "lcontactpoint" && contacts_[1].surfaceName == "rcontactpoint")
  {
    contacts_[0].pose = makeHorizontal(X_0_lf);
    contacts_[1].pose = makeHorizontal(X_0_rf);
  }
  else if(contacts_[0].surfaceName == "rcontactpoint" && contacts_[1].surfaceName == "lcontactpoint")
  {
    contacts_[0].pose = makeHorizontal(X_0_rf);
    contacts_[1].pose = makeHorizontal(X_0_lf);
  }
  else
  {
    std::cout<<"Invalid footstep plan: initial surfaces are \"" << contacts_[0].surfaceName << "\" and \""
                                                               << contacts_[1].surfaceName << "\""<<std::endl;
  }
  pinocchio::SE3 X_0_rise(
    Eigen::Matrix3d::Identity(), 
    Eigen::Vector3d{0., 0., initHeight} 
  );

  for(unsigned i = 0; i < contacts_.size(); i++)
  {
    contacts_[i].pose = contacts_[i].pose * X_0_rise;
  }
  X_0_init_ = X_delta * X_0_rise;
}
