// #include "DoubleSupport.h"

// void DoubleSupport::start()
// {
//   auto & ctl = controller();
//   auto & data_=ctl.data_;
//   auto & robot_=ctl.robot_;

//   double phaseDuration = ctl.doubleSupportDuration(); // careful! side effect here



//   duration_ = phaseDuration;
//   initLeftFootRatio_ = ctl.leftFootRatio();
//   remTime_ = (phaseDuration > ctl.timeStep) ? phaseDuration : -ctl.timeStep;
//   stateTime_ = 0.;
//   stopDuringThisDSP_ = ctl.pauseWalking;
//   if(phaseDuration > ctl.timeStep)
//   {
//     timeSinceLastPreviewUpdate_ = 2 * PREVIEW_UPDATE_PERIOD; // update at transition...
//   }
//   else // ... unless DSP duration is zero
//   {
//     timeSinceLastPreviewUpdate_ = 0.;
//   }

//   const std::string & targetSurfaceName = ctl.targetContact().surfaceName;

//   //q\v从传感器获得
//   //robot_->computeAllTerms(data_,q,v);
//   pinocchio::SE3 actualTargetPose = data_->oMf[robot_->model().getFrameId("targetSurfaceName")];

//   ctl.plan_.goToNextFootstep(actualTargetPose);
//   if(ctl.isLastDSP()) // called after goToNextFootstep
//   {
//     stopDuringThisDSP_ = true;
//   }

//   stabilizer().contactState(ContactState::DoubleSupport);
//   if(ctl.prevContact().surfaceName == "LeftFootCenter")
//   {
//     stabilizer().setContact(stabilizer().contactL, ctl.prevContact(), "lcontactpoint");
//     stabilizer().setContact(stabilizer().contactR, ctl.supportContact(), "rcontactpoint");
//     targetLeftFootRatio_ = 0.;
//   }
//   else // (ctl.prevContact().surfaceName == "RightFootCenter")
//   {
//     stabilizer().setContact(stabilizer().contactL, ctl.supportContact(), "lcontactpoint");
//     stabilizer().setContact(stabilizer().contactR, ctl.prevContact(), "rcontactpoint");
//     targetLeftFootRatio_ = 1.;
//   }
//   if(stopDuringThisDSP_)
//   {
//     targetLeftFootRatio_ = 0.5;
//   }

//   //leftFootInContact等变量在摆动足的状态机中定义
//   stabilizer().removeTasks(ctl.solver());
//   stabilizer().addTasks(ctl.solver());

//   if(stopDuringThisDSP_)
//   {
//     ctl.pauseWalking = false;
//   }

//   runState(); // don't wait till next cycle to update reference and tasks
// }

// void DoubleSupport::teardown()
// {
//   auto & ctl = controller();
//   stabilizer().removeTasks(ctl.solver());
// }

// void DoubleSupport::runState()
// {
//   auto & ctl = controller();
//   double dt = ctl.timeStep;

//   if(remTime_ > 0 && timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD
//      && !(stopDuringThisDSP_ && remTime_ < PREVIEW_UPDATE_PERIOD))
//   {
//     updatePreview();
//   }

//   double x =  MyWrapper::clamp(remTime_ / duration_, 0., 1.);
//   ctl.leftFootRatio(x * initLeftFootRatio_ + (1. - x) * targetLeftFootRatio_);

//   ctl.preview->integrate(pendulum(), dt);
//   pendulum().completeIPM(ctl.prevContact());
//   pendulum().resetCoMHeight(ctl.plan_.comHeight(), ctl.prevContact());
//   stabilizer().run();

//   remTime_ -= dt;
//   stateTime_ += dt;
//   timeSinceLastPreviewUpdate_ += dt;
// }

// bool DoubleSupport::checkTransitions()
// {
//   auto & ctl = controller();
//   if(!stopDuringThisDSP_ && remTime_ < 0.)
//   {
//     nextStateName_ = "SingleSupport";
//     return true;
//   }
//   if(stopDuringThisDSP_ && remTime_ < -0.5)
//   {
//     if(!ctl.isLastDSP())
//     {
//       ctl.plan_.restorePreviousFootstep(); // current one is for next SSP
//     }
//     nextStateName_ = "Standing"; 
//     return true;
//   }
//   return false;
// }