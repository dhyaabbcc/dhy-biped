#include "../../include/common/SwingFoot.h"

void SwingFoot::reset(const pinocchio::SE3 & initPose,
                      const pinocchio::SE3 & targetPose,
                      double duration,
                      double height)
{
  assert(0. <= takeoffDuration_ && takeoffDuration_ <= 0.5 * duration);
  assert(0. <= landingDuration_ && landingDuration_ <= 0.5 * duration);

  accel_.setZero();
  aerialStart_ = takeoffDuration_;
  duration_ = duration;
  height_ = height;
  initPose_ = initPose;
  ori_ = initPose.rotation();
  playback_ = 0.;
  pos_ = initPose.translation();
  targetPose_ = targetPose;
  vel_.setZero();

  const Eigen::Vector3d & initPos = initPose.translation();
  const Eigen::Vector3d & targetPos = targetPose.translation();
  double aerialDuration = duration - takeoffDuration_ - landingDuration_;
  double halfDuration = duration / 2;

  Eigen::Vector3d aerialStartPos = initPos + takeoffOffset_;
  xyTakeoffChunk_.reset(initPos.head<2>(), aerialStartPos.head<2>(), takeoffDuration_);
  xyAerialChunk_.reset(aerialStartPos.head<2>(), targetPos.head<2>(), aerialDuration);

  Eigen::Vector3d airPos = 0.5 * (initPos + targetPos);
  airPos.z() = std::min(initPos.z(), targetPos.z()) + height;
  zFirstChunk_.reset(initPos.z(), airPos.z(), halfDuration);
  zSecondChunk_.reset(airPos.z(), targetPos.z(), halfDuration);

  pitchTakeoffChunk_.reset(0., takeoffPitch_, takeoffDuration_);
  pitchAerialChunk1_.reset(takeoffPitch_, 0., aerialDuration / 2);
  pitchAerialChunk2_.reset(0., landingPitch_, aerialDuration / 2);
  pitchLandingChunk_.reset(landingPitch_, 0., landingDuration_);

  updatePose(0.);
}

void SwingFoot::integrate(double dt)
{
  playback_ += dt;
  updatePose(playback_);
}

void SwingFoot::updatePose(double t)
{
  updateZ(t);
  updateXY(t);
  updatePitch(t);
}

void SwingFoot::updateZ(double t)
{
  double t1 = t;
  double t2 = t - zFirstChunk_.duration();
  if(t1 <= zFirstChunk_.duration())
  {
    pos_.z() = zFirstChunk_.pos(t1);
    vel_.z() = zFirstChunk_.vel(t1);
    accel_.z() = zFirstChunk_.accel(t1);
  }
  else
  {
    pos_.z() = zSecondChunk_.pos(t2);
    vel_.z() = zSecondChunk_.vel(t2);
    accel_.z() = zSecondChunk_.accel(t2);
  }
}

void SwingFoot::updateXY(double t)
{
  double t1 = t;
  double t2 = t1 - xyTakeoffChunk_.duration();
  if(t1 <= xyTakeoffChunk_.duration())
  {
    pos_.head<2>() = xyTakeoffChunk_.pos(t1);
    vel_.head<2>() = xyTakeoffChunk_.vel(t1);
    accel_.head<2>() = xyTakeoffChunk_.accel(t1);
  }
  else
  {
    pos_.head<2>() = xyAerialChunk_.pos(t2);
    vel_.head<2>() = xyAerialChunk_.vel(t2);
    accel_.head<2>() = xyAerialChunk_.accel(t2);
  }
}

void SwingFoot::updatePitch(double t)
{
  Eigen::Matrix3d baseOri;
  double t1 = t;
  double t2 = t1 - pitchTakeoffChunk_.duration();
  double t3 = t2 - pitchAerialChunk1_.duration();
  double t4 = t3 - pitchAerialChunk2_.duration();

  double takeoffDuration = pitchTakeoffChunk_.duration();
  double aerialDuration = xyAerialChunk_.duration();

  if(t1 <= takeoffDuration)
  {
    baseOri = initPose_.rotation();
  }
  else if(t2 <= aerialDuration)
  {
    double s = xyAerialChunk_.s(t2);
    //显式处理slerp
    Eigen::Quaterniond q_init(initPose_.rotation());
    Eigen::Quaterniond q_target(targetPose_.rotation());
    q_init.normalize(); 
    q_target.normalize(); 
    Eigen::Quaterniond q_result = q_init.slerp(s, q_target);
    baseOri = q_result.toRotationMatrix();
  }
  else
  {
    baseOri = targetPose_.rotation();
  }

  if(t1 <= pitchTakeoffChunk_.duration())
  {
    pitch_ = pitchTakeoffChunk_.pos(t1);
  }
  else if(t2 <= pitchAerialChunk1_.duration())
  {
    pitch_ = pitchAerialChunk1_.pos(t2);
  }
  else if(t3 <= pitchAerialChunk2_.duration())
  {
    pitch_ = pitchAerialChunk2_.pos(t3);
  }
  else
  {
    pitch_ = pitchLandingChunk_.pos(t4);
  }

  Eigen::Matrix3d pitchRot;
  pitchRot = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX())   // roll = 0
         * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()) // pitch = pitch_
         * Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ());   // yaw = 0
  ori_ = pitchRot * baseOri;
}
