#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>
#include "Utilities/polynomials.h"


struct SwingFoot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void integrate(double dt);

  void reset(const pinocchio::SE3 & initPose,
             const pinocchio::SE3 & targetPose,
             double duration,
             double height);

  Eigen::Matrix<double, 6, 1> accel() const
  {
    Eigen::Matrix<double, 6, 1> a;
    a.head<3>().setZero();
    a.tail<3>() = accel_;
    return a;
  }

  double height() const
  {
    return pos_.z() - initPose_.translation().z();
  }

  void landingDuration(double duration)
  {
    landingDuration_ = duration;
  }

  void landingPitch(double pitch)
  {
    landingPitch_ = pitch;
  }

  pinocchio::SE3 pose() const
  {
    return pinocchio::SE3(ori_, pos_);
  }

  double remTime() const
  {
    return (duration_ - playback_);
  }

  void takeoffDuration(double duration)
  {
    takeoffDuration_ = duration;
  }

  void takeoffOffset(const Eigen::Vector3d & offset)
  {
    takeoffOffset_ = offset;
  }

  void takeoffPitch(double pitch)
  {
    takeoffPitch_ = pitch;
  }

  Eigen::Matrix<double, 6, 1> vel() const
  {
    Eigen::Matrix<double, 6, 1> v;
    v.head<3>().setZero();
    v.tail<3>() = vel_;
    return v;
  }

private:
  void updatePose(double t);
  void updateZ(double t);
  void updateXY(double t);
  void updatePitch(double t);

private:
  Eigen::Quaterniond ori_;
  Eigen::Vector3d accel_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d takeoffOffset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_;
  RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyAerialChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyTakeoffChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk1_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk2_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchLandingChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchTakeoffChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> zFirstChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> zSecondChunk_;
  double aerialStart_ = 0;
  double duration_ = 0;
  double height_ = 0;
  double landingDuration_ = 0;
  double landingPitch_ = 0;
  double pitch_ = 0;
  double playback_ = 0;
  double takeoffDuration_ = 0;
  double takeoffPitch_ = 0;
  pinocchio::SE3 initPose_;
  pinocchio::SE3 targetPose_;
  pinocchio::SE3 touchdownPose_;
};

