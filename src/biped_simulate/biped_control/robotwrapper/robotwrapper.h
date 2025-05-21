
#ifndef robot_wrapper_hpp__
#define robot_wrapper_hpp__


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/math/rpy.hpp>
#include "ufwd.hpp"
#include "common/cppTypes.h"

namespace pino = pinocchio;


class RobotWrapper
{
public:
  typedef pinocchio::Model Model;
  typedef pinocchio::Data Data;
  typedef pinocchio::Motion Motion;
  typedef pinocchio::Frame Frame;
  typedef pinocchio::SE3 SE3;
  typedef math::Vector Vector;
  typedef math::Vector3 Vector3;
  typedef math::Vector6 Vector6;
  typedef math::Matrix Matrix;
  typedef math::Matrix3x Matrix3x;
  typedef double Scalar;
  Motion frameClassicAcceleration(const Data &data,
                                  const Model::FrameIndex index) const;
  RobotWrapper(const std::string &filename,
               bool verbose = false);

  RobotWrapper(const std::string &filename,
               const pinocchio::JointModelVariant &rootJoint,
               bool verbose = false);

  void init();

  const Vector3& com(const Data& data) const;
  const Vector3& com_vel(const Data& data) const;
  const Vector3& com_acc(const Data& data) const;

  const SE3& position(const Data& data, const Model::JointIndex index) const;
  const Motion& velocity(const Data& data, const Model::JointIndex index) const;
  const Motion& acceleration(const Data& data, const Model::JointIndex index) const;

  const Matrix& mass(const Data& data);
  const Vector& nonLinearEffects(const Data& data) const;

  void jacobianWorld(const Data& data, const Model::JointIndex index, Data::Matrix6x& J) const;
  void jacobianLocal(const Data& data,const Model::JointIndex index,Data::Matrix6x& J) const;
  void jacobianWorld_Aligned(const Data& data,const Model::JointIndex index,Data::Matrix6x& J) const;

  SE3 framePosition(const Data& data,const Model::FrameIndex index) const;
  Motion frameVelocity(const Data& data,const Model::FrameIndex index) const;
  Motion frameAcceleration(const Data& data, const Model::FrameIndex index) const;

  
  template<typename T>
  static Quat<typename T::Scalar> rpytoquat(const Eigen::MatrixBase<T> &a)
  {
  //static_assert(std::is_floating_point<T>::value, "must use floating point value");
  Mat3<typename T::Scalar> rotationMatrix = pinocchio::rpy::rpyToMatrix(a[0], a[1], a[2]);
 // std::cout<<"RO matrix !!!:"<<rotationMatrix<<std::endl;
  Eigen::Quaternion<typename T::Scalar> quat(rotationMatrix);
  //std::cout << "Quaternion: (w, x, y, z) = (" << quat.w() << ", " <<quat.x() << ", " << quat.y() << ", " << quat.z() << ")" << std::endl;
  Quat<typename T::Scalar> returnmatrix;
  returnmatrix<<quat.x(),quat.y(),quat.z(),quat.w();
  return returnmatrix;
  }
 
  pino::Model m_model;
  std::string m_model_filename;
  int m_nq_actuated; /// dimension of the configuration space of the actuated
                     /// DoF (nq for fixed-based, nq-7 for floating-base
                     /// robots)
  int m_na;          /// number of actuators (nv for fixed-based, nv-6 for floating-base
                     /// robots)
  bool m_is_fixed_base;
  bool m_verbose;
  bool Driftcalculate_update=false;
  void computeAllTerms(Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v) const;

  void computeMainTerms(Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v ) const;

  void computedrift(Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v);

  Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> m_M;
};

#endif
