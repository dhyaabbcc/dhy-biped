#pragma once
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>

struct SwingPlan {
  Eigen::Vector3d takeoff_offset = Eigen::Vector3d::Zero();
  double takeoff_pitch = 0.0;
  double height = 0.0; // 可选参数 
};
 
struct ContactPlan {
  pinocchio::SE3 pose;
  std::string surface;
  SwingPlan swing; // 可选字段 
  
  // 构造函数 
  ContactPlan(const pinocchio::SE3& p, const std::string& s, 
             const SwingPlan& sw = SwingPlan{})
    : pose(p), surface(s), swing(sw) {}
};
 
inline std::vector<ContactPlan> loadSimpleFootstepPlan() {
  return {
    //站立(可能需要)
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(-0.04, 0.09, 0.0)), 
     "lcontactpoint"},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(-0.04, -0.09, 0.0)), 
     "rcontactpoint"},

    // 初始接触 
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(-0.04, 0.09, 0.0)), 
     "lcontactpoint"},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(-0.04, -0.09, 0.0)), 
     "rcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.03, 0.0, 0.0), 0.6}},
    
    // 第一步 
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.24, 0.09, 0.185)), 
     "lcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.02, 0.0, 0.0)}},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.24, -0.09, 0.185)), 
     "rcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.03, 0.0, 0.0), 0.6}},
    
    // 第二步  
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.48, 0.09, 0.370)), 
     "lcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.02, 0.0, 0.0)}},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.48, -0.09, 0.370)), 
     "rcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.03, 0.0, 0.0), 0.6}},
    
    // 第三步 
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.72, 0.09, 0.555)), 
     "lcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.02, 0.0, 0.0)}},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.72, -0.09, 0.555)), 
     "rcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.03, 0.0, 0.0), 0.6}},
    
    // 第四步（带高度参数）
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.96, 0.09, 0.740)), 
     "lcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.02, 0.0, 0.0), 0.0, 0.2}},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(0.96, -0.09, 0.740)), 
     "rcontactpoint",
     SwingPlan{Eigen::Vector3d(-0.03, 0.0, 0.0), 0.6, 0.2}},
    
    // 最终接触 
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(1.20, 0.09, 0.885)), 
     "lcontactpoint"},
     
    {pinocchio::SE3(Eigen::Matrix3d::Identity(), 
                   Eigen::Vector3d(1.20, -0.09, 0.885)), 
     "rcontactpoint"}
  };
}