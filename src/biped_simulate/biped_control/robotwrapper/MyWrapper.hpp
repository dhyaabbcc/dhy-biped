#pragma once 
#include <tsid/robots/robot-wrapper.hpp> 

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
 
class MyWrapper : public tsid::robots::RobotWrapper {
public:
    // 继承基类所有构造函数 
    using RobotWrapper::RobotWrapper; 
 
    // 扩展的新功能 
    void computeMainTerms(pinocchio::Data& data, 
                        const Eigen::VectorXd& q,
                        const Eigen::VectorXd& v) const;
};