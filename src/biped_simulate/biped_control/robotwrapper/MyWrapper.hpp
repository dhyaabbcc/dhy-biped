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

#include <pinocchio/spatial/fwd.hpp>

#include "../include/common/cppTypes.h"

#include <Eigen/Core>

 
class MyWrapper : public tsid::robots::RobotWrapper {
public:
    // 继承基类所有构造函数 
    using RobotWrapper::RobotWrapper; 
 
    // 扩展的新功能 
    void computeMainTerms(pinocchio::Data& data, 
                        const Eigen::VectorXd& q,
                        const Eigen::VectorXd& v) const;
                        

    template<typename T>
    static Eigen::Matrix<typename T::Scalar, 4, 1> rpytoquat(const Eigen::MatrixBase<T> &a) {
        // 使用Pinocchio将RPY角转换为旋转矩阵 
        Mat3<typename T::Scalar> rotationMatrix = pinocchio::rpy::rpyToMatrix(a[0], a[1], a[2]);
        
        // 从旋转矩阵生成四元数 
        Eigen::Quaternion<typename T::Scalar> quat(rotationMatrix);
        
        // 返回四元数（格式：[x, y, z, w]）
        Eigen::Matrix<typename T::Scalar, 4, 1> returnmatrix;
        returnmatrix << quat.x(), quat.y(), quat.z(), quat.w();
        return returnmatrix;
        //调用：Eigen::Vector4d quat = MyWrapper::rpytoquat(rpy);
    }

    //输出的是 XYZ轴顺序和Pinocchio和eign相反（输入的也需要是xyz形式：RotX(r) * RotY(p) * RotZ(y)）
    static Eigen::Vector3d rpyFromMat(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d rpy;
        
        // 检查奇异情况（pitch = ±π/2）
        if(std::abs(R(2,0)) > 0.9999) {
            // 处理奇异情况 
            rpy[0] = 0; // roll可以设为0 
            rpy[1] = (R(2,0) > 0) ? M_PI/2 : -M_PI/2; // pitch = ±π/2 
            rpy[2] = std::atan2(-R(0,1), R(1,1)); // yaw 
        }
        else {
            // 常规情况 
            rpy[0] = std::atan2(R(2,1), R(2,2));  // roll (X轴)
            rpy[1] = -std::asin(R(2,0));          // pitch (Y轴)
            rpy[2] = std::atan2(R(1,0), R(0,0));  // yaw (Z轴)
        }
        
        return rpy;
    }

    //clamp限制范围
    template<typename T>
    static const T& clamp(const T& value, const T& low, const T& high) {
        return (value < low) ? low : (high < value) ? high : value;
    }


    // 插值两个 SE3 变换
    static pinocchio::SE3 interpolate(const pinocchio::SE3& a,
                                      const pinocchio::SE3& b,
                                      double u) {
        // 插值 translation（线性插值）
        Eigen::Vector3d trans = (1.0 - u) * a.translation() + u * b.translation();

        // 插值 rotation（球面线性插值）
        Eigen::Quaterniond qa(a.rotation());
        Eigen::Quaterniond qb(b.rotation());
        Eigen::Quaterniond q_interp = qa.slerp(u, qb);

        return pinocchio::SE3(q_interp.toRotationMatrix(), trans);
    }

};