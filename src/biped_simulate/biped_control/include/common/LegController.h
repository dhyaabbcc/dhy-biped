/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Math/orientation_tools.h"
#include "Biped.h"
#include <array>
#include "../logdata/logdata.h"

/*!
 * Data sent from control algorithm to legs
 */
struct LegControllerCommand
{
    LegControllerCommand() { zero(); }

    void zero();

    Vec6<double> qDes, qdDes, tau;
    Vec3<double> pDes, vDes;
    Mat6<double> kpJoint, kdJoint;
    Vec6<double> feedforwardForce;
    Vec3<double> hiptoeforce;
    Mat3<double> kpCartesian;
    Mat3<double> kdCartesian;
    double kptoe;
    double kdtoe;
};

/*!
 * Data returned from legs to control code
 */
struct LegControllerData
{
    LegControllerData() { zero(); }
    void setBiped(Biped &biped) { hector = &biped; }
    void zero();
    Vec6<double> q, qd, qdd;
    // p:base 坐标系下足部位置。
    Vec3<double> p, v;
    Mat66<double> J_force_moment;
    Mat36<double> J_force;
    Vec6<double> tau;
    Biped *hector;
};

/*!
 * 
 */
class LegController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegController(Biped &biped) : _biped(biped)
    {
        for (auto &dat : data)
            dat.setBiped(_biped);
        for (int i = 0; i < 2; i++)
        {
            commands[i].zero();
            data[i].zero();
            Feetpose[i].resize(7, 1);
        }
    };

    void zeroCommand();
    void updateData(const LowlevelState *state);
    void updateCommand(LowlevelCmd *cmd,bool useWBC=0);
    void updateCommandwithKpKd(LowlevelCmd *cmd, Vec6<double> Kp, Vec6<double> Kd);
    //void setEnabled(bool enabled) { _legsEnabled = enabled; };
    void updatePosctrl(LowlevelCmd *cmd);
    void updateTorque(LowlevelCmd *cmd, const Vec6<double> &Rt, const Vec6<double> &Lt);
    void updatePose(LowlevelCmd *cmd, const Vec6<double> &Rt, const Vec6<double> &Lt);

    

    std::array<Vec6<double>, 2> Feettwist;
    /**
     * @brief 直接从geometry msg 获取的feetpose 为rj5,lj5处的位置
     * 
     */
    std::array<Eigen::VectorXd, 2> Feetpose;

    LegControllerCommand commands[2];
    // data:从ros读到的关节数据
    LegControllerData data[2];
    bool _legsEnabled = false;
    //std::string limbName[6] = {"Hip 1", "Hip 2", "Thigh", "Knee ", "ankle ", "feet"};
   // std::string Side[2] = {"Left ", "Right"};
    Biped &_biped;
    Mat33<double> R_foot_L;
    Mat33<double> R_foot_R; // 足部方位矩阵
    DataLogger &logger = DataLogger::GET();
};

void computeLegJacobianAndPosition(Biped &_biped, Vec6<double> &q, Mat66<double> *J_f_m, Mat36<double> *J_f,
                                   Vec3<double> *p, int leg);
void computeIK(Biped &_biped, Vec6<double> &q, Vec3<double> *p, int leg, Mat33<double> R = Mat33<double>::Identity());

void IKinbodyframe(Biped &_biped, Vec6<double> &q, Vec3<double> *p, int leg, Mat33<double> R = Mat33<double>::Identity());

template <typename T>
float sign(T num)
{
    if (num > 0)
        return 1;
    else if (num < 0)
        return -1;
    else
        return 0;
}

#endif