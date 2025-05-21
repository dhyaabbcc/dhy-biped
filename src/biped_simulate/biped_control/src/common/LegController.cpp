#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>

// upper level of joint controller
// send data to joint controller

void LegControllerCommand::zero()
{
    tau = Vec6<double>::Zero();
    qDes = Vec6<double>::Zero();
    qdDes = Vec6<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero();
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat6<double>::Zero();
    kdJoint = Mat6<double>::Zero();
    kptoe = 0;
    kdtoe = 0;
}

/*!
 * Zero leg data
 */
void LegControllerData::zero()
{
    q = Vec6<double>::Zero();
    qd = Vec6<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat66<double>::Zero();
    J_force = Mat36<double>::Zero();
    tau = Vec6<double>::Zero();
}

void LegController::zeroCommand()
{
    for (int i = 0; i < 2; i++)
    {
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState *state)
{
    Mat33<double> R[2];
    R[0] = Mat33<double>::Identity();
    R[1] = Mat33<double>::Identity();
    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < 6; j++)
        {
            data[leg].q(j) = state->motorState[leg * 6 + j].q;
            data[leg].qd(j) = state->motorState[leg * 6 + j].dq;
            data[leg].qdd(j) = state->motorState[leg * 6 + j].ddq;
            data[leg].tau(j) = state->motorState[leg * 6 + j].tauEst;

            // logger.jointTorque[leg*6+j]=data[leg].tau(j);
            // logger.jointvel[leg*6+j]=data[leg].qd(j);
            logger.jointpos[leg * 6 + j] = data[leg].q(j) + _biped.Initialq[j];

            Feettwist[leg][j]=state->feettwist[leg*6+j];
            Feetpose[leg][j] = state->feetpose[leg * 7 + j];
            R[leg] *= ori::coordinateRotation(_biped.axisname[j], data[leg].q[j]);
        }
        // std::cout << "关节电机Q" << leg<< ": \n"<< data[leg].q << std::endl;
        R_foot_R = R[0];
        R_foot_L = R[1];
        computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        data[leg].v = data[leg].J_force * data[leg].qd;

        Vec3<double> r_feet(0, 0, -1 * _biped.toeLinkLength);
        Feetpose[leg].block(0, 0, 3, 1) = Feetpose[leg].block(0, 0, 3, 1) + R[leg] * r_feet;
        Feettwist[leg].block<3,1>(0,0)=Feettwist[leg].block<3,1>(0,0)+ori::crossMatrix(Feettwist[leg].block<3,1>(3,0))*r_feet;
    }
//    std::cout << "右腿测量速度：\n"
//               << Feettwist[0] << '\n';

#ifdef DEBUG
    Vec6<double> qDES;
    std::cout << "姿态矩阵\n:" << R[1] << '\n';
    computeIK(_biped, qDES, &(data[1].p), 0, R[1]);
    std::cout << "逆运动学解算的q:\n"
              << qDES << '\n';
    std::cout << "真实q:\n"
              << data[1].q << '\n';

    std::cout << "R转换的姿态四元数:\n"
              << ori::rotationMatrixToQuaternion(R[1].transpose()) << '\n';
    std::cout << "实际四元数:\n"
              << Feetpose[1].block<4, 1>(3, 0) << '\n';
    std::cout << "左腿测量速度：\n"
              << Feettwist[1] << '\n';
    std::cout << "雅可比矩阵测算的左腿速度:\n"
              << data[1].J_force_moment * data[1].qd << '\n';
#endif
}

void LegController::updatePosctrl(LowlevelCmd *cmd)
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            cmd->motorCmd[i * 6 + j].tau = 110 * (commands[i].qDes(j) - data[i].q[j]) + 1 * (commands[i].qdDes[j] - data[i].qd[j]);
            cmd->motorCmd[i * 6 + j].q = commands[i].qDes(j);
            cmd->motorCmd[i * 6 + j].dq = commands[i].qdDes[j];
            cmd->motorCmd[i * 6 + j].Kp = 400;
            cmd->motorCmd[i * 6 + j].Kd = 10;
            if (j == 5)
            {
                cmd->motorCmd[i * 6 + j].tau = 10 * (commands[i].qDes(j) - data[i].q[j]) + 1 * (commands[i].qdDes[j] - data[i].qd[j]);
                cmd->motorCmd[i * 6 + j].Kp = 200;
                cmd->motorCmd[i * 6 + j].Kd = 5;
            }
        }
    }
}

void LegController::updateCommand(LowlevelCmd *cmd, bool useWBC)
{

    std::array<std::vector<double>, 2> qKp, qKd;
    qKp[0].assign(6, 0);
    qKp[1].assign(6, 0);
    qKd[0].assign(6, 0);
    qKd[1].assign(6, 0);
    for (int i = 0; i < 2; i++)
    {
        Vec6<double> footForce = commands[i].feedforwardForce;
        Vec6<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg
        for (int j = 0; j < 6; j++)
        {
            std::cout << "legtau" << j << ": " << legtau(j) << std::endl;
        }

        if (commands[i].kpCartesian(0, 0) != 0 || commands[i].kdCartesian(0, 0) != 0) // 摆动项
        {
            if (useWBC)
            {
            }
            else
            {
                Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                            commands[i].kdCartesian * (commands[i].vDes - data[i].v);

                Vec6<double> swingtau = data[i].J_force.transpose() * footForce_3d;
                double kphip1 = 3;
                double kdhip1 = 0.3;

                commands[i].kdtoe = 0.1;
                swingtau(0) = kphip1 * (0 - data[i].q(0)) + kdhip1 * (0 - data[i].qd(0));
                swingtau(4) = commands[i].kptoe * (-data[i].q(3) - data[i].q(2) - data[i].q(4)) + commands[i].kdtoe * (0 - data[i].qd(4));

                for (int j = 0; j < 6; j++)
                {
                    legtau(j) += swingtau(j);
                }

                     for (int j = 0; j < 6; j++)
                commands[i].qDes[j] -= _biped.Initialq[j];

            }

       
            qKp[i][0] = 50;
            qKp[i][1] = 50;
            qKp[i][2] = 50;
            qKp[i][3] = 50;
            qKp[i][4] = 50;
            qKp[i][5] = 5;

            qKd[i][0] = 1;
            qKd[i][1] = 1;
            qKd[i][2] = 1;
            qKd[i][3] = 1;
            qKd[i][4] = 1;
            qKd[i][5] = 0.5;
        }

        commands[i].tau += legtau;

        for (int j = 0; j < 6; j++)
        {
            cmd->motorCmd[i * 6 + j].tau = commands[i].tau(j);
            cmd->motorCmd[i * 6 + j].q = commands[i].qDes(j);
            cmd->motorCmd[i * 6 + j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i * 6 + j].Kp = qKp[i].at(j);
            cmd->motorCmd[i * 6 + j].Kd = qKd[i].at(j);
        }

        commands[i].tau << 0, 0, 0, 0, 0, 0; // zero torque command to prevent interference
    }
}

void LegController::updateCommandwithKpKd(LowlevelCmd *cmd, Vec6<double> Kp, Vec6<double> Kd)
{
    for (int i = 0; i < 2; i++)
    {
        Vec6<double> footForce = commands[i].feedforwardForce;
        Vec6<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg
        for (int j = 0; j < 6; j++)
        {
            std::cout << "legtau" << j << ": " << legtau(j) << std::endl;
        }

        if (commands[i].kpCartesian(0, 0) != 0 || commands[i].kdCartesian(0, 0) != 0) // 摆动项
        {
            Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                        commands[i].kdCartesian * (commands[i].vDes - data[i].v);

            Vec6<double> swingtau = data[i].J_force.transpose() * footForce_3d;
            double kphip1 = 3;
            double kdhip1 = 0.3;

            commands[i].kdtoe = 0.1;
            // swingtau(0) = kphip1*(0-data[i].q(0)) + kdhip1*(0-data[i].qd(0));
            // swingtau(4) = commands[i].kptoe * (-data[i].q(3)-data[i].q(2)-data[i].q(4))+commands[i].kdtoe*(0-data[i].qd(4));

            for (int j = 0; j < 6; j++)
            {
                legtau(j) += swingtau(j);
            }

            for (int j = 0; j < 6; j++)
                commands[i].qDes[j] -= _biped.Initialq[j];
        }

        commands[i].tau += legtau;

        for (int j = 0; j < 6; j++)
        {
            cmd->motorCmd[i * 6 + j].tau = commands[i].tau(j);
            cmd->motorCmd[i * 6 + j].q = commands[i].qDes(j);
            cmd->motorCmd[i * 6 + j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i * 6 + j].Kp = Kp(j);
            cmd->motorCmd[i * 6 + j].Kd = Kd(j);
        }

        commands[i].tau << 0, 0, 0, 0, 0, 0; // zero torque command to prevent interference
    }
}

void LegController::updateTorque(LowlevelCmd *cmd, const Vec6<double> &Rt, const Vec6<double> &Lt)
{
    for (int j = 0; j < 6; j++)
    {
        cmd->motorCmd[j].tau = Rt[j];
        logger.jointTorque[j]=Rt[j];
        cmd->motorCmd[j + 6].tau = Lt[j];
    }
}

void computeLegJacobianAndPosition(Biped &_biped, Vec6<double> &q, Mat66<double> *J_f_m, Mat36<double> *J_f,
                                   Vec3<double> *p, int leg)
{
    using ori::CoordinateAxis;
    double side = -1.0; // 1 for Left legs; -1 for right legs
    if (leg == 1)
    {
        side = 1.0;
    }

    for (int i = 0; i < 6; i++)
        q[i] += _biped.Initialq[i];

    std::array<Vec3<double>, 6> axis_body;
    std::array<Vec3<double>, 6> r_body;
    Vec3<double> pbiped;

    r_body[0] << 0, 0, 0;                       // 0r01
    r_body[1] << 0, 0, 0;                       // 1r12
    r_body[2] << 0, 0, -_biped.thighLinkLength; // 2r23
    r_body[3] << 0, 0, -_biped.calfLinkLength;  // 3r34
    r_body[4] << 0, 0, 0;                       // 4r45;
    r_body[5] << 0, 0, -_biped.toeLinkLength;   // 5r5e

    pbiped << _biped.leg_offset_x, side * _biped.leg_offset_y, _biped.leg_offset_z; // trt0

    Mat3<double> trans = Mat3<double>::Identity(); // bRn=bR0*0R1*...*(n-1)Rn
    Mat3<double> transr = Mat3<double>::Identity();
    *p = pbiped;

    for (int i = 0; i < 6; i++)
    {
        if (i)
            trans *= ori::coordinateRotation(_biped.axisname[i - 1], q(i - 1));
        transr *= ori::coordinateRotation(_biped.axisname[i], q[i]);
        axis_body[i] = trans * _biped.axis[i];
#ifdef DEBUGMODE
        std::cout << "axis:\n"
                  << _biped.axis[i] << '\n';
#endif
        r_body[i] = transr * r_body[i];
        *p = *p + r_body[i];
    }
    for (int i = 4; i >= 0; i--)
    {
        r_body[i] += r_body[i + 1];
    }

    for (int i = 0; i < 6; i++)
    {
#ifdef DEBUGMODE
        std::cout << "rbody:\n"
                  << r_body[i] << '\n';
#endif
        J_f_m->block<3, 1>(0, i) = ori::crossMatrix(axis_body[i]) * r_body[i];
        J_f_m->block<3, 1>(3, i) = axis_body[i];
        J_f->block<3, 1>(0, i) = J_f_m->block<3, 1>(0, i);
    }

    *p = *p - pbiped; // 原点设在髋部
    for (int i = 0; i < 6; i++)
        q[i] -= _biped.Initialq[i];
}

void computeIK(Biped &_biped, Vec6<double> &q, Vec3<double> *p, int leg, Mat33<double> R) // 以髋关节为原点
{
    using namespace ori;

    Vec3<double> r_toe;
    r_toe << 0, 0, -_biped.toeLinkLength;
    Vec3<double> r = *p - R * r_toe; // BrB5=BrBE-BR5 * 5r5E

    r = R.transpose() * (-r);

    double C = r.norm();
    double Clknee = (C * C - pow(_biped.thighLinkLength, 2) - pow(_biped.calfLinkLength, 2)) / (2 * _biped.thighLinkLength * _biped.calfLinkLength);
    if (Clknee >= 1)
        q(3) = 0;
    else if (Clknee <= -1)
        q(3) = M_PI;
    else
        q(3) = acos(Clknee);

    q(5) = atan2(r(1), r(2));
    if (q(5) > 0.5 * M_PI)
        q(5) -= M_PI;
    else if (q(5) < -0.5 * M_PI)
        q(5) += M_PI;

    double alfa = asin(_biped.thighLinkLength / C * sin(q(3)));
    q(4) = -atan2(r(0), sign(r(2)) * sqrt((pow(r(1), 2)) + pow(r(2), 2))) - alfa;

    Mat33<double> R1 = R * coordinateRotation(CoordinateAxis::X, q(5)).transpose() * coordinateRotation(CoordinateAxis::Y, q(4)).transpose() *
                       coordinateRotation(CoordinateAxis::Y, q(3)).transpose();

    q(0) = atan2(-R1(0, 1), R1(1, 1));
    q(1) = atan2(R1(2, 1), -R1(0, 1) * sin(q[0]) + R1(1, 1) * cos(q[0]));
    q(2) = atan2(-R1(2, 0), R1(2, 2));
}
void IKinbodyframe(Biped &_biped, Vec6<double> &q, Vec3<double> *p, int leg, Mat33<double> R)
{
    double side = -1.0; // 1 for Left legs; -1 for right legs; leg0:rightleg;
    if (leg == 1)
    {
        side = 1.0;
    }
    Vec3<double> pbiped;
    pbiped << _biped.leg_offset_x, side * _biped.leg_offset_y, _biped.leg_offset_z; // trt0
    Vec3<double> r;
    r = *p - pbiped;
    // std::cout<<"IKinbodyframe,r:\n"<<r<<'\n';
    computeIK(_biped, q, &r, leg, R);
}