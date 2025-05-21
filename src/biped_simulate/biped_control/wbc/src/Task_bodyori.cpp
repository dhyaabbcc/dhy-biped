#include "../inc/taskset/Task_bodyori.h"

template<typename T>
BipTaskBodyOri<T>::BipTaskBodyOri(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_Data)
:BipTask<T>(3),FB_Model(fb_model),FB_Data(fb_data),_data(_Data)
{
    BT::Jt =DMat<T>::Zero(BT::dimTask, this->dimConfig);
    BT::Jt.block(0,0, 3, 3).setIdentity();
    BT::JtDotQdot=DVec<T>::Zero(BT::dimTask);

    errScale = DVec<T>::Constant(BT::dimTask, 1.);
    Kp = DVec<T>::Constant(BT::dimTask, 50.);
    Kd = DVec<T>::Constant(BT::dimTask, 1.0);
    
}

template<typename T>
bool BipTaskBodyOri<T>::UpdateTaskJacobian()
{
    pinocchio::JointIndex float_joint=FB_Model->m_model.getJointId("root_joint");
    pino::Data::Matrix6x WJ=pino::Data::Matrix6x::Zero(6,18);
    FB_Model->jacobianWorld_Aligned(*FB_Data,float_joint,WJ);
    BT::Jt.block(0,0,3,3)=WJ.block(3,3,3,3).cast<T>();
  //  std::cout<<"任务TASK_BODY ORI 更新\n";
    return true;
    
}

template<typename T>
bool BipTaskBodyOri<T>::UpdateTaskJDotQdot()
{
    if(!FB_Model->Driftcalculate_update) return false;
    
    auto frameindex=FB_Model->m_model.getFrameId("root_joint");
    
    auto Drift=FB_Model->frameClassicAcceleration(*FB_Data,frameindex);

    BT::JtDotQdot=Drift.angular().cast<T>();

   // std::cout<<"任务TASK_BODY ORI 更新\n";

    return true;
    
}

template<typename T>
bool BipTaskBodyOri<T>::UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc)
{
    Quat<T> *ori_cmd = (Quat<T> *)des_pos;
    Quat<T> link_ori;
    auto &seResult = _data->_stateEstimator->getResult();
    link_ori=seResult.orientation.cast<T>();

    Quat<T> link_ori_inv;
    link_ori_inv[0] = link_ori[0];
    link_ori_inv[1] = -link_ori[1];
    link_ori_inv[2] = -link_ori[2];
    link_ori_inv[3] = -link_ori[3];

    Quat<T> ori_err = robotics::math::quatProduct(*ori_cmd, link_ori_inv);
    if (ori_err[0] < 0.) {
        ori_err *= (-1.);
    }

    Vec3<T> ori_err_so3;
    robotics::math::quaternionToso3(ori_err, ori_err_so3);


    Vec3<T> curr_vel=seResult.omegaBody.cast<T>();
    Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(link_ori); /* Rot transform world frame to base frame. */

    Vec3<T> vel_err=Rot.transpose()*(BT::desiredVel-curr_vel);


    for(int i=0;i<3;i++)
    {
        BT::posErr[i] = errScale[i] * ori_err_so3[i];
        BT::desiredVel[i] = des_vel[i];
        BT::desiredAcc[i] = des_acc[i];
        BT::xddotCmd[i] = Kp[i] * ori_err_so3[i] + Kd[i] * vel_err[i] + BT::desiredAcc[i];
        BT::xddotCmd[i] = std::min(std::max(BT::xddotCmd[i], (T)-10), (T)10);
    }

    //std::cout<<"任务TASK_BODY ORI 更新\n";

    return true;
    
}

template class BipTaskBodyOri<float>;
//template class BipTaskBodyOri<double>;