#include "../inc/taskset/Task_bodypos.h"

template<typename T>
BipTaskBodyPos<T>::BipTaskBodyPos(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_Data)
:BipTask<T>(3),FB_Model(fb_model),FB_Data(fb_data),_data(_Data)
{
    BT::Jt =DMat<T>::Zero(BT::dimTask, this->dimConfig);
    BT::Jt.block(0,0, 3, 3).setIdentity();
    BT::JtDotQdot=DVec<T>::Zero(BT::dimTask);

    errScale = DVec<T>::Constant(BT::dimTask,2);

    Kp = DVec<T>::Constant(BT::dimTask, 50.);
    Kd = DVec<T>::Constant(BT::dimTask, 1.0);
    
}

template<typename T>
bool BipTaskBodyPos<T>::UpdateTaskJacobian()
{
    pinocchio::JointIndex float_joint=FB_Model->m_model.getJointId("root_joint");
    pino::Data::Matrix6x WJ=pino::Data::Matrix6x::Zero(6,18);
    FB_Model->jacobianWorld_Aligned(*FB_Data,float_joint,WJ);
    BT::Jt.block(0,0,3,3)=WJ.block(0,0,3,3).cast<T>();
    return true;
    
}

template<typename T>
bool BipTaskBodyPos<T>::UpdateTaskJDotQdot()
{
    if(!FB_Model->Driftcalculate_update) return false;
    
    auto frameindex=FB_Model->m_model.getFrameId("root_joint");
    
    auto Drift=FB_Model->frameClassicAcceleration(*FB_Data,frameindex);

    BT::JtDotQdot=Drift.linear().cast<T>();

    return true;
    
}

template<typename T>
bool BipTaskBodyPos<T>::UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc)
{
    Vec3<T> *pos_cmd = (Vec3<T> *)des_pos;
    Vec3<T> link_pos;
    auto &seResult = _data->_stateEstimator->getResult();
    link_pos=seResult.position.cast<T>();
   
    Vec3<T> curr_vel=seResult.vWorld.cast<T>();
   // BT::desiredVel=des_vel;
   // Mat3<T> Rot=seResult.rBody.cast<T>();
   // Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(link_ori); /* Rot transform world frame to base frame. */

    //Vec3<T> vel_err=BT::desiredVel-curr_vel;
    for(int i=0;i<3;i++)
    {
        BT::posErr[i] = errScale[i] * ((*pos_cmd)[i]-link_pos[i]);
        BT::desiredVel[i] = des_vel[i];
        BT::desiredAcc[i] = des_acc[i];
        BT::xddotCmd[i] = Kp[i] *((*pos_cmd)[i]-link_pos[i]) + Kd[i] * (des_vel[i]-curr_vel[i]) + BT::desiredAcc[i];
        BT::xddotCmd[i] = std::min(std::max(BT::xddotCmd[i], (T)-10), (T)10);
    }

    return true;
}

template class BipTaskBodyPos<float>;
//template class BipTaskBodyOri<double>;