#include "../inc/taskset/Task_linkpos.h"

template<typename T>
bipTaskLinkPos<T>::bipTaskLinkPos(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_Data,short legindex)
:BipTask<T>(6),FB_Model(fb_model),FB_Data(fb_data),_data(_Data)
{
    if(legindex<0||legindex>1) throw std::runtime_error("leg index wrong");

    if(legindex) linkindex=FB_Model->m_model.getFrameId("lcontactpoint");
    else linkindex=FB_Model->m_model.getFrameId("rcontactpoint");

    LEGindex=legindex;

    BT::Jt = DMat<T>::Zero(BT::dimTask, this->dimConfig);
    BT::JtDotQdot = DVec<T>::Zero(BT::dimTask);

    errScale = DVec<T>::Constant(BT::dimTask, 0.1);

    // errScale[3]=0.5;
    // errScale[4]=0.5;
    // errScale[5]=0.5;
    Kp = DVec<T>::Constant(BT::dimTask,10.);
    Kd = DVec<T>::Constant(BT::dimTask, 0.5);

}

template<typename T>
bool bipTaskLinkPos<T>::UpdateTaskJacobian()
{
    pino::Data::Matrix6x WJ=pino::Data::Matrix6x::Zero(6,18);
    pinocchio::getFrameJacobian(FB_Model->m_model,*FB_Data,linkindex,pinocchio::LOCAL_WORLD_ALIGNED,WJ);
    BT::Jt=WJ.cast<T>();
    return true;

}

template<typename T>
bool bipTaskLinkPos<T>::UpdateTaskJDotQdot()
{
    if(!FB_Model->Driftcalculate_update) return false;

    auto Drift=FB_Model->frameClassicAcceleration(*FB_Data,linkindex);

    BT::JtDotQdot.segment(0,3)=Drift.linear().cast<T>();
    BT::JtDotQdot.segment(3,3)=Drift.angular().cast<T>();
    
    return true;

}


template<typename T>
bool bipTaskLinkPos<T>::UpdateCommand(const void *des_pos,const DVec<T> &des_vel, const DVec<T> &des_acc)
{
    Vec6<T> *DESIRED = (Vec6<T> *)des_pos;//期望位置与姿态
    Vec3<T> pos_cmd=DESIRED->block(0,0,3,1);
    Vec3<T> desrpy=DESIRED->block(3,0,3,1);
    //std::cout<<"ORI——命令:\n"<<desrpy<<'\n';
    Quat<T> ori_cmdt=RobotWrapper::rpytoquat(desrpy);
    Quat<T> ori_cmd;
    ori_cmd[0]=ori_cmdt[3];
    ori_cmd.block(1,0,3,1)=ori_cmdt.block(0,0,3,1);

    pinocchio::SE3 link_placement=FB_Model->framePosition(*FB_Data,linkindex);
    Vec3<T> linkpos=link_placement.translation().cast<T>();//当前位置
    Vec3<T> pos_err=pos_cmd-linkpos;

    Quat<T> linkori=robotics::math::rotationMatrixToQuaternion(link_placement.rotation().transpose()).cast<T>();//当前姿态四元数

    Quat<T> link_ori_inv;
    link_ori_inv[0] = linkori[0];
    link_ori_inv[1] = -linkori[1];
    link_ori_inv[2] = -linkori[2];
    link_ori_inv[3] = -linkori[3];

    Quat<T> ori_err = robotics::math::quatProduct(ori_cmd, link_ori_inv);
    if (ori_err[0] < 0.) {
        ori_err *= (-1.);
    }
    
    Eigen::VectorXd feetpose=_data->_legController->Feetpose[LEGindex];
   // std::cout<<"腿"<<LEGindex<<"期望位置:\n"<<pos_cmd<<'\n';

   // std::cout<<"测算的 feetpose:\n"<<feetpose<<'\n';

   // std::cout<<"PINOCCHIO LINK ori计算:\n"<<linkori<<'\n';

   // std::cout<<"PINOCCHIO LINK pos计算:\n"<<linkpos<<'\n';
   //Vec6<T> Vpinocchio=Jt*

  //  std::cout<<"LINK ori 期望:\n"<<ori_cmd<<'\n';
  //  std::cout<<"LINK 误差:\n"<<ori_err<<'\n';

    Vec3<T> ori_err_so3;
    robotics::math::quaternionToso3(ori_err, ori_err_so3);

    
    for(int i=0;i<3;i++)
    {
        BT::posErr[i] = errScale[i] * pos_err[i];
        BT::posErr[i+3]=errScale[i]* ori_err_so3[i];
  
        BT::desiredVel[i] = des_vel[i];
        BT::desiredAcc[i] = des_acc[i];
        BT::desiredVel[i+3] = des_vel[i+3];
        BT::desiredAcc[i+3] = des_acc[i+3];

        BT::xddotCmd[i] = Kp[i] * pos_err[i]+des_acc[i];
        BT::xddotCmd[i+3]= Kp[i]  * ori_err[i]+des_acc[i+3];
        BT::xddotCmd[i] = std::min(std::max(BT::xddotCmd[i], (T)-10), (T)10);
        BT::xddotCmd[i+3] = std::min(std::max(BT::xddotCmd[i], (T)-10), (T)10);

    }    

    return true;


}


template class bipTaskLinkPos<float>;




