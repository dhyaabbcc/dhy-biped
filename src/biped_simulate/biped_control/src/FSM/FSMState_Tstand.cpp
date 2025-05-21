#include "../../include/FSM/FSMState_Tstand.h"

FSMState_Tstand::FSMState_Tstand(ControlFSMData *data, ConvexMPCLocomotion &cmpc)
    : FSMState(data, FSMStateName::QPSTAND, "walking"),
      Cmpc(cmpc) {}

template <typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1)
{
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

void FSMState_Tstand::enter()
{
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run();
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;
}

void FSMState_Tstand::exit()
{
    counter = 0;
}

void FSMState_Tstand::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();

    auto &seResult = _data->_stateEstimator->getResult();
    auto dyptr = _data->_biped->_Dyptr;
    auto dataptr = _data->_biped->_Dataptr;
    Eigen::Vector3d Base_pos = seResult.position;
    Eigen::Vector4d Base_ori;
    Eigen::Vector3d Base_omega = seResult.omegaWorld;
    Eigen::Vector3d Base_vel = seResult.vWorld;
    Eigen::VectorXd q(dyptr->m_model.nq);
    Eigen::VectorXd v(dyptr->m_model.nv);
    Eigen::VectorXd a(dyptr->m_model.nv);
    Base_ori.block(0, 0, 3, 1) = seResult.orientation.block(1, 0, 3, 1); // pinocchio 四元数顺序为 [x,y,z,w]
    Base_ori[3] = seResult.orientation[0];
    q.setZero();
    q.block(0, 0, 3, 1) = Base_pos;
    //  q[2]=1.12;
    // q.block(0,0,3,1)[2]=(double)1.12;
    q.block(3, 0, 4, 1) = Base_ori;
    v.block(0, 0, 3, 1).setZero();
    v.block(3, 0, 3, 1) = Base_omega;
    a.block(0, 0, 3, 1).setZero();
    a.block(3, 0, 3, 1).setZero();

    for (int i = 0; i < 6; i++)
    {
        q(7 + i) = _data->_legController->data[1].q(i);
        q(7 + 6 + i) = _data->_legController->data[0].q(i) ; // 0腿为右腿，在kinematic tree 中索引值为8
        v(6 + i) = _data->_legController->data[1].qd(i);
        v(6 + 6 + i) = _data->_legController->data[0].qd(i);
        a(6 + i) = _data->_legController->data[1].qdd(i);
        a(6 + 6 + i) = _data->_legController->data[0].qdd(i);

        // v(6 + i) =0;
        // v(6 + 6 + i) = 0;
        // a(6 + i) = 0;
        // a(6 + 6 + i) =0;
    }
    std::cout << "Q:\n"
              << q << '\n';

    pino::container::aligned_vector<pino::Force> Fext;
    pino::Force f = pino::Force::Zero(), fext;
    Fext.assign(14, f);
    dyptr->computeMainTerms(*dataptr, q, v);
    Cmpc.setGaitNum(1); // 2 for walking
    Cmpc.stand(*_data);

    auto FrameIDr = dyptr->m_model.getFrameId(std::string("rcontactpoint"));
    auto FrameIDl = dyptr->m_model.getFrameId(std::string("lcontactpoint"));

    Fext.at(13).linear() = dyptr->m_model.frames[FrameIDr].placement.act(Cmpc.rawF_linear[0]);
    Fext.at(13).angular() = dyptr->m_model.frames[FrameIDr].placement.act(Cmpc.rawF_angular[0]);
    Fext.at(7).linear() = dyptr->m_model.frames[FrameIDl].placement.act(Cmpc.rawF_linear[1]);
    Fext.at(7).angular() = dyptr->m_model.frames[FrameIDl].placement.act(Cmpc.rawF_angular[1]);

    pino::rnea(dyptr->m_model, *dataptr, q, v, a, Fext);
    std::cout<<"所有关节力:\n"<<dataptr->tau<<'\n';
    Vec6<double> Lt = dataptr->tau.block(6, 0, 6, 1);
    Vec6<double> Rt = dataptr->tau.block(12, 0, 6, 1);
    std::cout << "rnea求得的左腿关节力为：\n"
              << Lt << std::endl;
    std::cout << "rnea求得的右腿关节力为：\n"
              << Rt << std::endl;

     //_data->_legController->updateTorque(_data->_lowCmd,Rt,Lt);

    _data->_legController->updateCommand(_data->_lowCmd);
     _data->_lowCmd->motorCmd[3].tau=(float)Rt(3);
   // _data->_lowCmd->motorCmd[2].tau=Rt(2);
    //_data->_lowCmd->motorCmd[4].tau=(float)Rt(4);

    _data->_lowCmd->motorCmd[9].tau=(float)Lt(3);
    // _data->_lowCmd->motorCmd[8].tau=Lt(2);
    // _data->_lowCmd->motorCmd[10].tau=(float)Lt(4);
    

}