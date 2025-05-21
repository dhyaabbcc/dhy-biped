#include "../../include/FSM/FSMState_Pstand.h"

// FSMState_PStand::FSMState_PStand(ControlFSMData *data) : FSMState(data, FSMStateName::PDSTAND, "Pstand")
// {
//     Kp << 100, 0, 0,
//       0, 100, 0,
//       0, 0, 100;

//   Kd << 0, 0, 0,
//       0, 0, 0,
//       0, 0, 0;

// }

void FSMState_PStand::enter()
{
    std::cout<<"站立FSM"<<std::endl;
}

// void FSMState_PStand::exit()
// {
// }

void FSMState_PStand::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run();

    auto &seResult = _data->_stateEstimator->getResult();
    world_position = seResult.position;
    // world_foot_position_desired=world_position-Vec3<double>(0,0,1.12);
    for (int foot = 0; foot < 2; foot++)
    {
        // world_foot_position[foot] = seResult.position + seResult.rBody.transpose() 
        // * (_data->_biped->getHip2Location(foot) + _data->_legController->data[foot].p);

        // world_foot_position_desired[foot]=seResult.position+seResult.rBody.transpose()
        // *_data->_biped->getHip2Location(foot)+Vec3<double>(0,0,-1.124-_data->_biped->leg_offset_z);

        base_foot_position[foot] = _data->_biped->getHip2Location(foot)+Vec3<double>(-0.01,0,-1.124-_data->_biped->leg_offset_z);

        Vec3<double> Pdes= base_foot_position[foot];
      
        IKinbodyframe(*_data->_biped, QDes[foot], &Pdes, foot);

        Pdes-=_data->_legController->_biped.getHip2Location(foot);

        _data->_legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
        _data->_legController->commands[foot].pDes =Pdes;
   
        _data->_legController->commands[foot].kpCartesian = Kp;
        _data->_legController->commands[foot].kdCartesian = Kd;
        _data->_legController->commands[foot].qDes=QDes[foot];
        _data->_legController->commands[foot].qdDes=0*QDes[foot];
  
    }

    Vec6<double> KP,KD;
    KP[0]=25;
    KP[1]=25;
    KP[2]=100;
    KP[3]=450;
    KP[4]=300;
    KP[5]=50;
    KD=0.001*KP;
    _data->_legController->updateCommandwithKpKd(_data->_lowCmd,2.5*KP,KD);

}

// FSMStateName FSMState_PStand::checkTransition()
// {
//     if(_lowState->userCmd == UserCommand::L2_B){
//         return FSMStateName::PASSIVE;
//     }
//     else{
//         return FSMStateName::PDSTAND;
//     }
// }