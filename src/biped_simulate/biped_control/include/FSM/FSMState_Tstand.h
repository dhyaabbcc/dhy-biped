#ifndef TSTAND_H
#define TSTAND_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
#include "logdata/logdata.h"
//#include "pinocchio/fwd.hpp"

class FSMState_Tstand : public FSMState
{
public:
    FSMState_Tstand(ControlFSMData *data,ConvexMPCLocomotion& cmpc);
    ~FSMState_Tstand() {}
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkTransition()
    {
       
        return FSMStateName::QPSTAND;
        
    }

private:
    ConvexMPCLocomotion& Cmpc;
   // std::shared_ptr<ConvexMPCLocomotion> cmpc;
    int counter;
    Vec3<double> v_des_body;
    double turn_rate = 0;
    double pitch, roll;
    DataLogger &logger = DataLogger::GET();
    //pino::Data dynamicdata;
    //std::shared_ptr<RobotWrapper> _dynamicptr;
    //std::shared_ptr<pinocchio::Data> _dydataptr;
};

#endif