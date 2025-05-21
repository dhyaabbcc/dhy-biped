#ifndef PPSTAND1_H
#define PPSTAND1_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
#include "logdata/logdata.h"


class FSMState_PStand : public FSMState
{
public:
  FSMState_PStand(ControlFSMData *data) : FSMState(data, FSMStateName::PDSTAND, "Pstand")
  {
    Kp << 10, 0, 0,
        0, 10, 0,
        0, 0, 10;

    Kd << 0, 0, 0,
        0, 0, 0,
        0, 0, 0;
  }

  void enter() override;

  void run() override;

  void exit() override
  {
  }
  FSMStateName checkTransition()
  {
    return FSMStateName::PDSTAND;
  }

  //  ConvexMPCLocomotion Cmpc;
  Vec3<double> world_position;
  Vec3<double> world_foot_position[2];
  Vec3<double> world_foot_position_desired[2];

  Vec3<double> base_foot_position[2];
  Vec3<double> base_foot_position_desired[2];

  Mat3<double> Kp, Kd;

  //各个关节的期望角度
  Vec6<double> QDes[2];

  int counter;
};

#endif