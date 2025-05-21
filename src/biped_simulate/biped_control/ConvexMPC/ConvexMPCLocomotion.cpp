#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;
using Eigen::Dynamic;

/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    walking(12, Vec2<int>(0, 6), Vec2<int>(6, 6), "Walking"),
                                                                                    standing(10, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  Kp << 100, 0, 0,
      0, 100, 0,
      0, 0, 100;
 // Kp_stance = 0 * Kp;

  Kd << 5, 0, 0,
      0, 5, 0,
      0, 0, 5;
 // Kd_stance = 0 * Kd;

  gaitNumber = 1;
  dtMPC = dt * iterationsBetweenMPC;
  // rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
    // data._biped->dataflow = std::make_shared<bipWbcCtrlData>();
}

ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc, ControlFSMData *data, bool USEWBC) : ConvexMPCLocomotion::ConvexMPCLocomotion(_dt, _iterations_between_mpc)
{
  useWBC = USEWBC;
  if(useWBC)
  {
    WBChandle=std::make_unique<bipWBCLocomotionControl<float>>(data);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
void ConvexMPCLocomotion::stand(ControlFSMData &data)
{
  auto &seResult = data._stateEstimator->getResult();
  Gait *gait = &standing;

  Vec3<double> v_des_world(0, 0, 0);
  if (firstRun)
  {
    world_position_desired = seResult.position;
    world_position_desired[2] = 1.12;
    firstRun = false;
  }

  auto &stateCommand = data._desiredStateCommand;
  stateCommand->data.stateDes.setZero();
  // 躯干位置
  stateCommand->data.stateDes[0] = world_position_desired[0];
  stateCommand->data.stateDes[1] = world_position_desired[1];
  stateCommand->data.stateDes[2] = 1.12;

  std::cout << "期望位置指令: " << stateCommand->data.stateDes[0] << '\n';

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  int *mpcTable = gait->mpc_gait();

  double Q[12] = {200, 300, 200, 200, 200, 300, 0.5, 1, 0.5, 1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
  updateMPC4Stand(mpcTable, data, false, Q);

  iterationCounter++;

  for (int foot = 0; foot < 2; foot++)
  {

    data._legController->commands[foot].kpCartesian = 0 * Kp; // 0
    data._legController->commands[foot].kdCartesian =  0 * Kd;
    data._legController->commands[foot].kptoe = 0; // 0
    data._legController->commands[foot].kdtoe = 0;
    data._legController->commands[foot].feedforwardForce = f_ff[foot];
  }

  // data log:
  //  logger.pxBody_des=world_position_desired[0];
  //  logger.pyBody_des=world_position_desired[1];
  //  logger.pzBody_des=world_position_desired[2];

  // logger.pxBody=seResult.position[0];
  // logger.pyBody=seResult.position[1];
  // logger.pzBody=seResult.position[2];

  // logger.rBody=seResult.rpy[0];
  // logger.pBody=seResult.rpy[1];
  // logger.yBody=seResult.rpy[2];

  // logger.rBody_des=0;
  // logger.yBody_des=0;
  // logger.pBody_des=0;
}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;
  MPC_Update = false;
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;
  auto &dataflow=data._biped->dataflow;
  // pick gait
  Gait *gait = &standing;
  // gaitNumber=1;
  if (gaitNumber == 1)
    gait = &standing;
  else if (gaitNumber == 2)
    gait = &walking;

  current_gait = gaitNumber;

  UpdateDesiredState(data);

  std::cout<<"姿态矩阵"<<seResult.rBody<<'\n';

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data._biped->getHip2Location(i) + data._legController->data[i].p);
  }

  // some first time initialization
  if (firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;

  double side_sign[2] = {1, -1};
  double v_abs = std::fabs(seResult.vBody[0]);

  for (int i = 0; i < 2; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(0.20);
    Vec3<double> offset(0, side_sign[i] * 0.0, -0.0);
    // simple heuristic function
    Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);

    Vec3<double> Pf = seResult.position +
                      seResult.rBody.transpose() * pRobotFrame + seResult.vWorld * swingTimeRemaining[i];

    double p_rel_max = 1.4;
    double pfx_rel = -0.01 + seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                     0.335 * (seResult.vWorld[0] - dataflow->vBody_des[0]);

    double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                     0.02 * (seResult.vWorld[1] - dataflow->vBody_des[1]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    //std::cout << "步长：" << pfx_rel << '\n';
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
    Pf[2] = -0.0;

    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  logger.L_legstate = true;
  logger.R_legstate = true;

  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();

  int *mpcTable = gait->mpc_gait();
  double Q[12] = {200, 500, 200, 200, 200, 300, 0.5, 1, 0.5, 1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
  updateMPCIfNeeded(mpcTable, data, omniMode, Q);

  iterationCounter++;
  if(useWBC) dataflow->allowAfterMPC=!MPC_Update;

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot);
    std::cout << "swing " << foot << ": " << swingState << std::endl;
    std::cout << "Contact " << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {

      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition().cast<double>();
      vDesFootWorld[foot] = footSwingTrajectories[foot].getVelocity().cast<double>();

      std::cout<<"腿"<<foot<<"规划位置:\n"<< pDesFootWorld[foot] <<'\n';

      double side = 1.0;
      if (foot == 1)
      {
        side = -1.0;
      }

      Vec3<double> hipOffset = {0, side * -0.03, 0};
      pDesFoot[foot] = seResult.rBody * (pDesFootWorld[foot] - seResult.position) - hipOffset;
      vDesFoot[foot] = seResult.rBody * (vDesFootWorld[foot] - seResult.vWorld);


      if(useWBC)
      {
        dataflow->pFoot_des[foot].block(0,0,3,1)= footSwingTrajectories[foot].getPosition().cast<float>()-(seResult.rBody.transpose()*hipOffset).cast<float>();
        dataflow->pFoot_des[foot].block(3,0,3,1)=dataflow->pBody_RPY_des;

        dataflow->vFoot_des[foot].block(0,0,3,1)=vDesFootWorld[foot].cast<float>();
        dataflow->vFoot_des[foot].block(3,0,3,1)=dataflow->vBody_Ori_des;

        dataflow->aFoot_des[foot].block(0,0,3,1)=footSwingTrajectories[foot].getAcc().cast<float>();
        dataflow->aFoot_des[foot].block(3,0,3,1).setZero();
        dataflow->contact_state[foot]=false;
      }


      if (vDesFoot[foot].hasNaN())
      {
        vDesFoot[foot] << 0, 0, 0;
      }

      Vec3<double> Pdes = pDesFoot[foot];

      IKinbodyframe(*(data._biped), QDes[foot], &Pdes, foot);
      Vec6<double> vdess = Vec6<double>::Zero();
      vdess.block<3, 1>(0, 0) = vDesFoot[foot];
      QdDes[foot] = data._legController->data[foot].J_force_moment.inverse() * vdess;

      // for (int i = 0; i < 6; i++)
      // {
      //   logger.jointposdes[i + foot * 6] = QDes[foot][i];
      //   logger.jointveldes[i + foot * 6] = QdDes[foot][i];
      // }

      Pdes -= data._legController->_biped.getHip2Location(foot);

      data._legController->commands[foot].feedforwardForce << 0, 0, 0, 0, 0, 0;
      data._legController->commands[foot].pDes = Pdes;
      data._legController->commands[foot].vDes = vDesFoot[foot];
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      data._legController->commands[foot].qDes = QDes[foot];
      data._legController->commands[foot].qdDes = QdDes[foot];

      if (!foot)
        logger.R_legstate = false;
      else
        logger.L_legstate = false;
    }
    else if (contactState > 0) // foot is in stance
    {
      firstSwing[foot] = true;
      data._legController->commands[foot].kpCartesian =  0 * Kp; // 0
      data._legController->commands[foot].kdCartesian =  0 * Kd;
      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;
      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      if(useWBC)
      {
        dataflow->contact_state[foot]=true;
      //dataflow->Fr_des[foot]=f_ff[foot].cast<float>();

       // WBChandle->Run(data._biped->dataflow);
      }
      // for (int i = 0; i < 6; i++)
      // {
      //   logger.jointposdes[i + foot * 6] = data._biped->Initialq[i];
      //   logger.jointveldes[i + foot * 6] = data._biped->Initialq[i];
      // }
      // std::cout<<"foot"<<foot<<"反作用力:\n"<<f_ff[foot]<<'\n';
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode, double *Q)
{

  if ((iterationCounter % 5) == 0)
  {

    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;
    auto &dataflow=data._biped->dataflow;
    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *quat = seResult.orientation.data();

    // Joint angles to compute foot rotation
    Eigen::Matrix<double, 12, 1> q;
    for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 6; k++)
      {
        q(i * 6 + k) = data._legController->data[i].q(k) + data._biped->Initialq[k];
      }
    }

    double PI = 3.14159265359;
    double PI2 = 2 * PI;
    for (int i = 0; i < 12; i++)
    {
      q(i) = fmod(q(i), PI2);
    }
    double *joint_angles = q.data();

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }
    double Alpha[12] = {1e-4, 5e-4, 5e-4, 1e-4, 1e-4, 5e-4, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    double *weights = Q;
    double *Alpha_K = Alpha;

    double yaw = seResult.rpy[2];

    // std::cout << "current position: " << p[0] << "  " << p[1] << "  " << p[2] << std::endl;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    const double max_pos_error = 0.08;
    double xStart = dataflow->pBody_des[0];
    double yStart =dataflow->pBody_des[1];

    // if (xStart - p[0] > max_pos_error)
    //   xStart = p[0] + max_pos_error;
    // if (p[0] - xStart > max_pos_error)
    //   xStart = p[0] - max_pos_error;
    // if (yStart - p[1] > max_pos_error)
    //   yStart = p[1] + max_pos_error;
    // if (p[1] - yStart > max_pos_error)
    //   yStart = p[1] - max_pos_error;

    // world_position_desired[0] = xStart;
    // world_position_desired[1] = yStart;

    // Vec3<double> ori_des_world;
   // ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];

    double trajInitial[12] = {dataflow->pBody_RPY_des[0], // 0
                              dataflow->pBody_RPY_des[1], // 1
                              dataflow->pBody_RPY_des[2],                                   // 2
                              dataflow->pBody_des[0],                                            // 3
                              dataflow->pBody_des[1],                                            // 4
                              dataflow->pBody_des[2],                         // 5
                              0,                                                 // 6
                              0,                                                 // 7
                              dataflow->vBody_Ori_des[2],                   // 8
                              dataflow->vBody_des[0],                                    // 9
                              dataflow->vBody_des[1],                                    // 10
                              0};                                                // 11

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if (i == 0) // start at current position  TODO consider not doing this
      {
        trajAll[0] = seResult.rpy[0];
        trajAll[1] = seResult.rpy[1];
        trajAll[2] = seResult.rpy[2];
        trajAll[3] = seResult.position[0];
        trajAll[4] = seResult.position[1];
        trajAll[5] = seResult.position[2];
      }
      else
      {
        if (v_des_world[0] == 0)
        {
          trajAll[12 * i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else
        {
          trajAll[12 * i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0];
        }
        if (v_des_world[1] == 0)
        {
          trajAll[12 * i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else
        {
          trajAll[12 * i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1];
        }
        if (stateCommand->data.stateDes[11] == 0)
        {
          trajAll[12 * i + 2] = trajInitial[2];
        }
        else
        {
          trajAll[12 * i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        }
      }
      std::cout << "traj " << i << std::endl;
      for (int j = 0; j < 12; j++)
      {
        std::cout << trajAll[12 * i + j] << "  ";
      }
      std::cout << " " << std::endl;
    }
    std::cout << "初始traj " << std::endl;
    for (int j = 0; j < 12; j++)
    {
      std::cout << trajInitial[j] <<"  ";
    }

    // MPC Solver Setup
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 2.5, 250);

    // Solve MPC
    Timer t_mpc_solve;
    t_mpc_solve.start();
    update_problem_data(p, v, quat, w, r, joint_angles, yaw, weights, trajAll, Alpha_K,
                        mpcTable, data._legController->R_foot_R.cast<float>(), data._legController->R_foot_L.cast<float>());
    printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());
    MPC_Update = true;
    // Get solution and update foot forces
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      GRF_R = -seResult.rBody * GRF;
      GRM_R = -seResult.rBody * GRM;
      std::cout << "RBody: " << seResult.rBody << std::endl;

      for (int i = 0; i < 3; i++)
      {
        f(i) = GRF_R(i);
        f(i + 3) = GRM_R(i);
        if(useWBC)
        {
          dataflow->Fr_des[leg](i)=GRF(i);
          dataflow->Fr_des[leg](i+3)=GRM(i);
          if(leg==0)
          {
            logger.Fmpc[i]=GRF(i);
            logger.Fmpc[i+3]=GRM(i);
          }
         
        }
      }
      f_ff[leg] = f;
    }
  }
}

void ConvexMPCLocomotion::updateMPC4Stand(int *mpcTable, ControlFSMData &data, bool omniMode, double *Q)
{
  if ((iterationCounter % 6) == 0)
  {

    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *quat = seResult.orientation.data();
    // Joint angles to compute foot rotation
    Eigen::Matrix<double, 12, 1> q;
    for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 6; k++)
      {
        q(i * 6 + k) = data._legController->data[i].q(k) + data._biped->Initialq[k];
      }
    }

    double PI = 3.14159265359;
    double PI2 = 2 * PI;
    for (int i = 0; i < 12; i++)
    {
      q(i) = fmod(q(i), PI2);
    }
    double *joint_angles = q.data();

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }

    double Alpha[12] = {1e-4, 5e-4, 5e-4, 1e-4, 1e-4, 5e-4, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    double *weights = Q;
    double *Alpha_K = Alpha;

    double yaw = seResult.rpy[2];

    std::cout << "current position: " << p[0] << "  " << p[1] << "  " << p[2] << std::endl;

    v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    const double max_pos_error = 0.08;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    // Vec3<double> ori_des_world;
    ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];

    double trajInitial[12] = {/*rpy_comp[0] + */ stateCommand->data.stateDes[3], // 0
                              /*rpy_comp[1] + */ stateCommand->data.stateDes[4], // 1
                              seResult.rpy[2],                                   // 2
                              xStart,                                            // 3
                              yStart,                                            // 4
                              world_position_desired[2],                         // 5
                              0,                                                 // 6
                              0,                                                 // 7
                              stateCommand->data.stateDes[11],                   // 8
                              v_des_world[0],                                    // 9
                              v_des_world[1],                                    // 10
                              0};                                                // 11

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if (i == 0) // start at current position  TODO consider not doing this
      {
        // trajAll[0] = seResult.rpy[0];
        // trajAll[1] = seResult.rpy[1];
        trajAll[2] = seResult.rpy[2];
        trajAll[3] = seResult.position[0];
        trajAll[4] = seResult.position[1];
        trajAll[5] = seResult.position[2];
      }
      else
      {
        if (v_des_world[0] == 0)
        {
          trajAll[12 * i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else
        {
          trajAll[12 * i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0];
        }
        if (v_des_world[1] == 0)
        {
          trajAll[12 * i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else
        {
          trajAll[12 * i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1];
        }
        if (stateCommand->data.stateDes[11] == 0)
        {
          trajAll[12 * i + 2] = trajInitial[2];
        }
        else
        {
          trajAll[12 * i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
          // std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
      std::cout << "traj " << i << std::endl;
      for (int j = 0; j < 12; j++)
      {
        std::cout << trajAll[12 * i + j] << "  ";
      }
      std::cout << " " << std::endl;
    }
    std::cout << "初始traj " << std::endl;
    for (int j = 0; j < 12; j++)
    {
      std::cout << trajInitial[j] << "  ";
    }

    // MPC Solver Setup
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 2.5, 250);

    // Solve MPC
    Timer t_mpc_solve;
    t_mpc_solve.start();
    update_problem_data(p, v, quat, w, r, joint_angles, yaw, weights, trajAll, Alpha_K,
                        mpcTable, data._legController->R_foot_R.cast<float>(), data._legController->R_foot_L.cast<float>());
    printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());

    // Get solution and update foot forces
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      // std::cout<<"MPC计算线性力\n"<<GRF<<'\n';
      // std::cout<<"MPC计算力矩\n"<<GRM<<'\n';

      GRF_R = -seResult.rBody * GRF;
      GRM_R = -seResult.rBody * GRM;
      // std::cout << "RBody: " << seResult.rBody << std::endl;

      for (int i = 0; i < 3; i++)
      {
        f(i) = GRF_R(i);
        f(i + 3) = GRM_R(i);
      }
      f_ff[leg] = f;
      rawF_angular[leg] = GRM;
      rawF_linear[leg] = GRF;

    }
  }
}

void ConvexMPCLocomotion::UpdateDesiredState(ControlFSMData &data)
{
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;
  auto &dataflow=data._biped->dataflow;

  const double *p = seResult.position.data();
  // const double *v = seResult.vWorld.data();
  // const double *w = seResult.omegaWorld.data();
  // const double *quat = seResult.orientation.data();

  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0);

  Vec3<double> v_des_world;
  v_des_world = seResult.rBody.transpose() * v_des_robot;

  

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 1.11; 
  double xStart = world_position_desired[0];
  double yStart = world_position_desired[1];
  const double max_pos_error = 0.08;
    if (xStart - p[0] > max_pos_error)
      xStart = p[0] + max_pos_error;
    if (p[0] - xStart > max_pos_error)
      xStart = p[0] - max_pos_error;
    if (yStart - p[1] > max_pos_error)
      yStart = p[1] + max_pos_error;
    if (p[1] - yStart > max_pos_error)
      yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    dataflow->vBody_des=v_des_world.cast<float>();
    dataflow->pBody_des=world_position_desired.cast<float>();

    dataflow->aBody_des.setZero();
    dataflow->pBody_RPY_des<<stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];
    dataflow->pBody_RPY_des[2]=seResult.rpy[2];
    
    dataflow->vBody_Ori_des<<0,0,stateCommand->data.stateDes[11];
  //dataflow->allowAfterMPC = !MPC_Update;

  Vec3<double> BodyRPY=ori::quatToRPY( seResult.orientation);

  logger.rBody=BodyRPY[0];
  logger.pBody=BodyRPY[1];
  logger.yBody=BodyRPY[2];

  logger.vxBody_des=v_des_world[0];
  logger.vxBody=seResult.vWorld[0];

  logger.vyBody_des=v_des_world[1];
  logger.vyBody=seResult.vWorld[1];

  logger.vzBody_des=v_des_world[2];
  logger.vzBody=seResult.vWorld[2];

  logger.pxBody=seResult.position[0];
  logger.pyBody=seResult.position[1];
  logger.pzBody=seResult.position[2];



}