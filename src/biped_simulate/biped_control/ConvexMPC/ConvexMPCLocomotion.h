#ifndef CONVEXMPCLOCOMOTION_H
#define CONVEXMPCLOCOMOTION_H

#include <eigen3/Eigen/Dense>
#include "../include/common/FootSwingTrajectory.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include "GaitGenerator.h"
#include <fstream>
#include "wbc_data_flow.h"
#include "logdata/logdata.h"
#include "wbc_locomotion_controller.h"
//#include "../robotwrapper/robotwrapper.h"

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;

using namespace std;

/**
 * @file ConvexMPCLocomotion.h
 * @brief Convex Model Predictive Control (MPC) for Bipedal Locomotion
 *
 * This file defines the ConvexMPCLocomotion class, which implements a convex MPC
 * approach to generate and control gait patterns for bipedal robots. The class
 * offers functionality to set gait patterns, update the MPC as needed, and track 
 * the state of the robot in terms of position, orientation, and contact with the ground.
 */


struct CMPC_Result {
  LegControllerCommand commands[2];
  Vec2<float> contactPhase;
};

class ConvexMPCLocomotion {
public:
    // Constructors
    ConvexMPCLocomotion(double _dt, int _iterations_between_mpc);
    ConvexMPCLocomotion(double _dt, int _iterations_between_mpc,ControlFSMData *data,bool USEWBC);
   
    // Main Functionalities
    void run(ControlFSMData& data);
    void stand(ControlFSMData& data);
    void setGaitNum(int gaitNum) { gaitNumber = gaitNum; }
    bool firstRun = true;
    Vec3<double> rawF_linear[2];
    Vec3<double> rawF_angular[2];
    bool useWBC=false;
private:
    void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode,double* Q);
    //void GenerateTrajectory(int* mpcTable, ControlFSMData& data, bool omniMode);
    void updateMPC4Stand(int* mpcTable, ControlFSMData& data, bool omniMode,double* Q);
    void UpdateDesiredState(ControlFSMData& data);
    std::unique_ptr<bipWBCLocomotionControl<float>> WBChandle;
    // Locomotion and Gait Parameters
    int iterationsBetweenMPC;
    int horizonLength;
    double dt;
    double dtMPC;
    int iterationCounter = 0;
    Vec6<double> f_ff[2];
   
    //Vec12<double> Forces_Sol;
    Vec2<double> swingTimes;
    FootSwingTrajectory<double> footSwingTrajectories[2];
    Gait walking, standing;

    // Feedback and Control Variables
    Mat3<double> Kp, Kd;
    bool firstSwing[2] = {true, true};
    double swingTimeRemaining[2];
    int current_gait;
    int gaitNumber;
    Vec3<double> world_position_desired;
    Vec3<double> pFoot[2];
    //CMPC_Result result;
    double trajAll[12*10];
    Vec3<double> a;  
    Vec4<double> pz;

    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;
    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;
    Vec3<double> pDesFootWorld[2] ;
    Vec3<double> vDesFootWorld[2] ;
    Vec3<double> pDesFoot[2] ;
    Vec3<double> vDesFoot[2] ;
    Vec6<double> QDes[2];
    Vec6<double> QdDes[2];
    Vec2<double> contact_state;
    Vec3<double> v_des_robot;
    //ofstream foot_position;
    Vec3<double> ori_des_world;   
    DataLogger& logger=DataLogger::GET(); 
    bool MPC_Update=false;

};


#endif //CONVEXMPCLOCOMOTION_H
