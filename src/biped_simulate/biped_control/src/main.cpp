#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include "../robotwrapper/robotwrapper.h"
#include "../include/interface/CheatIO.h"
#include "../include/common/Biped.h"
#include "../include/common/Checkjoint.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/common/cppTypes.h"
#include "../include/common/ControlFSMData.h"
#include "../include/FSM/FSM.h"
#include "logdata/logdata.h"
#include "ros/ros.h"



//#include "../include/FSM/FSMState_Pstand.h"
#ifndef PINOCCHIO_BIPED
  #define PINOCCHIO_BIPED "src/biped_simulate/biped_gazebo/urdf/biped_gazebo.urdf"
#endif

using std::shared_ptr;
using std::unique_ptr;

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

int main(int argc, char **argv)
{
    namespace pino=pinocchio;
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    signal(SIGINT, ShutDown);
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    ros::Publisher scope1=nh.advertise<std_msgs::Float32>( "scope1", 10);
    DataLogger& logger=DataLogger::GET();

    //pinochio dynamic
    double dt = 0.001;
    auto root_joint=pino::JointModelFreeFlyer();
    std::shared_ptr<RobotWrapper> robotptr=std::make_shared<RobotWrapper>(PINOCCHIO_BIPED,root_joint,true);
    std::shared_ptr<pino::Data> dataptr=std::make_shared<pino::Data>(robotptr->m_model);

    Biped robot(robotptr,dataptr);
    auto IOptr = std::make_shared<CheatIO>("biped");
    auto Legctrlptr = std::make_shared<LegController>(robot);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    StateEstimate stateEstimate;
    shared_ptr<StateEstimatorContainer>stateEstimator= std::make_shared<StateEstimatorContainer>(state,
                                                                          Legctrlptr->data,
                                                                          &stateEstimate);
                                                  
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();


    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);



    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &robot;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController =  Legctrlptr;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = IOptr;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;


    FSM* _FSMController = new FSM(_controlData);

 

    ros::Rate looprate1(1000);
    signal(SIGINT, ShutDown);


    

    while (running)
    {

    _FSMController->run();
   //logger.Savedata();
    logger.Senddata();
    looprate1.sleep();
    cout<<"\n------------------------------\n";

    }


    delete cmd;
    delete state;
    delete _controlData;
    delete _FSMController;
    system("stty sane"); // Terminal back to normal
    return 0;
}
