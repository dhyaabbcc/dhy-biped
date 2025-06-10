#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include "../robotwrapper/MyWrapper.hpp"
#include "../include/interface/CheatIO.h"
#include "../include/common/Biped.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/common/cppTypes.h"
#include "../include/common/ControlFSMData.h"
#include "../include/State/FSM.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-qpData.hpp>
#include <tsid/solvers/solver-HQP-output.hpp>
#include <tsid/contacts/contact-6d.hpp>



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
    double dt = 0.005;
    const std::vector<std::string> package_dirs{};
    auto root_joint=pino::JointModelFreeFlyer();
    std::shared_ptr<MyWrapper> robotptr=std::make_shared<MyWrapper>(PINOCCHIO_BIPED, package_dirs, root_joint, true);
    std::shared_ptr<pino::Data> dataptr=std::make_shared<pino::Data>(robotptr->model());

    Biped robot(robotptr,dataptr);
    auto IOptr = std::make_shared<CheatIO>("biped");

    auto Legctrlptr = std::make_shared<LegController>(robot);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    StateEstimate stateEstimate;
    shared_ptr<StateEstimatorContainer> stateEstimator= std::make_shared<StateEstimatorContainer>(state,
                                                                          Legctrlptr->data,
                                                                          &stateEstimate);
    //直接从imu、gazebo读取位姿                                     
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

    //模块初始化
    tsid::InverseDynamicsFormulationAccForce tsid("tsid_controller", *robot._Dyptr);
    FootstepPlan plan;
    Pendulum pendulum;
    FloatingBaseObserver baseObs(state, stateEstimator, robot);
    NetWrenchObserver wrenchObs;
    Stabilizer stabilizer(robot, pendulum, dt, *state);
    ModelPredictiveControl mpc;
    
    ControlFSMData *_controlData = new ControlFSMData(robot, tsid, stateEstimator, plan, pendulum,
                                                      baseObs, wrenchObs, stabilizer, *state, mpc, dt);

    std::cout<<"\nbegin to set FSM\n";
    FSM* _FSMController = new FSM(_controlData);

      

    ros::Rate looprate1(200);
    signal(SIGINT, ShutDown);

    std::cout<<"\nbegin to run\n";
    

    while (running)
    {

    _FSMController->run();
   //logger.Savedata();
   // logger.Senddata();
    looprate1.sleep();
    std::cout<<"\n------------------------------\n";
    }


    delete cmd;
    delete state;
    delete _controlData;
    //delete _FSMController;
    system("stty sane"); // Terminal back to normal
    return 0;
}
