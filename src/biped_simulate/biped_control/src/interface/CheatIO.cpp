#include "../../include/interface/CheatIO.h"
#include "../../include/interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

inline void RosShutDown(int sig)
{
    ROS_INFO("ROS interface shutting down!");
    ros::shutdown();
}

CheatIO::CheatIO(std::string robot_name) : IOInterface()
{
    // int argc; char **argv;
    // ros::init(argc, argv, "unitree_gazebo_servo");
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;
    _robot_name = robot_name;

    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(3000); // wait for subscribers start
    // initialize publisher
    initSend();

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
    ros::spinOnce();
}

CheatIO::~CheatIO()
{
    ros::shutdown();
}

void CheatIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);
    cmdPanel->updateVelCmd(state);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void CheatIO::sendCmd(const LowlevelCmd *cmd)
{
    for (int i = 0; i < 12; i++)
    {
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
    }
    for (int m = 0; m < 12; m++)
    {
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }

    ros::spinOnce();
}

void CheatIO::recvState(LowlevelState *state)
{
    for (int i = 0; i < 12; i++)
    {
        state->motorState[i].q = _highState.motorState[i].q;
        state->motorState[i].dq = _highState.motorState[i].dq;
        state->motorState[i].tauEst = _highState.motorState[i].tauEst;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _highState.imu.quaternion[i];
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];
       // _highState.imu.accelerometer
         state->imu.accelerometer[i]=_highState.imu.accelerometer[i];
        state->position[i] = _highState.position[i];
        state->vWorld[i] = _highState.velocity[i];
    }
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
    recvfeetState(state);
}

void CheatIO::recvfeetState(LowlevelState *state)
{
    for (int leg = 0; leg < 2; leg++)
    {
        state->feetpose[0+7*leg]=Footpose[leg].position.x;//右0-6，左，7-12；
        state->feetpose[1+7*leg]=Footpose[leg].position.y;
        state->feetpose[2+7*leg]=Footpose[leg].position.z;
        state->feetpose[3+7*leg]=Footpose[leg].orientation.w;
        state->feetpose[4+7*leg]=Footpose[leg].orientation.x;
        state->feetpose[5+7*leg]=Footpose[leg].orientation.y;
        state->feetpose[6+7*leg]=Footpose[leg].orientation.z;

        state->feettwist[0+6*leg]=Foottwist[leg].linear.x;
        state->feettwist[1+6*leg]=Foottwist[leg].linear.y;
        state->feettwist[2+6*leg]=Foottwist[leg].linear.z;
        state->feettwist[3+6*leg]=Foottwist[leg].angular.x;
        state->feettwist[4+6*leg]=Foottwist[leg].angular.y;
        state->feettwist[5+6*leg]=Foottwist[leg].angular.z;

    }
}

void CheatIO::initSend()
{
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j0_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j1_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j2_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j3_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j4_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/R_j5_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j0_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j1_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j2_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j3_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j4_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/L_j5_controller/command", 1);
}

void CheatIO::initRecv()
{
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &CheatIO::StateCallback, this);
    linkstate_sub = _nm.subscribe("/gazebo/link_states", 1, &CheatIO::LinkStatesCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j0_controller/state", 1, &CheatIO::RJ0Callback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j1_controller/state", 1, &CheatIO::RJ1Callback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j2_controller/state", 1, &CheatIO::RJ2Callback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j3_controller/state", 1, &CheatIO::RJ3Callback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j4_controller/state", 1, &CheatIO::RJ4Callback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/R_j5_controller/state", 1, &CheatIO::RJ5Callback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j0_controller/state", 1, &CheatIO::LJ0Callback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j1_controller/state", 1, &CheatIO::LJ1Callback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j2_controller/state", 1, &CheatIO::LJ2Callback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j3_controller/state", 1, &CheatIO::LJ3Callback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j4_controller/state", 1, &CheatIO::LJ4Callback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/L_j5_controller/state", 1, &CheatIO::LJ5Callback, this);
}

void CheatIO::LinkStatesCallback(const gazebo_msgs::LinkStates &msg)
{
    Foottwist[1] = msg.twist.at(7);
    Foottwist[0] = msg.twist.at(13);
    Footpose[1] = msg.pose.at(7);//左腿
    Footpose[0] = msg.pose.at(13);
    
}

void CheatIO::StateCallback(const gazebo_msgs::ModelStates &msg)
{
    int robot_index;
    // std::cout << msg.name.size() << std::endl;
    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == _robot_name + "_gazebo")
        {
            robot_index = i;
        }
    }

    _highState.position[0] = msg.pose[robot_index].position.x;
    _highState.position[1] = msg.pose[robot_index].position.y;
    _highState.position[2] = msg.pose[robot_index].position.z;

    _highState.velocity[0] = msg.twist[robot_index].linear.x;
    _highState.velocity[1] = msg.twist[robot_index].linear.y;
    _highState.velocity[2] = msg.twist[robot_index].linear.z;

    _highState.imu.quaternion[0] = msg.pose[robot_index].orientation.w;
    _highState.imu.quaternion[1] = msg.pose[robot_index].orientation.x;
    _highState.imu.quaternion[2] = msg.pose[robot_index].orientation.y;
    _highState.imu.quaternion[3] = msg.pose[robot_index].orientation.z;

    _highState.imu.gyroscope[0] = msg.twist[robot_index].angular.x;
    _highState.imu.gyroscope[1] = msg.twist[robot_index].angular.y;
    _highState.imu.gyroscope[2] = msg.twist[robot_index].angular.z;
}

void CheatIO::LJ0Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[6].mode = msg.mode;
    _highState.motorState[6].q = msg.q;
    _highState.motorState[6].dq = msg.dq;
    _highState.motorState[6].tauEst = msg.tauEst;
}

void CheatIO::LJ1Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[7].mode = msg.mode;
    _highState.motorState[7].q = msg.q;
    _highState.motorState[7].dq = msg.dq;
    _highState.motorState[7].tauEst = msg.tauEst;
}
void CheatIO::LJ2Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[8].mode = msg.mode;
    _highState.motorState[8].q = msg.q;
    _highState.motorState[8].dq = msg.dq;
    _highState.motorState[8].tauEst = msg.tauEst;
}

void CheatIO::LJ3Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[9].mode = msg.mode;
    _highState.motorState[9].q = msg.q;
    _highState.motorState[9].dq = msg.dq;
    _highState.motorState[9].tauEst = msg.tauEst;
}

void CheatIO::LJ4Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[10].mode = msg.mode;
    _highState.motorState[10].q = msg.q;
    _highState.motorState[10].dq = msg.dq;
    _highState.motorState[10].tauEst = msg.tauEst;
}

void CheatIO::LJ5Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[11].mode = msg.mode;
    _highState.motorState[11].q = msg.q;
    _highState.motorState[11].dq = msg.dq;
    _highState.motorState[11].tauEst = msg.tauEst;
}

void CheatIO::RJ0Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[0].mode = msg.mode;
    _highState.motorState[0].q = msg.q;
    _highState.motorState[0].dq = msg.dq;
    _highState.motorState[0].tauEst = msg.tauEst;
}

void CheatIO::RJ1Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[1].mode = msg.mode;
    _highState.motorState[1].q = msg.q;
    _highState.motorState[1].dq = msg.dq;
    _highState.motorState[1].tauEst = msg.tauEst;
}
void CheatIO::RJ2Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[2].mode = msg.mode;
    _highState.motorState[2].q = msg.q;
    _highState.motorState[2].dq = msg.dq;
    _highState.motorState[2].tauEst = msg.tauEst;
}

void CheatIO::RJ3Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[3].mode = msg.mode;
    _highState.motorState[3].q = msg.q;
    _highState.motorState[3].dq = msg.dq;
    _highState.motorState[3].tauEst = msg.tauEst;
}

void CheatIO::RJ4Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[4].mode = msg.mode;
    _highState.motorState[4].q = msg.q;
    _highState.motorState[4].dq = msg.dq;
    _highState.motorState[4].tauEst = msg.tauEst;
}

void CheatIO::RJ5Callback(const unitree_legged_msgs::MotorState &msg)
{
    _highState.motorState[5].mode = msg.mode;
    _highState.motorState[5].q = msg.q;
    _highState.motorState[5].dq = msg.dq;
    _highState.motorState[5].tauEst = msg.tauEst;
}
