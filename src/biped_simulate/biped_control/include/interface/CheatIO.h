#ifndef CHEATIO_H
#define CHEATIO_H

#include "ros/ros.h"
#include <ros/time.h>
#include<boost/array.hpp>
#include "IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_msgs/HighState.h"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <geometry_msgs/Twist.h>


class CheatIO : public IOInterface
{
    public:
        CheatIO(std::string robot_name);
        ~CheatIO();
        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state)override;
      
    private:
        void sendCmd(const LowlevelCmd *cmd);
        void recvState(LowlevelState *state);
        void recvfeetState(LowlevelState *state);
        ros::NodeHandle _nm;
        ros::Subscriber _servo_sub[12], _state_sub,linkstate_sub;
        ros::Publisher _servo_pub[12];
        unitree_legged_msgs::LowCmd _lowCmd;
        unitree_legged_msgs::HighState _highState;
        geometry_msgs::Twist Foottwist[2];
        geometry_msgs::Pose Footpose[2];

        std::string _robot_name;
        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void StateCallback(const gazebo_msgs::ModelStates & msg);
        void LinkStatesCallback(const gazebo_msgs::LinkStates &msgs);

        void LJ0Callback(const unitree_legged_msgs::MotorState& msg);
        void LJ1Callback(const unitree_legged_msgs::MotorState& msg);
        void LJ2Callback(const unitree_legged_msgs::MotorState& msg);
        void LJ3Callback(const unitree_legged_msgs::MotorState& msg);
        void LJ4Callback(const unitree_legged_msgs::MotorState& msg);
        void LJ5Callback(const unitree_legged_msgs::MotorState& msg);

        void RJ0Callback(const unitree_legged_msgs::MotorState& msg);
        void RJ1Callback(const unitree_legged_msgs::MotorState& msg);
        void RJ2Callback(const unitree_legged_msgs::MotorState& msg);
        void RJ3Callback(const unitree_legged_msgs::MotorState& msg);
        void RJ4Callback(const unitree_legged_msgs::MotorState& msg);
        void RJ5Callback(const unitree_legged_msgs::MotorState& msg);
   

};   

#endif