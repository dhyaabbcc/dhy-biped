
#ifndef CHECK_JOINT_H
#define CHECK_JOINT_H
#include <vector>
#include "../interface/CheatIO.h"
#include "../interface/IOInterface.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "LegController.h"
#include <iostream>
#include <memory>
#include <cmath>
#include "../../ConvexMPC/GaitGenerator.h"
#include "FootSwingTrajectory.h"
#include "ros/ros.h"
#include <ros/time.h>
#include "std_msgs/Float32.h"
#include "StateEstimatorContainer.h"

enum class JOINT
{
    RJ0,
    RJ1,
    RJ2,
    RJ3,
    RJ4,
    RJ5,
    LJ0,
    LJ1,
    LJ2,
    LJ3,
    LJ4,
    LJ5
};
using std::shared_ptr;

class Checkjoint
{
public:
    Checkjoint() = default;
    Checkjoint(shared_ptr<IOInterface> io,shared_ptr<LegController> ctrl,LowlevelCmd* cmdex,LowlevelState* statex,
    shared_ptr<StateEstimatorContainer> esptr) : ioptr(io),statectrl(ctrl),_stateEstimator(esptr)
    {
        for(int i=0;i<2;i++) statectrl->data[i].zero();
        walking=std::make_shared<Gait>(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking");
        standing=std::make_shared<Gait>(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing");
        dtMPC = dt * iterationsBetweenMPC;
        _lowCmd=cmdex;
        _lowState=statex;
        plotsub[0]=_nc.advertise<std_msgs::Float32>( "plot1", 10);
        plotsub[1]=_nc.advertise<std_msgs::Float32>( "plot2", 10);
        plotsub[2]=_nc.advertise<std_msgs::Float32>( "plot3", 10);
        Footpos[0].setZero();
        Footpos[1].setZero();

    }
    ~Checkjoint()
    {
        _lowCmd=nullptr;
        _lowState=nullptr;

    }
    void checkgait();

    void checkgait1();

    void staystill()
    {
        ioptr->sendRecv(_lowCmd,_lowState);
        statectrl->updateData(_lowState);
        for(int i=0;i<6;i++)
        {
            statectrl->commands[0].qDes(i)=0;
            statectrl->commands[0].qdDes(i)=0;
            statectrl->commands[1].qDes(i)=0;
            statectrl->commands[1].qdDes(i)=0;
        }

         statectrl->updatePosctrl(_lowCmd);
        
    }
   
    void checkOnejoint(unsigned short joint)
    {
        ioptr->sendRecv(_lowCmd,_lowState);
        statectrl->updateData(_lowState);
        float q,v,qmax=1.57,dq,dqdes;
        v=qmax/2;
        if(joint<=static_cast<unsigned short>(JOINT::RJ5)) 
        {
            q=statectrl->data[0].q(joint);
            dq=statectrl->data[0].qd(joint);
        }
        else
        {q=statectrl->data[1].q(joint%6);
        dq=statectrl->data[1].qd(joint%6);
        } 
        if(qdes>=0&&qdes<=1&&up>0) 
            {qdes+=v*dt;dqdes=v;}
        else if(qdes>=0&&qdes<=1&&up<0)
            {qdes-=v*dt;dqdes=-v;}
        else 
        {
            up*=-1;
            qdes+=up*0.005;
        }

         if(joint<=static_cast<unsigned short>(JOINT::RJ5)) 
         {
            statectrl->commands[0].qDes(joint)=qdes;
            statectrl->commands[0].qdDes(joint)=dqdes;
         }
        else
        {
            statectrl->commands[1].qDes(joint%6)=qdes;
            statectrl->commands[1].qdDes(joint%6)=dqdes;
        }

        statectrl->commands[1].qDes(5)=0.3;
        statectrl->commands[1].qdDes(5)=0;
        statectrl->updatePosctrl(_lowCmd);
    }
    void plot_publish(int i,float value)
    {
        std_msgs::Float32 Val;
        Val.data=value;
        plotsub.at(i).publish(Val);
    }

    ros::NodeHandle _nc;
    std::array<ros::Publisher,3>  plotsub;
    shared_ptr<Gait> walking, standing;
    int iterationsBetweenMPC=40;
    int horizonLength=10;
    FootSwingTrajectory<double> footSwingTrajectories[2];
    double dt=0.001;
    double dtMPC;
    bool firstRun = true;
    Vec3<double> world_position_desired;
    int iterationCounter = 0;
    Vec3<double> pFoot[2];
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    shared_ptr<LegController> statectrl;
    shared_ptr<IOInterface> ioptr;
    Vec2<double> swingTimes;
    std::array<Vec3<float>,2> Footpos;
    shared_ptr<StateEstimatorContainer> _stateEstimator;
    Vec3<double> pDesFootWorld[2] ;
    Vec3<double> vDesFootWorld[2] ;
    Vec3<double> pDesFoot[2] ;
    Vec3<double> vDesFoot[2] ;

    bool firstSwing[2] = {true, true};
    
    double swingTimeRemaining[2];
    float qdes=0;
    short up=1;
};

#endif
