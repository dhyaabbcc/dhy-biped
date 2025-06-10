#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

//#include "pinocchio/fwd.hpp"
#include <vector>
#include "cppTypes.h"
#include "Math/orientation_tools.h"
#include "../../robotwrapper/MyWrapper.hpp"


#define PINOCCHIO_BIPED "src/biped_simulate/biped_gazebo/urdf/biped_gazebo.urdf"




class Biped
{
public:
    Biped(std::shared_ptr<MyWrapper>rptr,std::shared_ptr<pinocchio::Data>dptr):_Dyptr(rptr),_Dataptr(dptr)
    {
        setBiped();
    }
    ~Biped()
    {
    }
    void setBiped()
    {

        mass = 63.6;
        waistwidth = 0.310 / 2;

        leg_offset_x = 0.0;
        leg_offset_y = waistwidth ; // 
        leg_offset_z = -0.23243;     

        thighLinkLength = 0.460; //
        calfLinkLength = 0.37;
        toeLinkLength = 0.123;
        
        std::cout<<"setbiped"<<'\n';
        axis.push_back(Vec3<double>(0,0,1));
        axis.push_back(Vec3<double>(1,0,0));
        axis.push_back(Vec3<double>(0,1,0));
        axis.push_back(Vec3<double>(0,1,0));
        axis.push_back(Vec3<double>(0,1,0));
        axis.push_back(Vec3<double>(1,0,0));

        axisname.push_back(ori::CoordinateAxis::Z);
        axisname.push_back(ori::CoordinateAxis::X);
        axisname.push_back(ori::CoordinateAxis::Y);
        axisname.push_back(ori::CoordinateAxis::Y);
        axisname.push_back(ori::CoordinateAxis::Y);
        axisname.push_back(ori::CoordinateAxis::X);
    }
    double toeLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double mass;
    double waistwidth;
    double leftAnkleOffset;
    std::vector<Vec3<double>> axis;
    std::vector<ori::CoordinateAxis> axisname;
    std::array<double,6> Initialq{0,0,-0.338,0.7616,-0.425,0};

    //control config
    double comHeight=0.85;
    double maxCoMHeight=0.90;
    double minCoMHeight=0.65;
    double postureStiffness=1.0;
    double postureWeight=10.0;


    Vec3<double> getHipLocation(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0)  
        {
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        return pHip;
    };
     Vec3<double> getHip2Location(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 1)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 0)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        return pHip;
    };


    
    std::shared_ptr<MyWrapper> _Dyptr;
    std::shared_ptr<pinocchio::Data> _Dataptr;
protected:
  
    friend class FSMState_Tstand;
    friend class FSMState_Walking;
};



#endif
