#include "../inc/bip_contact.h"


template<typename T>
bipSingleContact<T>::bipSingleContact(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_data,short CONTACTLEG):
    maxFz(2*12*9.81),
    contact_leg(CONTACTLEG),
    dimU(10),
    mu(0.4f),
    dimContact(6),
    FB_Model(fb_model),
    FB_Data(fb_data)
{
    if(CONTACTLEG==0||CONTACTLEG==1) indexContact=fb_model->m_model.getFrameId(contact_frame[CONTACTLEG]);
    else throw std::runtime_error("Invalid leg id!");

    desiredFr = DVec<T>::Zero(dimContact);
    Jc = DMat<T>(dimContact, 18);
    JcDotQdot = DVec<T>::Zero(dimContact);
    Uf = DMat<T>::Zero(dimU, dimContact);

       /* Uf matrix seems like:
     * |  0   0   1  0  0  0 |
     * |  1   0   mu  0  0  0|
     * | -1   0   mu  0  0  0|
     * |  0   1   mu  0  0  0|
     * |  0  -1   mu  0  0  0|
     * |  0   0  -1  0  0  0|
     * |  0   0  feetwidth  1  0  0|
     * |  0   0  feetwidth  -1  0  0|
     * |  0   0  feetL2  0  1  0|
     * |  0   0  feetL1  0  1  0|
     * 
     */
    Uf(0, 2) = 1.;//Fz>0

    Uf(1, 0) = 1.;//Fx+muFz>0
    Uf(1, 2) = mu;

    Uf(2, 0) = -1.;
    Uf(2, 2) = mu;//-Fx+muFz>0

    Uf(3, 1) = 1.;//Fy+muFz>0
    Uf(3, 2) = mu;

    Uf(4, 1) = -1.;//-Fx+muFz>0
    Uf(4, 2) = mu;

    Uf(5, 2) = -1.;//-Fz>-FzMax

    Uf(6,3)=1;
    Uf(6, 2)=feetwidth; //Mx+Fz*ly/2>0
    
    Uf(7,3)=-1;
    Uf(7,2)=feetwidth; //-Mx+Fz*ly/2>0

    Uf(8,4)=1;
    Uf(8,2)=feetL2;// My+Fz lx2>0

    Uf(9,4)=-1;
    Uf(9,2)=feetL1;// -My+Fz lx1>0

}

template<typename T>
bool bipSingleContact<T>::UpdateContactSpec()
{
    UpdateJc();
    UpdateJcDotQdot();
    UpdateUf();
    UpdateIneqVec();
    return true;
}

template<typename T>
bool bipSingleContact<T>::UpdateJc()
{
    pino::Data::Matrix6x WJ=pino::Data::Matrix6x::Zero(dimContact,18);
    //FB_Model->jacobianWorld_Aligned(*FB_Data,indexContact,WJ);
    pinocchio::getFrameJacobian(FB_Model->m_model,*FB_Data,indexContact,pinocchio::LOCAL_WORLD_ALIGNED,WJ);
    Jc=WJ.cast<T>();
    return true;
}

template<typename T>
bool bipSingleContact<T>::UpdateJcDotQdot()
{
    if(!FB_Model->Driftcalculate_update) return false;
    JcDotQdot.block(0,0,3,1)=FB_Model->frameClassicAcceleration(*FB_Data,indexContact).linear().cast<T>();
    JcDotQdot.block(3,0,3,1)=FB_Model->frameClassicAcceleration(*FB_Data,indexContact).angular().cast<T>();
    return true;
}


template<typename T>
bool bipSingleContact<T>::UpdateUf()
{
    return true;
}

template<typename T>
bool bipSingleContact<T>::UpdateIneqVec()
{
    ineqVec = DVec<T>::Zero(dimU);
    ineqVec[5] = -maxFz;
    return true;
}

template class bipSingleContact<float>;



