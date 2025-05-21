#ifndef TASK_BODYPOS_H
#define TASK_BODYPOS_H

#include "Bipedtask.h"
#include "../../../robotwrapper/robotwrapper.h"
#include "common/ControlFSMData.h"
#include "common/Math/bip_se3.h"

template<typename T>
class BipTaskBodyPos:public BipTask <T>{

public:

    /**
     * @brief Constructor of class BipTaskBodyOri.
     * @param fbModel: pointer to MIT floating base model.
     */
    BipTaskBodyPos(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_data);

    virtual ~BipTaskBodyPos() = default;

    /**
     * @brief A scale factor that will mutiply posErr.
     */
    DVec<T> errScale;

    /**
     * @brief KP for position gains.
     * Used in PD control to get acceleration command.
     */
    DVec<T> Kp;

    /**
     * @brief KP for velocity gains
     * Used in PD control to get acceleration command.
     */
    DVec<T> Kd;

protected:

    /**
     * @see qrTask::UpdateCommand
     */
    virtual bool UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc)override;

    /**
     * @see qrTask::UpdateTaskJacobian
     */
    virtual bool UpdateTaskJacobian() override;

    /**
     * @see qrTask::UpdateTaskJDotQdot
     * @attention In bod orientation task, just set JDotQdot to zero matrix.
     */
    virtual bool UpdateTaskJDotQdot() override;

    std::shared_ptr<RobotWrapper> FB_Model;
    std::shared_ptr<pinocchio::Data> FB_Data;
    ControlFSMData *_data;

};




#endif