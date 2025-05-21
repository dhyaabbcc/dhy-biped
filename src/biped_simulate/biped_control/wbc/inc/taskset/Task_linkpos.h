

#ifndef TASK_LINK_POS
#define TASK_LINK_POS

#include "Bipedtask.h"
#include "../../../robotwrapper/robotwrapper.h"
#include "common/ControlFSMData.h"
#include "common/Math/bip_se3.h"

template<typename T>
class bipTaskLinkPos : public BipTask <T> {

public:

    /**
     * @brief Constructor of class qrTaskLinkPosition.
     * @param fbModel: pointer to MIT floating base model.
     * @param legindex: 0 for right foot;1 for left foot
     *
     */
    bipTaskLinkPos(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_Data,short legindex);

    virtual ~bipTaskLinkPos() = default;

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
    
    /**
     * @brief 右腿0，左腿1
     * 
     */
    short LEGindex;

protected:

    /**
     * @see qrTask::UpdateCommand
     */
    virtual bool UpdateCommand(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc);

    /**
     * @see qrTask::UpdateTaskJacobian
     */
    virtual bool UpdateTaskJacobian();

    /**
     * @see qrTask::UpdateTaskJDotQdot
     */
    virtual bool UpdateTaskJDotQdot();

    /**
     * @brief Pointer to MIT floating base model.
     * Used to get Jc and JDotQDot.
     */
    std::shared_ptr<RobotWrapper> FB_Model;
    std::shared_ptr<pinocchio::Data> FB_Data;
    ControlFSMData *_data;
    /**
     * @brief The link index of current link position task.
     */
    pinocchio::FrameIndex linkindex;

    // /**
    //  * @brief 右腿0，左腿1
    //  * 
    //  */
    // short LEGindex;

    /**
     * @brief If the link virtual depend. the jacobian will relate to floating base.
     */
    bool virtualDepend;
};

#endif // QR_TASK_LINK_POSITION_H
