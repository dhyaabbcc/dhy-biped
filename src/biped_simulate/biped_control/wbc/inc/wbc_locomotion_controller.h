#ifndef QR_WBC_LOCOMOTION_CONTROLLER_H
#define QR_WBC_LOCOMOTION_CONTROLLER_H

#include "multitask_projection.h"
#include "bip_wholebody_impulse_ctrl.hpp"
#include "bip_contact.h"
#include "taskset/Bipedtask.h"
#include "taskset/Task_bodyori.h"
#include "taskset/Task_bodypos.h"
#include "taskset/Task_linkpos.h"
#include "common/enumClass.h"
#include "wbc_data_flow.h"
#include "pinocchio/algorithm/joint-configuration.hpp"

template<typename T>
class bipWBCLocomotionControl {

public:

    /**
     * @brief Constructor of class bipWBCLocomotionControl .
     * @param fb_model: the MIT floating base model.
     * @param control_fsm_data: some information of robot, like gait and estimators.
     */
    bipWBCLocomotionControl(ControlFSMData *_data);

    /**
     * @brief Destructor of class bipWBCLocomotionControl .
     */
    ~bipWBCLocomotionControl();

    /**
     * @brief Compute desired joint position and velocity using null-space projection,
     * then caculate the desired torque by QP formulation and make it into torque conmmands.
     * @param precomputeData: pointer to qrWbcCtrlData @see qrWbcLocomotionCtrl::wbcCtrlData
     */
    void Run(void *precomputeData);

protected:

    /**
     * @brief Update the floating base model dynamics.
     * @param robot: the robot class for update.
     */
    void UpdateModel();

    /**
     * @brief Update the task classes, let the leg joint.
     * statisfied the reaction force if the leg stance.
     * Maybe the controlFSMData need to be deleted.
     * @param ctrlData: pointer to wbcCtrlData, desired state of the robot.
     * @param controlFSMData: pointer to controlFSMData that stores some information of robot like gait and estimators.
     */
    void ContactTaskUpdate(bipWbcCtrlData *ctrl_data);

    /**
     * @brief Write the torque into member jointTorqueCmd.
     * @param control_fsm_data: stores desired command of robot
     */
    void UpdateLegCMD();

    /**
     * @brief Task that set the body position. Used in multi-task.
     * @see qrTaskBodyOrientation
     */
    BipTask<T>* taskBodyPos;

    /**
     * @brief Task that set the body orientation. Used in multi-task.
     * @see qrTaskBodyOrientation
     */
    BipTask<T>* taskBodyOri;

    /**
     * @brief Task that set foothold position. Used in multi-task.
     * @see qrTaskLinkPosition.
     */
    BipTask<T>* taskFootPos[2];

    /**
     * @brief Foot contact states.
     * @see qrSingleContact.
     */
    bipSingleContact<T>* footContact[2];

    /**
     * @brief Contact constraints. The list includes contact constraints of the current contact footholds.
     */
    std::vector<bipSingleContact<T> *> contactList;

    /**
     * @brief Prioritized tasks including body orientation and position and link positions.
     */
    std::vector<BipTask<T> *> taskList;

    /**
     * @brief Dimension of configuration, 6 floating base and 12 joints.
     */
    const size_t dimConfig;

    /**
     * @brief Stores desired robot state and MPC results.
     */
    //WbcCtrlData* wbcCtrlData;

    /**
     * @brief Pointer to controlFSMData, which stores some information like gait and estimators.
     */
    ControlFSMData* controlFSMData;

    /**
     * @brief Compute position, velocity and acceleration commands with null space projection.
     * @see qrMultitaskProjection
     */
    bipMultitaskProjection<T>* multitask;

    /**
     * @brief Whole body impulse controller which is used to calculate a relaxation target.
     */
    bipWBCimpulseCtrl<T>* wbic;

    /**
     * @brief Extra data, including output of QP problem and weight for QP formulation.
     */
    bipWBICExtraData <T>* wbicExtraData;

    /**
     * @brief Pointer to MIT floating base model.
     * Used to get mass matrix, coriolis matrix and other information used in multitask.
     */
    std::shared_ptr<RobotWrapper> FB_Model;
    std::shared_ptr<pinocchio::Data> FB_Data;

    /**
     * @brief Currunt joint states with floating base.
     */
    DVec<T> fullConfig;
    DVec<T> jointvel; 

    /**
     * @brief Output joint torque commands from WBC control.
     */
    DVec<T> jointTorqueCmd;

    /**
     * @brief Desired joint positions computed from multitask.
     */
    DVec<T> desiredJPos;

    /**
     * @brief Desired joint velocity computed from multitask.
     */
    DVec<T> desiredJVel;
    
    /**
     * @brief Counter for locomotion loop. Also used to adjust the frequency of WBC calculate.
     */
    unsigned long long iteration;

    /**
     * @brief 3 dim zero vector
     */
    Vec3<T> zeroVec3;

    DataLogger &logger = DataLogger::GET();
};





#endif