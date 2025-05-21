#ifndef BIPED_TASK_H
#define BIPED_TASK_H

//#include "../../robotwrapper/robotwrapper.h"
#include "common/cppTypes.h"
#include <iostream>
#define BT BipTask<T>

template<typename T>
class BipTask {

public:

    /**
     * @brief Constructor of class task.
     * @param dim: the dimension of the task.
     */
    BipTask(size_t dim):
        dimTask(dim),
        xddotCmd(dim),
        posErr(dim),
        desiredVel(dim),
        desiredAcc(dim) {
    }

    virtual ~BipTask() = default;

    /**
     * @brief Update the task constraint, including Jt, JtDotQdot and xddotCmd.
     * @param des_pos: desired position of the task.
     * @param des_vel: desired velocity of the task.
     * @param des_acc: desired acceleration of the task.
     * @return true if update has finished
     */
    bool UpdateTask(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc)
    {
       // std::cout<<"BeFORE1\n";
        UpdateTaskJacobian();
       // std::cout<<"BeFORE2\n";
        UpdateTaskJDotQdot();

	   // std::cout<<"BeFORE3\n";
        UpdateCommand(des_pos, des_vel, des_acc);
	   // std::cout<<"AFTER\n";
        return true;
    }

    /**
     * @brief Getter method of member xddotCmd.
     */
    void GetXddotCmd(DVec<T> &xddot_cmd) const {
        xddot_cmd = this->xddotCmd;
    }

    /**
     * @brief Getter method of member Jt.
     */
    void GetJt(DMat<T> &Jt) const {
        Jt = this->Jt;
    }

    /**
     * @brief Getter method of member JtDotQdot.
     */
    void GetJtDotQdot(DVec<T> &JtDot_Qdot) const {
        JtDot_Qdot = this->JtDotQdot;
    }

    /**
     * @brief Getter method of member posErr.
     */
    const DVec<T> &GetPosErr() const {
        return posErr;
    }

    /**
     * @brief Getter method of member desiredVel.
     */
    const DVec<T> &GetDesiredVel() const {
        return desiredVel;
    }

    /**
     * @brief Getter method of member desiredAcc.
     */
    const DVec<T> &GetDesiredAcc() const {
        return desiredAcc;
    }

public:

    /**
     * @brief Update the desired acceleration command or position error if needed.
     * @return true if update has finished.
     */
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) = 0;

    /**
     * @brief Update task jacobian.
     * @return true if update has finished.
     */
    virtual bool UpdateTaskJacobian() = 0;

    /**
     * @brief Update JtDotQdot. JtDotQDdot is usually from MIT floating base model.
     * @return true if update has finished.
     */
    virtual bool UpdateTaskJDotQdot() = 0;

    /**
     * @brief The dimension of the task.
     * If dim=3, the robot needs to satisfy command on all three directions.
     */
    size_t dimTask;
    /**
     * @brief The optimized acceleration command.
     * The acceleration command is computed from PD control of desired position and desired velocity.
     */
    DVec<T> xddotCmd;
    /**
     * @brief Derivative of Jt dot Derivative of q.
     * Used in null-space projection.
     */
    DVec<T> JtDotQdot;
    /**
     * @brief Task jacobian.
     */
    DMat<T> Jt;
    /**
     * @brief Position error of the task. Will be used in null-space projection.
     * Computed from (desired position/orientation - current  position/orientation.)
     */
    DVec<T> posErr;
    /**
     * @brief Desired velocity of the task.
     * Used in PD control to compute the acceleration command.
     */
    DVec<T> desiredVel;
    /**
     * @brief Desired acceleration of the task.
     * Used in PD control to compute the acceleration command.
     */
    DVec<T> desiredAcc;
    /**
     * @brief The dimension of the configuration space.
     * Including 6 dimension of floating base and 12 dimension of joints.
     */
    int dimConfig = 18;
    /**
     * @brief 动力学模型
     * 
     */
    // std::shared_ptr<RobotWrapper> _Dyptr;
    // /**
    //  * @brief pinocchio动力学数据对象
    //  * 
    //  */
    // std::shared_ptr<pinocchio::Data> _Dataptr;

};





#endif
