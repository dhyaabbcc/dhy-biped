#ifndef BIP_STATE_DATAFLOW_H
#define BIP_STATE_DATAFLOW_H

#include "../../include/common/Math/bip_se3.h"


struct bipWbcCtrlData {

public:

    /**
     * @brief Desired body position in world frame.
     */
    Vec3<float> pBody_des;

    /**
     * @brief Desired body velocity in world frame.
     */
    Vec3<float> vBody_des;

    /**
     * @brief Desired body acceleration in world frame.
     */
    Vec3<float> aBody_des;

    /**
     * @brief Desired body roll pitch yaw.
     */
    Vec3<float> pBody_RPY_des;

    /**
     * @brief Desired body angular velocity in world frame.
     */
    Vec3<float> vBody_Ori_des;

    /**
     * @brief Desired foothold position in world frame.
     */
    Vec6<float> pFoot_des[2];

    /**
     * @brief Desired foothold velocity in world frame.
     */
    Vec6<float> vFoot_des[2];

    /**
     * @brief Desired foothold acceleration in world frame.
     */
    Vec6<float> aFoot_des[2];

    /**
     * @brief Desired foothold force in world frame.
     */
    Vec6<float> Fr_des[2];

    /**
     * @brief Current contact state of 4 foothold.
     */
    Vec2<bool> contact_state;

    /**
     * @brief Whether to conduct WBC.
     * If MPC and WBC are conducted in one iteration, this iteration will consume so much time,
     * so if MPC is conducted in this iteration, WBC will be conducted and vice versa.
     */
    bool allowAfterMPC = true;

};





#endif