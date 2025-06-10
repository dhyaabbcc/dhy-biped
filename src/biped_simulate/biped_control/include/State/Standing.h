#pragma once

#include "State.h"

struct Standing : BaseState
{
public:
    void start() override;

    void teardown() override;

    bool checkTransitions() override;

    void runState() override;

    void updateTarget(double leftFootRatio);

    void startWalking();

private:
    Contact leftFootContact_;   /**< Current left foot contact handle in plan */
    Contact rightFootContact_;  /**< Current right foot contact handle in plan */
    Eigen::Vector3d copTarget_; /**< CoP target computed from GUI input */
    bool isMakingFootContact_;  /**< Is the robot going back to double support? */
    bool startWalking_;         /**< Has the user clicked on "Start walking"? */
    double leftFootRatio_;      /**< Left foot ratio from GUI input */
    double releaseHeight_;      /**< Desired height when lifting one foot in the air */
    Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
};