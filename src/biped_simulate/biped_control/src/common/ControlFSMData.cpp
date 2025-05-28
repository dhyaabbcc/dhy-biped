#include "../../include/common/ControlFSMData.h"

void loadFootstepPlan(FootstepPlan &plan)
{
    pinocchio::SE3 X_0_lf, X_0_rf;
    X_0_lf = _Dataptr->oMf[_Dyptr->model().getFrameId("lcontactpoint")];
    X_0_rf = _Dataptr->oMf[_Dyptr->model().getFrameId("rcontactpoint")];
    double initHeight = X_0_lf.translation().z();
    plan.updateInitialTransform(X_0_lf, X_0_rf, initHeight);
    plan.rewind();
}

//estimator_需要替代realrobot实现功能
void updateRealFromKinematics()
{
  baseObs_.updateRobot(estimator_);
  realCom_ = estimator_.com();
  if(!estimator_.leftFootRatioJumped_)
  {
    comVelFilter_.update(realCom_);
  }
  else // don't update velocity when CoM position jumped  突变时仅更新位置：1.步态切换2.外力冲击
  {
    comVelFilter_.updatePositionOnly(realCom_);
    estimator_.leftFootRatioJumped_ = false;
  }
  realComd_ = comVelFilter_.vel();
}

void resetController(MyWrapper & robot,
                     tsid::InverseDynamicsFormulationAccForce & tsid,
                     pinocchio::Model & model,
                     pinocchio::Data & data,
                     StateEstimator & estimator,
                     StateEstimatorContainer &estimatorcontainer
                     FootstepPlanner & planner,
                     PendulumModel & pendulum,
                     CoMVelocityFilter & comVelFilter,
                     FloatingBaseObserver & baseObs,
                     NetWrenchObserver & wrenchObs,
                     Stabilizer & stabilizer,
                     LowlevelState& lowState,
                     ModelPredictiveControl& mpc,
                     double leftFootRatio = 0.5)
{
    // --- (1) 设置浮动基位姿，利用双足估计初始base位姿
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_.nq());
    q.head<3>() = Eigen::Vector3d(0, 0, -1.12);    // 位置 (x,y,z) 1.12
    q.segment<4>(3) = Eigen::Vector4d(1, 0, 0, 0); // 旋转 (单位四元数)
    q[7] = -0.338;                                 // r_j0
    q[10] = 0.762;                                 // r_j3
    q[11] = -0.419;                                // r_j4
    q[12] = -0.338;                                // l_j0
    q[15] = 0.762;                                 // l_j3
    q[16] = -0.419;                                // l_j4
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_.nv());

    robot_.model().setData(model_, data_);
    robot_.model().setRobotState(q, v);

    // --- (2) 同步 contact 规划器的支持面位姿
    loadFootstepPlan(planner_);

    // --- (3) TSID 任务重置
    Eigen::VectorXd halfSit = q.segment(7,12);

    auto postureTask = std::dynamic_pointer_cast<tsid::tasks::TaskPosture>(tsid_.getTask("posture"));
    if(postureTask)
        postureTask->setReference(halfSit);

    stabilizer_.reset();

    // --- (4) 控制状态变量重置
    estimator_.leftFootRatioJumped_ = true;
    estimator_.leftFootRatio = leftFootRatio_;
    estimator_.pauseWalking = false;
    estimator_.pauseWalkingRequested = false;
    estimator_.nbMPCFailures = 0;
    estimator_.lowState=_lowState;

    Eigen::Vector3d com = robot_.com(data_);
    comVelFilter_.reset(com);
    pendulum_.reset(com);

    // --- (5) 浮动基观测器
    pinocchio::SE3 X0_base(
        Eigen::Quaterniond(1, 0, 0, 0),  // 单位四元数
        Eigen::Vector3d(0, 0, -1.12)     // 初始位置 
    );
    baseObs_.reset(X0_base);
    baseObs_.setLeftFootRatio(leftFootRatio_);
    baseObs_.run(estimator_); // 融合 realRobot 状态

    updateRealFromKinematics();  // 估计 realCoM, realCoMd

    // --- (6) 力/状态估计器更新 stabilizer
    wrenchObs_.update(estimator_, estimator_.supportContact());
    stabilizer_.updateState(estimator_.realCoM,
                           estimator_.realCoMd,
                           wrenchObs_.wrench(),
                           leftFootRatio_);

}

void leftFootRatio(double ratio)
{
  double maxRatioVar = 1.5 * timeStep / plan.doubleSupportDuration();
  if(std::abs(ratio - leftFootRatio_) > maxRatioVar)
  {
    estimator_.leftFootRatioJumped_ = true;
  }
  leftFootRatio_ = MyWrapper::clamp(ratio, 0., 1.);
}

//仅报警作用
void warnIfRobotIsInTheAir()
{
  static bool isInTheAir = false;
  constexpr double CONTACT_THRESHOLD = 30.; // [N]
  double leftFootForce = estimator_.footforce[0].z();
  double rightFootForce = estimator_.footforce[1].z();
  if(leftFootForce < CONTACT_THRESHOLD && rightFootForce < CONTACT_THRESHOLD)
  {
    if(!isInTheAir)
    {
      std::cout<<"Robot is in the air"<<std::endl;
      isInTheAir = true;
    }
  }
  else
  {
    if(isInTheAir)
    {
      std::cout<<"Robot is on the ground again"<<std::endl;
      isInTheAir = false;
    }
  }
}

bool updatePreview()
{
  mpc_.initState(pendulum());
  mpc_.comHeight(planner_.comHeight());
  if(mpc_.buildAndSolve())
  {
    preview = mpc_.solution();
    return true;
  }
  else
  {
    nbMPCFailures_++;
    return false;
  }
}