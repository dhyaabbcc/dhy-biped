#include "../../include/common/ControlFSMData.h"

void ControlFSMData::internalReset()
{
    // --- (1) 设置浮动基位姿，利用双足估计初始base位姿
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_->nq());
    q.head<3>() = Eigen::Vector3d(0, 0, -1.12);    // 位置 (x,y,z) 1.12
    q.segment<4>(3) = Eigen::Vector4d(1, 0, 0, 0); // 旋转 (单位四元数)
    q[7] = -0.338;                                 // r_j0
    q[10] = 0.762;                                 // r_j3
    q[11] = -0.419;                                // r_j4
    q[12] = -0.338;                                // l_j0
    q[15] = 0.762;                                 // l_j3
    q[16] = -0.419;                                // l_j4
    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_->nv());
    robot_->computeAllTerms(*data_, q, v);

     std::cout << "computeAllTerms" << std::endl;

    for (int i = 0; i < 12; i++)
    {
      _lowCmd.motorCmd[i].q = q[7 + i] - _biped.Initialq[i];
    }
    // --- (2) 同步 contact 规划器的支持面位姿
    std::cout << "_lowCmd.motorCmd" << std::endl;
    loadFootstepPlan(plan_);
    std::cout << "loadFootstepPlan_end" << std::endl;


    // --- (3) TSID 任务重置
    postureTask_ = std::make_shared<tsid::tasks::TaskJointPosture>("posture_task", *robot_);

    Eigen::VectorXd halfSit = q.segment(7,12);
    tsid::trajectories::TrajectorySample ref(halfSit.size());
    ref.setValue(halfSit);
    postureTask_->setReference(ref);

    stabilizer_.reset(*robot_);

    std::cout << "stabilizer_.reset_end" << std::endl;

    // --- (4) 控制状态变量重置

    leftFootRatioJumped_ = true;
    leftFootRatio_ = 0.5;
    nbMPCFailures_ = 0;
    pauseWalking = false;
    pauseWalkingRequested = false;

    Eigen::Vector3d com = robot_->com(*data_);
    comVelFilter_.reset(com);
    pendulum_.reset(com);

    // --- (5) 浮动基观测器
    pinocchio::SE3 X0_base(
        Eigen::Quaterniond(1, 0, 0, 0),  // 单位四元数
        Eigen::Vector3d(0, 0, -1.12)     // 初始位置 
    );

    baseObs_.reset(X0_base);
    baseObs_.setLeftFootRatio(leftFootRatio_);
    baseObs_.run(); 
    std::cout << " baseObs_.run()_end" << std::endl;

    updateRealFromKinematics();  // 估计 realCoM, realCoMd

    std::cout << " updateRealFromKinematics_end" << std::endl;

    // --- (6) 力/状态估计器更新 stabilizer
    wrenchObs_.update(_lowState, supportContact());
    stabilizer_.updateState(realCom_,
                           realComd_,
                           wrenchObs_.wrench(),
                           leftFootRatio_);
    std::cout << " internalReset_end" << std::endl;

}

void ControlFSMData::loadFootstepPlan(FootstepPlan &plan)
{
    pinocchio::SE3 X_0_lf, X_0_rf;
    X_0_lf = data_->oMf[robot_->model().getFrameId("lcontactpoint")];
    X_0_rf = data_->oMf[robot_->model().getFrameId("rcontactpoint")];
    double initHeight = X_0_lf.translation().z();
    plan.complete(sole_);
     std::cout << "plan.complete(sole_);" << std::endl;
      std::cout << X_0_lf<< X_0_rf<<initHeight<< std::endl;
    plan.updateInitialTransform(X_0_lf, X_0_rf, initHeight);
     std::cout << " plan.updateInitialTransform" << std::endl;
    plan.rewind();
    std::cout << " plan.rewind" << std::endl;
}

//estimator_需要替代realrobot实现功能
void ControlFSMData::updateRealFromKinematics()
{
  baseObs_.run();
  realCom_ = baseObs_.position();
  if(!leftFootRatioJumped_)
  {
    comVelFilter_.update(realCom_);
  }
  else // don't update velocity when CoM position jumped  突变时仅更新位置：1.步态切换2.外力冲击
  {
    comVelFilter_.updatePositionOnly(realCom_);
    leftFootRatioJumped_ = false;
  }
  realComd_ = comVelFilter_.vel();
}

void ControlFSMData::leftFootRatio(double ratio)
{
  double maxRatioVar = 1.5 * timeStep / plan_.doubleSupportDuration();
  if(std::abs(ratio - leftFootRatio_) > maxRatioVar)
  {
    leftFootRatioJumped_ = true;
  }
  leftFootRatio_ = MyWrapper::clamp(ratio, 0., 1.);
}

//仅报警作用
void ControlFSMData::warnIfRobotIsInTheAir()
{
  static bool isInTheAir = false;
  constexpr double CONTACT_THRESHOLD = 30.; // [N]
  double leftFootForce = _lowState.feettwist[8];
  double rightFootForce = _lowState.feettwist[2];
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

bool ControlFSMData::updatePreview()
{
  mpc_.initState(pendulum());
  mpc_.comHeight(plan_.comHeight());
  if(mpc_.solve())
  {
    preview_ = mpc_.solution();
    return true;
  }
  else
  {
    nbMPCFailures_++;
    return false;
  }
}

void ControlFSMData::pauseWalkingCallback(bool verbose)
{
  constexpr double MAX_HEIGHT_DIFF = 0.02; // [m]
  if(pauseWalking)
  {
    std::cout << "Already pausing, how did you get there?" << std::endl;
    return;
  }
  else if(std::abs(supportContact().z() - targetContact().z()) > MAX_HEIGHT_DIFF)
  {
    if(!pauseWalkingRequested || verbose)
    {
      std::cout <<"Cannot pause on uneven ground, will pause later" << std::endl;
    }
    pauseWalkingRequested = true;
  }
  else if(pauseWalkingRequested)
  {
    std::cout <<"Pausing now that contacts are at same level" << std::endl;
    pauseWalkingRequested = false;
    pauseWalking = true;
  }
  else // (!pauseWalkingRequested)
  {
    pauseWalking = true;
  }
}

void ControlFSMData::tsidsolve(){

  //求解
  double t=0.0;       //如果task的轨迹是时间相关的，则t需要改为外部控制器时间----t=0等于每次求解的目标值都是目标轨迹的初始值
  Eigen::VectorXd q=Eigen::VectorXd::Zero(robot_->nq());
  Eigen::VectorXd dq=Eigen::VectorXd::Zero(robot_->nv());
  auto &seResult = this->_stateEstimator->getResult();

  Eigen::Quaterniond quat(baseObs_.orientation());
  quat.normalize();
  q.head<3>() = baseObs_.position();
  q.segment<4>(3) << quat.w(), quat.x(), quat.y(), quat.z();
  dq.head<3>().setZero();
  dq.segment<3>(3) = seResult.omegaWorld;
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      q[6 * i + 7 + j] = _legController->data[i].q[j] +  _biped.Initialq[j];// 可能需要加初始值q
      dq[6 * i + 6 + j] = _legController->data[i].qd[j];
    }
  }

  std::cout << "编码器 q" << q << std::endl;

  tsid::solvers::HQPData hqpData = tsid_.computeProblemData(t, q, dq);
  tsid::solvers::SolverHQuadProg solver("qp_solver");
  solver.resize(tsid_.nVar(), tsid_.nEq(), tsid_.nIn());

  tsid::solvers::HQPOutput result = solver.solve(hqpData);

  Eigen::VectorXd dv = tsid_.getAccelerations(result);

  std::cout << "Optimal joint accelerations dv:\n"
            << dv.transpose() << std::endl;

  std::cout << "TSID controller initialized with CoM task." << std::endl;
  //求解完成

  // 数值积分
  dq += dv * timeStep;
  q = pinocchio::integrate(_biped._Dyptr->model(), q, dq * timeStep); // model 必须匹配

  std::cout << "TSID result q" << q << std::endl;

  // 可选：更新
  for (int i = 0; i < 12; i++)
  {
    _lowCmd.motorCmd[i].q = q[7 + i] - _biped.Initialq[i];
    _lowCmd.motorCmd[i].dq = dq[6 + i]; // 速度可以不要
  }

  robot_->computeAllTerms(*data_, q, dq);

  std::cout << "TSID integration done. q and dq updated." << std::endl;
}