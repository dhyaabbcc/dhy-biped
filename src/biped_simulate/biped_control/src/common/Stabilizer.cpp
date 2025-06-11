#include "../../include/common/Stabilizer.h"

using namespace qpOASES;

Stabilizer::Stabilizer(Biped &controlRobot, Pendulum &pendulum, double dt, LowlevelState &state)
    : dcmIntegrator_(dt, /* timeConstant = */ 5.), dcmDerivator_(dt, /* timeConstant = */ 1.), pendulum_(pendulum),
      controlRobot_(controlRobot), dt_(dt), mass_(controlRobot.mass), state_(state)
{
}

void Stabilizer::disable()
{
  comAdmittance_.setZero();
  copAdmittance_.setZero();
  dcmDerivGain_ = 0.;
  dcmIntegralGain_ = 0.;
  dcmPropGain_ = 0.;
  dfzAdmittance_ = 0.;
}

void Stabilizer::reconfigure()
{
  fdqpWeights_.netWrenchSqrt=10000.0;
  fdqpWeights_.ankleTorqueSqrt=100.0;
  fdqpWeights_.forceRatioSqrt=1.0;
  // admittance
  comAdmittance_ = Eigen::Vector2d(0.0, 0.0);
  copAdmittance_ = Eigen::Vector2d(0.01, 0.01);
  dfzAdmittance_ = 0.0001;
  dfzDamping_ = 0.0;
  // dcm_tracking
  dcmPropGain_ = 5.0;
  dcmIntegralGain_ = 10.0;
  dcmDerivGain_ = 0.0;
  dcmDerivator_.timeConstant(1.0);
  dcmIntegrator_.timeConstant(10.0);
  //tasks
  comStiffness_ = Eigen::Vector3d(1000.0, 1000.0, 100.0);
  comWeight_=1000.0;
  double d=300.0;
  double k=1.0;
  contactDamping_ = pinocchio::Motion(Eigen::Vector3d::Constant(d), Eigen::Vector3d::Constant(d)); 
  contactStiffness_ = pinocchio::Motion(Eigen::Vector3d::Constant(k), Eigen::Vector3d::Constant(k));
  swingFootStiffness_= 2000.0;
  swingFootWeight_=500.0;

  zmpccIntegrator_.rate(0.1);
}

void Stabilizer::reset(MyWrapper & robot) 
{
  // === 1. 创建 CoM 任务 ===
  comTask = std::make_shared<tsid::tasks::TaskComEquality>("task-com", robot);
  comTask->Kp(comStiffness_);
  comTask->Kd(2.0 * comStiffness_.cwiseSqrt());

  // === 2. 创建左右足 CoP 任务 ===
  leftFootTask = std::make_shared<tsid::tasks::TaskSE3Equality>("LeftFoot", robot, "lcontactpoint");
  rightFootTask = std::make_shared<tsid::tasks::TaskSE3Equality>("RightFoot", robot, "rcontactpoint");

  // 将左右足设为接触任务（可加接触力约束）
  double X = sole_.halfLength;
  double Y = sole_.halfWidth;
  contactPoints << 
    +X, +X, -X, -X,
    +Y, -Y, -Y, +Y,
     0,  0,  0,  0;
  frictionCoeff_=sole_.friction;

  contactL = std::make_shared<tsid::contacts::Contact6d>(
      "contact_L",
      robot,                    // tsid::robots::RobotWrapper 实例
      "lcontactpoint",          // 接触 frame 名字
      contactPoints,           // 3x4 矩阵，定义接触点在脚底坐标系中的位置  Eigen::Matrix<double, 3, 4> contactPoints;
      Eigen::Vector3d::UnitZ(), // 接触法线，一般为 Z 轴方向
      frictionCoeff_,           // 摩擦系数
      minNormalForce_,          // 最小法向力
      maxNormalForce_           // 最大法向力
  );
  contactR = std::make_shared<tsid::contacts::Contact6d>(
      "contact_R",
      robot,
      "rcontactpoint",
      contactPoints,
      Eigen::Vector3d::UnitZ(),
      frictionCoeff_,
      minNormalForce_,
      maxNormalForce_
    );

  // === 3. 控制器内部状态变量清零 ===
  dcmDerivator_.setZero();
  dcmIntegrator_.setZero();
  dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);

  zmpccIntegrator_.setZero();
  zmpccIntegrator_.saturation(MAX_ZMPCC_COM_OFFSET);

  Eigen::Vector3d staticForce = -mass_ * gravity_;

  dcmAverageError_.setZero();
  dcmError_.setZero();
  dcmVelError_.setZero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  distribWrench_ = pinocchio::Force(staticForce, staticForce.cross(pendulum_.com()));
  zmpError_.setZero();
  zmpccCoMAccel_.setZero();
  zmpccCoMOffset_.setZero();
  zmpccCoMVel_.setZero();
  zmpccError_.setZero();
}


void Stabilizer::checkGains()
{
  MyWrapper::clamp(comAdmittance_.x(), 0., MAX_COM_ADMITTANCE);
  MyWrapper::clamp(comAdmittance_.y(), 0., MAX_COM_ADMITTANCE);
  MyWrapper::clamp(copAdmittance_.x(), 0., MAX_COP_ADMITTANCE);
  MyWrapper::clamp(copAdmittance_.y(), 0., MAX_COP_ADMITTANCE);
  MyWrapper::clamp(dcmDerivGain_, 0., MAX_DCM_D_GAIN);
  MyWrapper::clamp(dcmIntegralGain_, 0., MAX_DCM_I_GAIN);
  MyWrapper::clamp(dcmPropGain_, 0., MAX_DCM_P_GAIN);
  MyWrapper::clamp(dfzAdmittance_, 0., MAX_DFZ_ADMITTANCE);
}

void Stabilizer::addTasks(tsid::InverseDynamicsFormulationAccForce& solver)
{
  solver.addMotionTask(*comTask, comWeight_, 1.0);

  if(leftFootInContact) //判断当前支撑需要修改
  {
    solver.addRigidContact(*contactL, 1e5); // 添加左脚接触
  }
  else
  {
    solver.addMotionTask(*leftFootTask, swingFootWeight_, footTaskPriority_);
  }

  if(rightFootInContact)
  {
    solver.addRigidContact(*contactR, 1e5); // 添加右脚接触
  }
  else
  {
    solver.addMotionTask(*rightFootTask, swingFootWeight_, footTaskPriority_);
  }
}

void Stabilizer::removeTasks(tsid::InverseDynamicsFormulationAccForce& solver)
{
  solver.removeTask("task-com");

  if(leftFootInContact)   //注意是否初始化，避免空指针访问
  {
    solver.removeRigidContact("contact_L");
  }
  else
  {
    solver.removeTask("LeftFoot");
  }

  if(rightFootInContact)
  {
    solver.removeRigidContact("contact_R");
  }
  else
  {
    solver.removeTask("RightFoot");
  }
}

void Stabilizer::setContact(std::shared_ptr<tsid::contacts::Contact6d> &contactTask, 
                            const Contact & contact, 
                            const std::string & contactFrameName)    //Contact6d不能输出对应的足名称
{
  // 2. 设置刚度和阻尼
// 设置刚度系数 (Kp)
  Eigen::VectorXd contact_kp(6);
  contact_kp.head<3>() = contactStiffness_.linear();  // 前3个：线性刚度 (x, y, z)
  contact_kp.tail<3>() = contactStiffness_.angular(); // 后3个：角刚度 (rx, ry, rz)
  Eigen::VectorXd contact_kd(6);
  contact_kd.head<3>() = contactDamping_.linear();  // 前3个：线性阻尼 (x, y, z)
  contact_kd.tail<3>() = contactDamping_.angular(); // 后3个：角阻尼 (rx, ry, rz)

  contactTask->Kp(contact_kp);
  contactTask->Kd(contact_kd);

  // 3. 设置目标位姿
  contactTask->setReference(contact.pose);  // contact.pose 应为 SE3 形式

  // 4. 根据接触帧名判断是哪只脚，并缓存
  if(contactFrameName == "lcontactpoint")
  {
    leftFootContact = contact;
  }
  else if(contactFrameName == "rcontactpoint")
  {
    rightFootContact = contact;
  }
  else
  {
    std::cerr << "[Stabilizer::setContact] Unknown contact frame: " << contactFrameName << std::endl;
  }
}

void Stabilizer::setSwingFoot(std::shared_ptr<tsid::tasks::TaskSE3Equality> footTask)
{
  // 1. 重置任务（清空参考轨迹等）
  tsid::trajectories::TrajectorySample empty_ref;
  Eigen::VectorXd zero6 = Eigen::VectorXd::Zero(6);
  empty_ref.setValue(zero6);
  empty_ref.setDerivative(zero6);
  empty_ref.setSecondDerivative(zero6);
  footTask->setReference(empty_ref);
  // 2. 设置任务刚度
  Eigen::VectorXd swing_kp(6);
  swing_kp.head<3>() = Eigen::Vector3d::Constant(vdcStiffness_);
  swing_kp.tail<3>() = Eigen::Vector3d::Constant(vdcStiffness_);
  footTask->Kp(swing_kp);
  // 3. 添加时设置任务权重  swingFootWeight_ 
}

//状态机参考流程：1.在空中 2.seekTouchdown 3.detectTouchdown 4.解除姿态任务添加接触任务
//判断摆动足是否落地接触  contact为落足点（或者为摆动的轨迹终点）
bool Stabilizer::detectTouchdown(const Contact & contact, 
                                 const pinocchio::SE3 & footPoseMeasured,
                                 const Eigen::Vector3d & footForceWorld)
{
  // footPoseMeasured: 当前足部姿态，来源于 pinocchio::updateFramePlacement
  // footForceWorld: 足部当前测得的力（通常为世界系下的力）

  // 1. 相对误差：足部目标和实际之间的位移差
  pinocchio::SE3 diff = footPoseMeasured.actInv(contact.pose);
  Eigen::Vector3d transError = diff.translation();

  double xDist = std::abs(transError.x());
  double yDist = std::abs(transError.y());
  double zDist = std::abs(transError.z());

  // 2. 垂直方向力（通常在世界 z 轴方向）
  double Fz = footForceWorld.z();

  // 3. 判定阈值（可调）
  const double posThresh = 0.03;  // 3cm
  const double forceThresh = 50.; // 50N

  // 4. 判断是否触地（近似贴近目标 + 有较大支持力）
  return (xDist < posThresh && yDist < posThresh && zDist < posThresh && Fz > forceThresh);
}

//落足速度的简易实现（阻抗控制）   
void Stabilizer::seekTouchdown(std::shared_ptr<tsid::tasks::TaskSE3Equality> footTask,
                               tsid::trajectories::TrajectorySample & ref,
                               const Eigen::Vector3d & footForceWorld)
{
  constexpr double MAX_VEL = 0.01;         // 最大下降速度 [m/s]
  constexpr double TOUCHDOWN_FORCE = 50.;  // 落地判定所需的法向力 [N]

  if(footForceWorld.z() < TOUCHDOWN_FORCE)
  {
    // 逐步降低目标 z 位置
    ref.pos.z() -= MAX_VEL * dt_; // dt_ 是控制器周期
    ref.vel.setZero();
    ref.acc.setZero();

    // 更新任务参考
    footTask->setReference(ref);
  }
}

void Stabilizer::setSupportFootGains()
{
  Eigen::VectorXd vdc_kp(6);
  vdc_kp.head<3>()  = Eigen::Vector3d::Constant(vdcStiffness_); // 线性刚度
  vdc_kp.tail<3>()  = contactStiffness_.angular();              // 角刚度 

  Eigen::VectorXd regular_kp(6), regular_kd(6);
  regular_kp << contactStiffness_.linear(), contactStiffness_.angular();
  regular_kd << contactDamping_.linear(), contactDamping_.angular();

  switch(contactState_)
  {
    case ContactState::DoubleSupport:
      contactL->Kp(regular_kp);
      contactL->Kd(regular_kd);
      contactR->Kp(regular_kp);
      contactR->Kd(regular_kd);
      break;
    case ContactState::LeftFoot:
      contactL->Kp(vdc_kp);       // 左脚使用VDC刚度
      contactL->Kd(regular_kd);   // 阻尼保持常规
      break;
    case ContactState::RightFoot:
      contactR->Kp(vdc_kp);       // 右脚使用VDC刚度 
      contactR->Kd(regular_kd);   // 阻尼保持常规
      break;
  }
}

void Stabilizer::checkInTheAir()
{
  double LFz = state_.feettwist[2];
  double RFz = state_.feettwist[8];
  inTheAir_ = (LFz < MIN_DSP_FZ && RFz < MIN_DSP_FZ);
}

void Stabilizer::updateZMPFrame() 
{
  const pinocchio::SE3 & X_0_lc = leftFootContact.pose;
  const pinocchio::SE3 & X_0_rc = rightFootContact.pose;
  switch(contactState_)
  {
    case ContactState::DoubleSupport:
      zmpFrame_ = MyWrapper::interpolate(X_0_lc, X_0_rc, 0.5);
      break;
    case ContactState::LeftFoot:
      zmpFrame_ = X_0_lc;
      break;
    case ContactState::RightFoot:
      zmpFrame_ = X_0_rc;
      break;
  }
  measuredZMP_ = computeZMP(measuredWrench_);
}

Eigen::Vector3d Stabilizer::computeZMP(const pinocchio::Force & wrench) const
{
  Eigen::Vector3d n = zmpFrame_.rotation().row(2);
  Eigen::Vector3d p = zmpFrame_.translation();
  const Eigen::Vector3d & force = wrench.linear();
  double normalForce = n.dot(force);
  if(normalForce < 1.)
  {
    double lambda = std::pow(pendulum_.omega(), 2);
    return measuredCoM_ + gravity_ / lambda; // default for logging
  }
  const Eigen::Vector3d & moment_0 = wrench.angular();
  Eigen::Vector3d moment_p = moment_0 - p.cross(force);
  return p + n.cross(moment_p) / normalForce;
}

void Stabilizer::run()
{
  auto startTime = std::chrono::high_resolution_clock::now();
  checkGains();
  checkInTheAir();
  setSupportFootGains();
  updateZMPFrame();
  auto desiredWrench = computeDesiredWrench();
  switch (contactState_) // 条件判断在何处
  {
  case ContactState::DoubleSupport:
    distributeWrench(desiredWrench);
    break;
  case ContactState::LeftFoot:
    saturateWrench(desiredWrench, leftFootContact, contactL);
    break;
  case ContactState::RightFoot:
    saturateWrench(desiredWrench, rightFootContact, contactR);
    break;
  }
  updateCoMTaskZMPCC();
  auto endTime = std::chrono::high_resolution_clock::now();
  runTime_ = std::chrono::duration<double, std::milli>(endTime - startTime).count();
}

void Stabilizer::updateState(const Eigen::Vector3d &com,
                             const Eigen::Vector3d &comd,
                             const pinocchio::Force &wrench,
                             double leftFootRatio)
{
  leftFootRatio_ = leftFootRatio;
  measuredCoM_ = com;
  measuredCoMd_ = comd;
  measuredWrench_ = wrench;
}

pinocchio::Force Stabilizer::computeDesiredWrench()
{
  double omega = pendulum_.omega();
  Eigen::Vector3d comError = pendulum_.com() - measuredCoM_;
  Eigen::Vector3d comdError = pendulum_.comd() - measuredCoMd_;
  dcmError_ = comError + comdError / omega;
  dcmError_.z() = 0.;

  if (inTheAir_)
  {
    dcmDerivator_.setZero();
    dcmIntegrator_.append(Eigen::Vector3d::Zero());
  }
  else
  {
    zmpError_ = pendulum_.zmp() - measuredZMP_;
    zmpError_.z() = 0.;
    dcmDerivator_.update(omega * (dcmError_ - zmpError_));
    dcmIntegrator_.append(dcmError_);
  }
  dcmAverageError_ = dcmIntegrator_.eval();
  dcmVelError_ = dcmDerivator_.eval();

  Eigen::Vector3d desiredCoMAccel = pendulum_.comdd();
  desiredCoMAccel += omega * (dcmPropGain_ * dcmError_ + comdError);
  desiredCoMAccel += omega * dcmIntegralGain_ * dcmAverageError_;
  desiredCoMAccel += omega * dfzAdmittance_ * dcmVelError_;
  auto desiredForce = mass_ * (desiredCoMAccel - gravity_);

  return {desiredForce, measuredCoM_.cross(desiredForce)};
}

void Stabilizer::distributeWrench(const pinocchio::Force &desiredWrench) // 用于contact初始化controlRobot_._Dataptr->oMf[controlRobot_.model().getFrameId("lcontactpoint")]
{

  const pinocchio::SE3 &X_0_lc = leftFootContact.pose;
  const pinocchio::SE3 &X_0_rc = rightFootContact.pose;
  pinocchio::SE3 X_0_lankle = leftFootContact.anklePose(sole_);
  pinocchio::SE3 X_0_rankle = rightFootContact.anklePose(sole_);

  constexpr unsigned NB_VAR = 6 + 6;
  constexpr unsigned COST_DIM = 6 + NB_VAR + 1;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A.setZero(COST_DIM, NB_VAR);
  b.setZero(COST_DIM);

  // |w_l_0 + w_r_0 - desiredWrench|^2
  A.block<6, 6>(0, 0).setIdentity();
  A.block<6, 6>(0, 6).setIdentity();
  b.segment<6>(0) = desiredWrench.toVector();

  // |ankle torques|^2
  Eigen::Matrix<double, 6, 6> W_lankle = Eigen::Matrix<double, 6, 6>::Zero();
  W_lankle.diagonal() << 1e-3, 1e-3, 1e-4, 1.0, 1.0, 1e-4;
  Eigen::Matrix<double, 6, 6> W_rankle = W_lankle;

  A.block<6, 6>(6, 0) = W_lankle * X_0_lankle.toDualActionMatrix();
  A.block<6, 6>(12, 6) = W_rankle * X_0_rankle.toDualActionMatrix();

  // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
  double lfr = leftFootRatio_;
  auto A_fratio = A.block<1, 12>(18, 0);

  Eigen::Matrix<double, 1, 6> forceZ;
  forceZ << 0, 0, 1, 0, 0, 0; // spatial wrench 的第3项是 force z

  A.block<1, 6>(18, 0) = (1 - lfr) * (forceZ * X_0_lc.toDualActionMatrix());
  A.block<1, 6>(18, 6) = -lfr * (forceZ * X_0_rc.toDualActionMatrix());

  // Apply weights
  A.block<6, 12>(0, 0) *= fdqpWeights_.netWrenchSqrt;
  b.segment<6>(0) *= fdqpWeights_.netWrenchSqrt;

  A.block<6, 6>(6, 0) *= fdqpWeights_.ankleTorqueSqrt;
  A.block<6, 6>(12, 6) *= fdqpWeights_.ankleTorqueSqrt;

  A.block<1, 12>(18, 0) *= fdqpWeights_.forceRatioSqrt;

  // === QP cost matrix ===
  Eigen::MatrixXd Q = A.transpose() * A;
  Eigen::VectorXd c = -A.transpose() * b;

  // === Inequality constraints ===
  constexpr unsigned NB_CONS = 16 + 16 + 2;
  Eigen::Matrix<double, NB_CONS, NB_VAR> A_ineq = Eigen::Matrix<double, NB_CONS, NB_VAR>::Zero();
  Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(NB_CONS);
  // CWC * w_l_lc <= 0
  A_ineq.block<16, 6>(0, 0) = wrenchFaceMatrix_ * X_0_lc.toDualActionMatrix(); // wrenchFaceMatrix_需要先力再扭矩
  // CWC * w_r_rc <= 0
  A_ineq.block<16, 6>(16, 6) = wrenchFaceMatrix_ * X_0_rc.toDualActionMatrix();
  // w_l_lc.force().z() >= MIN_DSP_FZ
  A_ineq.block<1, 6>(32, 0) = -forceZ * X_0_lc.toDualActionMatrix();
  b_ineq(32) = -MIN_DSP_FZ;
  // w_r_rc.force().z() >= MIN_DSP_FZ
  A_ineq.block<1, 6>(33, 6) = -forceZ * X_0_rc.toDualActionMatrix();
  b_ineq(33) = -MIN_DSP_FZ;

  //real_t[] 类型
  real_t H[NB_VAR * NB_VAR];
  real_t g[NB_VAR];
  real_t A_qp[NB_CONS * NB_VAR];
  real_t lb[NB_VAR], ub[NB_VAR];
  real_t lbA[NB_CONS], ubA[NB_CONS];

  Eigen::Matrix<real_t, NB_VAR, NB_VAR, Eigen::RowMajor> Q_qp = Q.cast<real_t>();
  Eigen::Matrix<real_t, NB_VAR, 1> c_qp = c.cast<real_t>();
  Eigen::Matrix<real_t, NB_CONS, NB_VAR, Eigen::RowMajor> A_qp_eigen = A_ineq.cast<real_t>();

  std::memcpy(H, Q_qp.data(), sizeof(real_t) * NB_VAR * NB_VAR);
  std::memcpy(g, c_qp.data(), sizeof(real_t) * NB_VAR);
  std::memcpy(A_qp, A_qp_eigen.data(), sizeof(real_t) * NB_CONS * NB_VAR);

  for (int i = 0; i < NB_VAR; ++i) {
    lb[i] = -1e20;
    ub[i] = 1e20;
  }

  for (int i = 0; i < NB_CONS; ++i) {
    lbA[i] = -1e20;
    ubA[i] = b_ineq(i);
  }

  QProblem qp(NB_VAR, NB_CONS);
  Options options;
  options.setToDefault();
  options.printLevel = PL_NONE;
  qp.setOptions(options);

  int nWSR = 100;
  auto ret = qp.init(H, g, A_qp, lb, ub, lbA, ubA, nWSR);
  if (ret != SUCCESSFUL_RETURN)
  {
    std::cout << "QP failed in distributeWrench() with return code: " << ret << std::endl;
    return;
  }

  Eigen::VectorXd x(NB_VAR);
  qp.getPrimalSolution(x.data());

  pinocchio::Force w_l_0(x.segment<6>(0));
  pinocchio::Force w_r_0(x.segment<6>(6));
  distribWrench_ = pinocchio::Force(w_l_0.linear() + w_r_0.linear(), w_l_0.angular() + w_r_0.angular());

  pinocchio::Force w_l_lc = X_0_lc.act(w_l_0);
  pinocchio::Force w_r_rc = X_0_rc.act(w_r_0);

  contactL->setForceReference(w_l_lc.toVector());
  contactR->setForceReference(w_r_rc.toVector());
}

void Stabilizer::saturateWrench(const pinocchio::Force &desiredWrench,
                                const Contact &FootContact,
                                std::shared_ptr<tsid::contacts::Contact6d> &contactTask)
{
  constexpr unsigned NB_CONS = 16;
  constexpr unsigned NB_VAR = 6;

  const pinocchio::SE3 &X_0_c = FootContact.pose;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd c = -desiredWrench.toVector();

  Eigen::MatrixXd A_ineq = wrenchFaceMatrix_ * X_0_c.toDualActionMatrix();
  Eigen::VectorXd b_ineq;
  b_ineq.setZero(NB_CONS);

  //real_t[] 类型
  real_t H[NB_VAR * NB_VAR];
  real_t g[NB_VAR];
  real_t A_qp[NB_CONS * NB_VAR];
  real_t lb[NB_VAR], ub[NB_VAR];
  real_t lbA[NB_CONS], ubA[NB_CONS];

  Eigen::Matrix<real_t, NB_VAR, NB_VAR, Eigen::RowMajor> Q_qp = Q.cast<real_t>();
  Eigen::Matrix<real_t, NB_VAR, 1> c_qp = c.cast<real_t>();
  Eigen::Matrix<real_t, NB_CONS, NB_VAR, Eigen::RowMajor> A_qp_eigen = A_ineq.cast<real_t>();

  std::memcpy(H, Q_qp.data(), sizeof(real_t) * NB_VAR * NB_VAR);
  std::memcpy(g, c_qp.data(), sizeof(real_t) * NB_VAR);
  std::memcpy(A_qp, A_qp_eigen.data(), sizeof(real_t) * NB_CONS * NB_VAR);

  for (int i = 0; i < NB_VAR; ++i) {
    lb[i] = -1e20;
    ub[i] = 1e20;
  }

  for (int i = 0; i < NB_CONS; ++i) {
    lbA[i] = -1e20;
    ubA[i] = b_ineq(i);
  }

  QProblem qp(NB_VAR, NB_CONS);
  Options options;
  options.setToDefault();
  options.printLevel = PL_NONE;
  qp.setOptions(options);

  int nWSR = 100;
  auto ret = qp.init(H, g, A_qp, lb, ub, lbA, ubA, nWSR);
  if (ret != SUCCESSFUL_RETURN)
  {
    std::cout << "QP failed in saturateWrench() with return code: " << ret << std::endl;
    return;
  }

  Eigen::VectorXd x(NB_VAR);
  qp.getPrimalSolution(x.data());

  pinocchio::Force w_0(x.segment<6>(0));
  pinocchio::Force w_c = X_0_c.actInv(w_0);
  contactTask->setForceReference(w_c.toVector());
  distribWrench_ = w_0;
}

void Stabilizer::updateCoMTaskZMPCC()
{
  if (zmpccOnlyDS_ && contactState_ != ContactState::DoubleSupport) // 单足期禁止zmp误差补偿
  {
    zmpccCoMAccel_.setZero();
    zmpccCoMVel_.setZero();
    zmpccIntegrator_.add(Eigen::Vector3d::Zero(), dt_); // leak to zero
  }
  else
  {
    auto distribZMP = computeZMP(distribWrench_);
    zmpccError_ = distribZMP - measuredZMP_;
    const Eigen::Matrix3d &R_0_c = zmpFrame_.rotation();
    const Eigen::Transpose<const Eigen::Matrix3d> R_c_0 = R_0_c.transpose();
    Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
    Eigen::Vector3d newVel = -R_c_0 * comAdmittance.cwiseProduct(R_0_c * zmpccError_);
    Eigen::Vector3d newAccel = (newVel - zmpccCoMVel_) / dt_;
    zmpccIntegrator_.add(newVel, dt_);
    zmpccCoMAccel_ = newAccel;
    zmpccCoMVel_ = newVel;
  }
  zmpccCoMOffset_ = zmpccIntegrator_.eval(); // 积分器将速度积分为位置

  tsid::trajectories::TrajectorySample com_ref;
  com_ref.pos = pendulum_.com() + zmpccCoMOffset_;
  com_ref.vel = pendulum_.comd() + zmpccCoMVel_;
  com_ref.acc = pendulum_.comdd() + zmpccCoMAccel_;

  comTask->setReference(com_ref);
}
