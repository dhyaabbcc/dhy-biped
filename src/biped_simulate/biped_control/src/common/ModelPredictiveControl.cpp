#include "../../include/common/ModelPredictiveControl.h"

using namespace Eigen;
using namespace qpOASES;

constexpr double ModelPredictiveControl::SAMPLING_PERIOD;
constexpr unsigned ModelPredictiveControl::INPUT_SIZE;
constexpr unsigned ModelPredictiveControl::NB_STEPS;
constexpr unsigned ModelPredictiveControl::STATE_SIZE;

ModelPredictiveControl::ModelPredictiveControl()
  : N_(NB_STEPS), T_(SAMPLING_PERIOD) {
  A_.resize(STATE_SIZE, STATE_SIZE);
  B_.resize(STATE_SIZE, INPUT_SIZE);
  initializeSystemMatrices();
}

void ModelPredictiveControl::initializeSystemMatrices()
{
  double S = T_ * T_ / 2.0;
  double C = T_ * T_ * T_ / 6.0;
  A_ << 1, 0, T_, 0, S, 0,
        0, 1, 0, T_, 0, S,
        0, 0, 1, 0, T_, 0,
        0, 0, 0, 1, 0, T_,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
  // clang-format on
  B_ << C, 0,
        0, C,
        S, 0,
        0, S,
        T_, 0,
        0, T_;
  // clang-format on
  initState_=Eigen::VectorXd::Zero(STATE_SIZE);   
  zmpRef_ = VectorXd::Zero(2 * (NB_STEPS + 1));
  velRef_ = VectorXd::Zero(2 * (NB_STEPS + 1));
}

void ModelPredictiveControl::buildQP() {
  int nU = INPUT_SIZE * N_;
  int nX = STATE_SIZE * (N_ + 1);

  int nEq = nX + termDCMCons_.A.rows() + termZMPCons_.A.rows();
  int nIneq = termZMPCons_.A.rows();

  H_ = MatrixXd::Identity(nU, nU);
  g_ = VectorXd::Zero(nU);

  Aeq_ = MatrixXd::Zero(nEq, nU);
  beq_ = VectorXd::Zero(nEq);

  Aineq_=MatrixXd::Zero(nIneq, Tmat_.cols());
  bineq_=VectorXd::Zero(nIneq);

  S_ = MatrixXd::Zero(nX, STATE_SIZE);
  Tmat_ = MatrixXd::Zero(nX, nU);

  // 1. 构造状态展开公式： x = S x0 + T u
  MatrixXd A_pow = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  for (int i = 0; i <= N_; ++i) {
    S_.block(i * STATE_SIZE, 0, STATE_SIZE, STATE_SIZE) = A_pow;
    for (int j = 0; j < i; ++j) {
      MatrixXd Ap = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
      for (int k = 0; k < i - j - 1; ++k) Ap = A_ * Ap;
      Tmat_.block(i * STATE_SIZE, j * INPUT_SIZE, STATE_SIZE, INPUT_SIZE) = Ap * B_;
    }
    A_pow = A_ * A_pow;
  }

  // 2. 代价项：H u^2 + g u
  H_ += jerkCost_.weight * jerkCost_.Q;
  g_ += jerkCost_.weight * jerkCost_.c;

  const Eigen::MatrixXd &Q_zmp = zmpCost_.Q; 
  const Eigen::VectorXd &c_zmp = zmpCost_.c; 
  const double w_zmp = zmpCost_.weight;   

  const Eigen::MatrixXd &Q_vel = velCost_.Q; 
  const Eigen::VectorXd &c_vel = velCost_.c; 
  const double w_vel = velCost_.weight;   

  Eigen::MatrixXd CT_zmp = Q_zmp * T_;                      // Q_zmp * T
  Eigen::VectorXd Cr_zmp = Q_zmp * S_ * initState_ - c_zmp; // Q_zmp * S * x0 - c_zmp

  Eigen::MatrixXd CT_vel = Q_vel * T_;                      // Q_vel * T
  Eigen::VectorXd Cr_vel = Q_vel * S_ * initState_ - c_vel; // Q_vel * S * x0 - c_vel
 
  H_ += w_zmp * CT_zmp.transpose() * CT_zmp; // w_zmp * T^T Q_zmp^T Q_zmp T
  g_ += w_zmp * CT_zmp.transpose() * Cr_zmp; // w_zmp * (Q_zmp T)^T (Q_zmp S x0 - c_zmp)

  H_ += w_vel * CT_vel.transpose() * CT_vel; // w_vel * T^T Q_vel^T Q_vel T
  g_ += w_vel * CT_vel.transpose() * Cr_vel; // w_vel * (Q_vel T)^T (Q_vel S x0 - c_vel)

  // 3. 状态等式约束：Aeq * u = beq
  int row_offset = nX;
  Aeq_.topRows(nX) = Tmat_;
  beq_.head(nX) = S_ * initState_;

  Eigen::MatrixXd Aeq_dcm = termDCMCons_.A * T_;
  Eigen::VectorXd beq_dcm = termDCMCons_.b - termDCMCons_.A * S_ * initState_;
  Eigen::MatrixXd Aeq_zmp = termZMPCons_.A * T_;
  Eigen::VectorXd beq_zmp = termZMPCons_.b - termZMPCons_.A * S_ * initState_;

  Aeq_.middleRows(row_offset, Aeq_dcm.rows()) = Aeq_dcm;
  beq_.segment(row_offset, beq_dcm.size()) = beq_dcm;
  row_offset += Aeq_dcm.rows();
  Aeq_.middleRows(row_offset, Aeq_zmp.rows()) = Aeq_zmp;
  beq_.segment(row_offset, beq_zmp.size()) = beq_zmp;

  // 4. 不等式约束：Aineq_ * u <= bineq_
  Eigen::MatrixXd Aineq_zmp = zmpCons_.A * T_;
  Eigen::VectorXd bineq_zmp = zmpCons_.b - zmpCons_.A * S_ * initState_;

  Aineq_.topRows(Aineq_zmp.rows()) = Aineq_zmp;
  bineq_.head(bineq_zmp.size()) = bineq_zmp;

  //5. 拼接
  A_qp.resize(Aeq_.rows() + Aineq_.rows(), Aeq_.cols());
  A_qp << Aeq_,
      Aineq_;

  lbA_qp.resize(beq_.size() + bineq_.size());
  ubA_qp.resize(beq_.size() + bineq_.size());

  lbA_qp << beq_,
      bineq_;

  ubA_qp << beq_,
      bineq_;
}

void ModelPredictiveControl::solve() {
  //1. 构建QP
  computeZMPRef();

  updateTerminalConstraint();
  updateZMPConstraint();
  updateJerkCost();
  updateVelCost();
  updateZMPCost();

  buildQP();
  //2. 初始化求解器
  int nU = INPUT_SIZE * N_;
  QProblem qp(nU, Aineq_.rows());
  Options options;
  options.setToMPC();
  options.printLevel  = PL_NONE; 
  qp.setOptions(options);
  int nWSR = 50;

  //3. 准备QP输入参数
  real_t *H_qp = H_.data();
  real_t *g_qp = g_.data();
  real_t *A_data = A_qp.data();
  real_t *lbA_data = lbA_qp.data();
  real_t *ubA_data = ubA_qp.data();

  // 4. 求解QP问题
  returnValue status = qp.init(
      H_qp, g_qp,
      A_data,
      nullptr, nullptr, // 无变量上下界
      lbA_data, ubA_data,
      nWSR);

  if (status != SUCCESSFUL_RETURN)
  {
      std::cout << "QP求解失败! 错误码: " << status << std::endl;
      solution_.reset(new Preview());
      return;
  }

  //5. 处理结果
  VectorXd sol(nU);
  qp.getPrimalSolution(sol.data());

  int state_dim = STATE_SIZE * (N_ + 1); // 轨迹状态维度
  int control_dim = INPUT_SIZE * N_;     // 控制输入维度

  auto trajectory = buildTrajectoryFromSolution(sol); 
  auto control = buildControlFromSolution(sol);       

  // 创建新的Preview对象
  solution_.reset(new Preview(trajectory, control));
}


void ModelPredictiveControl::phaseDurations(double initSupportDuration,
                                            double doubleSupportDuration,
                                            double targetSupportDuration)
{
  constexpr double T = SAMPLING_PERIOD;

  unsigned nbStepsSoFar = 0;
  nbInitSupportSteps_ = std::min(static_cast<unsigned>(std::round(initSupportDuration / T)), NB_STEPS - nbStepsSoFar);
  nbStepsSoFar += nbInitSupportSteps_;
  nbDoubleSupportSteps_ =
      std::min(static_cast<unsigned>(std::round(doubleSupportDuration / T)), NB_STEPS - nbStepsSoFar);
  nbStepsSoFar += nbDoubleSupportSteps_;
  nbTargetSupportSteps_ =
      std::min(static_cast<unsigned>(std::round(targetSupportDuration / T)), NB_STEPS - nbStepsSoFar);
  nbStepsSoFar += nbTargetSupportSteps_;
  if(nbTargetSupportSteps_ > 0) // full preview mode
  {
    nbNextDoubleSupportSteps_ = NB_STEPS - nbStepsSoFar; // always positive
  }
  for(long i = 0; i <= NB_STEPS; i++)
  {
    // SSP constraint is enforced at the very first step of DSP
    if(i < nbInitSupportSteps_ || (0 < i && i == nbInitSupportSteps_))
    {
      indexToHrep_[i] = 0;
    }
    else if(i - nbInitSupportSteps_ < nbDoubleSupportSteps_)
    {
      indexToHrep_[i] = 1;
    }
    else if(nbTargetSupportSteps_ > 0)
    {
      if(i - nbInitSupportSteps_ - nbDoubleSupportSteps_ <= nbTargetSupportSteps_)
      {
        indexToHrep_[i] = 2;
      }
      else if(nbNextDoubleSupportSteps_ > 0)
      {
        indexToHrep_[i] = 3;
      }
      else // (nbNextDoubleSupportSteps_ == 0)
      {
        indexToHrep_[i] = 2;
      }
    }
    else // (nbTargetSupportSteps_ == 0)
    {
      indexToHrep_[i] = 1;
    }
  }
}

void ModelPredictiveControl::computeZMPRef()
{
  zmpRef_.setZero();
  Eigen::Vector2d p_0 = initContact_.anklePos(sole_).head<2>();
  Eigen::Vector2d p_1 = targetContact_.anklePos(sole_).head<2>();
  Eigen::Vector2d p_2 = nextContact_.anklePos(sole_).head<2>();
  if(nbTargetSupportSteps_ < 1) // half preview mode (@see phaseDurations)
  {
    p_1 = 0.5 * (initContact_.anklePos(sole_) + targetContact_.anklePos(sole_)).head<2>();
  }
  for(long i = 0; i <= NB_STEPS; i++)
  {
    if(indexToHrep_[i] <= 1)
    {
      long j = i - nbInitSupportSteps_;
      double x = (nbDoubleSupportSteps_ > 0) ? static_cast<double>(j) / nbDoubleSupportSteps_ : 0.;
      x = clamp(x, 0., 1.);
      zmpRef_.segment<2>(2 * i) = (1. - x) * p_0 + x * p_1;
    }
    else // (indexToHrep_[i] <= 3), which implies nbTargetSupportSteps_ > 0
    {
      long j = i - nbInitSupportSteps_ - nbDoubleSupportSteps_ - nbTargetSupportSteps_;
      double x = (nbNextDoubleSupportSteps_ > 0) ? static_cast<double>(j) / nbNextDoubleSupportSteps_ : 0;
      x = clamp(x, 0., 1.);
      zmpRef_.segment<2>(2 * i) = (1. - x) * p_1 + x * p_2;
    }
  }
}

void ModelPredictiveControl::updateTerminalConstraint()
{
  Eigen::MatrixXd E_dcm = Eigen::MatrixXd::Zero(2, STATE_SIZE * (NB_STEPS + 1));
  Eigen::MatrixXd E_zmp = Eigen::MatrixXd::Zero(2, STATE_SIZE * (NB_STEPS + 1));
  if(nbTargetSupportSteps_ < 1) // half preview mode (@see phaseDurations)
  {
    unsigned i = nbInitSupportSteps_ + nbDoubleSupportSteps_;
    E_dcm.block<2, 6>(0, 6 * i) = dcmFromState_;
    E_zmp.block<2, 6>(0, 6 * i) = zmpFromState_;
  }
  else // full preview
  {
    E_dcm.rightCols<6>() = dcmFromState_;
    E_zmp.rightCols<6>() = zmpFromState_;
  }
  Eigen::Vector2d dcmTarget = zmpRef_.tail<2>();
  Eigen::Vector2d zmpTarget = zmpRef_.tail<2>();
  termDCMCons_.A=E_dcm;
  termDCMCons_.b=dcmTarget;
  termDCMCons_.isInequality=false;
  termZMPCons_.A=E_zmp;
  termZMPCons_.b=zmpTarget;
  termZMPCons_.isInequality=false;
}

void ModelPredictiveControl::updateZMPConstraint()
{
  hreps_[0] = initContact_.hrep();
  hreps_[2] = targetContact_.hrep();
  unsigned totalRows = 0;
  for(long i = 0; i <= NB_STEPS; i++)
  {
    unsigned hrepIndex = indexToHrep_[i];
    if(hrepIndex % 2 == 0)
    {
      const auto & hrep = hreps_[hrepIndex];
      totalRows += static_cast<unsigned>(hrep.first.rows());
    }
  }
  Eigen::MatrixXd A{totalRows, STATE_SIZE * (NB_STEPS + 1)};
  Eigen::VectorXd b{totalRows};
  A.setZero();
  long nextRow = 0;
  for(long i = 0; i <= NB_STEPS; i++)
  {
    unsigned hrepIndex = indexToHrep_[i];
    if(hrepIndex % 2 == 0)
    {
      const auto & hrep = hreps_[indexToHrep_[i]];
      unsigned consRows = static_cast<unsigned>(hrep.first.rows());
      A.block(nextRow, STATE_SIZE * i, consRows, STATE_SIZE) = hrep.first * zmpFromState_;
      b.segment(nextRow, consRows) = hrep.second;
      nextRow += consRows;
    }
  }
  zmpCons_.A=A;
  zmpCons_.b=b;
  zmpCons_.isInequality=true;
}

void ModelPredictiveControl::updateJerkCost()
{
  Eigen::Matrix2d jerkMat = Eigen::Matrix2d::Identity();
  Eigen::Vector2d jerkVec = Eigen::Vector2d::Zero();
  jerkCost_.Q=jerkMat;
  jerkCost_.c=jerkVec;
  jerkCost_.weight=jerkWeight_;
}

void ModelPredictiveControl::updateVelCost()
{
  velRef_.setZero();
  const Eigen::Matrix3d & R_0 = initContact_.pose.rotation();
  const Eigen::Matrix3d & R_1 = targetContact_.pose.rotation();
  const Eigen::Matrix3d & R_2 = nextContact_.pose.rotation();
  Eigen::Vector2d v_0 = initContact_.refVel.head<2>();
  Eigen::Vector2d v_1 = targetContact_.refVel.head<2>();
  Eigen::Vector2d v_2 = nextContact_.refVel.head<2>();
  if(nbTargetSupportSteps_ < 1) // half preview mode (@see phaseDurations)
  {
    v_1 = {0., 0.};
  }
  Eigen::Matrix2d R;
  Eigen::Vector2d v;
  for(long i = 0; i <= NB_STEPS; i++)
  {
    if(indexToHrep_[i] <= 1)
    {
      double w = static_cast<double>(i) / (nbInitSupportSteps_ + nbDoubleSupportSteps_);
      w = clamp(w, 0., 1.);
      R = slerp(R_0, R_1, w).topLeftCorner<2, 2>();
      v = (1. - w) * v_0 + w * v_1;
    }
    else // (indexToHrep_[i] <= 3), which implies nbTargetSupportSteps_ > 0
    {
      long i2 = i - nbInitSupportSteps_ - nbDoubleSupportSteps_; // >= 0
      double w = static_cast<double>(i2) / (nbTargetSupportSteps_ + nbNextDoubleSupportSteps_);
      w = clamp(w, 0., 1.);
      R = slerp(R_1, R_2, w).topLeftCorner<2, 2>();
      v = (1. - w) * v_1 + w * v_2;
    }
    velCostMat_.block<2, STATE_SIZE>(2 * i, STATE_SIZE * i).block<2, 2>(0, 2) = R;
    velRef_.segment<2>(2 * i) = R * v;
  }
  velCost_.Q=velCostMat_;
  velCost_.c=velRef_;
  velCost_.weight=velWeights_;
}

void ModelPredictiveControl::updateZMPCost()
{
  zmpCost_.Q=autoSpanQ(zmpFromState_, NB_STEPS);
  zmpCost_.c=autoSpanC(zmpRef_, NB_STEPS);
  zmpCost_.weight=zmpWeight_;
}

Eigen::MatrixXd autoSpanQ(const Eigen::MatrixXd& zmpFromState, int horizon) {
    const int nz = zmpFromState.rows();   // ZMP 维度（2）
    const int nx = zmpFromState.cols();   // 状态维度（13）
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nz * horizon, nx * horizon);
    
    for (int i = 0; i < horizon; ++i) {
        Q.block(i  * nz, i * nx, nz, nx) = zmpFromState;
    }
    
    return Q;
}

Eigen::VectorXd autoSpanC(const Eigen::VectorXd& zmpRef, int horizon) {
    const int nz = zmpRef.rows();   // ZMP 维度（2）
    
    Eigen::VectorXd c = Eigen::VectorXd::Zero(nz * horizon);
    
    for (int i = 0; i < horizon; ++i) {
        c.segment(i  * nz, nz) = zmpRef;
    }
    
    return c;
}

Eigen::VectorXd ModelPredictiveControl::buildTrajectoryFromSolution(const VectorXd& sol) {
    Eigen::VectorXd traj(STATE_SIZE * (N_ + 1));
    for (int i = 0; i <= N_; ++i) {
        traj.segment(i  * STATE_SIZE, STATE_SIZE) = sol.segment(i  * STATE_SIZE, STATE_SIZE);
    }
    return traj;
}
 
Eigen::VectorXd ModelPredictiveControl::buildControlFromSolution(const VectorXd& sol) {
    Eigen::VectorXd ctrl(INPUT_SIZE * N_);
    int state_part = STATE_SIZE * (N_ + 1);  // 跳过状态部分 
    for (int i = 0; i < N_; ++i) {
        ctrl.segment(i  * INPUT_SIZE, INPUT_SIZE) = sol.segment(state_part  + i * INPUT_SIZE, INPUT_SIZE);
    }
    return ctrl;
}