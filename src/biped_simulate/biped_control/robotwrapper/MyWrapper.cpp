 #include "MyWrapper.hpp" 
 
void MyWrapper::computeMainTerms(pinocchio::Data& data, 
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& v) const {
    // 调用Pinocchio算法 
    pinocchio::nonLinearEffects(model(), data, q, v);
    pinocchio::computeJointJacobians(model(), data, q);
    pinocchio::crba(model(), data, q);
    data.M.triangularView<Eigen::StrictlyLower>()  = 
    data.M.transpose().triangularView<Eigen::StrictlyLower>(); 
}