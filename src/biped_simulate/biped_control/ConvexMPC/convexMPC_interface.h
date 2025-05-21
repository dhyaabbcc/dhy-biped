#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif
#include <eigen3/Eigen/Dense>

struct problem_setup
{
  float dt;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[6];
  float joint_angles[12];
  float yaw;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  Eigen::MatrixXf R_foot;//足部方位矩阵
  Eigen::MatrixXf L_foot;
  
  // float alpha;
  float Alpha_K[12];
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC double get_solution(int index);
void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
void update_problem_data(double* p, double* v, double* q, double* w, double* r, double* joint_angles, double yaw, double* weights, double* state_trajectory, double* Alpha_K, int* gait);
void update_problem_data(double* p, double* v, double* q, double* w, double* r, 
double* joint_angles, double yaw, double* weights, double* state_trajectory, double* Alpha_K, int* gait,Eigen::MatrixXf RL,Eigen::MatrixXf RR);
#endif