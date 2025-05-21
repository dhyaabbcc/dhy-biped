​     

# StateEstimator

速度位置估计：直接读取世界坐标系下速度和位置。

姿态角速度估计：直接读取世界坐标系下姿态，body frame下角速度

## StateEstimate

```c++
struct StateEstimate {
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody; 
    Quat<double> orientation;
    Vec3<double> omegaBody;//body frame 下角速度
    RotMat<double> rBody;//base 在世界坐标系下姿态
    Vec3<double> rpy;//base 在世界坐标系下姿态 rpy表示
	Vec3<double> omegaWorld;//世界坐标系下角速度
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;


};
```

## StateEstimatorData

```c++
struct StateEstimatorData {
    StateEstimate* result;
    LowlevelState* lowState;
    LegControllerData* legControllerData;
};
```



# LowlevelState

```c++
struct LowlevelState
{
    IMU imu;
    MotorState motorState[10];
    UserCommand userCmd;
    UserValue userValue;

    float position[3];
    float vWorld[3];
    float rpy[3];
    
    LowlevelState()
    {
        for(int i = 0; i < 3; i++)
        {
            position[i] = 0;
            vWorld[i] = 0;
            rpy[i] = 0;
        }
    }

};
```

# 传感器

## IMU

```c++
struct IMU
{
    float quaternion[4];//获得base姿态
    float gyroscope[3];//获得base角速度
    float accelerometer[3];

    IMU()
    {
        for(int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

};
```

# 用户

## UserCommand

```c++
enum class UserCommand{
    // EXIT,
    NONE,
    START,      // walking
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // pushing
    L2_Y,       // probe
    L1_X,       // QPStand 
    L1_A,      
    L1_Y       
};
```

## UserValue

```c++
struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    float vx; // vx in body frame
    float vy; // vy in body frame
    float turn_rate;

    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
        vx = 0;
        vy = 0;
        turn_rate = 0;
    }

};
```

# Controller

## LegControllerData

```c++
struct LegControllerData{
        LegControllerData() {zero();}
        void setBiped(Biped& biped) { hector = &biped; }

       void zero();
       Vec5<double> q, qd;
       Vec3<double> p, v;// body frame 下足部位置速度
       Mat65<double> J_force_moment;
       Mat35<double> J_force;
       Vec5<double> tau;
       Biped* hector;
};

void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}
```

## LegControllerCommand

```c++
struct LegControllerCommand{
        LegControllerCommand() {zero();}

        void zero();
    
        Vec5<double> qDes, qdDes, tau;
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;
    };


```

# DesiredState

## DesiredStateData

```c++
struct DesiredStateData{

    DesiredStateData() { zero(); }
    
    // Zero all data
    void zero();
    
    // Instataneous desired state comman
    Vec12<double> stateDes;
    Vec12<double> pre_stateDes;
    
    int mode;

};
```

* stateDes[0]~stateDes[2]:期望xyz位置
* stateDes[3]~state[5]:期望姿态，roll pitch yaw形式
* stateDes[6]~stateDes[8]:期望线速度
* stateDes[9]~stateDes[11]:期望角速度

# FSM

## ControlFSMData

```c++
struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Biped *_biped;
  StateEstimatorContainer *_stateEstimator;
  LegController *_legController;
  DesiredStateCommand *_desiredStateCommand;
  IOInterface *_interface;
  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
  }
};
```

## ControlFSMData

```c++
struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Biped *_biped;
  StateEstimatorContainer *_stateEstimator;
  LegController *_legController;
  DesiredStateCommand *_desiredStateCommand;
  IOInterface *_interface;
  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
  }
};
```

## FSMMode

```c++
enum class FSMMode{
    NORMAL,
    CHANGE
};
```



## FSMStateName

```c++
enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    PDSTAND,
    QPSTAND,
    WALKING,
    PUSHING,
    PROBE,
    SLAM,       // slam
};

# lowl
```

# lowlevel



## Lowlevelcmd

```c++
struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }

};

struct LowlevelCmd{
    MotorCmd motorCmd[10];
};
```

## Lowlevelstate

```c++
struct LowlevelState
{
    IMU imu;
    MotorState motorState[10];
    UserCommand userCmd;
    UserValue userValue;

    float position[3];
    float vWorld[3];
    float rpy[3];
    
    LowlevelState()
    {
        for(int i = 0; i < 3; i++)
        {
            position[i] = 0;
            vWorld[i] = 0;
            rpy[i] = 0;
        }
    }

};

```

# MPC

## ConvexMPClocomotion

```c++
class ConvexMPCLocomotion {

public:

  // Constructors

  ConvexMPCLocomotion(double _dt, int _iterations_between_mpc);

 

  // Main Functionalities

  void run(ControlFSMData& data);

  void setGaitNum(int gaitNum) { gaitNumber = gaitNum; }

  bool firstRun = true;

private:

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode);

  void GenerateTrajectory(int* mpcTable, ControlFSMData& data, bool omniMode);

  // Locomotion and Gait Parameters

  int iterationsBetweenMPC;//控制器迭代步间隔

  int horizonLength;

  double dt;//控制迭代步

  double dtMPC;//MPC迭代步

  int iterationCounter = 0;

  Vec6<double> f_ff[2];

  Vec12<double> Forces_Sol;

  Vec2<double> swingTimes;

  FootSwingTrajectory<double> footSwingTrajectories[2];

  Gait walking, standing;

  // Feedback and Control Variables

  Mat3<double> Kp, Kd, Kp_stance, Kd_stance;

  bool firstSwing[2] = {true, true};

  double swingTimeRemaining[2];

  double stand_traj[6];

  int current_gait;

  int gaitNumber;

  Vec3<double> world_position_desired;//body frame在世界坐标系位置world_position_desired[0] += dt * v_des_world[0];

  Vec3<double> rpy_int;

  Vec3<double> rpy_comp;

  Vec3<double> pFoot[2];

  CMPC_Result result;

  double trajAll[12*10];

  Mat43<double> W;  

  Vec3<double> a;  

  Vec4<double> pz;

  double ground_pitch;

  Vec3<double> pBody_des;//body frame 在世界坐标系下位置

  Vec3<double> vBody_des;//body frame 在世界坐标系下速度

  Vec3<double> aBody_des;

  Vec3<double> pBody_RPY_des;

  Vec3<double> vBody_Ori_des;

  Vec3<double> pFoot_des[2];

  Vec3<double> vFoot_des[2];

  Vec3<double> aFoot_des[2];

  Vec3<double> Fr_des[2];

  Vec2<double> contact_state;

  Vec3<double> v_des_robot;

  bool climb = 0;

  ofstream foot_position;

  Vec3<double> ori_des_world;   

};
```

## MPC状态量

```c++
  double trajInitial[12] = {/*rpy_comp[0] + */stateCommand->data.stateDes[3],  // 0 期望姿态
          /*rpy_comp[1] + */stateCommand->data.stateDes[4],   // 1
          seResult.rpy[2],   // 2
          xStart,                  // 3期望世界坐标系下位置
          yStart,                  // 4
          0.55 ,  // 5
          0,                     // 6 期望角速度 wx
          0,                     // 7
          stateCommand->data.stateDes[11],  // 8
          v_des_world[0],              // 9 期望速度
          v_des_world[1],              // 10
          0};                    // 11
```

## MPC_interface

### problem_setup

```c++
struct problem_setup
{
 float dt;
 float mu;
 float f_max;
 int horizon;
};
```

### update_data_t

```c++
#define K_MAX_GAIT_SEGMENTS 36
struct update_data_t
{
 float p[3];
 float v[3];
 float q[4];//姿态四元数
 float w[3];
 float r[6];//两个足的位置
 float joint_angles[10];
 float yaw;
 float weights[12];
 float traj[12*K_MAX_GAIT_SEGMENTS];
 // float alpha;
 float Alpha_K[12];
 unsigned char gait[K_MAX_GAIT_SEGMENTS];
 unsigned char hack_pad[1000];
 int max_iterations;
 double rho, sigma, solver_alpha, terminate;
};
```

## RobotState

```c++
typedef float fpt;

class RobotState
{
  public:
   void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
   //void compute_rotations();
   void print();
   Matrix<fpt,3,1> p,v,w;
   Matrix<fpt,3,2> r_feet;
   Matrix<fpt,3,3> R;
   Matrix<fpt,3,3> R_yaw;
   Matrix<fpt,3,3> I_body;
   Quaternionf q;
   fpt yaw;
   //fpt m = 19; // Aliengo
   fpt m = 13; // A1

  //private:

};
```



# FootSwingTrajectory

```c++
template<typename T>
class FootSwingTrajectory {
public:
       FootSwingTrajectory() {
         _p0.setZero();
         _pf.setZero();
         _p.setZero();
         _v.setZero();
         _height = 0;
       }
   * Set the starting location of the foot
   * @param p0 : the initial foot position
     */
       void setInitialPosition(Vec3<T> p0) {
         _p0 = p0;
       }
  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
     */
       void setFinalPosition(Vec3<T> pf) {
         _pf = pf;
       }
  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
     */
       void setHeight(T h) {
         _height = h;
       }
  void computeSwingTrajectoryBezier(T phase, T swingTime);
  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
     */
       Vec3<T> getPosition() {
         return _p;
       }
  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
     */
       Vec3<T> getVelocity() {
         return _v;
       }
private:
  Vec3<T> _p0, _pf, _p, _v;
  T _height;
};
```



# Beizer Interpolate

## Linear Beizer

$$
\mathbf{B}(t)=\mathbf{P}_0 + t(\mathbf{P}_1-\mathbf{P}_0)=(1-t)\mathbf{P}_0 + t\mathbf{P}_1,\ 0 \le t\le1
$$



```c++
template <typename y_t, typename x_t>

y_t lerp(y_t y0, y_t yf, x_t x) {

 static_assert(std::is_floating_point<x_t>::value,"must use floating point value");

 assert(x >= 0 && x <= 1);

 return y0 + (yf - y0) * x;

}
```

## Cubic Beizer

$$
\mathbf{B}(t) = (1-t)^3\mathbf{P}_0+3(1-t)^2t\mathbf{P}_1+3(1-t)t^2\mathbf{P}_2+t^3\mathbf{P}_3,\ 0 \le t \le 1.
$$



首尾两端导数为0，P0=01；P2=P3；

```c++
template <typename y_t, typename x_t>

y_t cubicBezier(y_t y0, y_t yf, x_t x) {

 static_assert(std::is_floating_point<x_t>::value,  "must use floating point value");

 assert(x >= 0 && x <= 1);

 y_t yDiff = yf - y0;

 x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));

 return y0 + bezier * yDiff;

}
```

# Gait

```c++
class Gait

{

public:

 Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int>  durations, const std::string& name="");

 ~Gait();

 Vec2<double> getContactSubPhase();

 Vec2<double> getSwingSubPhase();

 int* mpc_gait();

 void setIterations(int iterationsPerMPC, int currentIteration);

 int _stance;//站立项MPC迭代数

 int _swing;//摆动项MPC迭代数

private:

 int _nMPC_segments;

 int* _mpc_table;

 Array2i _offsets;      // offset in mpc segments

 Array2i _durations;     // duration of step in mpc segments

 Array2d _offsetsPhase;    // offsets in phase (0 to 1) =_stance/_nIterations

 Array2d _durationsPhase;   // durations in phase (0 to 1)

 int _iteration;//当前的MPC迭代步数
 int _nIterations;//步态周期的总MPC迭代步数
 int currentIteration;//总控制迭代步
 double _phase;//0到1，当前步态相位

};
```

