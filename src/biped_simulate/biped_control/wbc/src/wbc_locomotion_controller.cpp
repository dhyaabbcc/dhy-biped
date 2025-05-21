#include "wbc_locomotion_controller.h"

#define NumMotor 12
#define BaseFreedomDim 6
#define NumLeg 2

template <typename T>
bipWBCLocomotionControl<T>::bipWBCLocomotionControl(ControlFSMData *_data) : controlFSMData(_data), fullConfig(NumMotor + 7), 
jointTorqueCmd(NumMotor), desiredJPos(NumMotor), desiredJVel(NumMotor), iteration(0), dimConfig(NumMotor + BaseFreedomDim),jointvel(NumMotor + BaseFreedomDim)
{
   FB_Model = _data->_biped->_Dyptr;
   FB_Data = _data->_biped->_Dataptr;

   fullConfig.setZero();
   zeroVec3.setZero();

   multitask = new bipMultitaskProjection<T>(dimConfig);
   wbic = new bipWBCimpulseCtrl<T>(dimConfig, &contactList, &taskList);

   wbicExtraData = new bipWBICExtraData<T>();
   wbicExtraData->weightFb = DVec<T>::Constant(BaseFreedomDim, 0.1);
   wbicExtraData->weightFr = DVec<T>::Constant(12, 1);

   taskBodyOri = new BipTaskBodyOri<T>(FB_Model, FB_Data, controlFSMData);
   taskBodyPos = new BipTaskBodyPos<T>(FB_Model, FB_Data, controlFSMData);

   footContact[0] = new bipSingleContact<T>(FB_Model, FB_Data, controlFSMData, linkID::RLEG);
   footContact[1] = new bipSingleContact<T>(FB_Model, FB_Data, controlFSMData, linkID::LLEG);

   taskFootPos[0] = new bipTaskLinkPos<T>(FB_Model, FB_Data, controlFSMData, linkID::RLEG);
   taskFootPos[1] = new bipTaskLinkPos<T>(FB_Model, FB_Data, controlFSMData, linkID::LLEG);

  // jointvel.size(18);
   //jointvel(18);
   // for (size_t i = 0; i < 3; ++i)
   // {

   //    ((BipTaskBodyOri<T> *)taskBodyOri)->Kp[i] = 100.;
   //    ((BipTaskBodyOri<T> *)taskBodyOri)->Kd[i] = 10.;
   // }
}

template <typename T>
bipWBCLocomotionControl<T>::~bipWBCLocomotionControl()
{
   delete taskBodyOri;
   delete taskBodyPos;
   for (int i = 0; i < 2; i++)
   {
      delete footContact[i];
      delete taskFootPos[i];
   }

   delete multitask;
   delete wbic;
   delete wbicExtraData;

   typename std::vector<BipTask<T> *>::iterator iter = taskList.begin();
   while (iter < taskList.end())
   {
      delete (*iter);
      ++iter;
   }
   taskList.clear();

   typename std::vector<bipSingleContact<T> *>::iterator iter2 = contactList.begin();
   while (iter2 < contactList.end())
   {
      delete (*iter2);
      ++iter2;
   }
   contactList.clear();
}

template <typename T>
void bipWBCLocomotionControl<T>::Run(void *precomputeData)
{
   bipWbcCtrlData *beforeWbcData = static_cast<bipWbcCtrlData *>(precomputeData);
   if (iteration % 1 == 0&&beforeWbcData->allowAfterMPC == true)
   {
      // if (beforeWbcData->allowAfterMPC == false)
      //    return;

      UpdateModel();

      ContactTaskUpdate(beforeWbcData);

      multitask->FindConfiguration(fullConfig, taskList, contactList, desiredJPos, desiredJVel);

      for (int i = 0; i < 6; i++)
      {
         logger.jointpos[i] = fullConfig[7+ i];
         logger.jointposdes[i]=desiredJPos[i];
       //  std::cout<<"右腿实际角度:\n"<<fullConfig[7+i]<<'\n';
        // std::cout<<"右腿规划角度:\n"<<desiredJPos[i]<<'\n';
      }
      //logger.Senddata();
       wbic->MakeTorque(jointTorqueCmd, wbicExtraData);
   }
   UpdateLegCMD();
   iteration++;
}

template <typename T>
void bipWBCLocomotionControl<T>::UpdateModel()
{
   // base 的实际状态
   ControlFSMData *_data = controlFSMData;
   auto &seResult = _data->_stateEstimator->getResult();
   Eigen::Vector3d Base_pos = seResult.position;
   Eigen::Vector4d Base_ori;
   Eigen::Vector3d Base_omega = seResult.omegaWorld;
   Eigen::Vector3d Base_vel = seResult.vWorld;

   Base_ori.block(0, 0, 3, 1) = seResult.orientation.block(1, 0, 3, 1); // pinocchio 四元数顺序为 [x,y,z,w]
   Base_ori[3] = seResult.orientation[0];

   Eigen::VectorXd q(FB_Model->m_model.nq);
   Eigen::VectorXd v(FB_Model->m_model.nv);
  
   // q.setZero();
   // v.setZero();
   // Eigen::Vector3d ORI(.0,.0,M_PI/2);

   //  Eigen::Vector4d frpy=RobotWrapper::rpytoquat(ORI);
   // q.block<4,1>(3,0)=frpy;

   std::cout << "Base_ori:\n"
             << Base_ori << '\n';

   q.block(0, 0, 3, 1) = Base_pos;
   q.block(3, 0, 4, 1) = Base_ori;
   v.block(0, 0, 3, 1) = Base_vel;
   v.block(3, 0, 3, 1) = Base_omega;
   for (int i = 0; i < 6; i++)
   {
      q(7 + i) = _data->_legController->data[1].q(i);
      q(7 + 6 + i) = _data->_legController->data[0].q(i); // 0腿为右腿，在kinematic tree 中索引值为8
      v(6 + i) = _data->_legController->data[1].qd(i);
      v(6 + 6 + i) = _data->_legController->data[0].qd(i);
   }
   pino::normalize(FB_Model->m_model, q);
   fullConfig = q.cast<T>();
   jointvel=v.cast<T>();
   // std::cout<<"pinocchio q:\n"<<q<<'\n';
   // std::cout<<"pinocchio v:\n"<<v<<'\n';

   FB_Model->computeMainTerms(*FB_Data, q, v);
   FB_Model->computedrift(*FB_Data, q, v);
   wbic->GetModelRes(FB_Data);
}

template <typename T>
void bipWBCLocomotionControl<T>::ContactTaskUpdate(bipWbcCtrlData *ctrl_data)
{
   // std::cout<<"contactTASK UPDATASE AAAAAAAAAAAAAAA\n";
   contactList.clear();
   taskList.clear();
   // Quat<T> quatDes = RobotWrapper::rpytoquat(ctrl_data->pBody_RPY_des);
   Quat<T> quatDes;
   RobotWrapper::rpytoquat(ctrl_data->pBody_RPY_des);

   taskBodyOri->UpdateTask(&quatDes, ctrl_data->vBody_Ori_des, zeroVec3);
   taskBodyPos->UpdateTask(&ctrl_data->pBody_des, zeroVec3, ctrl_data->aBody_des);

   taskList.push_back(taskBodyOri);
   taskList.push_back(taskBodyPos);

   for (size_t leg = 0; leg < NumLeg; leg++)
   {
      if (ctrl_data->contact_state[leg])
      {
         footContact[leg]->SetDesiredFr(ctrl_data->Fr_des[leg].cast<T>());
         footContact[leg]->UpdateContactSpec();
         contactList.push_back(footContact[leg]);
      }
      else
      {
         // std::cout<<"WBC 接受摆腿位置:\n"<<ctrl_data->pFoot_des[leg]<<'\n';
         taskFootPos[leg]->UpdateTask(&(ctrl_data->pFoot_des[leg]), ctrl_data->vFoot_des[leg], ctrl_data->aFoot_des[leg]);
         if(leg==0)
         {
           // std::cout<<"右腿雅可比:\n"<<taskFootPos[leg]->Jt<<'\n';
           // std::cout<<"计算右腿速度:\n"<<taskFootPos[leg]->Jt*jointvel<<'\n';
           // std::cout<<"测量右腿速度:\n"<<controlFSMData->_legController->Feettwist[0]<<'\n';
         }
            
         taskList.push_back(taskFootPos[leg]);

      }
   }
}

template <typename T>
void bipWBCLocomotionControl<T>::UpdateLegCMD()
{
   Vec6<double> Lt ;
   Vec6<double> Rt ;
   
   for(int i=0;i<6;i++)
   {
      Lt[i]=(double)jointTorqueCmd[i];
      Rt[i]=(double)jointTorqueCmd[i+6];
   }

   controlFSMData->_legController->updateTorque(controlFSMData->_lowCmd,Rt,Lt);
  
//   for(int i=0;i<6;i++)
//   {

//    controlFSMData->_legController->commands[0].qDes[i] = (double)desiredJPos[6+i];
//    controlFSMData->_legController->commands[0].qdDes[i] =(double)desiredJVel[6+i];

//    controlFSMData->_legController->commands[1].qDes[i] = (double)desiredJPos[i];
//    controlFSMData->_legController->commands[1].qdDes[i] =(double)desiredJVel[i];
//   }
  
   
}

template class bipWBCLocomotionControl<float>;