#include "bip_wholebody_impulse_ctrl.hpp"

#define NumMotor 12
#define BaseFreedomDim 6

template <typename T>
bipWBCimpulseCtrl<T>::bipWBCimpulseCtrl(size_t dim_qdot,
                                        const std::vector<bipSingleContact<T> *> *contact_list, const std::vector<BipTask<T> *> *task_list) : dimFb(BaseFreedomDim), dimQdot(dim_qdot), numActJoint(dim_qdot - dimFb)
{
    Sf = DMat<T>::Zero(BaseFreedomDim, dimQdot);
    Sf.block(0, 0, 6, 6).setIdentity();
    identityMat = DMat<T>::Identity(dimQdot, dimQdot);

    contactList = contact_list;
    taskList = task_list;
}

template <typename T>
void bipWBCimpulseCtrl<T>::GetModelRes(std::shared_ptr<pinocchio::Data> FB_Data)
{
    A = FB_Data->M.cast<T>();
   // Gravity = FB_Data->g.cast<T>();
    //Coriolis = FB_Data->C.cast<T>();
    Nle=FB_Data->nle.cast<T>();
    Ainv = A.inverse();

    settingUpdated = true;
}

template <typename T>
void bipWBCimpulseCtrl<T>::SetOptimizationSize()
{
    dimFr = 0;
    dimIneqConstraint = 0;
    for (size_t i(0); i < contactList->size(); ++i)
    {
        dimFr += (*contactList)[i]->GetDimContact();
        dimIneqConstraint += (*contactList)[i]->GetDimUf();
    }

    dimOptimal = dimFb + dimFr; /* 6 + 6 * ContactNum */
    dimEqConstraint = dimFb;

    qpG.resize(0., dimOptimal, dimOptimal);
    qpg0.resize(0., dimOptimal);
    qpCE.resize(0., dimOptimal, dimEqConstraint);
    qpce0.resize(0., dimEqConstraint);

    CE = DMat<T>::Zero(dimEqConstraint, dimOptimal);
    ce0 = DVec<T>(dimEqConstraint);
    if (dimFr > 0)
    {
        qpCI.resize(0., dimOptimal, dimIneqConstraint);
        qpci0.resize(0., dimIneqConstraint);
        CI = DMat<T>::Zero(dimIneqConstraint, dimOptimal);
        ci0 = DVec<T>(dimIneqConstraint);

        JC = DMat<T>(dimFr, dimQdot);
        JCDotQdot = DVec<T>(dimFr);
        desiredFr = DVec<T>(dimFr);

        UF = DMat<T>(dimIneqConstraint, dimFr);
        UF.setZero();
        ineqVec = DVec<T>(dimIneqConstraint);
    }
    else
    {
        qpCI.resize(0., dimOptimal, 1);
        qpci0.resize(0., 1);
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::SetCost()
{
    size_t idx_offset(0);

    /* The cost includes two parts, the floating base part and reaction force part.
     * All stored into a diagnose matrix.
     */
    for (size_t i = 0; i < dimFb; ++i)
    {
        qpG[i + idx_offset][i + idx_offset] = extraData->weightFb[i];
    }

    idx_offset += dimFb;
    for (size_t i = 0; i < dimFr; ++i)
    {
        qpG[i + idx_offset][i + idx_offset] = extraData->weightFr[i];
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::MakeTorque(DVec<T> &cmd, void *extraInput )
{
    if (!settingUpdated)
    {
        printf("[Wanning] WBIC setting is not done\n");
    }

    if (extraInput)
    {   
        extraData = static_cast<bipWBICExtraData<T> *>(extraInput);
    }

    SetOptimizationSize();
    SetCost();

    DVec<T> qddot_pre;
    DMat<T> JcBar;
    DMat<T> Npre;
    if (dimFr > 0)
    {
        ContactBuilding();
        SetInequalityConstraint(); /* Setup Inequality constraints by the way. */
        WeightedInverse(JC, Ainv, JcBar);
        qddot_pre = JcBar * (-JCDotQdot);
        Npre.noalias() = identityMat - JcBar * JC;
    }
    else
    {
        qddot_pre = DVec<T>::Zero(dimQdot);
        Npre = identityMat;
    }

    BipTask<T> *task;
    DMat<T> Jt, JtBar, JtPre;
    DVec<T> JtDotQdot, xddot;
    for (size_t i = 0; i < taskList->size(); ++i)
    {
        task = (*taskList)[i];
        task->GetJt(Jt);
        task->GetJtDotQdot(JtDotQdot);
        task->GetXddotCmd(xddot);
        JtPre.noalias() = Jt * Npre;
        WeightedInverse(JtPre, Ainv, JtBar);
        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        if (i < taskList->size() - 1)
        {
            Npre = Npre * (identityMat - JtBar * JtPre);
        }
    }

    SetEqualityConstraint(qddot_pre); /* Setup the dynamics constraint. */
    T f = solve_quadprog(qpG, qpg0, qpCE, qpce0, qpCI, qpci0, qpz);

    for (size_t i = 0; i < dimFb; ++i)
    {
        qddot_pre[i] += qpz[i];
    }
    GetSolution(qddot_pre, cmd);
    extraData->optimizedResult = DVec<T>(dimOptimal);
    for (size_t i = 0; i < dimOptimal; ++i)
    {
        extraData->optimizedResult[i] = qpz[i];
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::ContactBuilding()
{
    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    size_t dim_accumul_rf = 0, dim_accumul_uf = 0;
    size_t dim_new_rf = 0, dim_new_uf = 0;

    for (size_t i(0); i < contactList->size(); ++i)
    {
        (*contactList)[i]->GetJc(Jc); /* Jc is 6x18 matrix. */
        (*contactList)[i]->GetJcDotQdot(JcDotQdot);
        dim_new_rf = (*contactList)[i]->GetDimContact();
        dim_new_uf = (*contactList)[i]->GetDimUf();

        /* Stacked contact jacobian. */
        JC.block(dim_accumul_rf, 0, dim_new_rf, dimQdot) = Jc;

        /* Stacked JCDotQdot. */
        JCDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        /* Force part of matrix of inequality constraint. */
        (*contactList)[i]->GetUf(Uf);
        UF.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        /* Vector of inequality constraint. */
        (*contactList)[i]->GetIneqVec(Uf_ieq_vec);
        ineqVec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        /* Desired reaction force frame MPC. */
        desiredFr.segment(dim_accumul_rf, dim_new_rf) = (*contactList)[i]->GetDesiredFr();

        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::SetEqualityConstraint(const DVec<T> &qddot)
{
    if (dimFr > 0)
    {
        /* The dynamicsCE will seems like: [ A6x6 | -Sf * JC^T ]. */
        CE.block(0, 0, dimEqConstraint, dimFb) = A.block(0, 0, dimFb, dimFb);
        CE.block(0, dimFb, dimEqConstraint, dimFr) = -Sf * JC.transpose();
        ce0 = -Sf * (A * qddot +Nle - JC.transpose() * desiredFr);
    }
    else
    {
        CE.block(0, 0, dimEqConstraint, dimFb) = A.block(0, 0, dimFb, dimFb);
        ce0 = -Sf * (A * qddot + Nle);
    }

    /* Convert to quadprogpp matrix/vector. */
    for (size_t i = 0; i < dimEqConstraint; ++i)
    {
        for (size_t j = 0; j < dimOptimal; ++j)
        {
            qpCE[j][i] = CE(i, j);
        }
        qpce0[i] = -ce0[i];
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::SetInequalityConstraint()
{
    /* Force limitation and friction cone constraints.
     * W * fr > 0; fr = fr_MPC + v_fr.
     */
    CI.block(0, dimFb, dimIneqConstraint, dimFr) = UF;
    ci0 = ineqVec - UF * desiredFr;

    /* Convert to quadprogpp matrix/vector. */
    for (size_t i(0); i < dimIneqConstraint; ++i)
    {
        for (size_t j(0); j < dimOptimal; ++j)
        {
            qpCI[j][i] = CI(i, j);
        }
        qpci0[i] = -ci0[i];
    }
}

template <typename T>
void bipWBCimpulseCtrl<T>::WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv, double threshold)
{
    /* J_bar = A_inv * J^T ( J * A_inv * J^T)^(-1). */
    DMat<T> temp(Winv * J.transpose());
    DMat<T> lambda(J * temp);
    DMat<T> lambda_inv;
    robotics::math::pseudoInverse(lambda, threshold, lambda_inv);
    Jinv.noalias() = temp * lambda_inv;
}

template <typename T>
void bipWBCimpulseCtrl<T>::GetSolution(const DVec<T> &qddot, DVec<T> &cmd)
{
    DVec<T> tot_tau;

    if (dimFr > 0)
    {
        extraData->optimalFr = DVec<T>(dimFr);

        /* Store total reaction force to extra data. */
        for (size_t i = 0; i < dimFr; ++i)
        {
            extraData->optimalFr[i] = qpz[i + dimFb] + desiredFr[i];
        }
        tot_tau = A * qddot +Nle - JC.transpose() * extraData->optimalFr;
        if(!((*contactList)[0]->contact_leg))
        {
            std::cout<<"WBC计算的不同值:\n";
            for(int i=0;i<6;i++)
            {
                logger.Fwbc[i]= extraData->optimalFr[i];
                std::cout<<desiredFr[i]<<'\n';
            }
            
        }
        else
        {
             for(int i=0;i<6;i++)
            {
                logger.Fwbc[i]=logger.Fmpc[i];
                std::cout<<desiredFr[i]<<'\n';
            }
        }
    }
    else
    {
        tot_tau = A * qddot + Nle;

    }
    cmd = tot_tau.tail(numActJoint);
    std::cout<<"WBC Torque:\n"<<cmd<<'\n';
}
   
template class bipWBCimpulseCtrl<float>;