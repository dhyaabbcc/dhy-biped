#include "Estimator.h"

void Estimator::update()
{
    if (!container || !lowState)
    {
        std::cerr << "[Estimator] Missing container or lowState.\n";
        return;
    }

    container->run();
    const StateEstimate* est = container->_data.result;

    realCoM = est->position;
    realCoMd = est->vWorld;

    // 更新足端力
    for (int i = 0; i < 2; ++i)
    {
        Eigen::Matrix<double, 6, 1> force;
        for (int j = 0; j < 6; ++j)
        {
            force(j) = lowState->feettwist[i * 6 + j];
        }
        footforce[i] = force;
    }
}

Eigen::Vector3d Estimator::com() const
{
    return realCoM;
}

Eigen::Vector3d Estimator::comd() const
{
    return realCoMd;
}

std::string Estimator::supportContact() const
{
    return (footforce[0][2]  > footforce[1][2] ) ? "lcontactpoint" : "rcontactpoint";
}

void Estimator::setSensorForce(const std::string foot, const Eigen::Matrix<double, 6, 1> &force)
{
    if (foot == "lcontactpoint")
        footforce[0] = force;
    else
        footforce[1] = force;
}