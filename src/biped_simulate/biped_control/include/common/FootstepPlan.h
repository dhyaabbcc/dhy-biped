#include "Contact.h"
#include "Biped.h"
#include <algorithm>

struct FootstepPlan
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void complete(const Sole & sole);
    void goToNextFootstep();
    void goToNextFootstep(const pinocchio::SE3 & actualTargetPose);
    void load();
    void reset(unsigned startIndex);
    void restorePreviousFootstep();
    void updateInitialTransform(const pinocchio::SE3 & X_0_lf, const pinocchio::SE3 & X_0_rf, double initHeight);

    void appendContact(Contact step)
    {
        contacts_.push_back(step);
    }
    double comHeight() const
    {
        return comHeight_;
    }
    void comHeight(double height)
    {
        comHeight_ = MyWrapper::clamp(height, 0., 2.);
    }
    const std::vector<Contact> & contacts() const
    {
        return contacts_;
    }
    double doubleSupportDuration() const
    {
        return doubleSupportDuration_;
    }
    double finalDSPDuration() const
    {
        return finalDSPDuration_;
    }
    void finalDSPDuration(double duration)
    {
        finalDSPDuration_ = MyWrapper::clamp(duration, 0.1, 1.6);
    }
    double initDSPDuration() const
    {
        return initDSPDuration_;
    }
    void initDSPDuration(double duration)
    {
        initDSPDuration_ = MyWrapper::clamp(duration, 0.1, 1.6);
    }
    const pinocchio::SE3 & initPose()
    {
        return X_0_init_;
    }
    double landingDuration()
    {
        landingDuration_=0.1;
        return landingDuration_;
    }
    void landingDuration(double duration)
    {
        landingDuration_ = MyWrapper::clamp(duration, 0., 0.5);
    }
    double landingPitch() const
    {
        return landingPitch_;
    }
    void landingPitch(double pitch)
    {
        constexpr double MIN_LANDING_PITCH = -1.;
        constexpr double MAX_LANDING_PITCH = 1.;
        landingPitch_ = MyWrapper::clamp(pitch, MIN_LANDING_PITCH, MAX_LANDING_PITCH);
    }
    const Contact & nextContact() const
    {
        return nextContact_;
    }
    const Contact & prevContact() const
    {
        return prevContact_;
    }
    void resetContacts(const std::vector<Contact> & contacts)
    {
        contacts_ = contacts;
    }
    double singleSupportDuration() const
    {
        return singleSupportDuration_;
    }
    void singleSupportDuration(double duration)
    {
        singleSupportDuration_ = MyWrapper::clamp(duration, 0., 2.);
    }
    const Contact & supportContact() const
    {
        return supportContact_;
    }
    void swapFirstTwoContacts()
    {
        std::swap(contacts_[0], contacts_[1]);
    }
    double swingHeight()
    {
        swingHeight_=0.24;
        return swingHeight_;
    }
    void swingHeight(double height)
    {
        constexpr double MIN_SWING_FOOT_HEIGHT = 0.;
        constexpr double MAX_SWING_FOOT_HEIGHT = 0.25;
        swingHeight_ = MyWrapper::clamp(height, MIN_SWING_FOOT_HEIGHT, MAX_SWING_FOOT_HEIGHT);
    }
    double takeoffDuration()
    {
        takeoffDuration_=0.42;
        return takeoffDuration_;
    }
    void takeoffDuration(double duration)
    {
        takeoffDuration_ = MyWrapper::clamp(duration, 0., 0.5);
    }
    Eigen::Vector3d takeoffOffset()
    {
        takeoffOffset_ = prevContact_.swing.takeoff_offset;
        return takeoffOffset_;
    }
    void takeoffOffset(const Eigen::Vector3d & offset)
    {
        takeoffOffset_ = offset;
    }
    double takeoffPitch()
    {
        takeoffPitch_ = prevContact_.swing.takeoff_pitch;
        return takeoffPitch_;
    }
    void takeoffPitch(double pitch)
    {
        constexpr double MIN_TAKEOFF_PITCH = -1.;
        constexpr double MAX_TAKEOFF_PITCH = 1.;
        takeoffPitch_ = MyWrapper::clamp(pitch, MIN_TAKEOFF_PITCH, MAX_TAKEOFF_PITCH);
    }
    const Contact & targetContact() const
    {
        return targetContact_;
    }
    double torsoPitch() const
    {
        return torsoPitch_;
    }
    void rewind()
    {
        reset(0);
    }





    Contact nextContact_;
    Contact prevContact_;
    Contact supportContact_;
    Contact targetContact_;
    double comHeight_ = 0.8;
    std::vector<Contact> contacts_;
    double doubleSupportDuration_ = 0.2;
    double finalDSPDuration_ =  0.6;
    double initDSPDuration_ = 0.6;
    double landingDuration_ = 0.15;
    double singleSupportDuration_ = 0.8;
    double swingHeight_ = 0.04;
    double takeoffDuration_ = 0.05;
    double takeoffPitch_ = 0.0;
    double torsoPitch_ = -100.0;
    double landingPitch_=0.0;
    Eigen::Vector3d takeoffOffset_=Eigen::Vector3d::Zero();
    pinocchio::SE3 X_0_init_;
    unsigned nextFootstep_ = 0;
};
