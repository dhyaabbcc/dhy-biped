#ifndef BIP_CONTACT_H
#define BIP_CONTACT_H

#include "../../robotwrapper/robotwrapper.h"
#include "common/ControlFSMData.h"
#include "common/Math/bip_se3.h"
#include <array>



template<typename T>
class bipSingleContact {

public:

    /**
     * @brief Constructor of the class bipSingleContact.
     * @param robot: Class to represent a floating base rigid body model with rotors and ground contacts. No concept of state.
     * @param contact_pt: the number of contact points.
     */
    bipSingleContact(std::shared_ptr<RobotWrapper> fb_model,std::shared_ptr<pinocchio::Data> fb_data,ControlFSMData *_data,short CONTACTLEG);

    virtual ~bipSingleContact() = default;

    /**
     * @brief Update contact jacobian and inequivalent constraint vector.
     * @return true if update has finished.
     */
    bool UpdateContactSpec();

    /**
     * @brief Get the row count/dimension of member Uf.
     */
    size_t GetDimUf() const {
        return Uf.rows();
    }

    /**
     * @brief Getter method of member dimContact.
     */
    size_t GetDimContact() const {
        return dimContact;
    }

    /**
     * @brief Getter method of member Jc.
     */
    void GetJc(DMat<T>& Jc) const {
        Jc = this->Jc;
    }

    /**
     * @brief Getter method of member JcDotQDot.
     */
    void GetJcDotQdot(DVec<T>& JcDotQdot) const {
        JcDotQdot = this->JcDotQdot;
    }

    /**
     * @brief Getter method of member Uf.
     */
    void GetUf(DMat<T>& Uf) const {
        Uf = this->Uf;
    }

    /**
     * @brief Getter method of member ineqVec.
     */
    void GetIneqVec(DVec<T>& ineqVec) const {
        ineqVec = this->ineqVec;
    }

    /**
     * @brief Getter method of member desiredFr
     */
    const DVec<T>& GetDesiredFr() const {
        return desiredFr;
    }

    /**
     * @brief Setter method of member desiredFr.
     */
    void SetDesiredFr(const DVec<T>& desiredFr) {
        this->desiredFr = desiredFr;
    }
    pinocchio::JointIndex indexContact;
      /**
     * @brief Current index of contact point.
     * 0, 1 for R, L
     */

    short contact_leg;

protected:

    /**
     * @brief Update Jc by reading the contact jacobian from MIT floating base model.
     * @return true if the update has finished.
     */
    bool UpdateJc();

    /**
     * @brief Update JcDotQdot by reading the contact jacobian from MIT floating base model.
     * @return true if the update has finished.
     */
    bool UpdateJcDotQdot();

    /**
     * @brief Update Uf matrix.
     * Currently the matrix is constant, so the method just return true.
     * @todo Consider add dynamic terrain information to UpdateUf().
     * @return true if the update has finished.
     */
    bool UpdateUf();

    /**
     * @brief Update ineqVec.
     * Currently only limiting the force on Z axis not to be greater than the weight of quadruped,
     * or it will start bumping.
     * @return true if the update has finished.
     */
    bool UpdateIneqVec();
    

    /**
     * @brief Maximum force of the contact point along z-axis.
     */
    T maxFz;

    /**
     * @brief Current index of contact point.
     * 0, 1 for R, L
     */
    //short contact_leg;

   // pinocchio::JointIndex indexContact;

    std::array<std::string,2> contact_frame={std::string("rcontactpoint"),std::string("lcontactpoint")};

    /**
     * @brief Dimension of constraint matrix.
     * Usually set to 6, including 4 conic constraints and 2 boundary constraints.
     */
    int dimU;

    /**
     * @brief Friction factor of the terrain.
     * @todo Consider add dynamic terrain information to mu.
     */
    T mu;

    /**
     * @brief z-force index in force vector.
     * Usually set to 2.
     */
    int indexFz;

    /**
     * @brief 6x3 inequivalent constraint martix, including conic and boundary constraints.
     *
     */
    DMat<T> Uf;


    /**
     * @brief Desired reaction force.
     * This is set to the result force from MPC solver.
     */
    DVec<T> desiredFr;

    /**
     * @brief 6x1 inequivalent vector.
     * The sixth entry is set to -maxFz to satisfy fz < maxFz.
     */
    DVec<T> ineqVec;

    /**
     * @brief Single contact jacobian of corresponding contact point.
     */
    DMat<T> Jc;

    /**
     * @brief Derivative of Jc dot derivative of q.
     * Used in null-space projection.
     */
    DVec<T> JcDotQdot;

    /**
     * @brief Dimension of contact point.
     * Usually set to 3. (x, y, z)
     */
    size_t dimContact;

    std::shared_ptr<RobotWrapper> FB_Model;
    std::shared_ptr<pinocchio::Data> FB_Data;
    ControlFSMData *_data;

    const float feetwidth=0.05;

    // feetL1 前脚掌长度，L2后脚掌长度
    const float feetL1=0.09,feetL2=0.06;

};



#endif
