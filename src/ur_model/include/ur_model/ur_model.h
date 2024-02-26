/*! \file ur_model.h
 *
 * \brief Description of URModel class for Universal Robots kinematics and dynamics.
 *
 * \author Yanbing LIU
 *
 * \date 17/02/2024
 *
 * \copyright Your Copyright
 *
 * License details.
 *
 */
#ifndef UR_MODEL_H
#define UR_MODEL_H

//////////////////////////////////////////////////////////////////////////////////////////////
// Reference library
//////////////////////////////////////////////////////////////////////////////////////////////
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ur_model/model_base.h>
#include <ros/node_handle.h>
#include <control_core/types.h>


namespace ur_model_namespace
{
    class URModel : public model_interface::ModelBase
    {
    //////////////////////////////////////////////////////////////////////////////////////////////
    // Variable
    // MatrixX, VectorX: [dynamically] sized matrix
    //                   Matrix(m,n), Vector(x,1)
    // MatrixDof, VectorDof: [robot] sized matrix
    //////////////////////////////////////////////////////////////////////////////////////////////

    public:
        ////////////////////////////////
        // Types
        ////////////////////////////////

        typedef model_interface::ModelBase Base;
        typedef cc::VectorX Regressor_Theta;    // 61*1
        typedef cc::MatrixX Regressor_Yr;       // 6*61
        

    private:
        ////////////////////////////////
        // Variable
        // M, C, G
        // Theta, Yr
        // Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        // Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////
        cc::Scalar L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12;
        cc::Scalar m1, m2, m3, m4, m5, m6;
        // g = [gx, gy, gz];
        cc::Scalar gx, gy, gz;

        cc::Scalar I111, I112, I113, I122, I123, I133;
        cc::Scalar I211, I212, I213, I222, I223, I233;
        cc::Scalar I311, I312, I313, I322, I323, I333;
        cc::Scalar I411, I412, I413, I422, I423, I433;
        cc::Scalar I511, I512, I513, I522, I523, I533;
        cc::Scalar I611, I612, I613, I622, I623, I633;

        cc::MatrixDof M_;
        cc::MatrixDof C_;
        cc::VectorDof G_;

        Regressor_Theta Theta_;
        Regressor_Yr Yr_;

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Function
    //////////////////////////////////////////////////////////////////////////////////////////////

    public:
        // Constructor & Destructor
        URModel(const std::string& name = "ur_model");
        virtual ~URModel();

        // Write the [init-method] in ModelBase
        virtual bool init(ros::NodeHandle& nh) override;

        ////////////////////////////////
        /// M, C, G Matrix caculet
        /// Update(used [private] funciton) & Return
        ////////////////////////////////
        const cc::MatrixDof &Inertia_Matrix(const cc::JointPosition &q);
        const cc::MatrixDof &Coriolis_Matrix(const cc::JointPosition &q, const cc::JointVelocity &qp);
        const cc::VectorDof &Gravity_Matrix(const cc::JointPosition &q);
        
        ////////////////////////////////
        /// Yr, Theta caculet
        ////////////////////////////////
        const Regressor_Theta &Theta_function();
        const Regressor_Yr &Yr_function(const cc::JointPosition &q, 
                                            const cc::JointVelocity &qp, 
                                            const cc::JointVelocity &qrp, 
                                            const cc::JointAcceleration &qrpp);

        ////////////////////////////////
        /// Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        ////////////////////////////////
        cc::HomogeneousTransformation Tef_0(const cc::JointPosition &q) const;
        cc::HomogeneousTransformation Ti_0(const cc::JointPosition &q, int i) const;
        cc::HomogeneousTransformation Tcmi_0(const cc::JointPosition &q, int i) const;


        ////////////////////////////////
        /// Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////
        cc::Jacobian Jef_0(const cc::JointPosition &q) const;
        cc::Jacobian Jef_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qp) const;

        cc::Jacobian Ji_0(const cc::JointPosition &q, int i) const;
        cc::Jacobian Jcmi_0(const cc::JointPosition &q, int i) const;

        cc::Jacobian Ji_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qp, int i) const;
        cc::Jacobian Jcmi_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qp, int i) const;


    private:

        ////////////////////////////////
        /// M, C, G Matrix
        /// Update
        ////////////////////////////////
        void caculet_M(cc::MatrixDof &M, const cc::JointPosition &q) const;
        void caculet_C(cc::MatrixDof &C,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_G(cc::VectorDof &G,
                        const cc::JointPosition &q) const;


        ////////////////////////////////
        /// Yr, Theta caculet
        ////////////////////////////////
        void caculet_Theta(Regressor_Theta &Theta) const;
        void caculet_Yr(Regressor_Yr &Y,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp,
                        const cc::JointVelocity &qrp,
                        const cc::JointAcceleration &qrpp) const;


        ////////////////////////////////
        /// Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        ////////////////////////////////
        void caculet_T1_0(cc::HomogeneousTransformation &T1_0, const cc::JointPosition &q) const;
        void caculet_T2_0(cc::HomogeneousTransformation &T2_0, const cc::JointPosition &q) const;
        void caculet_T3_0(cc::HomogeneousTransformation &T3_0, const cc::JointPosition &q) const;
        void caculet_T4_0(cc::HomogeneousTransformation &T4_0, const cc::JointPosition &q) const;
        void caculet_T5_0(cc::HomogeneousTransformation &T5_0, const cc::JointPosition &q) const;
        void caculet_T6_0(cc::HomogeneousTransformation &T6_0, const cc::JointPosition &q) const;

        void caculet_Tcm1_0(cc::HomogeneousTransformation &Tcm1_0, const cc::JointPosition &q) const;
        void caculet_Tcm2_0(cc::HomogeneousTransformation &Tcm2_0, const cc::JointPosition &q) const;
        void caculet_Tcm3_0(cc::HomogeneousTransformation &Tcm3_0, const cc::JointPosition &q) const;
        void caculet_Tcm4_0(cc::HomogeneousTransformation &Tcm4_0, const cc::JointPosition &q) const;
        void caculet_Tcm5_0(cc::HomogeneousTransformation &Tcm5_0, const cc::JointPosition &q) const;
        void caculet_Tcm6_0(cc::HomogeneousTransformation &Tcm6_0, const cc::JointPosition &q) const;


        ////////////////////////////////
        /// Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////
        void caculet_J1_0(cc::Jacobian &J1_0, const cc::JointPosition &q) const;
        void caculet_J2_0(cc::Jacobian &J2_0, const cc::JointPosition &q) const;
        void caculet_J3_0(cc::Jacobian &J3_0, const cc::JointPosition &q) const;
        void caculet_J4_0(cc::Jacobian &J4_0, const cc::JointPosition &q) const;
        void caculet_J5_0(cc::Jacobian &J5_0, const cc::JointPosition &q) const;
        void caculet_J6_0(cc::Jacobian &J6_0, const cc::JointPosition &q) const;

        void caculet_Jcm1_0(cc::Jacobian &Jcm1_0, const cc::JointPosition &q) const;
        void caculet_Jcm2_0(cc::Jacobian &Jcm2_0, const cc::JointPosition &q) const;
        void caculet_Jcm3_0(cc::Jacobian &Jcm3_0, const cc::JointPosition &q) const;
        void caculet_Jcm4_0(cc::Jacobian &Jcm4_0, const cc::JointPosition &q) const;
        void caculet_Jcm5_0(cc::Jacobian &Jcm5_0, const cc::JointPosition &q) const;
        void caculet_Jcm6_0(cc::Jacobian &Jcm6_0, const cc::JointPosition &q) const;

        ////////////////////////////////

        void caculet_J1_0_dot(cc::Jacobian &J1_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_J2_0_dot(cc::Jacobian &J2_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_J3_0_dot(cc::Jacobian &J3_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_J4_0_dot(cc::Jacobian &J4_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_J5_0_dot(cc::Jacobian &J5_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_J6_0_dot(cc::Jacobian &J6_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;

        void caculet_Jcm1_0_dot(cc::Jacobian &Jcm1_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_Jcm2_0_dot(cc::Jacobian &Jcm2_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_Jcm3_0_dot(cc::Jacobian &Jcm3_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_Jcm4_0_dot(cc::Jacobian &Jcm4_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_Jcm5_0_dot(cc::Jacobian &Jcm5_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;
        void caculet_Jcm6_0_dot(cc::Jacobian &Jcm6_0_dot,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const;

    };

} // namespace ur_model_namespace

#endif // UR_MODEL_H


