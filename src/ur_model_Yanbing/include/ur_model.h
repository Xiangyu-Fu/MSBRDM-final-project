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
        typedef cc::MatrixX Regressor_theta;
        typedef cc::VectorX Regressor_Yr;
        


    private:
        ////////////////////////////////
        // Variable
        // M, C, G
        // Theta, Yr
        // Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        // Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////
        cc::Scalar L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11;
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

        Regressor_theta theta_;
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
        // Member function
        // Inverse Kinematics
        // M, C, G
        // Theta, Yr
        // Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        // Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////

        ///
        /// \brief M, C, G Matrix caculet
        /// \param M_, C_, G_
        /// \param joint state q
        /// \return Update & Return
        ///
        const cc::MatrixDof &URModel::Mass_Matrix(const cc::JointPosition &q);
        const cc::MatrixDof &URModel::Coriolis_Matrix(const cc::JointPosition &q);
        const cc::MatrixDof &URModel::Gravity_Matrix(const cc::JointPosition &q);




    private:
        ////////////////////////////////
        // Member function
        // M, C, G, Tau
        // Theta, Yr
        // Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
        // Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
        //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
        ////////////////////////////////





    };

} // namespace ur_model_namespace

#endif // UR_MODEL_H


