#include <ur_model/ur_model.h>
#include <ur_model/model_base.h>
#include <math.h>

namespace ur_model_namespace
{
    //////////////////////////////////////////////////////////////////////////////////////////
    /// Initial
    //////////////////////////////////////////////////////////////////////////////////////////

    URModel::URModel(const std::string &name): 
        Base(name), 
        M_(Matrix6d::Zero()),
        C_(Matrix6d::Zero()),
        G_(Vector6D::Zero()),
        Theta_.setZero(61, 1),
        Yr_.setZero(6, 61)
    {
    }

    URModel::~URModel()
    {
    }

    bool URModel::init(ros::NodeHandle &nh)
    {
        ////////////////////////////////
        /// Initial Constant
        ////////////////////////////////
        std::string ns = Base::name() + '/';
        std::cout << "name: " << Base::name() << std::endl;

        for (int i = 1; i <= 6; ++i) {
            cc::load(ns + "L" + std::to_string(i));
        }
        L7 = L8 = L9 = L10 = L11 = L12 = 0.0;
        m1 = m2 = m3 = m4 = m5 = m6 = 0.0;
        I111 = I112 = I113 = I122 = I123 = I133 = 0.0;
        I211 = I212 = I213 = I222 = I223 = I233 = 0.0;
        I311 = I312 = I313 = I322 = I323 = I333 = 0.0;
        I411 = I412 = I413 = I422 = I423 = I433 = 0.0;
        I511 = I512 = I513 = I522 = I523 = I533 = 0.0;
        I611 = I612 = I613 = I622 = I623 = I633 = 0.0;
        cc::load(ns+"gx", gx);
        cc::load(ns+"gy", gy);
        cc::load(ns+"gz", gz);

        ////////////////////////////////
        /// Initial Parameter
        ////////////////////////////////
        caculet_Theta(Theta_);  // constant

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////
    /// M, C, G
    //////////////////////////////////////////////////////////////////////////////////////////

    const cc::MatrixDof &URModel::Mass_Matrix(const cc::JointPosition &q)
    {
        // Update M & Return
        caculet_M(M_, q);
        return M_;
    }

    const cc::MatrixDof &URModel::Coriolis_Matrix(const cc::JointPosition &q)
    {
        // Update M & Return
        caculet_C(C_, q);
        return C_;
    }

    const cc::MatrixDof &URModel::Gravity_Matrix(const cc::JointPosition &q)
    {
        // Update M & Return
        caculet_G(G_, q);
        return G_;
    }

    ////////////////////////////////
    /// Yr, Theta caculet
    ////////////////////////////////
    const URModel::Regressor_Theta &URModel::Theta_function()
    {
        caculet_Theta(Theta_);
        return Theta_;
    }

    const URModel::Regressor_Yr &URModel::Yr_function(const cc::JointPosition &q, 
                                            const cc::JointVelocity &qp, 
                                            const cc::JointVelocity &qrp, 
                                            const cc::JointAcceleration &qrpp);
    {
        caculet_Yr(Yr_, q, qp, qrp, qrpp);
        return Yr_;
    }

    ////////////////////////////////
    /// Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
    ////////////////////////////////
    cc::HomogeneousTransformation Ti_0(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::HomogeneousTransformation T;
        
        std::array<std::function<void(cc::HomogeneousTransformation&, const cc::JointPosition&)>, 6> matrixTFunctions = {
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T1_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T2_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T3_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T4_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T5_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T6_0(T, q); },
        };

        if (i >= 0 && i < matrixTFunctions.size()) {
            matrixTcmFunctions[i](T, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ti_0");
        }

        return T;
    }

    cc::HomogeneousTransformation Tcmi_0(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::HomogeneousTransformation T;
        
        std::array<std::function<void(cc::HomogeneousTransformation&, const cc::JointPosition&)>, 6> matrixTFunctions = {
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm1_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm2_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm3_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm4_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm5_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm6_0(T, q); },
        };

        if (i >= 0 && i < matrixTFunctions.size()) {
            matrixTcmFunctions[i](T, q);
        } else {
            throw std::out_of_range("Index i is out of range for Tcmi_0");
        }

        return T;
    }

    ////////////////////////////////
    /// Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
    //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
    ////////////////////////////////
    cc::HomogeneousTransformation Ji_0(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::Jacobian J = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> calculateJFunctions = {
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J1_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J2_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J3_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J4_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J5_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_J6_0(J, q); },
        };

        if (i >= 0 && i < calculateJcmFunctions.size()) {
            calculateJFunctions[i](J, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0");
        }

        return J;
    }

    cc::HomogeneousTransformation Jcmi_0(const cc::JointPosition &q, 
                                        int i) const
    {
        cc::Jacobian J = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> calculateJFunctions = {
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm1_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm2_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm3_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm4_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm5_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { calculat_Jcm6_0(J, q); },
        };

        if (i >= 0 && i < calculateJFunctions.size()) {
            calculateJcmFunctions[i](J, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0");
        }

        return J;
    }

    ////////////////////////////////
    cc::HomogeneousTransformation Ji_0_dot(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::Jacobian J_dot = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> calculateJdotFunctions = {
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J1_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J2_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J3_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J4_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J5_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_J6_0_dot(J_dot, q); },
        };

        if (i >= 0 && i < calculateJdotFunctions.size()) {
            calculateJcmFunctions[i](J_dot, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0_dot");
        }

        return J_dot;
    }

    cc::HomogeneousTransformation Jcmi_0_dot(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::Jacobian J_dot = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> calculateJdotFunctions = {
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm1_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm2_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm3_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm4_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm5_0_dot(J_dot, q); },
            [this](cc::Jacobian &J_dot, const cc::JointPosition &q) { calculat_Jcm6_0_dot(J_dot, q); },
        };

        if (i >= 0 && i < calculateJdotFunctions.size()) {
            calculateJcmFunctions[i](J_dot, q);
        } else {
            throw std::out_of_range("Index i is out of range for Jcmi_0_dot");
        }

        return J_dot;
    }

    







} // namespace ur_model_namespace

#endif // UR_MODEL_CPP
