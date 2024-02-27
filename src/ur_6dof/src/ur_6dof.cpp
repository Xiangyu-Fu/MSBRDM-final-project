#include <ur_6dof/ur_6dof.h>
#include <control_core/common/parameter.h>
#include <math.h>
#include <string>
#include <Eigen/Dense>


namespace tum_ics_ur_model
{
    //////////////////////////////////////////////////////////////////////////////////////////
    /// Initial
    //////////////////////////////////////////////////////////////////////////////////////////

    URModel::URModel(const std::string &name): 
        Base(name), 
        M_(cc::MatrixDof::Zero()),
        C_(cc::MatrixDof::Zero()),
        G_(cc::VectorDof::Zero()),
        Theta_(61, 1),
        Yr_(6, 61)  
    {
    Theta_.setZero();
    Yr_.setZero();
    }

    URModel::~URModel()
    {
    }

    bool URModel::init(ros::NodeHandle &nh)
    {
        ////////////////////////////////
        /// Initial Constant
        ////////////////////////////////
        std::string ns = Base::get_name() + '/';
        std::cout << "name: " << Base::get_name() << std::endl;

        // for (int i = 1; i <= 6; ++i) {
        //     cc::load(ns + "L" + std::to_string(i));
        // }
        cc::load(ns+"L1", L1);
        cc::load(ns+"L2", L2);
        cc::load(ns+"L3", L3);
        cc::load(ns+"L4", L4);
        cc::load(ns+"L5", L5);
        cc::load(ns+"L6", L6);
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
        // caculet_Theta(Theta_);  // constant
        Theta_(0,0) = 0.538671;
        Theta_(1,0) = 0.0495189;
        Theta_(2,0) = -0.137352;
        Theta_(3,0) = 0.00369289;
        Theta_(4,0) = -0.00449387;
        Theta_(5,0) = 0.0763342;
        Theta_(6,0) = -8.26096e-05;
        Theta_(7,0) = 0.0389543;
        Theta_(8,0) = 0.0306582;
        Theta_(9,0) = 0.000803445;
        Theta_(10,0) = -0.228603;
        Theta_(11,0) = -0.06977;
        Theta_(12,0) = -0.000473192;
        Theta_(13,0) = -0.0643962;
        Theta_(14,0) = -0.00132254;
        Theta_(15,0) = -0.00446129;
        Theta_(16,0) = -0.0919834;
        Theta_(17,0) = 0.108403;
        Theta_(18,0) = 0.0019377;
        Theta_(19,0) = -0.00698126;
        Theta_(20,0) = 0.106904;
        Theta_(21,0) = 0.0287031;
        Theta_(22,0) = 0.0155653;
        Theta_(23,0) = 0.156894;
        Theta_(24,0) = -0.115319;
        Theta_(25,0) = 0.0131442;
        Theta_(26,0) = 0.00621417;
        Theta_(27,0) = -0.0441391;
        Theta_(28,0) = -0.00730768;
        Theta_(29,0) = -0.0404186;
        Theta_(30,0) = 0.158079;
        Theta_(31,0) = -0.41046;
        Theta_(32,0) = 0.102432;
        Theta_(33,0) = -0.0339328;
        Theta_(34,0) = 0.0807778;
        Theta_(35,0) = -3.11026;
        Theta_(36,0) = -0.642361;
        Theta_(37,0) = 0.749634;
        Theta_(38,0) = -0.216106;
        Theta_(39,0) = -0.795434;
        Theta_(40,0) = -0.691179;
        Theta_(41,0) = -0.859666;
        Theta_(42,0) = -7.05368;
        Theta_(43,0) = 8.15735;  
        Theta_(44,0) = -15.8328;
        Theta_(45,0) = -4.37181;
        Theta_(46,0) = 0.0420231;
        Theta_(47,0) = -0.0384988;
        Theta_(48,0) = 0.404748;
        Theta_(49,0) = -0.921156;
        Theta_(50,0) = -0.353958;
        Theta_(51,0) = -0.0412944;
        Theta_(52,0) = -0.331908;
        Theta_(53,0) = 0.055763;
        Theta_(54,0) = 8.04245;
        Theta_(55,0) = -7.09376;
        Theta_(56,0) = 15.8786;
        Theta_(57,0) = 8.21114;
        Theta_(58,0) = 1.01367;
        Theta_(59,0) = -0.520889;
        Theta_(60,0) = 1.28733;

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////
    /// M, C, G
    //////////////////////////////////////////////////////////////////////////////////////////

    const cc::MatrixDof &URModel::Inertia_Matrix(const cc::JointPosition &q)
    {
        // Update M & Return
        caculet_M(M_, q);
        return M_;
    }

    const cc::MatrixDof &URModel::Coriolis_Matrix(const cc::JointPosition &q, const cc::JointVelocity &qp)
    {
        // Update M & Return
        caculet_C(C_, q, qp);
        return C_;
    }

    const cc::VectorDof &URModel::Gravity_Matrix(const cc::JointPosition &q)
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
        // caculet_Theta(Theta_);
        return Theta_;
    }

    const URModel::Regressor_Yr &URModel::Yr_function(const cc::JointPosition &q, 
                                            const cc::JointVelocity &qp, 
                                            const cc::JointVelocity &qrp, 
                                            const cc::JointAcceleration &qrpp)
    {
        caculet_Yr(Yr_, q, qp, qrp, qrpp);
        return Yr_;
    }

    ////////////////////////////////
    /// Transformation Matrix: Ti_0 & Tcmi_0, i=1...6
    ////////////////////////////////
    cc::HomogeneousTransformation URModel::Tef_0(const cc::JointPosition &q) const
    {
        cc::HomogeneousTransformation T;
        caculet_T6_0(T, q);
        return T;
    }
    
    cc::HomogeneousTransformation URModel::Ti_0(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::HomogeneousTransformation T;
        
        std::array<std::function<void(cc::HomogeneousTransformation&, const cc::JointPosition&)>, 6> matrixTFunctions = {
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T1_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T2_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T3_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T4_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T5_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_T6_0(T, q); }
        };

        if (i >= 0 && i < matrixTFunctions.size()) {
            matrixTFunctions[i](T, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ti_0");
        }

        return T;
    }

    cc::HomogeneousTransformation URModel::Tcmi_0(const cc::JointPosition &q, 
                                    int i) const
    {
        cc::HomogeneousTransformation T;
        
        std::array<std::function<void(cc::HomogeneousTransformation&, const cc::JointPosition&)>, 6> matrixTFunctions = {
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm1_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm2_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm3_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm4_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm5_0(T, q); },
            [this](cc::HomogeneousTransformation &T, const cc::JointPosition &q) { caculet_Tcm6_0(T, q); }
        };

        if (i >= 0 && i < matrixTFunctions.size()) {
            matrixTFunctions[i](T, q);
        } else {
            throw std::out_of_range("Index i is out of range for Tcmi_0");
        }

        return T;
    }

    ////////////////////////////////
    /// Jacobian Matrix: Ji_0 & Jcmi_0, i=1...6
    //                  Jdot_i_0 & Jdot_cmi_0, i=1...6
    ////////////////////////////////
    cc::Jacobian URModel::Jef_0(const cc::JointPosition &q) const
    {
        cc::Jacobian J = cc::Jacobian::Zero();
        caculet_J6_0(J, q);
        return J;
    }

    cc::Jacobian URModel::Jef_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qP) const
    {
        cc::Jacobian Jp = cc::Jacobian::Zero();
        caculet_J6_0_dot(Jp, q, qP);
        return Jp;
    }


    cc::Jacobian URModel::Ji_0(const cc::JointPosition &q, int i) const
    {
        cc::Jacobian J = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> caculetJFunctions = {
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J1_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J2_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J3_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J4_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J5_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_J6_0(J, q); }
        };

        if (i >= 0 && i < caculetJFunctions.size()) {
            caculetJFunctions[i](J, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0");
        }

        return J;
    }

    cc::Jacobian URModel::Jcmi_0(const cc::JointPosition &q, 
                                        int i) const
    {
        cc::Jacobian J = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&)>, 6> caculetJFunctions = {
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm1_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm2_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm3_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm4_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm5_0(J, q); },
            [this](cc::Jacobian &J, const cc::JointPosition &q) { caculet_Jcm6_0(J, q); }
        };

        if (i >= 0 && i < caculetJFunctions.size()) {
            caculetJFunctions[i](J, q);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0");
        }

        return J;
    }

    ////////////////////////////////
    cc::Jacobian URModel::Ji_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qp, int i) const 
    {
        cc::Jacobian J_dot = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>, 6> caculetJdotFunctions = {
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J1_0_dot(J_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J2_0_dot(J_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J3_0_dot(J_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J4_0_dot(J_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J5_0_dot(J_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &J_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_J6_0_dot(J_dot, q, qp); }},
        };

        if (i >= 0 && i < caculetJdotFunctions.size()) {
            caculetJdotFunctions[i](J_dot, q, qp);
        } else {
            throw std::out_of_range("Index i is out of range for Ji_0_dot");
        }

        return J_dot;
    }


    cc::Jacobian URModel::Jcmi_0_dot(const cc::JointPosition &q, const cc::JointVelocity &qp, int i) const 
    {
        cc::Jacobian Jcm_dot = cc::Jacobian::Zero();
        
        std::array<std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>, 6> caculetJcmdotFunctions = {
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm1_0_dot(Jcm_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm2_0_dot(Jcm_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm3_0_dot(Jcm_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm4_0_dot(Jcm_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm5_0_dot(Jcm_dot, q, qp); }},
            std::function<void(cc::Jacobian&, const cc::JointPosition&, const cc::JointVelocity&)>{[this](cc::Jacobian &Jcm_dot, const cc::JointPosition &q, const cc::JointVelocity &qp) { caculet_Jcm6_0_dot(Jcm_dot, q, qp); }}
        };

        if (i >= 0 && i < caculetJcmdotFunctions.size()) {
            caculetJcmdotFunctions[i](Jcm_dot, q, qp);
        } else {
            throw std::out_of_range("Index i is out of range for Jcmi_0_dot");
        }

        return Jcm_dot;
    }



} // namespace tum_ics_ur_model

// #endif // UR_6DOF_CPP
