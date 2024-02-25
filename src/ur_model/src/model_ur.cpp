#ifndef MATRIX_YR_H_
#define MATRIX_YR_H_

#include <ur_model/ur_model.h>
#include <control_core/common/parameter.h>
#include <string>

namespace ur
{

  URModel::URModel(const std::string &name) : Base(name),
                                              M_(cc::MatrixDof::Zero()),
                                              C_(cc::MatrixDof::Zero()),
                                              g_(cc::VectorDof::Zero())
  {
    // initalize correct size
    theta_.setZero(45, 1);
    Yr_.setZero(6, 45);
  }

  URModel::~URModel()
  {
  }

  const cc::MatrixDof &URModel::inertiaMatrix(const cc::JointPosition &q)
  {
    matrix_M(M_, q);
    return M_;
  }

  const cc::MatrixDof &URModel::centripetalMatrix(const cc::JointPosition &q, const cc::JointVelocity &qP)
  {
    matrix_C(C_, q, qP);
    return C_;
  }

  const cc::VectorDof &URModel::gravityVector(const cc::JointPosition &q)
  {
    matrix_G(g_, q);
    return g_;
  }

  const URModel::Regressor &URModel::regressor(const cc::JointPosition &q, const cc::JointVelocity &qP, const cc::JointVelocity &qrP, const cc::JointAcceleration &qrPP)
  {
    matrix_Y(Yr_, q, qP, qrP, qrPP);
    return Yr_;
  }

  const URModel::Parameters &URModel::parameterInitalGuess()
  {
    return theta_;
  }

  cc::HomogeneousTransformation URModel::T_ef_0(const cc::JointPosition &q) const
  {
    cc::HomogeneousTransformation T;
    matrix_T6_0(T, q);
    return T;
  }

  cc::HomogeneousTransformation URModel::T_j_0(const cc::JointPosition &q, int j) const
  {
    cc::HomogeneousTransformation T;
    switch (j)
    {
    case 0:
      matrix_T1_0(T, q);
      break;
    case 1:
      matrix_T2_0(T, q);
      break;
    case 2:
      matrix_T3_0(T, q);
      break;
    case 3:
      matrix_T4_0(T, q);
      break;
    case 4:
      matrix_T5_0(T, q);
      break;
    case 5:
      matrix_T6_0(T, q);
      break;
    }
    return T;
  }

  cc::HomogeneousTransformation URModel::T_cm_j_0(const cc::JointPosition &q, int j) const
  {
    cc::HomogeneousTransformation T;
    switch (j)
    {
    case 0:
      matrix_Tcm1_0(T, q);
      break;
    case 1:
      matrix_Tcm2_0(T, q);
      break;
    case 2:
      matrix_Tcm3_0(T, q);
      break;
    case 3:
      matrix_Tcm4_0(T, q);
      break;
    case 4:
      matrix_Tcm5_0(T, q);
      break;
    case 5:
      matrix_Tcm6_0(T, q);
      break;
    }
    return T;
  }

  cc::Jacobian URModel::J_ef_0(const cc::JointPosition &q) const
  {
    cc::Jacobian J = cc::Jacobian::Zero();
    matrix_J6_0(J, q);
    return J;
  }


  cc::Jacobian URModel::J_j_0(const cc::JointPosition &q, int j) const
  {
    cc::Jacobian J = cc::Jacobian::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0(J, q);
      break;
    case 1:
      matrix_J2_0(J, q);
      break;
    case 2:
      matrix_J3_0(J, q);
      break;
    case 3:
      matrix_J4_0(J, q);
      break;
    case 4:
      matrix_J5_0(J, q);
      break;
    case 5:
      matrix_J6_0(J, q);
      break;
    }
    return J;
  }


  cc::Jacobian URModel::Jcm_j_0(const cc::JointPosition &q, int j) const
  {
    cc::Jacobian J = cc::Jacobian::Zero();
    switch (j)
    {
    case 0:
      matrix_Jcm1_0(J, q);
      break;
    case 1:
      matrix_Jcm2_0(J, q);
      break;
    case 2:
      matrix_Jcm3_0(J, q);
      break;
    case 3:
      matrix_Jcm4_0(J, q);
      break;
    case 4:
      matrix_Jcm5_0(J, q);
      break;
    case 5:
      matrix_Jcm6_0(J, q);
      break;
    }
    return J;
  }

  cc::Jacobian URModel::Jp_ef_0(const cc::JointPosition &q, const cc::JointVelocity &qP) const
  {
    cc::Jacobian Jp = cc::Jacobian::Zero();
    matrix_J6_0p(Jp, q, qP);
    return Jp;
  }

  cc::Jacobian URModel::Jp_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const
  {
    cc::Jacobian Jp = cc::Jacobian::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0p(Jp, q, qP);
      break;
    case 1:
      matrix_J2_0p(Jp, q, qP);
      break;
    case 2:
      matrix_J3_0p(Jp, q, qP);
      break;
    case 3:
      matrix_J4_0p(Jp, q, qP);
      break;
    case 4:
      matrix_J5_0p(Jp, q, qP);
      break;
    case 5:
      matrix_J6_0p(Jp, q, qP);
      break;
    }
    return Jp;
  }


  cc::Jacobian URModel::Jpcm_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const
  {
    cc::Jacobian Jp = cc::Jacobian::Zero();
    switch (j)
    {
    case 0:
      matrix_Jcm1_0p(Jp, q, qP);
      break;
    case 1:
      matrix_Jcm2_0p(Jp, q, qP);
      break;
    case 2:
      matrix_Jcm3_0p(Jp, q, qP);
      break;
    case 3:
      matrix_Jcm4_0p(Jp, q, qP);
      break;
    case 4:
      matrix_Jcm5_0p(Jp, q, qP);
      break;
    case 5:
      matrix_Jcm6_0p(Jp, q, qP);
      break;
    }
    return Jp;
  }


  cc::Scalar URModel::lowerJointLimits_j(int j) const
  {
    switch (j)
    {
    case 0:
      return lo_jl1;
    case 1:
      return lo_jl2;
    case 2:
      return lo_jl3;
    case 3:
      return lo_jl4;
    case 4:
      return lo_jl5;
    case 5:
      return lo_jl6;
    }
    return -std::numeric_limits<cc::Scalar>::infinity();
  }

  cc::Scalar URModel::upperJointLimits_j(int j) const
  {
    switch (j)
    {
    case 0:
      return hi_jl1;
    case 1:
      return hi_jl2;
    case 2:
      return hi_jl3;
    case 3:
      return hi_jl4;
    case 4:
      return hi_jl5;
    case 5:
      return hi_jl6;
    }
    return std::numeric_limits<cc::Scalar>::infinity();
  }


  void URModel::inverseKinematics(IKSolutions &Qs, const cc::HomogeneousTransformation &Tef, cc::Scalar q6_d) const
  {
    matrix_IK(Qs, Tef, q6_d);
  }


  bool URModel::init(ros::NodeHandle &nh)
  {
    std::string ns = Base::name() + '/';

    std::cout << "name: " << Base::name() << std::endl;

    cc::load(ns+"L1", L1);
    cc::load(ns+"L2", L2);
    cc::load(ns+"L3", L3);
    cc::load(ns+"L4", L4);
    cc::load(ns+"L5", L5);
    cc::load(ns+"L6", L6);
    L7 = 0.0;
    L8 = 0.0;
    L9 = 0.0;
    L10 = 0.0;
    L11 = 0.0;
    L12 = 0.0;

    m1 = 0.0;
    m2 = 0.0;
    m3 = 0.0;
    m4 = 0.0;
    m5 = 0.0;
    m6 = 0.0;

    I111 = 0.0;
    I112 = 0.0;
    I113 = 0.0;
    I122 = 0.0;
    I123 = 0.0;
    I133 = 0.0;

    I211 = 0.0;
    I212 = 0.0;
    I213 = 0.0;
    I222 = 0.0;
    I223 = 0.0;
    I233 = 0.0;

    I311 = 0.0;
    I312 = 0.0;
    I313 = 0.0;
    I322 = 0.0;
    I323 = 0.0;
    I333 = 0.0;

    I411 = 0.0;
    I412 = 0.0;
    I413 = 0.0;
    I422 = 0.0;
    I423 = 0.0;
    I433 = 0.0;

    I511 = 0.0;
    I512 = 0.0;
    I513 = 0.0;
    I522 = 0.0;
    I523 = 0.0;
    I533 = 0.0;

    I611 = 0.0;
    I612 = 0.0;
    I613 = 0.0;
    I622 = 0.0;
    I623 = 0.0;
    I633 = 0.0;

    cc::load(ns+"gx", gx);
    cc::load(ns+"gy", gy);
    cc::load(ns+"gz", gz);
    // g_b_ << gx, gy, gz;

    // std::cout << "g_b_: " << g_b_ << std::endl;

    // cc::load(ns+"robot_0_frame", robot_0_frame_);
    // cc::load(ns+"base_frame", base_frame_);
    // cc::load(ns+"tool_frame", tool_frame_);

    // cc::CartesianPosition X_0_b;
    // cc::load(ns+"X_0_B", X_0_b);
    // T_0_b_ = X_0_b;
    // T_b_0_ = T_0_b_.inverse();

    // cc::CartesianPosition X_tool_ef;
    // cc::load(ns+"X_Tool_Ef", X_tool_ef);
    // T_tool_ef_ = X_tool_ef;

    // // gravity wrt base
    // g_0_ = T_0_b_.orientation().inverse() * g_b_;

    // set inital guess theta
    matrix_th(theta_);

    // setup frames
    // tf_stamped_transform_.resize(CC_ROBOT_DOF + 2);
    // for (size_t i = 0; i < CC_ROBOT_DOF; ++i)
    // {
    //   tf_stamped_transform_[i].frame_id_ = base_frame_;
    //   tf_stamped_transform_[i].child_frame_id_ = Base::name() + "_dh_" + std::to_string(i);
    // }
    // // tf_stamped_transform_[CC_ROBOT_DOF].frame_id_ = base_frame_;
    // // tf_stamped_transform_[CC_ROBOT_DOF].child_frame_id_ = Base::name() + "_dh_tool";
    // tf_stamped_transform_[CC_ROBOT_DOF+1].frame_id_ = base_frame_;
    // tf_stamped_transform_[CC_ROBOT_DOF+1].child_frame_id_ = Base::name() + "_0";

    return true;
  }

} // namespace ur

#endif