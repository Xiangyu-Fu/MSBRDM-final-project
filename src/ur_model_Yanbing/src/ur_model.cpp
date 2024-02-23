#include <ur_model/ur_model.h>
#include <ur_model/model_base.h>
#include <math.h>

namespace ur_model_namespace
{
    URModel::URModel(const std::string &name): 
        Base(name), 
        M_(Matrix6d::Zero()),
        C_(Matrix6d::Zero()),
        G_(Vector6D::Zero()),
        // theta_.setZero(45, 1),
        // Yr_.setZero(6, 45)
    {
    }

    URModel::~URModel()
    {
    }

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

} // namespace ur_model_namespace

#endif // UR_MODEL_CPP
