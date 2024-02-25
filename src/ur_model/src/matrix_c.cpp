#ifndef MATRIX_C_H_
#define MATRIX_C_H_

#include <cmath>
#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_C(cc::MatrixDof &C,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp) const
{
}

#endif