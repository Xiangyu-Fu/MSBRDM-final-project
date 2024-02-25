#ifndef MATRIX_YR_H_
#define MATRIX_YR_H_

#include <cmath>
#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_Y(Regressor &Y,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qp,
                        const cc::JointVelocity &qrp,
                        const cc::JointAcceleration &qrpp) const
{
}

#endif
