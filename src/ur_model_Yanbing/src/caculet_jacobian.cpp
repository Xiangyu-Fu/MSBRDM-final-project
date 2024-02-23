#include <ur_model/ur_model.h>

using namespace ur_model_namespace;

void URModel::caculet_J1_0(cc::Jacobian &J1_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J1_0(i, j) = 0;
        }
    }

    J1_0(5, 0) = 1;
}

void URModel::caculet_J2_0(cc::Jacobian &J2_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J2_0(i, j) = 0;
        }
    }

    J2_0(0, 0) = L3*cos(q(1))*sin(q(0));
    J2_0(0, 1) = L3*cos(q(0))*sin(q(1));
    J2_0(1, 0) = -L3*cos(q(0))*cos(q(1));
    J2_0(1, 1) = L3*sin(q(0))*sin(q(1));
    J2_0(2, 1) = -L3*cos(q(1));
    J2_0(3, 1) = sin(q(0));
    J2_0(4, 1) = -cos(q(0));
    J2_0(5, 0) = 1;
}

void URModel::caculet_J3_0(cc::Jacobian &J3_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J3_0(i, j) = 0;
        }
    }

    J3_0(0, 0) = L3*cos(q(1))*sin(q(0)) + cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) - sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);
    J3_0(0, 1) = cos(q(0))*(sin(q(1) + q(2))*(L5 - L11) + L3*sin(q(1)));
    J3_0(0, 2) = sin(q(1) + q(2))*cos(q(0))*(L5 - L11);
    
    J3_0(1, 0) = cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) - L3*cos(q(0))*cos(q(1));
    J3_0(1, 1) = sin(q(0))*(sin(q(1) + q(2))*(L5 - L11) + L3*sin(q(1)));
    J3_0(1, 2) = sin(q(1) + q(2))*sin(q(0))*(L5 - L11);
    
    J3_0(2, 1) = L11*cos(q(1) + q(2)) - L5*cos(q(1) + q(2)) - L3*cos(q(1));
    J3_0(2, 2) = -cos(q(1) + q(2))*(L5 - L11)
    
    J3_0(3, 1) = sin(q(0));
    J3_0(3, 2) = sin(q(0));
    
    J3_0(4, 1) = -cos(q(0));
    J3_0(4, 2) = -cos(q(0));
    J3_0(5, 0) = 1;
}

void URModel::caculet_J4_0(cc::Jacobian &J4_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J4_0(i, j) = 0;
        }
    }

    J4_0(0, 0) = L2*cos(q1) + L3*cos(q2)*sin(q1) + cos(q2)*cos(q3)*sin(q1)*(L5 - L11) - sin(q1)*sin(q2)*sin(q3)*(L5 - L11);
    J4_0(0, 1) = cos(q1)*(sin(q2 + q3)*(L5 - L11) + L3*sin(q2));
    J4_0(0, 2) = sin(q2 + q3)*cos(q1)*(L5 - L11);

    J4_0(1, 0) = L2*cos(q1) + L3*cos(q2)*sin(q1) + cos(q2)*cos(q3)*sin(q1)*(L5 - L11) - sin(q1)*sin(q2)*sin(q3)*(L5 - L11);
    J4_0(1, 1) = sin(q1)*(sin(q2 + q3)*(L5 - L11) + L3*sin(q2));
    J4_0(1, 2) = sin(q2 + q3)*sin(q1)*(L5 - L11);

    J4_0(2, 1) = L11*cos(q2 + q3) - L5*cos(q2 + q3) - L3*cos(q2);
    J4_0(2, 2) = -cos(q2 + q3)*(L5 - L11);

    J4_0(3, 1) = sin(q1);
    J4_0(3, 2) = sin(q1);
    J4_0(3, 3) = sin(q1);

    J4_0(4, 1) = -cos(q1);
    J4_0(4, 2) = -cos(q1);
    J4_0(4, 3) = -cos(q1);

    J4_0(5, 0) = 1;
}

void URModel::caculet_J5_0(cc::Jacobian &J5_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J5_0(i, j) = 0;
        }
    }

    J5_0(0, 0) = L2*cos(q(0)) + L3*cos(q(1))*sin(q(0)) - L11*sin(q(1) + q(2) + q(3))*sin(q(0)) + cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) - sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);
    J5_0(0, 1) = cos(q(0))*(L5*sin(q(1) + q(2)) - L11*sin(q(1) + q(2)) + L3*sin(q(1)) + L11*cos(q(1) + q(2) + q(3)));
    J5_0(0, 2) = cos(q(0))*(sin(q(1) + q(2))*(L5 - L11) + L11*cos(q(1) + q(2) + q(3)));
    J5_0(0, 3) = L11*cos(q(1) + q(2) + q(3))*cos(q(0));

    J5_0(1, 0) = L2*sin(q(0)) - L3*cos(q(0))*cos(q(1)) + L11*sin(q(1) + q(2) + q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);
    J5_0(1, 1) = sin(q(0))*(L5*sin(q(1) + q(2)) - L11*sin(q(1) + q(2)) + L3*sin(q(1)) + L11*cos(q(1) + q(2) + q(3)));
    J5_0(1, 2) = sin(q(0))*(sin(q(1) + q(2))*(L5 - L11) + L11*cos(q(1) + q(2) + q(3)));
    J5_0(1, 3) = L11*cos(q(1) + q(2) + q(3))*sin(q(0));

    J5_0(2, 1) = L11*cos(q(1) + q(2)) - L5*cos(q(1) + q(2)) - L3*cos(q(1)) + L11*sin(q(1) + q(2) + q(3));
    J5_0(2, 2) = L11*cos(q(1) + q(2)) - L5*cos(q(1) + q(2)) + L11*sin(q(1) + q(2) + q(3));
    J5_0(2, 3) = L11*sin(q(1) + q(2) + q(3));

    J5_0(3, 1) = sin(q(0));
    J5_0(3, 2) = sin(q(0));
    J5_0(3, 3) = sin(q(0));
    J5_0(3, 4) = sin(q(1) + q(2) + q(3))*cos(q(0));

    J5_0(4, 1) = -cos(q(0));
    J5_0(4, 2) = -cos(q(0));
    J5_0(4, 3) = -cos(q(0));
    J5_0(4, 4) = sin(q(1) + q(2) + q(3))*sin(q(0));

    J5_0(5, 0) = 1;
    J5_0(5, 4) = -cos(q(1) + q(2) + q(3));
}

void URModel::caculet_J6_0(cc::Jacobian &J6_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            J6_0(i, j) = 0;
        }
    }

    J6_0(0, 0) = L4*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L2*cos(q1) + L3*cos(q2)*sin(q1) - L11*sin(q2 + q3 + q4)*sin(q1) + cos(q2)*cos(q3)*sin(q1)*(L5 - L11) - sin(q1)*sin(q2)*sin(q3)*(L5 - L11);
    J6_0(0, 1) = cos(q1)*(L5*sin(q2 + q3) - L11*sin(q2 + q3) + L3*sin(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(0, 2) = cos(q1)*(sin(q2 + q3)*(L5 - L11) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(0, 3) = cos(q1)*(L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(0, 4) = L4*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - L4*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - L4*sin(q1)*sin(q5) + L4*cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + L4*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3);

    J6_0(1, 0) = L4*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + L2*sin(q1) - L3*cos(q1)*cos(q2) + L11*sin(q2 + q3 + q4)*cos(q1) - cos(q1)*cos(q2)*cos(q3)*(L5 - L11) + cos(q1)*sin(q2)*sin(q3)*(L5 - L11);
    J6_0(1, 1) = sin(q1)*(L5*sin(q2 + q3) - L11*sin(q2 + q3) + L3*sin(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(1, 2) = sin(q1)*(sin(q2 + q3)*(L5 - L11) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(1, 3) = sin(q1)*(L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
    J6_0(1, 4) = L4*cos(q1)*sin(q5) + L4*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) + L4*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) + L4*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) - L4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1);

    J6_0(2, 1) = L11*cos(q2 + q3) - L5*cos(q2 + q3) - (L4*sin(q2 + q3 + q4 + q5))/2 - L3*cos(q2) + (L4*sin(q2 + q3 + q4 - q5))/2 + L11*sin(q2 + q3 + q4);
    J6_0(2, 2) = L11*cos(q2 + q3) - L5*cos(q2 + q3) - (L4*sin(q2 + q3 + q4 + q5))/2 + (L4*sin(q2 + q3 + q4 - q5))/2 + L11*sin(q2 + q3 + q4);
    J6_0(2, 3) = (L4*sin(q2 + q3 + q4 - q5))/2 - (L4*sin(q2 + q3 + q4 + q5))/2 + L11*sin(q2 + q3 + q4);
    J6_0(2, 4) = -L4*(sin(q2 + q3 + q4 + q5)/2 + sin(q2 + q3 + q4 - q5)/2)

    J6_0(3, 1) = sin(q1);
    J6_0(3, 2) = sin(q1);
    J6_0(3, 3) = sin(q1);
    J6_0(3, 4) = sin(q2 + q3 + q4)*cos(q1);
    J6_0(3, 5) = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5);

    J6_0(4, 1) = -cos(q1);
    J6_0(4, 2) = -cos(q1);
    J6_0(4, 3) = -cos(q1);
    J6_0(4, 4) = sin(q2 + q3 + q4)*sin(q1);
    J6_0(4, 5) = - cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5);

    J6_0(5, 0) = 1;
    J6_0(5, 4) = -cos(q2 + q3 + q4);
    J6_0(5, 5) = -sin(q2 + q3 + q4)*sin(q5);
}


//////////////////////////////////////////////////////////////////////////////////////////////
// CM
//////////////////////////////////////////////////////////////////////////////////////////////

void URModel::caculet_Jcm1_0(cc::Jacobian &Jcm1_0,
                        const cc::JointPosition &q) const
{
    // Initial
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Jcm1_0(i, j) = 0;
        }
    }

    Jcm1_0(0, 0) = 
}