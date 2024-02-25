#include <cmath>
#include <ur_model/ur_model.h>

using namespace ur_model_namespace;

void URModel::caculet_J1_0(cc::Jacobian &J1_0,
                        const cc::JointPosition &q) const
{
    J1_0(5,0) = 1;
}

void URModel::caculet_J2_0(cc::Jacobian &J2_0,
                        const cc::JointPosition &q) const
{
    J2_0(0,0) = -L2*cos(q(1))*sin(q(0));
    J2_0(0,1) = -L2*cos(q(0))*sin(q(1));
    J2_0(1,0) = L2*cos(q(0))*cos(q(1));
    J2_0(1,1) = -L2*sin(q(0))*sin(q(1));
    J2_0(2,1) = L2*cos(q(1));
    J2_0(3,1) = sin(q(0));
    J2_0(4,1) = -cos(q(0));
    J2_0(5,0) = 1;
}

void URModel::caculet_J3_0(cc::Jacobian &J3_0,
                        const cc::JointPosition &q) const
{
    J3_0(0,0) = -sin(q(0))*(L3*cos(q(1) + q(2)) + L2*cos(q(1)));
    J3_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    J3_0(0,2) = -L3*sin(q(1) + q(2))*cos(q(0));
    J3_0(1,0) = cos(q(0))*(L3*cos(q(1) + q(2)) + L2*cos(q(1)));
    J3_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    J3_0(1,2) = -L3*sin(q(1) + q(2))*sin(q(0));
    J3_0(2,1) = L3*cos(q(1) + q(2)) + L2*cos(q(1));
    J3_0(2,2) = L3*cos(q(1) + q(2));
    J3_0(3,1) = sin(q(0));
    J3_0(3,2) = sin(q(0));
    J3_0(4,1) = -cos(q(0));
    J3_0(4,2) = -cos(q(0));
    J3_0(5,0) = 1;
}

void URModel::caculet_J4_0(cc::Jacobian &J4_0,
                        const cc::JointPosition &q) const
{
    J4_0(0,0) = L4*cos(q(0)) - L2*cos(q(1))*sin(q(0)) - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    J4_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    J4_0(0,2) = -L3*sin(q(1) + q(2))*cos(q(0));
    J4_0(1,0) = L4*sin(q(0)) + L2*cos(q(0))*cos(q(1)) + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    J4_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    J4_0(1,2) = -L3*sin(q(1) + q(2))*sin(q(0));
    J4_0(2,1) = L3*cos(q(1) + q(2)) + L2*cos(q(1));
    J4_0(2,2) = L3*cos(q(1) + q(2));
    J4_0(3,1) = sin(q(0));
    J4_0(3,2) = sin(q(0));
    J4_0(3,3) = sin(q(0));
    J4_0(4,1) = -cos(q(0));
    J4_0(4,2) = -cos(q(0));
    J4_0(4,3) = -cos(q(0));
    J4_0(5,0) = 1;
}

void URModel::caculet_J5_0(cc::Jacobian &J5_0,
                        const cc::JointPosition &q) const
{
    J5_0(0,0) = L4*cos(q(0)) - L5*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - L2*cos(q(1))*sin(q(0)) - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    J5_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)) - L5*cos(q(1) + q(2) + q(3)));
    J5_0(0,2) = -cos(q(0))*(L3*sin(q(1) + q(2)) - L5*cos(q(1) + q(2) + q(3)));
    J5_0(0,3) = L5*cos(q(1) + q(2) + q(3))*cos(q(0));
    J5_0(1,0) = L4*sin(q(0)) + L5*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + L2*cos(q(0))*cos(q(1)) + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    J5_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)) - L5*cos(q(1) + q(2) + q(3)));
    J5_0(1,2) = -sin(q(0))*(L3*sin(q(1) + q(2)) - L5*cos(q(1) + q(2) + q(3)));
    J5_0(1,3) = L5*cos(q(1) + q(2) + q(3))*sin(q(0));
    J5_0(2,1) = L3*cos(q(1) + q(2)) + L2*cos(q(1)) + L5*sin(q(1) + q(2) + q(3));
    J5_0(2,2) = L3*cos(q(1) + q(2)) + L5*sin(q(1) + q(2) + q(3));
    J5_0(2,3) = L5*sin(q(1) + q(2) + q(3));
    J5_0(3,1) = sin(q(0));
    J5_0(3,2) = sin(q(0));
    J5_0(3,3) = sin(q(0));
    J5_0(3,4) = sin(q(1) + q(2) + q(3))*cos(q(0));
    J5_0(4,1) = -cos(q(0));
    J5_0(4,2) = -cos(q(0));
    J5_0(4,3) = -cos(q(0));
    J5_0(4,4) = sin(q(1) + q(2) + q(3))*sin(q(0));
    J5_0(5,0) = 1;
    J5_0(5,4) = -cos(q(1) + q(2) + q(3));
}

void URModel::caculet_J6_0(cc::Jacobian &J6_0,
                        const cc::JointPosition &q) const
{
    J6_0(0,0) = L6*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + L4*cos(q(0)) - L2*cos(q(1))*sin(q(0)) - L5*sin(q(1) + q(2) + q(3))*sin(q(0)) - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    J6_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)) - L5*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - L6*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
    J6_0(0,2) = cos(q(0))*(L5*cos(q(1) + q(2) + q(3)) - L3*sin(q(1) + q(2)) + L6*sin(q(1) + q(2) + q(3))*sin(q(4)));
    J6_0(0,3) = cos(q(0))*(L5*cos(q(1) + q(2) + q(3)) + L6*sin(q(1) + q(2) + q(3))*sin(q(4)));
    J6_0(0,4) = L6*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(3)) - L6*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) - L6*sin(q(0))*sin(q(4)) + L6*cos(q(0))*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + L6*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2));
    J6_0(1,0) = L6*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + L4*sin(q(0)) + L2*cos(q(0))*cos(q(1)) + L5*sin(q(1) + q(2) + q(3))*cos(q(0)) + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    J6_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) + L2*sin(q(1)) - L5*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - L6*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
    J6_0(1,2) = sin(q(0))*(L5*cos(q(1) + q(2) + q(3)) - L3*sin(q(1) + q(2)) + L6*sin(q(1) + q(2) + q(3))*sin(q(4)));
    J6_0(1,3) = sin(q(0))*(L5*cos(q(1) + q(2) + q(3)) + L6*sin(q(1) + q(2) + q(3))*sin(q(4)));
    J6_0(1,4) = L6*cos(q(0))*sin(q(4)) + L6*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(3)) + L6*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) + L6*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2)) - L6*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0));
    J6_0(2,1) = L3*cos(q(1) + q(2)) - (L6*sin(q(1) + q(2) + q(3) + q(4)))/2 + L2*cos(q(1)) + (L6*sin(q(1) + q(2) + q(3) - q(4)))/2 + L5*sin(q(1) + q(2) + q(3));
    J6_0(2,2) = L3*cos(q(1) + q(2)) - (L6*sin(q(1) + q(2) + q(3) + q(4)))/2 + (L6*sin(q(1) + q(2) + q(3) - q(4)))/2 + L5*sin(q(1) + q(2) + q(3));
    J6_0(2,3) = (L6*sin(q(1) + q(2) + q(3) - q(4)))/2 - (L6*sin(q(1) + q(2) + q(3) + q(4)))/2 + L5*sin(q(1) + q(2) + q(3));
    J6_0(2,4) = -L6*(sin(q(1) + q(2) + q(3) + q(4))/2 + sin(q(1) + q(2) + q(3) - q(4))/2);
    J6_0(3,1) = sin(q(0));
    J6_0(3,2) = sin(q(0));
    J6_0(3,3) = sin(q(0));
    J6_0(3,4) = sin(q(1) + q(2) + q(3))*cos(q(0));
    J6_0(3,5) = cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4));
    J6_0(4,1) = -cos(q(0));
    J6_0(4,2) = -cos(q(0));
    J6_0(4,3) = -cos(q(0));
    J6_0(4,4) = sin(q(1) + q(2) + q(3))*sin(q(0));
    J6_0(4,5) = - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4));
    J6_0(5,0) = 1;
    J6_0(5,4) = -cos(q(1) + q(2) + q(3));
    J6_0(5,5) = -sin(q(1) + q(2) + q(3))*sin(q(4));
}


//////////////////////////////////////////////////////////////////////////////////////////////
// CM
//////////////////////////////////////////////////////////////////////////////////////////////

void URModel::caculet_Jcm1_0(cc::Jacobian &Jcm1_0,
                        const cc::JointPosition &q) const
{
    Jcm1_0(1,0) = 21/1000;
    Jcm1_0(5,0) = 1;
}

void URModel::caculet_Jcm2_0(cc::Jacobian &Jcm2_0,
                        const cc::JointPosition &q) const
{
    Jcm2_0(0,0) = (79*cos(q(0)))/500 - (19*sin(q(0)))/50;
    Jcm2_0(1,0) = (19*cos(q(0)))/50 + (79*sin(q(0)))/500;
    Jcm2_0(2,1) = 19/50;
    Jcm2_0(3,1) = sin(q(0));
    Jcm2_0(4,1) = -cos(q(0));
    Jcm2_0(5,0) = 1;

}

void URModel::caculet_Jcm3_0(cc::Jacobian &Jcm3_0,
                        const cc::JointPosition &q) const
{
    Jcm3_0(0,0) = (17*cos(q(0)))/250 - (6*cos(q(1))*sin(q(0)))/25 - L2*cos(q(1))*sin(q(0));
    Jcm3_0(0,1) = -cos(q(0))*sin(q(1))*(L2 + 6/25);
    Jcm3_0(0,2) = -(6*cos(q(0))*sin(q(1)))/25;
    Jcm3_0(1,0) = (17*sin(q(0)))/250 + (6*cos(q(0))*cos(q(1)))/25 + L2*cos(q(0))*cos(q(1));
    Jcm3_0(1,1) = -sin(q(0))*sin(q(1))*(L2 + 6/25);
    Jcm3_0(1,2) = -(6*sin(q(0))*sin(q(1)))/25;
    Jcm3_0(2,1) = cos(q(1))*(L2 + 6/25);
    Jcm3_0(2,2) = (6*cos(q(1)))/25;
    Jcm3_0(3,1) = sin(q(0));
    Jcm3_0(3,2) = sin(q(0));
    Jcm3_0(4,1) = -cos(q(0));
    Jcm3_0(4,2) = -cos(q(0));
    Jcm3_0(5,0) = 1;
}

void URModel::caculet_Jcm4_0(cc::Jacobian &Jcm4_0,
                        const cc::JointPosition &q) const
{
    Jcm4_0(0,0) = (9*cos(q(0)))/500 - L2*cos(q(1))*sin(q(0)) + (7*cos(q(1))*sin(q(0))*sin(q(2)))/1000 + (7*cos(q(2))*sin(q(0))*sin(q(1)))/1000 - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    Jcm4_0(0,1) = -cos(q(0))*((7*cos(q(1) + q(2)))/1000 + L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    Jcm4_0(0,2) = -cos(q(0))*((7*cos(q(1) + q(2)))/1000 + L3*sin(q(1) + q(2)));
    Jcm4_0(0,3) = -(7*cos(q(1) + q(2))*cos(q(0)))/1000;
    Jcm4_0(1,0) = (9*sin(q(0)))/500 + L2*cos(q(0))*cos(q(1)) - (7*cos(q(0))*cos(q(1))*sin(q(2)))/1000 - (7*cos(q(0))*cos(q(2))*sin(q(1)))/1000 + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    Jcm4_0(1,1) = -sin(q(0))*((7*cos(q(1) + q(2)))/1000 + L3*sin(q(1) + q(2)) + L2*sin(q(1)));
    Jcm4_0(1,2) = -sin(q(0))*((7*cos(q(1) + q(2)))/1000 + L3*sin(q(1) + q(2)));
    Jcm4_0(1,3) = -(7*cos(q(1) + q(2))*sin(q(0)))/1000;
    Jcm4_0(2,1) = L3*cos(q(1) + q(2)) - (7*sin(q(1) + q(2)))/1000 + L2*cos(q(1));
    Jcm4_0(2,2) = L3*cos(q(1) + q(2)) - (7*sin(q(1) + q(2)))/1000;
    Jcm4_0(2,3) = -(7*sin(q(1) + q(2)))/1000;
    Jcm4_0(3,1) = sin(q(0));
    Jcm4_0(3,2) = sin(q(0));
    Jcm4_0(3,3) = sin(q(0));
    Jcm4_0(4,1) = -cos(q(0));
    Jcm4_0(4,2) = -cos(q(0));
    Jcm4_0(4,3) = -cos(q(0));
    Jcm4_0(5,0) = 1;
}

void URModel::caculet_Jcm5_0(cc::Jacobian &Jcm5_0,
                        const cc::JointPosition &q) const
{
    Jcm5_0(0,0) = (7*cos(q(0)))/1000 + L4*cos(q(0)) - (9*cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))/500 + (9*sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))))/500 - L2*cos(q(1))*sin(q(0)) - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    Jcm5_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) - (9*cos(q(1) + q(2) + q(3)))/500 + L2*sin(q(1)));
    Jcm5_0(0,2) = cos(q(0))*((9*cos(q(1) + q(2) + q(3)))/500 - L3*sin(q(1) + q(2)));
    Jcm5_0(0,3) = (9*cos(q(1) + q(2) + q(3))*cos(q(0)))/500;
    Jcm5_0(0,4) = -(7*cos(q(1) + q(2) + q(3))*cos(q(0)))/1000;
    Jcm5_0(1,0) = (7*sin(q(0)))/1000 + (9*cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))/500 + (9*sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))))/500 + L4*sin(q(0)) + L2*cos(q(0))*cos(q(1)) + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    Jcm5_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) - (9*cos(q(1) + q(2) + q(3)))/500 + L2*sin(q(1)));
    Jcm5_0(1,2) = sin(q(0))*((9*cos(q(1) + q(2) + q(3)))/500 - L3*sin(q(1) + q(2)));
    Jcm5_0(1,3) = (9*cos(q(1) + q(2) + q(3))*sin(q(0)))/500;
    Jcm5_0(1,4) = -(7*cos(q(1) + q(2) + q(3))*sin(q(0)))/1000;
    Jcm5_0(2,1) = (9*sin(q(1) + q(2) + q(3)))/500 + L3*cos(q(1) + q(2)) + L2*cos(q(1));
    Jcm5_0(2,2) = (9*sin(q(1) + q(2) + q(3)))/500 + L3*cos(q(1) + q(2));
    Jcm5_0(2,3) = (9*sin(q(1) + q(2) + q(3)))/500;
    Jcm5_0(2,4) = -(7*sin(q(1) + q(2) + q(3)))/1000;
    Jcm5_0(3,1) = sin(q(0));
    Jcm5_0(3,2) = sin(q(0));
    Jcm5_0(3,3) = sin(q(0));
    Jcm5_0(3,4) = sin(q(1) + q(2) + q(3))*cos(q(0));
    Jcm5_0(4,1) = -cos(q(0));
    Jcm5_0(4,2) = -cos(q(0));
    Jcm5_0(4,3) = -cos(q(0));
    Jcm5_0(4,4) = sin(q(1) + q(2) + q(3))*sin(q(0));
    Jcm5_0(5,0) = 1;
    Jcm5_0(5,4) = -cos(q(1) + q(2) + q(3));
}

void URModel::caculet_Jcm6_0(cc::Jacobian &Jcm6_0,
                        const cc::JointPosition &q) const
{
    Jcm6_0(0,0) = L4*cos(q(0)) - (13*cos(q(0))*cos(q(4)))/500 - (13*cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)))/500 - L2*cos(q(1))*sin(q(0)) - L5*sin(q(1) + q(2) + q(3))*sin(q(0)) - L3*cos(q(1))*cos(q(2))*sin(q(0)) + L3*sin(q(0))*sin(q(1))*sin(q(2));
    Jcm6_0(0,1) = -cos(q(0))*(L3*sin(q(1) + q(2)) + sin(q(4))*((13*cos(q(1) + q(2))*sin(q(3)))/500 + (13*sin(q(1) + q(2))*cos(q(3)))/500) + L2*sin(q(1)) - L5*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))));
    Jcm6_0(0,2) = -cos(q(0))*(L3*sin(q(1) + q(2)) + (13*sin(q(1) + q(2) + q(3))*sin(q(4)))/500 - L5*cos(q(1) + q(2) + q(3)));
    Jcm6_0(0,3) = -(cos(q(0))*(13*sin(q(1) + q(2) + q(3))*sin(q(4)) - 500*L5*cos(q(1) + q(2) + q(3))))/500;
    Jcm6_0(0,4) = (13*sin(q(0))*sin(q(4)))/500 + (13*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))/500 - (13*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(3)))/500 - (13*cos(q(0))*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)))/500 - (13*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)))/500;
    Jcm6_0(1,0) = L4*sin(q(0)) - (13*cos(q(4))*sin(q(0)))/500 + (13*cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)))/500 + L2*cos(q(0))*cos(q(1)) + L5*sin(q(1) + q(2) + q(3))*cos(q(0)) + L3*cos(q(0))*cos(q(1))*cos(q(2)) - L3*cos(q(0))*sin(q(1))*sin(q(2));
    Jcm6_0(1,1) = -sin(q(0))*(L3*sin(q(1) + q(2)) + sin(q(4))*((13*cos(q(1) + q(2))*sin(q(3)))/500 + (13*sin(q(1) + q(2))*cos(q(3)))/500) + L2*sin(q(1)) - L5*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))));
    Jcm6_0(1,2) = -sin(q(0))*(L3*sin(q(1) + q(2)) + (13*sin(q(1) + q(2) + q(3))*sin(q(4)))/500 - L5*cos(q(1) + q(2) + q(3)));
    Jcm6_0(1,3) = -(sin(q(0))*(13*sin(q(1) + q(2) + q(3))*sin(q(4)) - 500*L5*cos(q(1) + q(2) + q(3))))/500;
    Jcm6_0(1,4) = (13*cos(q(1) + q(2) + q(3))*cos(q(4))*sin(q(0)))/500 - (13*cos(q(0))*sin(q(4)))/500;
    Jcm6_0(2,1) = (13*sin(q(1) + q(2) + q(3) + q(4)))/1000 - (13*sin(q(1) + q(2) + q(3) - q(4)))/1000 + L3*cos(q(1) + q(2)) + L2*cos(q(1)) + L5*sin(q(1) + q(2) + q(3));
    Jcm6_0(2,2) = (13*sin(q(1) + q(2) + q(3) + q(4)))/1000 - (13*sin(q(1) + q(2) + q(3) - q(4)))/1000 + L3*cos(q(1) + q(2)) + L5*sin(q(1) + q(2) + q(3));
    Jcm6_0(2,3) = (13*sin(q(1) + q(2) + q(3) + q(4)))/1000 - (13*sin(q(1) + q(2) + q(3) - q(4)))/1000 + L5*sin(q(1) + q(2) + q(3));
    Jcm6_0(2,4) = (13*sin(q(1) + q(2) + q(3) + q(4)))/1000 + (13*sin(q(1) + q(2) + q(3) - q(4)))/1000;
    Jcm6_0(3,1) = sin(q(0));
    Jcm6_0(3,2) = sin(q(0));
    Jcm6_0(3,3) = sin(q(0));
    Jcm6_0(3,4) = sin(q(1) + q(2) + q(3))*cos(q(0));
    Jcm6_0(3,5) = cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4));
    Jcm6_0(4,1) = -cos(q(0));
    Jcm6_0(4,2) = -cos(q(0));
    Jcm6_0(4,3) = -cos(q(0));
    Jcm6_0(4,4) = sin(q(1) + q(2) + q(3))*sin(q(0));
    Jcm6_0(4,5) = - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4));
    Jcm6_0(5,0) = 1;
    Jcm6_0(5,4) = -cos(q(1) + q(2) + q(3));
    Jcm6_0(5,5) = -sin(q(1) + q(2) + q(3))*sin(q(4));
}


#endif