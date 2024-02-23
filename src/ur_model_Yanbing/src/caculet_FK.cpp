#include <ur_model/ur_model.h>

using namespace ur_model_namespace;

void URModel::caculet_T1_0(cc::HomogeneousTransformation &T1_0,
                        const cc::JointPosition &q) const
{
    T1_0(0,0) = cos(q(0));
    T1_0(0,1) = 0;
    T1_0(0,2) = sin(q(0));
    T1_0(0,3) = 0;

    T1_0(1,0) = sin(q(0));
    T1_0(1,1) = 0;
    T1_0(1,2) = -cos(q(0));
    T1_0(1,3) = 0;

    T1_0(2,0) = 0;
    T1_0(2,1) = 1;
    T1_0(2,2) = 0;
    T1_0(2,3) = L1;

    T1_0(3,0) = 0;
    T1_0(3,1) = 0;
    T1_0(3,2) = 0;
    T1_0(3,3) = 1;
}


void URModel::caculet_T2_0(cc::HomogeneousTransformation &T2_0,
                        const cc::JointPosition &q) const
{
    T2_0(0,0) = cos(q(0))*cos(q(1));
    T2_0(0,1) = -cos(q(0))*sin(q(1));
    T2_0(0,2) = sin(q(0));
    T2_0(0,3) = -L3*cos(q(0))*cos(q(1));

    T2_0(1,0) = cos(q(1))*sin(q(0));
    T2_0(1,1) = -sin(q(0))*sin(q(1));
    T2_0(1,2) = -cos(q(0));
    T2_0(1,3) = -L3*cos(q(1))*sin(q(0));

    T2_0(2,0) = sin(q(1));
    T2_0(2,1) = cos(q(1));
    T2_0(2,2) = 0;
    T2_0(2,3) = L1 - L3*sin(q(1));

    T2_0(3,0) = 0;
    T2_0(3,1) = 0;
    T2_0(3,2) = 0;
    T2_0(3,3) = 1;
}


void URModel::caculet_T3_0(cc::HomogeneousTransformation &T3_0,
                        const cc::JointPosition &q) const
{
    T3_0(0,0) = cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2));
    T3_0(0,1) = - cos(q(0))*cos(q(1))*sin(q(2)) - cos(q(0))*cos(q(2))*sin(q(1));
    T3_0(0,2) = sin(q(0));
    T3_0(0,3) = cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) - L3*cos(q(0))*cos(q(1));

    T3_0(1,0) = cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2));
    T3_0(1,1) = - cos(q(1))*sin(q(0))*sin(q(2)) - cos(q(2))*sin(q(0))*sin(q(1));
    T3_0(1,2) = -cos(q(0));
    T3_0(1,3) = sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) - L3*cos(q(1))*sin(q(0));

    T3_0(2,0) = cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1));
    T3_0(2,1) = cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2));
    T3_0(2,2) = 0;
    T3_0(2,3) = L1 - L3*sin(q(1)) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    T3_0(3,0) = 0;
    T3_0(3,1) = 0;
    T3_0(3,2) = 0;
    T3_0(3,3) = 1;
}


void URModel::caculet_T4_0(cc::HomogeneousTransformation &T4_0,
                        const cc::JointPosition &q) const
{
    T4_0(0,0) = cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)));
    T4_0(0,1) = sin(q(0));
    T4_0(0,2) = cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
    T4_0(0,3) = L2*sin(q(0)) - L3*cos(q(0))*cos(q(1)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    T4_0(1,0) = - cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
    T4_0(1,1) = -cos(q(0));
    T4_0(1,2) = cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)));
    T4_0(1,3) = sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11) - L3*cos(q(1))*sin(q(0)) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) - L2*cos(q(0));

    T4_0(2,0) = cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
    T4_0(2,1) = 0;
    T4_0(2,2) = sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) - cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
    T4_0(2,3) = L1 - L3*sin(q(1)) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    T4_0(3,0) = 0;
    T4_0(3,1) = 0;
    T4_0(3,2) = 0;
    T4_0(3,3) = 1;
}

void URModel::caculet_T5_0(cc::HomogeneousTransformation &T5_0,
                        const cc::JointPosition &q) const
{
    T5_0(0,0) = sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    T5_0(0,1) = - cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) - sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
    T5_0(0,2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    T5_0(0,3) = L2*sin(q(0)) + L11*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - L3*cos(q(0))*cos(q(1)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    T5_0(1,0) = - cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))));
    T5_0(1,1) = sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
    T5_0(1,2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
    T5_0(1,3) = L11*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - L2*cos(q(0)) - L3*cos(q(1))*sin(q(0)) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) + sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    T5_0(2,0) = cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    T5_0(2,1) = cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)));
    T5_0(2,2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    T5_0(2,3) = L1 - L3*sin(q(1)) - L11*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    T5_0(3,0) = 0;
    T5_0(3,1) = 0;
    T5_0(3,2) = 0;
    T5_0(3,3) = 1;
}

void URModel::caculet_T6_0(cc::HomogeneousTransformation &T6_0,
                        const cc::JointPosition &q) const
{
    T6_0(0,0) = cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
    T6_0(0,1) = - sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
    T6_0(0,2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    T6_0(0,3) = L2*sin(q(0)) + L4*(cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + L11*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - L3*cos(q(0))*cos(q(1)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    T6_0(1,0) = - cos(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
    T6_0(1,1) = sin(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
    T6_0(1,2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
    T6_0(1,3) = L11*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - L2*cos(q(0)) - L4*(cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - L3*cos(q(1))*sin(q(0)) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) + sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    T6_0(2,0) = sin(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + cos(q(4))*cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    T6_0(2,1) = cos(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(4))*sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    T6_0(2,2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    T6_0(2,3) = L1 - L3*sin(q(1)) - L11*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11) - L4*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));

    T6_0(3,0) = 0;
    T6_0(3,1) = 0;
    T6_0(3,2) = 0;
    T6_0(3,3) = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// CM
//////////////////////////////////////////////////////////////////////////////////////////////

void URModel::caculet_Tcm1_0(cc::HomogeneousTransformation &Tcm1_0,
                        const cc::JointPosition &q) const
{
    Tcm1_0(0,0) = cos(q(0));
    Tcm1_0(0,1) = 0;
    Tcm1_0(0,2) = sin(q(0));
    Tcm1_0(0,3) = 21/1000;

    Tcm1_0(1,0) = sin(q(0));
    Tcm1_0(1,1) = 0;
    Tcm1_0(1,2) = -cos(q(0));
    Tcm1_0(1,3) = 0;

    Tcm1_0(2,0) = 0;
    Tcm1_0(2,1) = 1;
    Tcm1_0(2,2) = 0;
    Tcm1_0(2,3) = 27/1000;

    Tcm1_0(3,0) = 0;
    Tcm1_0(3,1) = 0;
    Tcm1_0(3,2) = 0;
    Tcm1_0(3,3) = 1;
}

void URModel::caculet_Tcm2_0(cc::HomogeneousTransformation &Tcm2_0,
                        const cc::JointPosition &q) const
{
    Tcm2_0(0,0) = cos(q(0))*cos(q(1));
    Tcm2_0(0,1) = -cos(q(0))*sin(q(1));
    Tcm2_0(0,2) = sin(q(0));
    Tcm2_0(0,3) = (19*cos(q(0)))/50 + (79*sin(q(0)))/500;

    Tcm2_0(1,0) = cos(q(1))*sin(q(0));
    Tcm2_0(1,1) = -sin(q(0))*sin(q(1));
    Tcm2_0(1,2) = -cos(q(0));
    Tcm2_0(1,3) = (19*sin(q(0)))/50 - (79*cos(q(0)))/500;

    Tcm2_0(2,0) = sin(q(1));
    Tcm2_0(2,1) = cos(q(1));
    Tcm2_0(2,2) = 0;
    Tcm2_0(2,3) = L1;

    Tcm2_0(3,0) = 0;
    Tcm2_0(3,1) = 0;
    Tcm2_0(3,2) = 0;
    Tcm2_0(3,3) = 1;
}

void URModel::caculet_Tcm3_0(cc::HomogeneousTransformation &Tcm3_0,
                        const cc::JointPosition &q) const
{
    Tcm3_0(0,0) = cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2));
    Tcm3_0(0,1) = - cos(q(0))*cos(q(1))*sin(q(2)) - cos(q(0))*cos(q(2))*sin(q(1));
    Tcm3_0(0,2) = sin(q(0));
    Tcm3_0(0,3) = (17*sin(q(0)))/250 + (6*cos(q(0))*cos(q(1)))/25 - L3*cos(q(0))*cos(q(1));

    Tcm3_0(1,0) = cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2));
    Tcm3_0(1,1) = - cos(q(1))*sin(q(0))*sin(q(2)) - cos(q(2))*sin(q(0))*sin(q(1));
    Tcm3_0(1,2) = -cos(q(0));
    Tcm3_0(1,3) = (6*cos(q(1))*sin(q(0)))/25 - (17*cos(q(0)))/250 - L3*cos(q(1))*sin(q(0));

    Tcm3_0(2,0) = cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1));
    Tcm3_0(2,1) = cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2));
    Tcm3_0(2,2) = 0;
    Tcm3_0(2,3) = L1 + (6*sin(q(1)))/25 - L3*sin(q(1));

    Tcm3_0(3,0) = 0;
    Tcm3_0(3,1) = 0;
    Tcm3_0(3,2) = 0;
    Tcm3_0(3,3) = 1;
}

void URModel::caculet_Tcm4_0(cc::HomogeneousTransformation &Tcm4_0,
                        const cc::JointPosition &q) const
{
    Tcm4_0(0,0) = cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)));
    Tcm4_0(0,1) = sin(q(0));
    Tcm4_0(0,2) = cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
    Tcm4_0(0,3) = (9*sin(q(0)))/500 - L3*cos(q(0))*cos(q(1)) - (7*cos(q(0))*cos(q(1))*sin(q(2)))/1000 - (7*cos(q(0))*cos(q(2))*sin(q(1)))/1000 - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    Tcm4_0(1,0) = - cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
    Tcm4_0(1,1) = -cos(q(0));
    Tcm4_0(1,2) = cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)));
    Tcm4_0(1,3) = sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11) - L3*cos(q(1))*sin(q(0)) - (7*cos(q(1))*sin(q(0))*sin(q(2)))/1000 - (7*cos(q(2))*sin(q(0))*sin(q(1)))/1000 - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) - (9*cos(q(0)))/500;

    Tcm4_0(2,0) = cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
    Tcm4_0(2,1) = 0;
    Tcm4_0(2,2) = sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) - cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
    Tcm4_0(2,3) = L1 + (7*cos(q(1))*cos(q(2)))/1000 - (7*sin(q(1))*sin(q(2)))/1000 - L3*sin(q(1)) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    Tcm4_0(3,0) = 0;
    Tcm4_0(3,1) = 0;
    Tcm4_0(3,2) = 0;
    Tcm4_0(3,3) = 1;
}

void URModel::caculet_Tcm5_0(cc::HomogeneousTransformation &Tcm5_0,
                        const cc::JointPosition &q) const
{
    Tcm5_0(0,0) = sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    Tcm5_0(0,1) = - cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) - sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
    Tcm5_0(0,2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    Tcm5_0(0,3) = (7*sin(q(0)))/1000 + (9*cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))/500 + (9*sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))))/500 + L2*sin(q(0)) - L3*cos(q(0))*cos(q(1)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    Tcm5_0(1,0) = - cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))));
    Tcm5_0(1,1) = sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
    Tcm5_0(1,2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
    Tcm5_0(1,3) = (9*cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))/500 - L2*cos(q(0)) - (7*cos(q(0)))/1000 - (9*sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))))/500 - L3*cos(q(1))*sin(q(0)) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) + sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    Tcm5_0(2,0) = cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    Tcm5_0(2,1) = cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)));
    Tcm5_0(2,2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    Tcm5_0(2,3) = L1 - L3*sin(q(1)) - (9*cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))))/500 + (9*sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))))/500 - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    Tcm5_0(3,0) = 0;
    Tcm5_0(3,1) = 0;
    Tcm5_0(3,2) = 0;
    Tcm5_0(3,3) = 1;
}

void URModel::caculet_Tcm6_0(cc::HomogeneousTransformation &Tcm6_0,
                        const cc::JointPosition &q) const
{
    Tcm6_0(0,0) = cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
    Tcm6_0(0,1) = - sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
    Tcm6_0(0,2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
    Tcm6_0(0,3) = (13*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))))/500 - (13*cos(q(4))*sin(q(0)))/500 + L2*sin(q(0)) + L11*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - L3*cos(q(0))*cos(q(1)) - cos(q(0))*cos(q(1))*cos(q(2))*(L5 - L11) + cos(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    Tcm6_0(1,0) = - cos(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
    Tcm6_0(1,1) = sin(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
    Tcm6_0(1,2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
    Tcm6_0(1,3) = (13*cos(q(0))*cos(q(4)))/500 + L11*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - L2*cos(q(0)) - (13*sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))))/500 - L3*cos(q(1))*sin(q(0)) - cos(q(1))*cos(q(2))*sin(q(0))*(L5 - L11) + sin(q(0))*sin(q(1))*sin(q(2))*(L5 - L11);

    Tcm6_0(2,0) = sin(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + cos(q(4))*cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    Tcm6_0(2,1) = cos(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(4))*sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    Tcm6_0(2,2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
    Tcm6_0(2,3) = L1 + (13*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))))/500 - L3*sin(q(1)) - L11*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(1))*sin(q(2))*(L5 - L11) - cos(q(2))*sin(q(1))*(L5 - L11);

    Tcm6_0(3,0) = 0;
    Tcm6_0(3,1) = 0;
    Tcm6_0(3,2) = 0;
    Tcm6_0(3,3) = 1;
}

#endif

