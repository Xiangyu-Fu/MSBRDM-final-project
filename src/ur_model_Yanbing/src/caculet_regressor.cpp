#include <ur_model/ur_model.h>

using namespace ur_model_namespace;

void URModel::caculet_Theta(Parameters &Theta) const
{
    Theta(0, 0) = 4*I622 - 4*I611;
    Theta(1, 0) = 8*I612;
    Theta(2, 0) = 4*I533 - 4*I511 - 4*I622 + 4*I633 - (169*m6)/62500;
    Theta(3, 0) = 8*I613;
    Theta(4, 0) = -8*I623;
    Theta(5, 0) = 8*I513;
    Theta(6, 0) = - 4*m6*L11^2 - 4*I411 + 4*I433 + 4*I522 - 4*I533 + 4*I611 - 4*I633 - (81*m5)/62500 + (169*m6)/62500;
    Theta(7, 0) = 8*I512 - 8*I612;
    Theta(8, 0) = (26*L11*m6)/125 - 8*I523;
    Theta(9, 0) = -8*I413;

    Theta(10, 0) = (13*L11*m6)/125 - (13*L5*m6)/125;
    Theta(11, 0) = (9*L11*m5)/125 - (9*L5*m5)/125 + 4*L11^2*m6 - 4*L5*L11*m6;
    Theta(12, 0) = 2*I322 - 2*I311 + 2*I411 - 2*I433 - 2*I522 + 2*I533 - 2*I611 + 2*I633 - (49*m4)/500000 + (81*m5)/125000 - (169*m6)/125000 + 2*L5^2*m4 + 2*L5^2*m5 + 2*L5^2*m6 + 2*L11^2*m4 + 2*L11^2*m5 + 4*L11^2*m6 - 4*L5*L11*m4 - 4*L5*L11*m5 - 4*L5*L11*m6;
    Theta(13, 0) = -(13*L3*m6)/250;
    Theta(14, 0) = 4*I312 + 4*I413 + (7*L5*m4)/250 - (7*L11*m4)/250;
    Theta(15, 0) = - (9*L3*m5)/250 - 2*L3*L11*m6;
    Theta(16, 0) = 2*L3*L5*m4 + 2*L3*L5*m5 + 2*L3*L5*m6 - 2*L3*L11*m4 - 2*L3*L11*m5 - 2*L3*L11*m6;
    Theta(17, 0) = (7*L3*m4)/500;
    Theta(18, 0) = I222 - I211 + I311 - I322 - I411 + I433 + I522 - I533 + I611 - I633 + (36*m3)/625 + (49*m4)/1000000 - (81*m5)/250000 + (169*m6)/250000 - (12*L3*m3)/25 + L3^2*m3 + L3^2*m4 + L3^2*m5 + L3^2*m6 - L5^2*m4 - L5^2*m5 - L5^2*m6 - L11^2*m4 - L11^2*m5 - 2*L11^2*m6 + 2*L5*L11*m4 + 2*L5*L11*m5 + 2*L5*L11*m6;
    Theta(19, 0) = 2*I212 - 2*I312 - 2*I413 - (7*L5*m4)/500 + (7*L11*m4)/500;

    Theta(20, 0) = -(13*L2*m6)/250;
    Theta(21, 0) = I122 + I211 + I322 + I411 + I533 + I633 + (441*m1)/1000000 + (42341*m2)/250000 + (289*m3)/62500 + (81*m4)/250000 + (373*m5)/1000000 + (7*L2*m5)/500 + L2^2*m5 + L2^2*m6 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L11^2*m4 + L11^2*m5 + 2*L11^2*m6 - 2*L5*L11*m4 - 2*L5*L11*m5 - 2*L5*L11*m6;
    Theta(22, 0) = (63*m5)/500000 - I423 + (9*L2*m5)/500 + L2*L11*m6;
    Theta(23, 0) = I412 - I513;
    Theta(24, 0) = I323 - (63*m4)/500000;
    Theta(25, 0) = I313 + (9*L5*m4)/500 + (7*L5*m5)/1000 - (9*L11*m4)/500 - (7*L11*m5)/1000 + L2*L5*m5 + L2*L5*m6 - L2*L11*m5 - L2*L11*m6;
    Theta(26, 0) = I223;
    Theta(27, 0) = I213 - (51*m3)/3125 + (17*L3*m3)/250 + (9*L3*m4)/500 + (7*L3*m5)/1000 + L2*L3*m5 + L2*L3*m6;
    Theta(28, 0) = -(51*m3)/3125;
    Theta(29, 0) = -(63*m4)/500000;

    Theta(30, 0) = - I522 - I611 - (49*m5)/1000000 - (169*m6)/250000 - (7*L2*m5)/1000;
    Theta(31, 0) = -I633;
    Theta(32, 0) = I522 - I511 + I533 + I611 - I622 + I633 + (49*m5)/1000000 + (7*L2*m5)/1000;
    Theta(33, 0) = I622 - I611 - I633;
    Theta(34, 0) = I611 - I622 - I633;
    Theta(35, 0) = (13*gy*m6)/500;
    Theta(36, 0) = (9*gy*m5)/500 + L11*gy*m6;
    Theta(37, 0) = L11*gy*m4 - L5*gy*m5 - L5*gy*m6 - L5*gy*m4 + L11*gy*m5 + L11*gy*m6;
    Theta(38, 0) = -(7*gy*m4)/1000;
    Theta(39, 0) = (6*gy*m3)/25 - L3*gy*m3 - L3*gy*m4 - L3*gy*m5 - L3*gy*m6;

    Theta(40, 0) = -(13*gx*m6)/500;
    Theta(41, 0) = (79*gx*m2)/500 + (17*gx*m3)/250 + (9*gx*m4)/500 + (7*gx*m5)/1000 + (19*gy*m2)/50 + L2*gx*m5 + L2*gx*m6;
    Theta(42, 0) = - (9*gx*m5)/500 - L11*gx*m6;
    Theta(43, 0) = L5*gx*m4 + L5*gx*m5 + L5*gx*m6 - L11*gx*m4 - L11*gx*m5 - L11*gx*m6;
    Theta(44, 0) = (7*gx*m4)/1000;
    Theta(45, 0) = L3*gx*m3 - (6*gx*m3)/25 + L3*gx*m4 + L3*gx*m5 + L3*gx*m6;
    Theta(46, 0) = (79*gy*m2)/500 - (19*gx*m2)/50 + (17*gy*m3)/250 + (9*gy*m4)/500 + (7*gy*m5)/1000 + L2*gy*m5 + L2*gy*m6;
    Theta(47, 0) = I233 + I333 + I422 + I511 + I622 + (361*m2)/2500 + (36*m3)/625 + (49*m4)/1000000 + (81*m5)/250000 + (169*m6)/250000 - (12*L3*m3)/25 + L3^2*m3 + L3^2*m4 + L3^2*m5 + L3^2*m6 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L11^2*m4 + L11^2*m5 + 2*L11^2*m6 - 2*L5*L11*m4 - 2*L5*L11*m5 - 2*L5*L11*m6;
    Theta(48, 0) = I333 + I422 + I511 + I622 + (36*m3)/625 + (49*m4)/1000000 + (81*m5)/250000 + (169*m6)/250000 - (6*L3*m3)/25 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L11^2*m4 + L11^2*m5 + 2*L11^2*m6 - 2*L5*L11*m4 - 2*L5*L11*m5 - 2*L5*L11*m6;
    Theta(49, 0) = m6*L11^2 + I422 + I511 + I622 + (49*m4)/1000000 + (81*m5)/250000 + (169*m6)/250000;

    Theta(50, 0) = (7*L3*m5)/1000;
    Theta(51, 0) = (7*L5*m5)/1000 - (7*L11*m5)/1000;
    Theta(52, 0) = -(63*m5)/500000;
    Theta(53, 0) = I533 - I522 - I511 - I611 - I622 + I633 - (49*m5)/1000000 - (169*m6)/125000 - (7*L2*m5)/1000;
    Theta(54, 0) = (13*gz*m6)/500;
    Theta(55, 0) = (9*gz*m5)/500 + L11*gz*m6;
    Theta(56, 0) = L11*gz*m4 - L5*gz*m5 - L5*gz*m6 - L5*gz*m4 + L11*gz*m5 + L11*gz*m6;
    Theta(57, 0) = -(7*gz*m4)/1000;
    Theta(58, 0) = (6*gz*m3)/25 - L3*gz*m3 - L3*gz*m4 - L3*gz*m5 - L3*gz*m6;
    Theta(59, 0) = I333 + I422 + I511 + I622 + (36*m3)/625 + (49*m4)/1000000 + (81*m5)/250000 + (169*m6)/250000 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L11^2*m4 + L11^2*m5 + 2*L11^2*m6 - 2*L5*L11*m4 - 2*L5*L11*m5 - 2*L5*L11*m6;

    Theta(60, 0) = I522 + I611 + (49*m5)/1000000 + (169*m6)/250000;
}

void URModel::matrix_Yr(Regressor &Y,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP,
                        const cc::JointVelocity &qrP,
                        const cc::JointAcceleration &qrPP) const
{
    Yr(0, 0) = qpp1r/32 - (qpp1r*cos(2*q5))/32 + (qpp1r*cos(2*q6))/32 + (3*qpp1r*cos(2*q2 + 2*q3 + 2*q4))/32 - (sin(q2 + q3 + q4 - q5 - 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 + (sin(q2 + q3 + q4 - q5 + 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 + (sin(q2 + q3 + q4 - 2*q5 - 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/32 + (sin(q2 + q3 + q4 - 2*q5 + 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/32 - (sin(q2 + q3 + q4 + 2*q5 - 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/32 - (sin(q2 + q3 + q4 + 2*q5 + 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/32 + (qp4^2*sin(q2 + q3 + q4 - 2*q5))/32 - (qp4^2*sin(q2 + q3 + q4 + 2*q5))/32 - (qpp4*cos(q2 + q3 + q4 + q5 - 2*q6))/32 + (qpp4*cos(q2 + q3 + q4 + q5 + 2*q6))/32 + (qpp5*cos(q2 + q3 + q4 + q5 - 2*q6))/32 - (qpp5*cos(q2 + q3 + q4 + q5 + 2*q6))/32 - (qpp2r*cos(q2 + q3 + q4 + q5 - 2*q6))/32 + (qpp2r*cos(q2 + q3 + q4 + q5 + 2*q6))/32 - (qpp3r*cos(q2 + q3 + q4 + q5 - 2*q6))/32 + (qpp3r*cos(q2 + q3 + q4 + q5 + 2*q6))/32 + (sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/64 + (sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/64 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/64 - (qpp1r*cos(2*q5 - 2*q6))/64 - (qpp1r*cos(2*q5 + 2*q6))/64 + (sin(q2 + q3 + q4 - 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 - (sin(q2 + q3 + q4 + 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 - (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/32 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/32 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/32 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/32 - (3*sin(2*q2 + 2*q3 + 2*q4 - 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/32 - (3*sin(2*q2 + 2*q3 + 2*q4 + 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/32 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/32 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/32 - (3*sin(2*q2 + 2*q3 + 2*q4 - 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/32 - (3*sin(2*q2 + 2*q3 + 2*q4 + 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/32 + (qpp4*cos(q2 + q3 + q4 - q5 - 2*q6))/32 - (qpp4*cos(q2 + q3 + q4 - q5 + 2*q6))/32 - (qpp4*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/64 - (qpp4*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/64 + (qpp4*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/64 + (qpp4*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/64 + (qpp5*cos(q2 + q3 + q4 - q5 - 2*q6))/32 - (qpp5*cos(q2 + q3 + q4 - q5 + 2*q6))/32 + (qpp2r*cos(q2 + q3 + q4 - q5 - 2*q6))/32 - (qpp2r*cos(q2 + q3 + q4 - q5 + 2*q6))/32 - (qpp2r*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/64 - (qpp2r*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/64 + (qpp2r*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/64 + (qpp2r*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/64 + (qpp3r*cos(q2 + q3 + q4 - q5 - 2*q6))/32 - (qpp3r*cos(q2 + q3 + q4 - q5 + 2*q6))/32 - (qpp3r*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/64 - (qpp3r*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/64 + (qpp3r*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/64 + (qpp3r*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/64 - (3*sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (3*sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (qp4^2*sin(q2 + q3 + q4 + q5 - 2*q6))/32 - (qp4^2*sin(q2 + q3 + q4 + q5 + 2*q6))/32 - (qp5^2*sin(q2 + q3 + q4 + q5 - 2*q6))/32 + (qp5^2*sin(q2 + q3 + q4 + q5 + 2*q6))/32 - (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/32 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/32 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/128 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/128 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/128 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/128 + (sin(q2 + q3 + q4 + q5 - 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 - (sin(q2 + q3 + q4 + q5 + 2*q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/16 - (qpp4*cos(q2 + q3 + q4 - 2*q5))/32 + (qpp4*cos(q2 + q3 + q4 + 2*q5))/32 - (qpp5*cos(q2 + q3 + q4 - 2*q6))/16 - (qpp5*cos(q2 + q3 + q4 + 2*q6))/16 - (qpp2r*cos(q2 + q3 + q4 - 2*q5))/32 + (qpp2r*cos(q2 + q3 + q4 + 2*q5))/32 - (qpp3r*cos(q2 + q3 + q4 - 2*q5))/32 + (qpp3r*cos(q2 + q3 + q4 + 2*q5))/32 + (sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 + (sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/64 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/64 + (3*qpp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q6))/64 + (3*qpp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q6))/64 - (qpp5*cos(q2 + q3 + q4))/8 - (qp4^2*sin(q2 + q3 + q4 - q5 - 2*q6))/32 + (qp4^2*sin(q2 + q3 + q4 - q5 + 2*q6))/32 + (qp4^2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/64 + (qp4^2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/64 - (qp4^2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/64 - (qp4^2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/64 + (qp5^2*sin(q2 + q3 + q4 - q5 - 2*q6))/32 - (qp5^2*sin(q2 + q3 + q4 - q5 + 2*q6))/32 - (qp2*qp4*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp2*qp4*sin(q2 + q3 + q4 - q5 + 2*q6))/16 + (qp2*qp4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 + (qp2*qp4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 - (qp2*qp4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp2*qp4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 - (qp3*qp4*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp3*qp4*sin(q2 + q3 + q4 - q5 + 2*q6))/16 - (qp2*qp5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 - (qp2*qp5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 - (qp2*qp5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp2*qp5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 + (qp2*qp6*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp2*qp6*sin(q2 + q3 + q4 - q5 + 2*q6))/16 + (qp3*qp4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 + (qp3*qp4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 - (qp3*qp4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp3*qp4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 - (qp2*qp6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 + (qp2*qp6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 + (qp2*qp6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp2*qp6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 - (qp3*qp5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 - (qp3*qp5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 - (qp3*qp5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp3*qp5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 + (qp3*qp6*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp3*qp6*sin(q2 + q3 + q4 - q5 + 2*q6))/16 - (qp3*qp6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 + (qp3*qp6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 + (qp3*qp6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp3*qp6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 - (qp4*qp5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 - (qp4*qp5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 - (qp4*qp5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp4*qp5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 + (qp4*qp6*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp4*qp6*sin(q2 + q3 + q4 - q5 + 2*q6))/16 - (qp4*qp6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/32 + (qp4*qp6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/32 + (qp4*qp6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/32 - (qp4*qp6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/32 + (qp5*qp6*sin(q2 + q3 + q4 - q5 - 2*q6))/16 + (qp5*qp6*sin(q2 + q3 + q4 - q5 + 2*q6))/16 - (qp2*qp2r*sin(q2 + q3 + q4 - q5 - 2*q6))/32 + (qp2*qp2r*sin(q2 + q3 + q4 - q5 + 2*q6))/32 + (qp2*qp2r*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/64 + (qp2*qp2r*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/64 - (qp2*qp2r*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/64 - (qp2*qp2r*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/64 - (qp3*qp3r*sin(q2 + q3 + q4 - q5 - 2*q6))/32 + (qp3*qp3r*sin(q2 + q3 + q4 - q5 + 2*q6))/32 + (qp3*qp3r*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/64 + (qp3*qp3r*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/64 - (qp3*qp3r*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/64 - (qp3*qp3r*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/64 + (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/64 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/64 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/64 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/64 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/32 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/32 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/64 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/64 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/64 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/64 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 + (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/64 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/64 + (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/64 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/64 + (qp2*qp4*sin(q2 + q3 + q4 - 2*q5))/16 - (qp2*qp4*sin(q2 + q3 + q4 + 2*q5))/16 - (qp2*qp5*sin(q2 + q3 + q4 - 2*q5))/16 - (qp2*qp5*sin(q2 + q3 + q4 + 2*q5))/16 + (qp2*qp6*sin(q2 + q3 + q4 - q5))/8 + (qp3*qp4*sin(q2 + q3 + q4 - 2*q5))/16 - (qp3*qp4*sin(q2 + q3 + q4 + 2*q5))/16 + (qp2*qp5*sin(q2 + q3 + q4 - 2*q6))/16 + (qp2*qp5*sin(q2 + q3 + q4 + 2*q6))/16 - (qp3*qp5*sin(q2 + q3 + q4 - 2*q5))/16 - (qp3*qp5*sin(q2 + q3 + q4 + 2*q5))/16 + (qp3*qp6*sin(q2 + q3 + q4 - q5))/8 + (qp3*qp5*sin(q2 + q3 + q4 - 2*q6))/16 + (qp3*qp5*sin(q2 + q3 + q4 + 2*q6))/16 - (qp4*qp5*sin(q2 + q3 + q4 - 2*q5))/16 - (qp4*qp5*sin(q2 + q3 + q4 + 2*q5))/16 + (qp4*qp6*sin(q2 + q3 + q4 - q5))/8 + (qp4*qp5*sin(q2 + q3 + q4 - 2*q6))/16 + (qp4*qp5*sin(q2 + q3 + q4 + 2*q6))/16 + (qp5*qp6*sin(q2 + q3 + q4 - q5))/8 - (qp5*qp6*sin(q2 + q3 + q4 - 2*q6))/8 + (qp5*qp6*sin(q2 + q3 + q4 + 2*q6))/8 + (qp2*qp2r*sin(q2 + q3 + q4 - 2*q5))/32 - (qp2*qp2r*sin(q2 + q3 + q4 + 2*q5))/32 + (qp3*qp3r*sin(q2 + q3 + q4 - 2*q5))/32 - (qp3*qp3r*sin(q2 + q3 + q4 + 2*q5))/32 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 - (3*qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/32 - (3*qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/32 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 + (3*qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/32 - (3*qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/32 + (qp2*qp5*sin(q2 + q3 + q4))/8 + (qp3*qp5*sin(q2 + q3 + q4))/8 + (qp4*qp5*sin(q2 + q3 + q4))/8 + (qp1*qp5*sin(2*q5))/16 - (qp1*qp6*sin(2*q6))/16 - (3*qp1*qp4*sin(2*q2 + 2*q3 + 2*q4))/16 + (qp2*qp4*sin(q2 + q3 + q4 + q5 - 2*q6))/16 - (qp2*qp4*sin(q2 + q3 + q4 + q5 + 2*q6))/16 + (qp3*qp4*sin(q2 + q3 + q4 + q5 - 2*q6))/16 - (qp3*qp4*sin(q2 + q3 + q4 + q5 + 2*q6))/16 - (qp2*qp6*sin(q2 + q3 + q4 + q5 - 2*q6))/16 - (qp2*qp6*sin(q2 + q3 + q4 + q5 + 2*q6))/16 - (qp3*qp6*sin(q2 + q3 + q4 + q5 - 2*q6))/16 - (qp3*qp6*sin(q2 + q3 + q4 + q5 + 2*q6))/16 - (qp4*qp6*sin(q2 + q3 + q4 + q5 - 2*q6))/16 - (qp4*qp6*sin(q2 + q3 + q4 + q5 + 2*q6))/16 + (qp5*qp6*sin(q2 + q3 + q4 + q5 - 2*q6))/16 + (qp5*qp6*sin(q2 + q3 + q4 + q5 + 2*q6))/16 + (qp2*qp2r*sin(q2 + q3 + q4 + q5 - 2*q6))/32 - (qp2*qp2r*sin(q2 + q3 + q4 + q5 + 2*q6))/32 + (qp3*qp3r*sin(q2 + q3 + q4 + q5 - 2*q6))/32 - (qp3*qp3r*sin(q2 + q3 + q4 + q5 + 2*q6))/32 + (qp1*qp5*sin(2*q5 - 2*q6))/32 + (qp1*qp5*sin(2*q5 + 2*q6))/32 - (qp1*qp6*sin(2*q5 - 2*q6))/32 + (qp1*qp6*sin(2*q5 + 2*q6))/32 + (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/32 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/32 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 - (qp1*qp6*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 - (qp2*qp6*sin(q2 + q3 + q4 + q5))/8 - (qp3*qp6*sin(q2 + q3 + q4 + q5))/8 - (qp4*qp6*sin(q2 + q3 + q4 + q5))/8 + (qp5*qp6*sin(q2 + q3 + q4 + q5))/8;
    Yr(0, 1) = (qpp1r*sin(2*q6))/32 - (qpp1r*cos(2*q5)*sin(2*q6))/32 + (qp1*qp6*cos(2*q6))/16 - (qp1*qp6*cos(2*q5)*cos(2*q6))/16 + (qp1*qp5*sin(2*q5)*sin(2*q6))/16 + (qpp4*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (qpp5*cos(q2)*cos(q3)*cos(q5)*sin(q4))/8 - (qpp5*cos(q2)*cos(q4)*cos(q5)*sin(q3))/8 - (qpp5*cos(q3)*cos(q4)*cos(q5)*sin(q2))/8 + (qpp2r*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 + (qpp3r*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (3*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (3*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (3*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (3*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (3*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (3*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (qpp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5))/8 + (qpp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5))/8 + (qpp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5))/8 - (qpp4*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp4*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp4*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (qpp5*cos(q5)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp2r*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp2r*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp2r*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 - (qpp3r*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp3r*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp3r*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (3*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 + (3*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (qpp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/8 - (qpp5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/8 - (qp4^2*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp4^2*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp4^2*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 + (qp5^2*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 + (qp5^2*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 + (qp5^2*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 + (3*qpp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6))/32 + (qpp5*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/8 + (qpp5*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/8 + (qpp5*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/8 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(q2)*cos(q3)*sin(q4)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(q2)*cos(q4)*sin(q3)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(q3)*cos(q4)*sin(q2)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qp4^2*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qp5^2*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (3*qpp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/32 - (3*qpp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6))/32 - (3*qpp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6))/32 - (cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (sin(q2)*sin(q3)*sin(q4)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qpp1r*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(2*q6))/32 - (qpp1r*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(2*q6))/32 - (qpp1r*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q6))/32 + (qpp4*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/16 + (qpp2r*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/16 + (qpp3r*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/16 - (cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (3*qp1*qp6*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6))/16 + (qp2*qp5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 + (qp2*qp5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 + (qp2*qp5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 + (qp3*qp5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 + (qp3*qp5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 + (qp3*qp5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 + (qp4*qp5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 + (qp4*qp5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 + (qp4*qp5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 + (qp5*qp6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4))/4 + (qp5*qp6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4))/4 + (qp5*qp6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3))/4 - (3*qp1*qp4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/16 - (3*qp1*qp4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/16 - (3*qp1*qp4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/16 - (3*qp1*qp6*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4))/16 - (3*qp1*qp6*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4))/16 - (3*qp1*qp6*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3))/16 - (qp2*qp5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp3*qp5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp4*qp5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp4^2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/16 - (sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (3*qp1*qp4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/16 + (qp4^2*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/16 + (qp4^2*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/16 + (qp4^2*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/16 + (qpp4*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (qpp5*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/8 - (qpp5*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/8 - (qpp5*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/8 + (qpp2r*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 + (qpp3r*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (qpp1r*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*cos(q5))/8 + (qpp1r*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*cos(q5))/8 + (qpp1r*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*cos(q5))/8 - (qpp4*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp4*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp4*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (qpp5*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp2r*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp2r*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp2r*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 - (qpp3r*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp3r*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp3r*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 + (cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (qpp1r*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/8 + (qp1*qp4*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q5))/4 - (qp2*qp4*cos(q2)*cos(q3)*sin(q4)*sin(q5))/4 - (qp2*qp4*cos(q2)*cos(q4)*sin(q3)*sin(q5))/4 - (qp2*qp4*cos(q3)*cos(q4)*sin(q2)*sin(q5))/4 - (qp3*qp4*cos(q2)*cos(q3)*sin(q4)*sin(q5))/4 - (qp3*qp4*cos(q2)*cos(q4)*sin(q3)*sin(q5))/4 - (qp3*qp4*cos(q3)*cos(q4)*sin(q2)*sin(q5))/4 - (qp2*qp2r*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp2*qp2r*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp2*qp2r*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 - (qp3*qp3r*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp3*qp3r*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp3*qp3r*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 - (qp1*qp4*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/4 - (qp1*qp4*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q5))/4 - (qp1*qp4*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q5))/4 - (qp1*qp5*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5))/8 - (qp1*qp5*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5))/8 - (qp1*qp5*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5))/8 + (qp2*qp4*sin(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp3*qp4*sin(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp2*qp2r*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 + (qp3*qp3r*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qp4^2*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp4^2*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp4^2*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 + (qp5^2*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 + (qp5^2*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 + (qp5^2*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 + (qpp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q6))/32 - (qpp4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/16 - (qpp4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/16 - (qpp4*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/16 - (qpp2r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/16 - (qpp2r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/16 - (qpp2r*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/16 - (qpp3r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/16 - (qpp3r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/16 - (qpp3r*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/16 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qp1*qp5*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5))/8 - (qp5*qp6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4))/4 + (qp4^2*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qp5^2*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qp1*qp4*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5))/4 - (qp1*qp4*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5))/4 - (qp1*qp4*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5))/4 - (qp1*qp5*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*sin(q5))/8 - (qp1*qp5*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*sin(q5))/8 - (qp1*qp5*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(q5))/8 - (qp1*qp6*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/4 - (qp1*qp6*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*cos(q5))/4 - (qp1*qp6*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*cos(q5))/4 + (qp2*qp4*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp3*qp4*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp2*qp6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp2*qp6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/4 + (qp2*qp6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/4 + (qp3*qp6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp3*qp6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/4 + (qp3*qp6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/4 + (qp4*qp6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp4*qp6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/4 + (qp4*qp6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/4 - (qp5*qp6*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp2*qp2r*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 + (qp3*qp3r*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 + (qp1*qp5*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5))/8 + (qp1*qp6*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/4 + (qp1*qp6*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6))/16 - (qp2*qp4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/8 - (qp2*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 - (qp2*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 - (qp2*qp5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 - (qp3*qp4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/8 - (qp2*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/8 - (qp2*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/8 - (qp2*qp6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/8 - (qp3*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 - (qp3*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 - (qp3*qp5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 - (qp3*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/8 - (qp3*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/8 - (qp3*qp6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/8 - (qp4*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 - (qp4*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 - (qp4*qp5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 - (qp4*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/8 - (qp4*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/8 - (qp4*qp6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/8 - (qp2*qp2r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/16 - (qp3*qp3r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/16 - (qp1*qp4*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/16 - (qp1*qp4*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/16 - (qp1*qp4*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/16 - (qp1*qp5*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(2*q6))/16 - (qp1*qp6*cos(2*q2)*cos(2*q5)*cos(2*q6)*sin(2*q3)*sin(2*q4))/16 - (qp1*qp6*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q4))/16 - (qp1*qp6*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3))/16 + (qp2*qp4*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/8 + (qp2*qp4*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/8 + (qp2*qp4*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/8 + (qp2*qp5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp3*qp4*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/8 + (qp3*qp4*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/8 + (qp3*qp4*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/8 + (qp2*qp6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/8 + (qp3*qp5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp3*qp6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/8 + (qp4*qp5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp4*qp6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/8 + (qp2*qp2r*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/16 + (qp2*qp2r*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/16 + (qp2*qp2r*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/16 + (qp3*qp3r*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/16 + (qp3*qp3r*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/16 + (qp3*qp3r*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/16 + (qp1*qp4*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/16 + (qp1*qp5*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(2*q6))/16 + (qp1*qp5*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(2*q6))/16 + (qp1*qp5*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(2*q6))/16 + (qp1*qp4*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5))/4 - (qp2*qp4*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/4 - (qp2*qp4*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/4 - (qp2*qp4*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/4 - (qp3*qp4*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/4 - (qp3*qp4*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/4 - (qp3*qp4*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/4 - (qp2*qp6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/4 - (qp3*qp6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/4 - (qp4*qp6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/4 + (qp5*qp6*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/4 + (qp5*qp6*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/4 + (qp5*qp6*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/4 - (qp2*qp2r*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp2*qp2r*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp2*qp2r*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8 - (qp3*qp3r*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5))/8 - (qp3*qp3r*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/8 - (qp3*qp3r*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/8;
    Yr(0, 2) = (qpp1r*cos(2*q2 + 2*q3 + 2*q4))/16 - (qpp1r*cos(2*q5))/16 - qpp1r/16 + (qp4^2*sin(q2 + q3 + q4 - 2*q5))/16 - (qp4^2*sin(q2 + q3 + q4 + 2*q5))/16 + (sin(q2 + q3 + q4 - 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 - (sin(q2 + q3 + q4 + 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 - (sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qpp4*cos(q2 + q3 + q4 - 2*q5))/16 + (qpp4*cos(q2 + q3 + q4 + 2*q5))/16 - (qpp2r*cos(q2 + q3 + q4 - 2*q5))/16 + (qpp2r*cos(q2 + q3 + q4 + 2*q5))/16 - (qpp3r*cos(q2 + q3 + q4 - 2*q5))/16 + (qpp3r*cos(q2 + q3 + q4 + 2*q5))/16 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 + (qp2*qp4*sin(q2 + q3 + q4 - 2*q5))/8 - (qp2*qp4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qp5*sin(q2 + q3 + q4 - 2*q5))/8 - (qp2*qp5*sin(q2 + q3 + q4 + 2*q5))/8 + (qp3*qp4*sin(q2 + q3 + q4 - 2*q5))/8 - (qp3*qp4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qp5*sin(q2 + q3 + q4 - 2*q5))/8 - (qp3*qp5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qp5*sin(q2 + q3 + q4 - 2*q5))/8 - (qp4*qp5*sin(q2 + q3 + q4 + 2*q5))/8 + (qp2*qp2r*sin(q2 + q3 + q4 - 2*q5))/16 - (qp2*qp2r*sin(q2 + q3 + q4 + 2*q5))/16 + (qp3*qp3r*sin(q2 + q3 + q4 - 2*q5))/16 - (qp3*qp3r*sin(q2 + q3 + q4 + 2*q5))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 - (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (qp2*qp5*sin(q2 + q3 + q4))/4 - (qp3*qp5*sin(q2 + q3 + q4))/4 - (qp4*qp5*sin(q2 + q3 + q4))/4 + (qp1*qp5*sin(2*q5))/8 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4))/8;
    Yr(0, 3) = (qp1*qp6*sin(2*q5)*sin(q6))/16 - (qp1*qp5*cos(2*q5)*cos(q6))/8 - (qpp1r*sin(2*q5)*cos(q6))/16 + (qpp6*cos(q2)*cos(q3)*cos(q4)*sin(q6))/8 - (qpp6*cos(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp6*cos(q3)*sin(q2)*sin(q4)*sin(q6))/8 - (qpp6*cos(q4)*sin(q2)*sin(q3)*sin(q6))/8 + (qp6^2*cos(q2)*cos(q3)*cos(q4)*cos(q6))/8 - (qp6^2*cos(q2)*cos(q6)*sin(q3)*sin(q4))/8 - (qp6^2*cos(q3)*cos(q6)*sin(q2)*sin(q4))/8 - (qp6^2*cos(q4)*cos(q6)*sin(q2)*sin(q3))/8 + (qpp4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/8 + (qpp6*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 + (qpp6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 + (qpp6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 + (qpp2r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/8 + (qpp3r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/8 - (cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qpp4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8 - (qpp4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/8 + (qpp5*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6))/8 + (qpp5*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6))/8 + (qpp5*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6))/8 - (qpp6*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp2r*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp2r*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8 - (qpp2r*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/8 - (qpp3r*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp3r*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8 - (qpp3r*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/8 + (sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 + (sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qpp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/8 - (qpp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6))/8 - (qpp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6))/8 - (qpp5*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6))/8 + (qpp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/8 + (qpp4*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qpp4*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qpp4*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qpp2r*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qpp2r*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qpp2r*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qpp3r*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qpp3r*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qpp3r*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 - (qp4^2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 - (qp4^2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 - (qp4^2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 + (qp5^2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 + (qp5^2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 + (qp5^2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 - (qp6^2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 - (qp6^2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 - (qp6^2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 + (qpp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/16 - (qpp4*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp2r*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp3r*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qp4^2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp5^2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp6^2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/16 - (qpp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/16 - (qpp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/16 - (cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qp2*qp6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp2*qp6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp2*qp6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp3*qp6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp3*qp6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp3*qp6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp4*qp6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp4*qp6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp4*qp6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 + (qp4^2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6))/8 + (cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qp2*qp6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp3*qp6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp4*qp6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp4^2*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/8 - (qp4^2*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4))/8 - (qp4^2*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3))/8 - (cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qp1*qp4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/8 - (qp1*qp4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/8 - (qp1*qp4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/8 - (qp1*qp5*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*cos(q6))/8 - (qp1*qp5*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*cos(q6))/8 - (qp1*qp5*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*cos(q6))/8 - (qp1*qp6*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/16 + (qp2*qp5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp2*qp6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp3*qp5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp3*qp6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp4*qp5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp4*qp6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp1*qp4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/8 + (qp1*qp6*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/16 + (qp1*qp6*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/16 + (qp1*qp6*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/16 + (qp2*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/4 + (qp3*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/4 + (qp4*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/4 - (qp2*qp4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/4 - (qp2*qp4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/4 - (qp2*qp4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/4 - (qp3*qp4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/4 - (qp3*qp4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/4 - (qp3*qp4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/4 - (qp2*qp6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/4 - (qp2*qp6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/4 - (qp2*qp6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/4 - (qp3*qp6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/4 - (qp3*qp6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/4 - (qp3*qp6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/4 - (qp4*qp6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/4 - (qp4*qp6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/4 - (qp4*qp6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/4 - (qp2*qp2r*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 - (qp2*qp2r*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 - (qp2*qp2r*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 - (qp3*qp3r*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 - (qp3*qp3r*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 - (qp3*qp3r*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 - (qp1*qp4*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6))/4 - (qp1*qp5*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/8 - (qp1*qp5*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*sin(q6))/8 - (qp1*qp5*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*sin(q6))/8 - (qp1*qp6*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/8 - (qp1*qp6*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5))/8 - (qp1*qp6*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5))/8 + (qp2*qp4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 + (qp3*qp4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 + (qp2*qp2r*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp3*qp3r*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp1*qp4*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/4 + (qp1*qp4*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6))/4 + (qp1*qp4*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6))/4 + (qp1*qp5*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/8 + (qp1*qp6*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/8 + (qp2*qp4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6))/4 + (qp3*qp4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6))/4 + (qp2*qp2r*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6))/8 + (qp3*qp3r*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6))/8 + (qp1*qp5*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(q6))/8 - (qp2*qp4*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/4 - (qp2*qp4*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4))/4 - (qp2*qp4*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3))/4 - (qp2*qp5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/4 - (qp2*qp5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/4 - (qp2*qp5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/4 - (qp3*qp4*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/4 - (qp3*qp4*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4))/4 - (qp3*qp4*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3))/4 - (qp2*qp6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp2*qp6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp2*qp6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp3*qp5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/4 - (qp3*qp5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/4 - (qp3*qp5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/4 - (qp3*qp6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp3*qp6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp3*qp6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp4*qp5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/4 - (qp4*qp5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/4 - (qp4*qp5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/4 - (qp4*qp6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp4*qp6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp4*qp6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp2*qp2r*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/8 - (qp2*qp2r*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4))/8 - (qp2*qp2r*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3))/8 - (qp3*qp3r*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/8 - (qp3*qp3r*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4))/8 - (qp3*qp3r*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3))/8;
    Yr(0, 4) = (qpp6*cos(q2)*cos(q6)*sin(q3)*sin(q4))/8 - (qp1*qp5*cos(2*q5)*sin(q6))/8 - (qp1*qp6*sin(2*q5)*cos(q6))/16 - (qpp6*cos(q2)*cos(q3)*cos(q4)*cos(q6))/8 - (qpp1r*sin(2*q5)*sin(q6))/16 + (qpp6*cos(q3)*cos(q6)*sin(q2)*sin(q4))/8 + (qpp6*cos(q4)*cos(q6)*sin(q2)*sin(q3))/8 + (qp6^2*cos(q2)*cos(q3)*cos(q4)*sin(q6))/8 - (qp6^2*cos(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp6^2*cos(q3)*sin(q2)*sin(q4)*sin(q6))/8 - (qp6^2*cos(q4)*sin(q2)*sin(q3)*sin(q6))/8 + (qpp4*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/8 + (qpp4*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/8 + (qpp4*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/8 - (qpp5*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5))/8 - (qpp5*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5))/8 - (qpp5*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5))/8 + (qpp6*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6))/8 + (qpp6*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/8 + (qpp6*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/8 + (qpp2r*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/8 + (qpp2r*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/8 + (qpp2r*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/8 + (qpp3r*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/8 + (qpp3r*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/8 + (qpp3r*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/8 - (cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 + (qpp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/8 + (qpp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5))/8 + (qpp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5))/8 + (qpp5*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp6*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 + (sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qpp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/8 + (qp4^2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 + (qp4^2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 + (qp4^2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 - (qp5^2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 - (qp5^2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 - (qp5^2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 + (qp6^2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 + (qp6^2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 + (qp6^2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 + (qpp4*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 + (qpp4*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 + (qpp4*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 + (qpp2r*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 + (qpp2r*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 + (qpp2r*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 + (qpp3r*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 + (qpp3r*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 + (qpp3r*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 + (cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qp4^2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp5^2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp6^2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qpp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/16 - (qpp4*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp2r*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp3r*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qp2*qp6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp2*qp6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp2*qp6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp3*qp6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp3*qp6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp3*qp6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp4*qp6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp4*qp6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp4*qp6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 - (qpp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/16 - (qpp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/16 - (qpp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/16 - (cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qp2*qp6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp3*qp6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp4*qp6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp4^2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6))/8 + (cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qp4^2*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp4^2*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6))/8 - (qp4^2*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6))/8 - (qpp4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/8 - (qpp2r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/8 - (qpp3r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/8 - (cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qp1*qp4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/8 - (qp1*qp4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/8 - (qp1*qp4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/8 - (qp1*qp5*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(q6))/8 - (qp1*qp5*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(q6))/8 - (qp1*qp5*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(q6))/8 - (qp1*qp6*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/16 - (qp1*qp6*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/16 - (qp1*qp6*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/16 + (qp2*qp5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 + (qp3*qp5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 + (qp4*qp5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 + (qp1*qp4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/8 + (qp2*qp4*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/4 + (qp2*qp4*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/4 + (qp2*qp4*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/4 + (qp3*qp4*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/4 + (qp3*qp4*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/4 + (qp3*qp4*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/4 + (qp2*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/4 + (qp3*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/4 + (qp4*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/4 + (qp2*qp2r*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 + (qp2*qp2r*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 + (qp2*qp2r*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 + (qp3*qp3r*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4))/8 + (qp3*qp3r*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/8 + (qp3*qp3r*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/8 + (qp1*qp4*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5))/4 + (qp1*qp5*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/8 + (qp1*qp5*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*cos(q6))/8 + (qp1*qp5*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*cos(q6))/8 - (qp2*qp4*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp3*qp4*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp2*qp6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/4 - (qp2*qp6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/4 - (qp2*qp6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/4 - (qp3*qp6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/4 - (qp3*qp6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/4 - (qp3*qp6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/4 - (qp4*qp6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/4 - (qp4*qp6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/4 - (qp4*qp6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/4 - (qp2*qp2r*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp3*qp3r*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp1*qp4*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/4 - (qp1*qp4*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5))/4 - (qp1*qp4*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5))/4 - (qp1*qp5*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/8 - (qp1*qp6*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/8 - (qp1*qp6*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6))/8 - (qp1*qp6*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6))/8 + (qp1*qp6*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/8 + (qp2*qp4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6))/4 + (qp3*qp4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6))/4 + (qp2*qp6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp2*qp6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp2*qp6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp3*qp6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp3*qp6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp3*qp6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp4*qp6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp4*qp6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp4*qp6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp2*qp2r*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6))/8 + (qp3*qp3r*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6))/8 + (qp1*qp5*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(q6))/8 + (qp1*qp6*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/16 - (qp2*qp4*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/4 - (qp2*qp4*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6))/4 - (qp2*qp4*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6))/4 - (qp2*qp5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/4 - (qp2*qp5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/4 - (qp2*qp5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/4 - (qp3*qp4*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/4 - (qp3*qp4*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6))/4 - (qp3*qp4*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6))/4 - (qp2*qp6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp3*qp5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/4 - (qp3*qp5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/4 - (qp3*qp5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/4 - (qp3*qp6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp4*qp5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/4 - (qp4*qp5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/4 - (qp4*qp5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/4 - (qp4*qp6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qp2*qp2r*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp2*qp2r*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6))/8 - (qp2*qp2r*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6))/8 - (qp3*qp3r*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp3*qp3r*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6))/8 - (qp3*qp3r*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6))/8;
    Yr(0, 5) = (qp4^2*cos(q2 + q3 + q4 - 2*q5))/16 - (qpp1r*sin(2*q5))/16 + (qp4^2*cos(q2 + q3 + q4 + 2*q5))/16 + (cos(q2 + q3 + q4 - 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (cos(q2 + q3 + q4 + 2*q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (qp4^2*cos(q2 + q3 + q4))/8 - (cos(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 + (cos(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/16 - (cos(2*q2 + 2*q3 + 2*q4 - 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (cos(2*q2 + 2*q3 + 2*q4 + 2*q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/16 + (cos(q2 + q3 + q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qpp4*sin(q2 + q3 + q4 - 2*q5))/16 + (qpp4*sin(q2 + q3 + q4 + 2*q5))/16 + (qpp2r*sin(q2 + q3 + q4 - 2*q5))/16 + (qpp2r*sin(q2 + q3 + q4 + 2*q5))/16 + (qpp3r*sin(q2 + q3 + q4 - 2*q5))/16 + (qpp3r*sin(q2 + q3 + q4 + 2*q5))/16 - (qpp1r*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 + (qpp1r*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 + (qpp4*sin(q2 + q3 + q4))/8 + (qpp2r*sin(q2 + q3 + q4))/8 + (qpp3r*sin(q2 + q3 + q4))/8 + (qp2*qp4*cos(q2 + q3 + q4 - 2*q5))/8 + (qp2*qp4*cos(q2 + q3 + q4 + 2*q5))/8 - (qp2*qp5*cos(q2 + q3 + q4 - 2*q5))/8 + (qp2*qp5*cos(q2 + q3 + q4 + 2*q5))/8 + (qp3*qp4*cos(q2 + q3 + q4 - 2*q5))/8 + (qp3*qp4*cos(q2 + q3 + q4 + 2*q5))/8 - (qp3*qp5*cos(q2 + q3 + q4 - 2*q5))/8 + (qp3*qp5*cos(q2 + q3 + q4 + 2*q5))/8 - (qp4*qp5*cos(q2 + q3 + q4 - 2*q5))/8 + (qp4*qp5*cos(q2 + q3 + q4 + 2*q5))/8 + (qp2*qp2r*cos(q2 + q3 + q4 - 2*q5))/16 + (qp2*qp2r*cos(q2 + q3 + q4 + 2*q5))/16 + (qp3*qp3r*cos(q2 + q3 + q4 - 2*q5))/16 + (qp3*qp3r*cos(q2 + q3 + q4 + 2*q5))/16 - (qp1*qp4*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qp4*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp1*qp5*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qp5*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp2*qp4*cos(q2 + q3 + q4))/4 + (qp3*qp4*cos(q2 + q3 + q4))/4 + (qp2*qp2r*cos(q2 + q3 + q4))/8 + (qp3*qp3r*cos(q2 + q3 + q4))/8 - (qp1*qp5*cos(2*q5))/8;
    Yr(0, 6) = (qpp1r*cos(2*q2 + 2*q3 + 2*q4))/8 + (qpp1r*cos(2*q2 + 2*q3))/8 - (sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (sin(2*q2 + 2*q3 + 2*q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (sin(2*q2 + 2*q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (sin(2*q2 + 2*q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4))/4;
    Yr(0, 7) = (qp5^2*cos(q2 + q3 + q4 - q5))/16 - (qp4^2*cos(q2 + q3 + q4 - q5))/16 - (cos(q2 + q3 + q4 - q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (cos(2*q2 + 2*q3 + 2*q4 - q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 + (cos(2*q2 + 2*q3 + 2*q4 - q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 + (qpp4*sin(q2 + q3 + q4 + q5))/16 - (qpp5*sin(q2 + q3 + q4 + q5))/16 + (qpp2r*sin(q2 + q3 + q4 + q5))/16 + (qpp3r*sin(q2 + q3 + q4 + q5))/16 + (qpp1r*sin(2*q2 + 2*q3 + 2*q4 + q5))/16 + (qp4^2*cos(q2 + q3 + q4 + q5))/16 - (qp5^2*cos(q2 + q3 + q4 + q5))/16 - (qpp4*sin(q2 + q3 + q4 - q5))/16 - (qpp5*sin(q2 + q3 + q4 - q5))/16 - (qpp2r*sin(q2 + q3 + q4 - q5))/16 - (qpp3r*sin(q2 + q3 + q4 - q5))/16 + (cos(q2 + q3 + q4 + q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (qpp1r*sin(2*q2 + 2*q3 + 2*q4 - q5))/16 + (cos(2*q2 + 2*q3 + 2*q4 + q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 + (cos(2*q2 + 2*q3 + 2*q4 + q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qp2*qp4*cos(q2 + q3 + q4 - q5))/8 - (qp3*qp4*cos(q2 + q3 + q4 - q5))/8 - (qp2*qp2r*cos(q2 + q3 + q4 - q5))/16 - (qp3*qp3r*cos(q2 + q3 + q4 - q5))/16 + (qp1*qp4*cos(2*q2 + 2*q3 + 2*q4 - q5))/8 - (qp1*qp5*cos(2*q2 + 2*q3 + 2*q4 - q5))/16 + (qp2*qp4*cos(q2 + q3 + q4 + q5))/8 + (qp3*qp4*cos(q2 + q3 + q4 + q5))/8 + (qp2*qp2r*cos(q2 + q3 + q4 + q5))/16 + (qp3*qp3r*cos(q2 + q3 + q4 + q5))/16 + (qp1*qp4*cos(2*q2 + 2*q3 + 2*q4 + q5))/8 + (qp1*qp5*cos(2*q2 + 2*q3 + 2*q4 + q5))/16;
    Yr(0, 8) = (qp4^2*sin(q2 + q3 + q4 - q5))/16 - (qp5^2*sin(q2 + q3 + q4 - q5))/16 + (sin(q2 + q3 + q4 - q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 - (qpp4*cos(q2 + q3 + q4 + q5))/16 + (qpp5*cos(q2 + q3 + q4 + q5))/16 - (qpp2r*cos(q2 + q3 + q4 + q5))/16 - (qpp3r*cos(q2 + q3 + q4 + q5))/16 - (sin(2*q2 + 2*q3 + 2*q4 - q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 - (sin(2*q2 + 2*q3 + 2*q4 - q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 - (qpp1r*cos(2*q2 + 2*q3 + 2*q4 + q5))/16 - (qpp4*cos(q2 + q3 + q4 - q5))/16 - (qpp5*cos(q2 + q3 + q4 - q5))/16 - (qpp2r*cos(q2 + q3 + q4 - q5))/16 - (qpp3r*cos(q2 + q3 + q4 - q5))/16 + (qp4^2*sin(q2 + q3 + q4 + q5))/16 - (qp5^2*sin(q2 + q3 + q4 + q5))/16 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4 - q5))/16 + (sin(q2 + q3 + q4 + q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/8 + (sin(2*q2 + 2*q3 + 2*q4 + q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/8 + (sin(2*q2 + 2*q3 + 2*q4 + q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/8 + (qp2*qp4*sin(q2 + q3 + q4 - q5))/8 + (qp3*qp4*sin(q2 + q3 + q4 - q5))/8 + (qp2*qp2r*sin(q2 + q3 + q4 - q5))/16 + (qp3*qp3r*sin(q2 + q3 + q4 - q5))/16 - (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 - q5))/8 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 - q5))/16 + (qp2*qp4*sin(q2 + q3 + q4 + q5))/8 + (qp3*qp4*sin(q2 + q3 + q4 + q5))/8 + (qp2*qp2r*sin(q2 + q3 + q4 + q5))/16 + (qp3*qp3r*sin(q2 + q3 + q4 + q5))/16 + (qp1*qp4*sin(2*q2 + 2*q3 + 2*q4 + q5))/8 + (qp1*qp5*sin(2*q2 + 2*q3 + 2*q4 + q5))/16;
    Yr(0, 9) = (qpp1r*sin(2*q2 + 2*q3 + 2*q4))/8 + (qpp1r*sin(2*q2 + 2*q3))/8 + (cos(2*q2 + 2*q3 + 2*q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2 + 2*q3 + 2*q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (cos(2*q2 + 2*q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2 + 2*q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (qp1*qp4*cos(2*q2 + 2*q3 + 2*q4))/4;

    Yr(0, 10) = (qpp1r*sin(2*q2 + 2*q3 + q4 + q5))/8 - (qp5^2*cos(q2 + q3 + q5))/8 + (qpp5*sin(q2 + q3 - q5))/8 + (qpp2r*sin(q2 + q3 - q5))/8 + (qpp3r*sin(q2 + q3 - q5))/8 + (cos(q2 + q3 + q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 + (qpp1r*sin(q4 + q5))/8 - (qpp1r*sin(2*q2 + 2*q3 + q4 - q5))/8 + (cos(2*q2 + 2*q3 + q4 + q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 + (cos(2*q2 + 2*q3 + q4 + q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qp5^2*cos(q2 + q3 - q5))/8 + (cos(q2 + q3 - q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/4 - (qpp1r*sin(q4 - q5))/8 - (cos(2*q2 + 2*q3 + q4 - q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/4 - (cos(2*q2 + 2*q3 + q4 - q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qpp5*sin(q2 + q3 + q5))/8 + (qpp2r*sin(q2 + q3 + q5))/8 + (qpp3r*sin(q2 + q3 + q5))/8 - (qp1*qp4*cos(q4 - q5))/8 + (qp1*qp5*cos(q4 - q5))/8 + (qp2*qp2r*cos(q2 + q3 + q5))/8 + (qp3*qp3r*cos(q2 + q3 + q5))/8 + (qp1*qp4*cos(2*q2 + 2*q3 + q4 + q5))/8 + (qp1*qp5*cos(2*q2 + 2*q3 + q4 + q5))/8 + (qp2*qp2r*cos(q2 + q3 - q5))/8 + (qp3*qp3r*cos(q2 + q3 - q5))/8 + (qp1*qp4*cos(q4 + q5))/8 + (qp1*qp5*cos(q4 + q5))/8 - (qp1*qp4*cos(2*q2 + 2*q3 + q4 - q5))/8 + (qp1*qp5*cos(2*q2 + 2*q3 + q4 - q5))/8;
    Yr(0, 11) = (cos(2*q2 + 2*q3 + q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 + (cos(2*q2 + 2*q3 + q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + (qpp1r*sin(q4))/4 + (qpp1r*sin(2*q2 + 2*q3 + q4))/4 + (qp1*qp4*cos(q4))/4 + (qp1*qp4*cos(2*q2 + 2*q3 + q4))/4;
    Yr(0, 12) = (qpp1r*cos(2*q2))/4 + (qpp1r*cos(2*q2 + 2*q3))/4 - (sin(2*q2)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 - (sin(2*q2 + 2*q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 - (sin(2*q2 + 2*q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2;
    Yr(0, 13) = (cos(2*q2 + q3 + q4 + q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 - (qpp1r*sin(2*q2 + q3 + q4 - q5))/4 + (cos(2*q2 + q3 + q4 + q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qp5^2*cos(q2 - q5))/4 - (qpp1r*sin(q3 + q4 - q5))/4 + (cos(q3 + q4 + q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (qpp5*sin(q2 + q5))/4 + (qpp2r*sin(q2 + q5))/4 - (cos(2*q2 + q3 + q4 - q5)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 - (cos(2*q2 + q3 + q4 - q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 - (cos(q3 + q4 - q5)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/4 + (qpp1r*sin(2*q2 + q3 + q4 + q5))/4 - (qp5^2*cos(q2 + q5))/4 + (qpp5*sin(q2 - q5))/4 + (qpp2r*sin(q2 - q5))/4 + (qpp1r*sin(q3 + q4 + q5))/4 + (qp1*qp4*cos(2*q2 + q3 + q4 + q5))/4 + (qp1*qp5*cos(2*q2 + q3 + q4 + q5))/4 + (qp2*qp2r*cos(q2 - q5))/4 + (qp1*qp4*cos(q3 + q4 + q5))/4 + (qp1*qp5*cos(q3 + q4 + q5))/4 - (qp1*qp4*cos(2*q2 + q3 + q4 - q5))/4 + (qp1*qp5*cos(2*q2 + q3 + q4 - q5))/4 - (qp1*qp4*cos(q3 + q4 - q5))/4 + (qp1*qp5*cos(q3 + q4 - q5))/4 + (qp2*qp2r*cos(q2 + q5))/4;
    Yr(0, 14) = (qpp1r*sin(2*q2))/4 + (qpp1r*sin(2*q2 + 2*q3))/4 + (cos(2*q2)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 + (cos(2*q2 + 2*q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2))/2 + (cos(2*q2 + 2*q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2;
    Yr(0, 15) = (qpp1r*sin(2*q2 + q3 + q4))/2 + (qpp1r*sin(q3 + q4))/2 + cos(2*q2 + q3 + q4)*((qp1*qp2r)/2 + (qp2*qp1r)/2) + (cos(2*q2 + q3 + q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + (cos(q3 + q4)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + (qp1*qp4*cos(2*q2 + q3 + q4))/2 + (qp1*qp4*cos(q3 + q4))/2;
    Yr(0, 16) = (qpp1r*cos(q3))/2 - sin(2*q2 + q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2) - (sin(2*q2 + q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 - (sin(q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + (qpp1r*cos(2*q2 + q3))/2;
    Yr(0, 17) = (cos(q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + cos(2*q2 + q3)*((qp1*qp2r)/2 + (qp2*qp1r)/2) + (cos(2*q2 + q3)*((qp1*qp3r)/2 + (qp3*qp1r)/2))/2 + (qpp1r*sin(q3))/2 + (qpp1r*sin(2*q2 + q3))/2;
    Yr(0, 18) = qpp1r*cos(q2)^2 - 2*cos(q2)*sin(q2)*((qp1*qp2r)/2 + (qp2*qp1r)/2);
    Yr(0, 19) = (qpp1r*sin(2*q2))/2 + cos(2*q2)*((qp1*qp2r)/2 + (qp2*qp1r)/2);

    Yr(0, 20) = qpp1r*cos(q5) - (qp5^2*sin(q2 + q3 + q4 - q5))/4 - (sin(q2 + q3 + q4 - q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/2 - (qpp4*cos(q2 + q3 + q4 + q5))/4 - (qpp5*cos(q2 + q3 + q4 + q5))/4 - (qpp2r*cos(q2 + q3 + q4 + q5))/4 - (qpp3r*cos(q2 + q3 + q4 + q5))/4 - (qp4^2*sin(q2 + q3 + q4 - q5))/4 + (qpp4*cos(q2 + q3 + q4 - q5))/4 - (qpp5*cos(q2 + q3 + q4 - q5))/4 + (qpp2r*cos(q2 + q3 + q4 - q5))/4 + (qpp3r*cos(q2 + q3 + q4 - q5))/4 + (qp4^2*sin(q2 + q3 + q4 + q5))/4 + (qp5^2*sin(q2 + q3 + q4 + q5))/4 + (sin(q2 + q3 + q4 + q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2))/2 - qp1*qp5*sin(q5) - (qp2*qp4*sin(q2 + q3 + q4 - q5))/2 + (qp2*qp5*sin(q2 + q3 + q4 - q5))/2 - (qp3*qp4*sin(q2 + q3 + q4 - q5))/2 + (qp3*qp5*sin(q2 + q3 + q4 - q5))/2 + (qp4*qp5*sin(q2 + q3 + q4 - q5))/2 - (qp2*qp2r*sin(q2 + q3 + q4 - q5))/4 - (qp3*qp3r*sin(q2 + q3 + q4 - q5))/4 + (qp2*qp4*sin(q2 + q3 + q4 + q5))/2 + (qp2*qp5*sin(q2 + q3 + q4 + q5))/2 + (qp3*qp4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qp5*sin(q2 + q3 + q4 + q5))/2 + (qp4*qp5*sin(q2 + q3 + q4 + q5))/2 + (qp2*qp2r*sin(q2 + q3 + q4 + q5))/4 + (qp3*qp3r*sin(q2 + q3 + q4 + q5))/4;
    Yr(0, 21) = qpp1r;
    Yr(0, 22) = qpp4*cos(q2 + q3 + q4) - 2*sin(q2 + q3 + q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2) - qp4^2*sin(q2 + q3 + q4) + qpp2r*cos(q2 + q3 + q4) + qpp3r*cos(q2 + q3 + q4) - 2*qp2*qp4*sin(q2 + q3 + q4) - 2*qp3*qp4*sin(q2 + q3 + q4) - qp2*qp2r*sin(q2 + q3 + q4) - qp3*qp3r*sin(q2 + q3 + q4);
    Yr(0, 23) = qp4^2*cos(q2 + q3 + q4) + 2*cos(q2 + q3 + q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + qpp4*sin(q2 + q3 + q4) + qpp2r*sin(q2 + q3 + q4) + qpp3r*sin(q2 + q3 + q4) + 2*qp2*qp4*cos(q2 + q3 + q4) + 2*qp3*qp4*cos(q2 + q3 + q4) + qp2*qp2r*cos(q2 + q3 + q4) + qp3*qp3r*cos(q2 + q3 + q4);
    Yr(0, 24) = qpp2r*cos(q2 + q3) + qpp3r*cos(q2 + q3) - 2*sin(q2 + q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2) - qp2*qp2r*sin(q2 + q3) - qp3*qp3r*sin(q2 + q3);
    Yr(0, 25) = qpp2r*sin(q2 + q3) + qpp3r*sin(q2 + q3) + 2*cos(q2 + q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + qp2*qp2r*cos(q2 + q3) + qp3*qp3r*cos(q2 + q3);
    Yr(0, 26) = qpp2r*cos(q2) - qp2*qp2r*sin(q2);
    Yr(0, 27) = qpp2r*sin(q2) + qp2*qp2r*cos(q2);
    Yr(0, 28) = cos(q2)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + qpp3r*sin(q2);
    Yr(0, 29) = qpp4*cos(q2 + q3) - qp2*qp4*sin(q2 + q3) - qp3*qp4*sin(q2 + q3);

    Yr(0, 30) = qpp5*cos(q2 + q3 + q4);
    Yr(0, 31) = -(qpp6*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;
    Yr(0, 32) = qp5*sin(q2 + q3 + q4)*(qp2 + qp3 + qp4);
    Yr(0, 33) = (qp6*(sin(q2 + q3 + q4 + q5) - sin(q2 + q3 + q4 - q5))*(qp2 + qp3 + qp4))/2;
    Yr(0, 34) = (qp5*qp6*(sin(q2 + q3 + q4 + q5) + sin(q2 + q3 + q4 - q5)))/2;
    Yr(0, 35) = cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q5)*sin(q1) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    Yr(0, 36) = sin(q1 + q2 + q3 + q4)/2 + sin(q2 - q1 + q3 + q4)/2;
    Yr(0, 37) = cos(q2 + q3)*cos(q1);
    Yr(0, 38) = sin(q2 + q3)*cos(q1);
    Yr(0, 39) = cos(q1)*cos(q2);

    Yr(0, 40) = cos(q1)*cos(q5) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);
    Yr(0, 41) = cos(q1);
    Yr(0, 42) = cos(q2 - q1 + q3 + q4)/2 - cos(q1 + q2 + q3 + q4)/2;
    Yr(0, 43) = cos(q2 + q3)*sin(q1);
    Yr(0, 44) = sin(q2 + q3)*sin(q1);
    Yr(0, 45) = cos(q2)*sin(q1);
    Yr(0, 46) = sin(q1);
    Yr(0, 47) = 0;
    Yr(0, 48) = 0;
    Yr(0, 49) = 0;

    Yr(0, 50) = 0;
    Yr(0, 51) = 0;
    Yr(0, 52) = 0;
    Yr(0, 53) = 0;
    Yr(0, 54) = 0;
    Yr(0, 55) = 0;
    Yr(0, 56) = 0;
    Yr(0, 57) = 0;
    Yr(0, 58) = 0;
    Yr(0, 59) = 0;
    Yr(0, 60) = 0;

    //////////////////////////////////////////////////////////////////////////////////////////////

    Yr(1, 0) = (qpp4*cos(q5)^2*cos(q6)^2)/4 - (qpp2r*cos(q6)^2)/4 - (qpp3r*cos(q6)^2)/4 - (qpp4*cos(q6)^2)/4 + (qpp2r*cos(q5)^2*cos(q6)^2)/4 + (qpp3r*cos(q5)^2*cos(q6)^2)/4 + (qp2*qp6*sin(2*q6))/4 + (qp3*qp6*sin(2*q6))/4 + (qp4*qp6*sin(2*q6))/4 + (qp5*qp6*cos(q6)^2*sin(q5))/2 + (qpp5*cos(q6)*sin(q5)*sin(q6))/4 + (qp5^2*cos(q5)*cos(q6)*sin(q6))/4 - (qp2*qp5*cos(q5)*cos(q6)^2*sin(q5))/2 - (qp3*qp5*cos(q5)*cos(q6)^2*sin(q5))/2 - (qp2*qp6*cos(q5)^2*cos(q6)*sin(q6))/2 - (qp4*qp5*cos(q5)*cos(q6)^2*sin(q5))/2 - (qp3*qp6*cos(q5)^2*cos(q6)*sin(q6))/2 - (qp4*qp6*cos(q5)^2*cos(q6)*sin(q6))/2 + (qp1*qp1r*cos(q2)*cos(q6)^2*sin(q2))/4 + (qp1*qp1r*cos(q3)*cos(q6)^2*sin(q3))/4 + (qp1*qp1r*cos(q4)*cos(q6)^2*sin(q4))/4 - (qp1*qp1r*cos(q5)*cos(q6)*sin(q6))/4 + (qp1*qp1r*cos(q2)^2*cos(q5)*cos(q6)*sin(q6))/2 + (qp1*qp1r*cos(q3)^2*cos(q5)*cos(q6)*sin(q6))/2 + (qp1*qp1r*cos(q4)^2*cos(q5)*cos(q6)*sin(q6))/2 - (qp1*qp1r*cos(q2)*cos(q3)^2*cos(q6)^2*sin(q2))/2 - (qp1*qp1r*cos(q2)*cos(q4)^2*cos(q6)^2*sin(q2))/2 - (qp1*qp1r*cos(q2)^2*cos(q3)*cos(q6)^2*sin(q3))/2 + (qp1*qp1r*cos(q2)*cos(q5)^2*cos(q6)^2*sin(q2))/4 - (qp1*qp1r*cos(q3)*cos(q4)^2*cos(q6)^2*sin(q3))/2 - (qp1*qp1r*cos(q2)^2*cos(q4)*cos(q6)^2*sin(q4))/2 + (qp1*qp1r*cos(q3)*cos(q5)^2*cos(q6)^2*sin(q3))/4 - (qp1*qp1r*cos(q3)^2*cos(q4)*cos(q6)^2*sin(q4))/2 + (qp1*qp1r*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q4))/4 + (qpp1r*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5)*sin(q6))/4 + (qpp1r*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5)*sin(q6))/4 + (qpp1r*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5)*sin(q6))/4 - (qp1*qp5*cos(q2)*cos(q3)*cos(q5)^2*cos(q6)^2*sin(q4))/2 - (qp1*qp5*cos(q2)*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q3))/2 - (qp1*qp5*cos(q3)*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q2))/2 - qp1*qp1r*cos(q2)^2*cos(q3)^2*cos(q5)*cos(q6)*sin(q6) - qp1*qp1r*cos(q2)^2*cos(q4)^2*cos(q5)*cos(q6)*sin(q6) - qp1*qp1r*cos(q3)^2*cos(q4)^2*cos(q5)*cos(q6)*sin(q6) - (qpp1r*cos(q2)*cos(q3)*cos(q5)*cos(q6)^2*sin(q4)*sin(q5))/4 - (qpp1r*cos(q2)*cos(q4)*cos(q5)*cos(q6)^2*sin(q3)*sin(q5))/4 - (qpp1r*cos(q3)*cos(q4)*cos(q5)*cos(q6)^2*sin(q2)*sin(q5))/4 + (qp1*qp5*cos(q5)^2*cos(q6)^2*sin(q2)*sin(q3)*sin(q4))/2 + (qpp1r*cos(q5)*cos(q6)^2*sin(q2)*sin(q3)*sin(q4)*sin(q5))/4 + qp1*qp1r*cos(q2)*cos(q3)^2*cos(q4)^2*cos(q6)^2*sin(q2) - (qp1*qp1r*cos(q2)*cos(q3)^2*cos(q5)^2*cos(q6)^2*sin(q2))/2 + qp1*qp1r*cos(q2)^2*cos(q3)*cos(q4)^2*cos(q6)^2*sin(q3) - (qp1*qp1r*cos(q2)*cos(q4)^2*cos(q5)^2*cos(q6)^2*sin(q2))/2 - (qp1*qp1r*cos(q2)^2*cos(q3)*cos(q5)^2*cos(q6)^2*sin(q3))/2 + qp1*qp1r*cos(q2)^2*cos(q3)^2*cos(q4)*cos(q6)^2*sin(q4) - (qp1*qp1r*cos(q3)*cos(q4)^2*cos(q5)^2*cos(q6)^2*sin(q3))/2 - (qp1*qp1r*cos(q2)^2*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q4))/2 - (qp1*qp1r*cos(q3)^2*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q4))/2 - (qp1*qp6*cos(q2)*cos(q3)*cos(q4)*cos(q6)^2*sin(q5))/2 - (qpp1r*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5)*sin(q6))/4 + (qp1*qp6*cos(q2)*cos(q6)^2*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qp6*cos(q3)*cos(q6)^2*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qp6*cos(q4)*cos(q6)^2*sin(q2)*sin(q3)*sin(q5))/2 - (qp1*qp5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6))/2 + 2*qp1*qp1r*cos(q2)^2*cos(q3)^2*cos(q4)^2*cos(q5)*cos(q6)*sin(q6) + (qp1*qp5*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qp5*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6))/2 + (qp1*qp5*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6))/2 + qp1*qp1r*cos(q2)*cos(q3)^2*cos(q4)^2*cos(q5)^2*cos(q6)^2*sin(q2) + qp1*qp1r*cos(q2)^2*cos(q3)*cos(q4)^2*cos(q5)^2*cos(q6)^2*sin(q3) + qp1*qp1r*cos(q2)^2*cos(q3)^2*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q4) + (qp1*qp6*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*sin(q5)*sin(q6))/2 + (qp1*qp6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q5)*sin(q6))/2 + (qp1*qp6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q5)*sin(q6))/2 + qp1*qp1r*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp1*qp1r*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp1*qp1r*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) - (qp1*qp6*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6))/2 - qp1*qp1r*cos(q2)*cos(q3)*cos(q4)*cos(q6)^2*sin(q2)*sin(q3)*sin(q4) - 2*qp1*qp1r*cos(q2)*cos(q3)*cos(q4)^2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) - 2*qp1*qp1r*cos(q2)*cos(q3)^2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) - 2*qp1*qp1r*cos(q2)^2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) - qp1*qp1r*cos(q2)*cos(q3)*cos(q4)*cos(q5)^2*cos(q6)^2*sin(q2)*sin(q3)*sin(q4);
    Yr(1, 1) = (qpp4*cos(2*q5)*sin(2*q6))/16 - (qpp4*sin(2*q6))/16 - (qpp2r*sin(2*q6))/16 - (qpp3r*sin(2*q6))/16 - (qpp5*sin(q5))/8 - (qp5^2*cos(2*q6)*cos(q5))/8 - (qp5^2*cos(q5))/8 + (qpp2r*cos(2*q5)*sin(2*q6))/16 + (qpp3r*cos(2*q5)*sin(2*q6))/16 - (qp2*qp6*cos(2*q6))/8 - (qp3*qp6*cos(2*q6))/8 - (qp4*qp6*cos(2*q6))/8 - (qpp5*cos(2*q6)*sin(q5))/8 + (qp5*qp6*sin(2*q6)*sin(q5))/4 + (qp2*qp6*cos(2*q5)*cos(2*q6))/8 + (qp3*qp6*cos(2*q5)*cos(2*q6))/8 + (qp4*qp6*cos(2*q5)*cos(2*q6))/8 - (qp2*qp5*sin(2*q5)*sin(2*q6))/8 - (qp3*qp5*sin(2*q5)*sin(2*q6))/8 - (qp4*qp5*sin(2*q5)*sin(2*q6))/8 + (qpp1r*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (qpp1r*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp1r*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp1r*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (qpp1r*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/16 - (qp1*qp5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 - (qp1*qp5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 - (qp1*qp5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 + (3*qp1*qp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/32 + (3*qp1*qp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/32 + (3*qp1*qp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/32 + (qp1*qp5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 - (3*qp1*qp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/32 + (qpp1r*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/8 - (qpp1r*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/8 - (qpp1r*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/8 - (qpp1r*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/8 + (qp1*qp5*cos(q2)*cos(q3)*cos(q4)*cos(q5))/4 - (qp1*qp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q5))/8 - (qp1*qp5*cos(q2)*cos(q5)*sin(q3)*sin(q4))/4 - (qp1*qp5*cos(q3)*cos(q5)*sin(q2)*sin(q4))/4 - (qp1*qp5*cos(q4)*cos(q5)*sin(q2)*sin(q3))/4 + (qp1*qp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/8 + (qp1*qp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q5))/8 + (qp1*qp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q5))/8 - (qpp1r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/16 - (qpp1r*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/16 - (qpp1r*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/16 + (qp1*qp1r*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5))/8 + (qp1*qp1r*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5))/8 + (qp1*qp1r*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5))/8 + (qp1*qp6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/4 + (qp1*qp6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/4 + (qp1*qp6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/4 - (qp1*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/8 - (qp1*qp5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/8 - (qp1*qp5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/8 - (qp1*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/8 - (qp1*qp6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/8 - (qp1*qp6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/8 + (qp1*qp1r*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/32 + (qp1*qp1r*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/32 + (qp1*qp1r*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/32 + (qp1*qp5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp1*qp6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/8 - (qp1*qp1r*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/32 + (qp1*qp5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/4 - (qp1*qp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5))/8 - (qp1*qp5*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/4 - (qp1*qp5*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/4 - (qp1*qp5*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/4 - (qp1*qp6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/4;
    Yr(1, 2) = qpp4/8 + qpp2r/8 + qpp3r/8 + (qpp4*cos(2*q5))/8 + (qpp2r*cos(2*q5))/8 + (qpp3r*cos(2*q5))/8 - (qpp1r*cos(q2 + q3 + q4 - 2*q5))/16 + (qpp1r*cos(q2 + q3 + q4 + 2*q5))/16 - (qp1*qp5*sin(q2 + q3 + q4 - 2*q5))/8 - (qp1*qp5*sin(q2 + q3 + q4 + 2*q5))/8 + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 - (qp1*qp5*sin(q2 + q3 + q4))/4 - (qp2*qp5*sin(2*q5))/4 - (qp3*qp5*sin(2*q5))/4 - (qp4*qp5*sin(2*q5))/4 + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/16;
    Yr(1, 3) = (qpp6*cos(q6)*sin(q5))/8 - (qpp5*cos(q5)*sin(q6))/8 + (qpp4*sin(2*q5)*cos(q6))/8 + (qpp2r*sin(2*q5)*cos(q6))/8 + (qpp3r*sin(2*q5)*cos(q6))/8 + (qp5^2*sin(q5)*sin(q6))/8 - (qp6^2*sin(q5)*sin(q6))/8 + (qp2*qp5*cos(2*q5)*cos(q6))/4 + (qp3*qp5*cos(2*q5)*cos(q6))/4 + (qp4*qp5*cos(2*q5)*cos(q6))/4 - (qp2*qp6*sin(2*q5)*sin(q6))/8 - (qp3*qp6*sin(2*q5)*sin(q6))/8 - (qp4*qp6*sin(2*q5)*sin(q6))/8 + (qpp1r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/8 - (qpp1r*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6))/8 - (qpp1r*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/8 - (qpp1r*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6))/8 + (qpp1r*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qpp1r*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qpp1r*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 - (qpp1r*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 + (qp1*qp6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 + (qp1*qp6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 + (qp1*qp6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qp1*qp6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 + (qp1*qp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/16 + (qp1*qp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/16 + (qp1*qp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/16 + (qp1*qp5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qp6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp1*qp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/16 - (qp1*qp5*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6))/4 + (qp1*qp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6))/8 + (qp1*qp5*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6))/4 + (qp1*qp5*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6))/4 + (qp1*qp5*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6))/4 - (qp1*qp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/8 - (qp1*qp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6))/8 - (qp1*qp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6))/8 - (qp1*qp5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/4 - (qp1*qp5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/4 - (qp1*qp5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/4 - (qp1*qp6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 - (qp1*qp6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 - (qp1*qp6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8;
    Yr(1, 4) = (qpp5*cos(q5)*cos(q6))/8 + (qpp6*sin(q5)*sin(q6))/8 - (qp5^2*cos(q6)*sin(q5))/8 + (qp6^2*cos(q6)*sin(q5))/8 + (qpp4*sin(2*q5)*sin(q6))/8 + (qpp2r*sin(2*q5)*sin(q6))/8 + (qpp3r*sin(2*q5)*sin(q6))/8 + (qp2*qp5*cos(2*q5)*sin(q6))/4 + (qp2*qp6*sin(2*q5)*cos(q6))/8 + (qp3*qp5*cos(2*q5)*sin(q6))/4 + (qp3*qp6*sin(2*q5)*cos(q6))/8 + (qp4*qp5*cos(2*q5)*sin(q6))/4 + (qp4*qp6*sin(2*q5)*cos(q6))/8 + (qpp1r*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4))/8 + (qpp1r*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/8 + (qpp1r*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3))/8 + (qpp1r*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/8 + (qpp1r*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/8 + (qpp1r*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/8 - (qpp1r*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/8 - (qp1*qp6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 - (qp1*qp6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 - (qp1*qp6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 + (qp1*qp6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8 - (qpp1r*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/8 + (qp1*qp1r*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/16 + (qp1*qp1r*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/16 + (qp1*qp1r*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/16 + (qp1*qp5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/4 - (qp1*qp1r*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/16 + (qp1*qp5*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5))/4 - (qp1*qp1r*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5))/8 - (qp1*qp5*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5))/4 - (qp1*qp5*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5))/4 - (qp1*qp5*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5))/4 + (qp1*qp1r*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/8 + (qp1*qp1r*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5))/8 + (qp1*qp1r*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5))/8 + (qp1*qp6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/8 + (qp1*qp6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/8 + (qp1*qp6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/8 - (qp1*qp5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/4 - (qp1*qp5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/4 - (qp1*qp5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/4 - (qp1*qp6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/8;
    Yr(1, 5) = (qpp4*sin(2*q5))/8 + (qpp2r*sin(2*q5))/8 + (qpp3r*sin(2*q5))/8 + (qpp1r*sin(q2 + q3 + q4 - 2*q5))/16 + (qpp1r*sin(q2 + q3 + q4 + 2*q5))/16 + (qpp1r*sin(q2 + q3 + q4))/8 - (qp1*qp5*cos(q2 + q3 + q4 - 2*q5))/8 + (qp1*qp5*cos(q2 + q3 + q4 + 2*q5))/8 + (qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/32 - (qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/32 + (qp2*qp5*cos(2*q5))/4 + (qp3*qp5*cos(2*q5))/4 + (qp4*qp5*cos(2*q5))/4;
    Yr(1, 6) = (qp1*qp1r*(sin(2*q2 + 2*q3 + 2*q4) + sin(2*q2 + 2*q3)))/8;
    Yr(1, 7) = (qpp1r*sin(q2 + q3 + q4 + q5))/16 - (qp5^2*cos(q5))/8 - (qpp5*sin(q5))/8 - (qpp1r*sin(q2 + q3 + q4 - q5))/16 + (qp1*qp5*cos(q2 + q3 + q4 - q5))/8 - (qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4 - q5))/16 + (qp1*qp5*cos(q2 + q3 + q4 + q5))/8 - (qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4 + q5))/16;
    Yr(1, 8) = (qpp5*cos(q5))/8 - (qpp1r*cos(q2 + q3 + q4 + q5))/16 - (qp5^2*sin(q5))/8 - (qpp1r*cos(q2 + q3 + q4 - q5))/16 - (qp1*qp5*sin(q2 + q3 + q4 - q5))/8 + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4 - q5))/16 + (qp1*qp5*sin(q2 + q3 + q4 + q5))/8 - (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4 + q5))/16;
    Yr(1, 9) = -(qp1*qp1r*(cos(2*q2 + 2*q3 + 2*q4) + cos(2*q2 + 2*q3)))/8;

    Yr(1, 10) = (qpp4*cos(q4)*sin(q5))/4 + (qpp5*cos(q5)*sin(q4))/4 + (qpp2r*cos(q4)*sin(q5))/2 + (qpp3r*cos(q4)*sin(q5))/2 - (qp4^2*sin(q4)*sin(q5))/4 - (qp5^2*sin(q4)*sin(q5))/4 - (qp2*qp4*sin(q4)*sin(q5))/2 - (qp3*qp4*sin(q4)*sin(q5))/2 + (qp1*qp1r*sin(q4)*sin(q5))/4 + (qpp1r*cos(q2)*cos(q5)*sin(q3))/4 + (qpp1r*cos(q3)*cos(q5)*sin(q2))/4 + (qp2*qp5*cos(q4)*cos(q5))/2 + (qp3*qp5*cos(q4)*cos(q5))/2 + (qp4*qp5*cos(q4)*cos(q5))/2 - (qp1*qp1r*cos(q2)^2*sin(q4)*sin(q5))/2 - (qp1*qp1r*cos(q3)^2*sin(q4)*sin(q5))/2 - (qp1*qp5*cos(q2)*sin(q3)*sin(q5))/2 - (qp1*qp5*cos(q3)*sin(q2)*sin(q5))/2 + qp1*qp1r*cos(q2)^2*cos(q3)^2*sin(q4)*sin(q5) - (qp1*qp1r*cos(q2)*cos(q4)*sin(q2)*sin(q5))/2 - (qp1*qp1r*cos(q3)*cos(q4)*sin(q3)*sin(q5))/2 + qp1*qp1r*cos(q2)*cos(q3)^2*cos(q4)*sin(q2)*sin(q5) + qp1*qp1r*cos(q2)^2*cos(q3)*cos(q4)*sin(q3)*sin(q5) - qp1*qp1r*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5);
    Yr(1, 11) = (qp4^2*cos(q4))/4 + (qpp4*sin(q4))/4 + (qpp2r*sin(q4))/2 + (qpp3r*sin(q4))/2 + (qp2*qp4*cos(q4))/2 + (qp3*qp4*cos(q4))/2 - (qp1*qp1r*cos(2*q2 + 2*q3 + q4))/4;
    Yr(1, 12) = (qp1*qp1r*(sin(2*q2) + sin(2*q2 + 2*q3)))/4;
    Yr(1, 13) = (qpp1r*cos(q5)*sin(q2))/2 - qp1*qp5*sin(q2)*sin(q5) + (qpp4*cos(q3)*cos(q4)*sin(q5))/2 + (qpp5*cos(q3)*cos(q5)*sin(q4))/2 + (qpp5*cos(q4)*cos(q5)*sin(q3))/2 + qpp2r*cos(q3)*cos(q4)*sin(q5) + (qpp3r*cos(q3)*cos(q4)*sin(q5))/2 - (qpp4*sin(q3)*sin(q4)*sin(q5))/2 - qpp2r*sin(q3)*sin(q4)*sin(q5) - (qpp3r*sin(q3)*sin(q4)*sin(q5))/2 - (qp4^2*cos(q3)*sin(q4)*sin(q5))/2 - (qp4^2*cos(q4)*sin(q3)*sin(q5))/2 - (qp5^2*cos(q3)*sin(q4)*sin(q5))/2 - (qp5^2*cos(q4)*sin(q3)*sin(q5))/2 - cos(q3)*sin(q4)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2) - cos(q4)*sin(q3)*sin(q5)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + qp2*qp5*cos(q3)*cos(q4)*cos(q5) + qp3*qp5*cos(q3)*cos(q4)*cos(q5) + qp4*qp5*cos(q3)*cos(q4)*cos(q5) - qp2*qp4*cos(q3)*sin(q4)*sin(q5) - qp2*qp4*cos(q4)*sin(q3)*sin(q5) - qp2*qp5*cos(q5)*sin(q3)*sin(q4) - qp3*qp4*cos(q3)*sin(q4)*sin(q5) - qp3*qp4*cos(q4)*sin(q3)*sin(q5) - qp3*qp5*cos(q5)*sin(q3)*sin(q4) - qp4*qp5*cos(q5)*sin(q3)*sin(q4) - (qp3*qp3r*cos(q3)*sin(q4)*sin(q5))/2 - (qp3*qp3r*cos(q4)*sin(q3)*sin(q5))/2 + (qp1*qp1r*cos(2*q2)*cos(q3)*sin(q4)*sin(q5))/2 + (qp1*qp1r*cos(2*q2)*cos(q4)*sin(q3)*sin(q5))/2 + (qp1*qp1r*sin(2*q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qp1r*sin(2*q2)*sin(q3)*sin(q4)*sin(q5))/2;
    Yr(1, 14) = -(qp1*qp1r*(cos(2*q2) + cos(2*q2 + 2*q3)))/4;
    Yr(1, 15) = (qpp4*sin(q3 + q4))/2 + qpp2r*sin(q3 + q4) + (qpp3r*sin(q3 + q4))/2 + (qp4^2*cos(q3 + q4))/2 + cos(q3 + q4)*((qp2*qp3r)/2 + (qp3*qp2r)/2) - (qp1*qp1r*cos(2*q2 + q3 + q4))/2 + qp2*qp4*cos(q3 + q4) + qp3*qp4*cos(q3 + q4) + (qp3*qp3r*cos(q3 + q4))/2;
    Yr(1, 16) = qpp2r*cos(q3) - sin(q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + (qpp3r*cos(q3))/2 - (qp3*qp3r*sin(q3))/2 + (qp1*qp1r*sin(2*q2 + q3))/2;
    Yr(1, 17) = cos(q3)*((qp2*qp3r)/2 + (qp3*qp2r)/2) + (qpp4*sin(q3))/2 + qpp2r*sin(q3) + (qpp3r*sin(q3))/2 + (qp3*qp4*cos(q3))/2 + (qp3*qp3r*cos(q3))/2 - (qp1*qp1r*cos(2*q2 + q3))/2;
    Yr(1, 18) = (qp1*qp1r*sin(2*q2))/2;
    Yr(1, 19) = -(qp1*qp1r*cos(2*q2))/2;

    Yr(1, 20) = -(qpp1r*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/4;
    Yr(1, 21) = 0;
    Yr(1, 22) = qpp1r*cos(q2 + q3 + q4);
    Yr(1, 23) = qpp1r*sin(q2 + q3 + q4);
    Yr(1, 24) = qpp1r*cos(q2 + q3);
    Yr(1, 25) = qpp1r*sin(q2 + q3);
    Yr(1, 26) = qpp1r*cos(q2);
    Yr(1, 27) = qpp1r*sin(q2);
    Yr(1, 28) = -cos(q2)*((qp1*qp3r)/2 + (qp3*qp1r)/2);
    Yr(1, 29) = qp1*qp4*sin(q2 + q3);

    Yr(1, 30) = 
    Yr(1, 31) = 
    Yr(1, 32) = 
    Yr(1, 33) = 
    Yr(1, 34) = 
    Yr(1, 35) = 
    Yr(1, 36) = 
    Yr(1, 37) = 
    Yr(1, 38) = 
    Yr(1, 39) = 

    Yr(1, 40) = 
    Yr(1, 41) = 
    Yr(1, 42) = 
    Yr(1, 43) = 
    Yr(1, 44) = 
    Yr(1, 45) = 
    Yr(1, 46) = 
    Yr(1, 47) = 
    Yr(1, 48) = 
    Yr(1, 49) = 

    Yr(1, 50) = 
    Yr(1, 51) = 
    Yr(1, 52) = 
    Yr(1, 53) = 
    Yr(1, 54) = 
    Yr(1, 55) = 
    Yr(1, 56) = 
    Yr(1, 57) = 
    Yr(1, 58) = 
    Yr(1, 59) = 
    Yr(1, 60) = 

    //////////////////////////////////////////////////////////////////////////////////////////////

    Yr(2, 0) = 
    Yr(2, 1) = 
    Yr(2, 2) = 
    Yr(2, 3) = 
    Yr(2, 4) = 
    Yr(2, 5) = 
    Yr(2, 6) = 
    Yr(2, 7) = 
    Yr(2, 8) = 
    Yr(2, 9) = 

    Yr(2, 10) = 
    Yr(2, 11) = 
    Yr(2, 12) = 
    Yr(2, 13) = 
    Yr(2, 14) = 
    Yr(2, 15) = 
    Yr(2, 16) = 
    Yr(2, 17) = 
    Yr(2, 18) = 
    Yr(2, 19) = 

    Yr(2, 20) = 
    Yr(2, 21) = 
    Yr(2, 22) = 
    Yr(2, 23) = 
    Yr(2, 24) = 
    Yr(2, 25) = 
    Yr(2, 26) = 
    Yr(2, 27) = 
    Yr(2, 28) = 
    Yr(2, 29) = 

    Yr(2, 30) = 
    Yr(2, 31) = 
    Yr(2, 32) = 
    Yr(2, 33) = 
    Yr(2, 34) = 
    Yr(2, 35) = 
    Yr(2, 36) = 
    Yr(2, 37) = 
    Yr(2, 38) = 
    Yr(2, 39) = 

    Yr(2, 40) = 
    Yr(2, 41) = 
    Yr(2, 42) = 
    Yr(2, 43) = 
    Yr(2, 44) = 
    Yr(2, 45) = 
    Yr(2, 46) = 
    Yr(2, 47) = 
    Yr(2, 48) = 
    Yr(2, 49) = 

    Yr(2, 50) = 
    Yr(2, 51) = 
    Yr(2, 52) = 
    Yr(2, 53) = 
    Yr(2, 54) = 
    Yr(2, 55) = 
    Yr(2, 56) = 
    Yr(2, 57) = 
    Yr(2, 58) = 
    Yr(2, 59) = 
    Yr(2, 60) = 

    //////////////////////////////////////////////////////////////////////////////////////////////

    Yr(3, 0) = 
    Yr(3, 1) = 
    Yr(3, 2) = 
    Yr(3, 3) = 
    Yr(3, 4) = 
    Yr(3, 5) = 
    Yr(3, 6) = 
    Yr(3, 7) = 
    Yr(3, 8) = 
    Yr(3, 9) = 

    Yr(3, 10) = 
    Yr(3, 11) = 
    Yr(3, 12) = 
    Yr(3, 13) = 
    Yr(3, 14) = 
    Yr(3, 15) = 
    Yr(3, 16) = 
    Yr(3, 17) = 
    Yr(3, 18) = 
    Yr(3, 19) = 

    Yr(3, 20) = 
    Yr(3, 21) = 
    Yr(3, 22) = 
    Yr(3, 23) = 
    Yr(3, 24) = 
    Yr(3, 25) = 
    Yr(3, 26) = 
    Yr(3, 27) = 
    Yr(3, 28) = 
    Yr(3, 29) = 

    Yr(3, 30) = 
    Yr(3, 31) = 
    Yr(3, 32) = 
    Yr(3, 33) = 
    Yr(3, 34) = 
    Yr(3, 35) = 
    Yr(3, 36) = 
    Yr(3, 37) = 
    Yr(3, 38) = 
    Yr(3, 39) = 

    Yr(3, 40) = 
    Yr(3, 41) = 
    Yr(3, 42) = 
    Yr(3, 43) = 
    Yr(3, 44) = 
    Yr(3, 45) = 
    Yr(3, 46) = 
    Yr(3, 47) = 
    Yr(3, 48) = 
    Yr(3, 49) = 

    Yr(3, 50) = 
    Yr(3, 51) = 
    Yr(3, 52) = 
    Yr(3, 53) = 
    Yr(3, 54) = 
    Yr(3, 55) = 
    Yr(3, 56) = 
    Yr(3, 57) = 
    Yr(3, 58) = 
    Yr(3, 59) = 
    Yr(3, 60) = 

    //////////////////////////////////////////////////////////////////////////////////////////////

    Yr(4, 0) = 
    Yr(4, 1) = 
    Yr(4, 2) = 
    Yr(4, 3) = 
    Yr(4, 4) = 
    Yr(4, 5) = 
    Yr(4, 6) = 
    Yr(4, 7) = 
    Yr(4, 8) = 
    Yr(4, 9) = 

    Yr(4, 10) = 
    Yr(4, 11) = 
    Yr(4, 12) = 
    Yr(4, 13) = 
    Yr(4, 14) = 
    Yr(4, 15) = 
    Yr(4, 16) = 
    Yr(4, 17) = 
    Yr(4, 18) = 
    Yr(4, 19) = 

    Yr(4, 20) = 
    Yr(4, 21) = 
    Yr(4, 22) = 
    Yr(4, 23) = 
    Yr(4, 24) = 
    Yr(4, 25) = 
    Yr(4, 26) = 
    Yr(4, 27) = 
    Yr(4, 28) = 
    Yr(4, 29) = 

    Yr(4, 30) = 
    Yr(4, 31) = 
    Yr(4, 32) = 
    Yr(4, 33) = 
    Yr(4, 34) = 
    Yr(4, 35) = 
    Yr(4, 36) = 
    Yr(4, 37) = 
    Yr(4, 38) = 
    Yr(4, 39) = 

    Yr(4, 40) = 
    Yr(4, 41) = 
    Yr(4, 42) = 
    Yr(4, 43) = 
    Yr(4, 44) = 
    Yr(4, 45) = 
    Yr(4, 46) = 
    Yr(4, 47) = 
    Yr(4, 48) = 
    Yr(4, 49) = 

    Yr(4, 50) = 
    Yr(4, 51) = 
    Yr(4, 52) = 
    Yr(4, 53) = 
    Yr(4, 54) = 
    Yr(4, 55) = 
    Yr(4, 56) = 
    Yr(4, 57) = 
    Yr(4, 58) = 
    Yr(4, 59) = 
    Yr(4, 60) = 

    //////////////////////////////////////////////////////////////////////////////////////////////

    Yr(5, 0) = 
    Yr(5, 1) = 
    Yr(5, 2) = 
    Yr(5, 3) = 
    Yr(5, 4) = 
    Yr(5, 5) = 
    Yr(5, 6) = 
    Yr(5, 7) = 
    Yr(5, 8) = 
    Yr(5, 9) = 

    Yr(5, 10) = 
    Yr(5, 11) = 
    Yr(5, 12) = 
    Yr(5, 13) = 
    Yr(5, 14) = 
    Yr(5, 15) = 
    Yr(5, 16) = 
    Yr(5, 17) = 
    Yr(5, 18) = 
    Yr(5, 19) = 

    Yr(5, 20) = 
    Yr(5, 21) = 
    Yr(5, 22) = 
    Yr(5, 23) = 
    Yr(5, 24) = 
    Yr(5, 25) = 
    Yr(5, 26) = 
    Yr(5, 27) = 
    Yr(5, 28) = 
    Yr(5, 29) = 

    Yr(5, 30) = 
    Yr(5, 31) = 
    Yr(5, 32) = 
    Yr(5, 33) = 
    Yr(5, 34) = 
    Yr(5, 35) = 
    Yr(5, 36) = 
    Yr(5, 37) = 
    Yr(5, 38) = 
    Yr(5, 39) = 

    Yr(5, 40) = 
    Yr(5, 41) = 
    Yr(5, 42) = 
    Yr(5, 43) = 
    Yr(5, 44) = 
    Yr(5, 45) = 
    Yr(5, 46) = 
    Yr(5, 47) = 
    Yr(5, 48) = 
    Yr(5, 49) = 

    Yr(5, 50) = 
    Yr(5, 51) = 
    Yr(5, 52) = 
    Yr(5, 53) = 
    Yr(5, 54) = 
    Yr(5, 55) = 
    Yr(5, 56) = 
    Yr(5, 57) = 
    Yr(5, 58) = 
    Yr(5, 59) = 
    Yr(5, 60) = 
}

#endif