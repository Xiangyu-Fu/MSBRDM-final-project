clear
syms q1 q2 q3 q4 q5 q6 L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 real
syms qp1 qp2 qp3 qp4 qp5 qp6 real
syms m1 m2 m3 m4 m5 m6 real
syms gx gy gz real
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real
syms I411 I412 I413 I422 I423 I433 real
syms I511 I512 I513 I522 I523 I533 real
syms I611 I612 I613 I622 I623 I633 real

%% DH-Table
DH = [q1,       L1,         0,          pi/2;
      q2,       0,          -L3,        0;
      q3,       0,          -(L5-L11),  0;
      q4,       L2,         0,          pi/2;
      q5,       L11,        0,          -pi/2;
      q6,       L4,         0,          0];

Center_of_Mass = [[0.021,   0.0,    0.027];
                  [0.38,    0.0,    0.158];
                  [0.24,    0.0,    0.068];
                  [0.0,     0.007,  0.018];
                  [0.0,     0.007,  0.018];
                  [0.0,     0.0,    -0.026]];

%% Transformation matrix
% O
T1_0 = relativeTrans(DH(1,:));
T2_1 = relativeTrans(DH(2,:));		
T3_2 = relativeTrans(DH(3,:));
T4_3 = relativeTrans(DH(4,:));
T5_4 = relativeTrans(DH(5,:));		
T6_5 = relativeTrans(DH(6,:));

T2_0 = T1_0 * T2_1;
T3_0 = T2_0 * T3_2;
T4_0 = T3_0 * T4_3;
T5_0 = T4_0 * T5_4;
T6_0 = T5_0 * T6_5;

% CM
Tcm1_0 = T1_0;
Tcm2_1 = T2_1;		
Tcm3_2 = T3_2;
Tcm4_3 = T4_3;
Tcm5_4 = T5_4;		
Tcm6_5 = T6_5;
Tcm1_0(1:3,4) = Center_of_Mass(1,:)';
Tcm2_1(1:3,4) = Center_of_Mass(2,:)';
Tcm3_2(1:3,4) = Center_of_Mass(3,:)';
Tcm4_3(1:3,4) = Center_of_Mass(4,:)';
Tcm5_4(1:3,4) = Center_of_Mass(5,:)';
Tcm6_5(1:3,4) = Center_of_Mass(6,:)';

Tcm2_0 = T1_0 * Tcm2_1;
Tcm3_0 = T2_0 * Tcm3_2;
Tcm4_0 = T3_0 * Tcm4_3;
Tcm5_0 = T4_0 * Tcm5_4;
Tcm6_0 = T5_0 * Tcm6_5;

%% Jacobian

t1_0 = T1_0(1:3, 4);
t2_0 = T2_0(1:3, 4);
t3_0 = T3_0(1:3, 4);
t4_0 = T4_0(1:3, 4);
t5_0 = T5_0(1:3, 4);
t6_0 = T6_0(1:3, 4); % {ee}

tcm1_0 = Tcm1_0(1:3, 4);
tcm2_0 = Tcm2_0(1:3, 4);
tcm3_0 = Tcm3_0(1:3, 4);
tcm4_0 = Tcm4_0(1:3, 4);
tcm5_0 = Tcm5_0(1:3, 4);
tcm6_0 = Tcm6_0(1:3, 4);

Z0 = [0 0 1]';
Z1 = T1_0(1:3,1:3) * Z0;
Z2 = T2_0(1:3,1:3) * Z0;
Z3 = T3_0(1:3,1:3) * Z0;
Z4 = T4_0(1:3,1:3) * Z0;
Z5 = T5_0(1:3,1:3) * Z0;


Jcm1 = simplify([cross(Z0, tcm1_0), [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]'; ...
                Z0, [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]']);
Jcm2 = simplify([cross(Z0, tcm2_0), cross(Z1, tcm2_0-t1_0), [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]'; ...
                Z0, Z1, [0 0 0]', [0 0 0]', [0 0 0]', [0 0 0]']);

Jcm3 = simplify([cross(Z0, tcm3_0), cross(Z1, tcm3_0-t1_0), cross(Z2, tcm3_0-t2_0), [0 0 0]', [0 0 0]', [0 0 0]'; ...
                Z0, Z1, Z2, [0 0 0]', [0 0 0]', [0 0 0]']);
Jcm4 = simplify([cross(Z0, tcm4_0), cross(Z1, tcm4_0-t1_0), cross(Z2, tcm4_0-t2_0), cross(Z3, tcm4_0-t3_0), [0 0 0]', [0 0 0]'; ...
                Z0, Z1, Z2, Z3, [0 0 0]', [0 0 0]']);
Jcm5 = simplify([cross(Z0, tcm5_0), cross(Z1, tcm5_0-t1_0), cross(Z2, tcm5_0-t2_0), cross(Z3, tcm5_0-t3_0), cross(Z4, tcm5_0-t4_0), [0 0 0]'; ...
                Z0, Z1, Z2, Z3, Z4, [0 0 0]']);
Jcm6 = simplify([cross(Z0, tcm6_0), cross(Z1, tcm6_0-t1_0), cross(Z2, tcm6_0-t2_0), cross(Z3, tcm6_0-t3_0), cross(Z4, tcm6_0-t4_0), cross(Z5, tcm6_0-t5_0); ...
                Z0, Z1, Z2, Z3, Z4, Z5]);

%% M, C, G

I1 = [I111, I112, I113;
      I112, I122, I123;
      I113, I123, I133];
I2 = [I211, I212, I213;
      I212, I222, I223;
      I213, I223, I233];
I3 = [I311, I312, I313;
      I312, I322, I323;
      I313, I323, I333];
I4 = [I411, I412, I413;
      I412, I422, I423;
      I413, I423, I433];
I5 = [I511, I512, I513;
      I512, I522, I523;
      I513, I523, I533];
I6 = [I611, I612, I613;
      I612, I622, I623;
      I613, I623, I633];
g = [gx, gy, gz];

M = m1*Jcm1(1:3,:)'*Jcm1(1:3,:) + m2*Jcm2(1:3,:)'*Jcm2(1:3,:) + m3*Jcm3(1:3,:)'*Jcm3(1:3,:) + m4*Jcm4(1:3,:)'*Jcm4(1:3,:) + m5*Jcm5(1:3,:)'*Jcm5(1:3,:) + m6*Jcm6(1:3,:)'*Jcm6(1:3,:);
M = M + Jcm1(4:6,:)'*Tcm1_0(1:3,1:3)*I1*Tcm1_0(1:3,1:3)'*Jcm1(4:6,:);
M = M + Jcm2(4:6,:)'*Tcm2_0(1:3,1:3)*I2*Tcm2_0(1:3,1:3)'*Jcm2(4:6,:);
M = M + Jcm3(4:6,:)'*Tcm3_0(1:3,1:3)*I3*Tcm3_0(1:3,1:3)'*Jcm3(4:6,:);
M = M + Jcm4(4:6,:)'*Tcm4_0(1:3,1:3)*I4*Tcm4_0(1:3,1:3)'*Jcm4(4:6,:);
M = M + Jcm5(4:6,:)'*Tcm5_0(1:3,1:3)*I5*Tcm5_0(1:3,1:3)'*Jcm5(4:6,:);
M = M + Jcm6(4:6,:)'*Tcm6_0(1:3,1:3)*I6*Tcm6_0(1:3,1:3)'*Jcm6(4:6,:);
M = simplify(expand(M));

C = sym(zeros(6, 6));
for i = 1:6
    for j = 1:6
        for k = 1:6
            qi = sym(['q', num2str(i)]);
            qj = sym(['q', num2str(j)]);
            qk = sym(['q', num2str(k)]);
            qpi = sym(['qp', num2str(i)]);

            C(k,j) = C(k,j) + 0.5*( diff(M(k,j),qi) + diff(M(k,i),qj) - diff(M(i,j),qk) )*qpi;
            % disp([i,j,k])
        end
    end
end
C = simplify(expand(C));

P = m1*g*Tcm1_0(1:3,4) + m2*g*Tcm2_0(1:3,4) + m3*g*Tcm3_0(1:3,4) + m4*g*Tcm4_0(1:3,4) + m5*g*Tcm5_0(1:3,4) + m6*g*Tcm6_0(1:3,4);
P = simplify(P);

G = sym(zeros(6, 1));
for i = 1:6
    qi = sym(['q', num2str(i)]);
    G(i) = diff(P, qi);
end
G = simplify(expand(G));

%% Check
if (M ~= M')
    disp('M is not symmetric matrix')
else
    disp('M is symmetric matrix')
end

% Check N
N = sym(zeros(6)); 
for i = 1:6
    for j = 1:6
        M_dot_ij = 0;
        for k = 1:6
            qpk = sym(['qp', num2str(k)]);
            qk = sym(['q', num2str(k)]);
            M_dot_ij = M_dot_ij + diff( M(i, j), qk ) * qpk;
        end
        N(i, j) = M_dot_ij - 2 * C(i, j);
    end
end

N = simplify(expand(N));
if (N ~= -N')
    disp('N is not skew symmetric matrix')
else
    disp('N is skew symmetric matrix')
end

% check N
X = sym('x%d', [6,1], 'real');
if (simplify(expand(X' * N * X)) ~= 0)
    disp('N is not skew symmetric matrix')
else
    disp('N is skew symmetric matrix')
end


function H = relativeTrans(dh)
    Hz = [RotZ(dh(1)) [0 0 dh(2)]'; 0 0 0 1];
    Hx = [RotX(dh(4)) [dh(3) 0 0]'; 0 0 0 1];
    H = Hz*Hx;
end

function R = RotZ(a)
    R=[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end

function R = RotX(a)
    R=[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end