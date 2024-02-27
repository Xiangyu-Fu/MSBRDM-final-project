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


%% Load Data
Theta_fileName = 'Theta.mat';
Y_fileName = 'Y.mat';
M_fileName = 'M.mat';
C_fileName = 'C.mat';
G_fileName = 'G.mat';

Y = load(Y_fileName).Y;
Theta = load(Theta_fileName).Theta;
M = load(M_fileName).M;
C = load(C_fileName).C;
G = load(G_fileName).G;


%% Check

syms qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 real
q_ddot = [qpp1; qpp2; qpp3; qpp4; qpp5; qpp6];
q_dot = [qp1; qp2; qp3; qp4; qp5; qp6];
q = [q1; q2; q3; q4; q5; q6];

caculet = M*q_ddot + C*q_dot + G;
regressor = Y*Theta;

if (simplify(expand(caculet ~= regressor)))
    disp('Error')
else
    disp('Correct')
end



