
clc; clear all;
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

%% M, C, G
%Inertia Matrix
load('M.mat');
load('C.mat');
load('G.mat');


%% Regressor

syms qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 real
q_ddot = [qpp1 qpp2 qpp3 qpp4 qpp5 qpp6]';
q_dot = [qp1; qp2; qp3; qp4; qp5; qp6];

tau = M*q_ddot+ C * q_dot + G;

n = 6;
q = transpose(sym('q', [1,n]));
qp = transpose(sym('qp', [1,n]));
qpp = transpose(sym('qpp', [1,n]));
f = [cos(q) sin(q)];

tau = expand(tau);


Theta = [];
Y = [];

for num_link = 1:n
    y = [];
    theta = [];
    [theta_qpp, y_qpp] = coeffs(tau(num_link), qpp);
    for i = 1:length(y_qpp)
        c_qpp = y_qpp(i);
        [theta_qp, y_qp] = coeffs(theta_qpp(i), qp);
        for j = 1:length(y_qp)
            c_qp = y_qp(j);
            [theta_q, y_q] = coeffs(theta_qp(j), f);
            c_qp = c_qp * y_q;

            % update for current n
            theta = [theta; theta_q'];
            y = [y c_qpp * c_qp];
        end
    end
    Theta = [Theta; theta];
    y_new = sym(zeros(n, length(y)));
    y_new(num_link, :) = y;
    Y = [Y y_new];
end

% Simplify
n = length(Theta);
i = 1;
while i <= n
    j = i + 1;
    while j <= n
        r = symvar(simplify(Theta(i) / Theta(j)));
        if isempty(r)
            c = double(simplify(Theta(i) / Theta(j)));
            Theta(j) = [];

            Y(:, i) = Y(:, i) + 1/c * Y(:, j); 
            Y(:, j) = [];

            n = length(Theta);
        else
            j = j + 1;
        end
    end
    i = i + 1;
end

Theta
Y

%% Check 
simplify(Y*Theta - tau)

load('Y.mat');
syms qp1r qp2r qp3r qp4r qp5r qp6r real
syms qpp1r qpp2r qpp3r qpp4r qpp5r qpp6r real
Yr = subs(Y, [qpp1 qpp2 qpp3 qpp4 qpp5 qpp6], [qpp1r qpp2r qpp3r qpp4r qpp5r qpp6r]);
Yr = simplify(subs(Yr, [qp1^2 qp2^2 qp3^2 qp4^2 qp5^2 qp6^2], [qp1*qp1r qp2*qp2r qp3*qp3r qp4*qp4r qp5*qp5r qp6*qp6r]));
% Yr = subs(Yr, [qp1*qp2 qp2*qp3 qp1*qp3], [0.5*qp1*qp2r+0.5*qp1r*qp2 0.5*qp2*qp3r+0.5*qp2r*qp3 0.5*qp1*qp3r+0.5*qp1r*qp3]);

for i = 1:length(qp)
    for j = 1:length(qp)
        if i ~= j
            qpi = sym(['qp', num2str(i)]);
            qpj = sym(['qp', num2str(j)]);
            qpir = sym(['qp', num2str(i), 'r']);
            qpjr = sym(['qp', num2str(j), 'r']);
            original_term = qpi*qpj;
            replaced_term = 0.5*qpi*qpjr + 0.5*qpir*qpj;
            Yr = subs(Yr, original_term, replaced_term);
        end
    end
end

Yr


save('Theta.mat', 'Theta');
save('Y.mat', 'Y');
save('Yr.mat', 'Yr');
