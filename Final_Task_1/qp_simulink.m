function rrf_body = qp_simulink(X, pf, t)
coder.extrinsic('quadprog');
m = 12;
F_max = 500;
F_min = 0;
mu = 0.5;
I_b = diag([0.0168, 0.0565, 0.064]);
legs = 4;
grav = [0; 0; -9.81];

% Control Parameters from HW
% Kp_pos = diag([30, 30, 400]);
% Kd_pos = diag([10, 10, 50]);
% Kp_ori = diag([100, 10, 10]);
% Kd_ori = diag([5, 5, 5]);
% S_qp = diag([2, 2, 10, 1, 2, 1]);
% alpha = 0.01;


% Control Parameters
Kp_pos = diag([30, 30, 400]);
Kd_pos = diag([10, 10, 50]);
Kp_ori = diag([2500, 700, 700]);
Kd_ori = diag([150, 100, 50]);
S_qp = diag([2, 2, 10, 1, 2, 1]);
alpha = 0.01;


% Control Parameters -- Junheng Advice
% Kp_pos = diag([100, 100, 100]);
% Kd_pos = diag([30, 30, 30]);
% Kp_ori = diag([100, 100, 100]);
% Kd_ori = diag([30, 30, 30]);
% S_qp = diag([2, 2, 10, 1, 2, 1]);
% alpha = 0.01;
%works with our custom jacobian
% Kp_pos = diag([100, 100, 300]);
% Kd_pos = diag([30, 30, 30]);
% Kp_ori = diag([100, 500, 100]);
% Kd_ori = diag([30, 30, 30]);
% S_qp = diag([2, 2, 10, 1, 2, 1]);
% alpha = 0.01;


% Desired Conditions
Pd = [0; 0; 0.2];
% [Yaw, Pitch, Roll]
euld = [0, 0, 0];
Rd = eul2rotm(euld);
dPd = [0; 0; 0];
wd = [0; 0; 0];

% PD Controller b_d
P = X(1:3);
dP = X(7:9);
ddPd = Kp_pos*(Pd - P) + Kd_pos*(dPd - dP);

ypr = X(4:6);
R = eul2rotm(ypr');
wb = X(10:12);
w = R*wb;
dwd = Kp_ori*rotmat2vec3d(Rd*R')' + Kd_ori*(wd - w);
I_w = R*I_b*R';

bd_pd = [m*(ddPd - grav); I_w*dwd];

% QP Inequality Constraints
Fiq_mat = [-1, 0, -mu; 1, 0, -mu; 0, -1, -mu; 0, 1, -mu; 0, 0, 1; 0, 0, -1];
A_iq = kron(eye(legs),Fiq_mat);
temp_vec = [0; 0; 0; 0; F_max; -F_min];
b_iq = [temp_vec; temp_vec; temp_vec; temp_vec];

% Position of feet in world frame ORIENTATION
r_f1 = pf(1:3) - P;
r_f2 = pf(4:6) - P;
r_f3 = pf(7:9) - P;
r_f4 = pf(10:12) - P;
r_fs = [r_f1, r_f2, r_f3, r_f4];

% QP QuadProg
A_eye = [eye(3), eye(3), eye(3), eye(3)];
temp_mat = zeros(3, 3*legs);
for ind = 1:legs
    temp_r = r_fs(:, ind);
    temp = [0, -temp_r(3), temp_r(2); temp_r(3), 0, -temp_r(1); -temp_r(2), temp_r(1), 0];
    temp_mat(:, ind*3 - 2:ind*3) = temp;
end
A = [A_eye; temp_mat];

H = A'*S_qp*A + alpha*eye(legs*3);
f = -A'*S_qp*bd_pd;


grf_legs = quadprog(H, f, A_iq, b_iq);

rrf_body = zeros(3*legs, 1);

for ind = 0:legs-1
    rrf_body(ind*3 + 1: ind*3 + 3,1) = -1 * R'*grf_legs(ind*3 + 1: ind*3 + 3,1);
end

disp(rrf_body)

end