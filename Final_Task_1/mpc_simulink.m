function rrf_legs = mpc_simulink(X, Xd, pf, t, N, dt, ftcontacts)
coder.extrinsic('quadprog');
warningState = warning('off', 'all');
P = X(1:3);
ypr = X(4:6);
X_bard = [Xd; 9.81];
X_bar = [X; 9.81];
legs = 4;
bar_states = 13;
% N = 10;
% dt = 0.03;
Xmpc_elem = bar_states*N + legs*3*N;
% Q, R values from HW4 #2(b)
% Q = diag([40, 50, 60, 10, 10, 10, 4, 4, 4, 1, 1, 1, 0]);

% test tuning for trotting in place
Q = diag([40, 50, 60, 10, 10, 10, 4, 4, 4, 1, 1, 1, 0]);

% test tuning for trotting forward
% Q = diag([40, 50, 60, 10, 10, 10, 4, 4, 4, 1, 1, 1, 0]);

R = 0.000001*eye(legs*3);
m = 12;
F_max = 500;
F_min = 10;
mu = 0.5;
I_b = diag([0.0168, 0.0565, 0.064]);

% Cost function parameters
vec_Q = diag(Q);
temp_Qs = vec_Q(:, ones(N, 1));
vec_R = diag(R);
temp_Rs = vec_R(:, ones(N, 1));
temp_fQ = -Q'*X_bard;
temp_fQs = temp_fQ(:, ones(N, 1));

H = diag([temp_Qs(:); temp_Rs(:)]);
f = [temp_fQs(:); zeros(3*legs*N, 1)];

% Inequality constraints A_iq
Fiq_mat = [-1, 0, -mu; 1, 0, -mu; 0, -1, -mu; 0, 1, -mu; 0, 0, 1; 0, 0, -1];
A_iq = [zeros(6*legs*N, bar_states*N), kron(eye(legs*N),Fiq_mat)];
% Inequality constraints b_iq
b_iq = zeros(6*N*legs, 1);
for ind = 1:6:6*N*legs
    alpha = ftcontacts(floor(ind/6) + 1);
    b_iq(ind:ind + 5, 1) = [0; 0; 0; 0; alpha*F_max; -alpha*F_min];
end

% Linearization
% [Yaw, Pitch, Roll]
psi = ypr(1);
theta = ypr(2);
phi  = ypr(3);
Rz_T = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1];
I_w = Rz_T'*I_b*Rz_T;
A_bar = zeros(bar_states);
A_bar(1:3, 3*2+1:3*3) = eye(3);
A_bar(3+1:3*2, 3*3+1:3*4) = Rz_T;
A_bar(3*3, bar_states) = -1;
% Position of feet in world frame
r_f1 = pf(1:3) - P;
r_f2 = pf(4:6) - P;
r_f3 = pf(7:9) - P;
r_f4 = pf(10:12) - P;
r_fs = [r_f1, r_f2, r_f3, r_f4];

temp_mat = zeros(3, 3*legs);
for ind = 1:legs
    temp_r = r_fs(:, ind);
    temp = [0, -temp_r(3), temp_r(2); temp_r(3), 0, -temp_r(1); -temp_r(2), temp_r(1), 0];
    temp_term = I_w\temp;
    temp_mat(:, 3*ind - 2:3*ind) = temp_term;
end
B_bar = [zeros(3*2, 3*legs); [eye(3)/m, eye(3)/m, eye(3)/m, eye(3)/m]; temp_mat; zeros(1, 3*legs)];

% Discretization
A_k = A_bar*dt + eye(bar_states);
B_k = B_bar*dt;

% MPC Dynamic Constraints
A_eq1 = zeros(bar_states*N, bar_states*N);
A_eq1(1:bar_states, 1:bar_states) = eye(bar_states);
A_Xrot = [-A_k, eye(bar_states), zeros(bar_states, bar_states*(N-2))];
% Shift for each
for ind = 1:N-1
    A_eq1(ind*bar_states + 1:ind*bar_states + bar_states, 1:bar_states*N) = A_Xrot;
    A_Xrot = circshift(A_Xrot,bar_states, 2);
end
A_eq2 = zeros(bar_states*N, 3*legs*N);
for ind = 1:N
    temp_vec = zeros(Xmpc_elem - 3*legs*N, 3*legs);
    start = 1 + bar_states*(ind - 1);
    temp_vec(start:start+bar_states-1, :) = -B_k;
    A_eq2(:, ind*3*legs - 3*legs + 1: ind*3*legs) = temp_vec;
end
A_eq = [A_eq1, A_eq2];
b_eq = zeros(Xmpc_elem - 3*legs*N, 1);
b_eq(1:bar_states) = A_k*X_bar;


X_mpc = quadprog(H, f, A_iq, b_iq, A_eq, b_eq);
grf_legs = X_mpc(N*bar_states + 1:N*bar_states + 3*legs, 1);
rrf_legs = zeros(3*legs, 1);

ypr = X(4:6);
Rot = eul2rotm(ypr');


for ind = 0:legs-1
    rrf_legs(ind*3 + 1: ind*3 + 3,1) = -1 * Rot'*grf_legs(ind*3 + 1: ind*3 + 3,1);
end

end

