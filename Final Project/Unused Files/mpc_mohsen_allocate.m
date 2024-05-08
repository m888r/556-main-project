function [rrf_legs, grf_legs] = mpc_mohsen_allocate(X, Xd, pf, t, N, dt, ftcontacts)
coder.extrinsic('quadprog');
warningState = warning('off', 'all');

%gaitname = 'standing';
m = 12;
Ib = diag([0.0168, 0.0565, 0.064]);
state_num = 13;
controller_num = 12;
legs = 4;
bar_states = 13;
F_max = 500;
F_min = 10;
X_bar = [X; 9.81];
Xmpc_elem = bar_states*N + legs*3*N;

% Q, R values from HW4 #2(b)
 Q_mpc = diag([40, 50, 60, 10, 10, 10, 4, 4, 4, 1, 1, 1]);

% test tuning for trotting in place
%Q = diag([10, 10, 10, 30, 30, 30, 4, 4, 4, 1, 1, 1, 0]);

% test tuning for trotting forward
% Q = diag([40, 50, 60, 10, 10, 10, 4, 4, 4, 1, 1, 1, 0]);

R_mpc = 0.00001*eye(4*3);

p = X(1:3);
eul = X(4:6);
R = eul2rotm(eul');

r1 = pf(1:3) - p;
r2 = pf(4:6) - p;
r3 = pf(7:9) - p;
r4 = pf(10:12) - p;

yaw = eul(1);
Rz_yaw = [cos(yaw) sin(yaw) 0;
    -sin(yaw) cos(yaw) 0;
    0 0 1];
Act = [zeros(3) zeros(3) eye(3) zeros(3) zeros(3,1); zeros(3) zeros(3) zeros(3) Rz_yaw zeros(3,1);
    zeros(3) zeros(3) zeros(3) zeros(3) [0;0;-1];
    zeros(4,13)];
Iw = R'*Ib*R;
D = [m*eye(3) zeros(3);
    zeros(3) Iw];
A = [eye(3), eye(3), eye(3), eye(3);
    Vec2Skewmat(r1),Vec2Skewmat(r2),Vec2Skewmat(r3),Vec2Skewmat(r4)];
Bct = [zeros(6,12);
    D\A;
    zeros(1,12)];
At = eye(state_num)+dt*Act;
Bt = dt*Bct;
Q_mpc = blkdiag(Q_mpc,0);
xd = [Xd; 9.81];


vec_Q = diag(Q_mpc);
temp_Qs = vec_Q(:, ones(N, 1));
vec_R = diag(R_mpc);
temp_Rs = vec_R(:, ones(N, 1));
temp_fQ = -Q_mpc'*xd;
temp_fQs = temp_fQ(:, ones(N, 1));
H = diag([temp_Qs(:); temp_Rs(:)]);

f_blk = -Q_mpc*xd;
f1 = zeros(state_num*N, 1);
for i = 1:N
    f1(i*state_num - 12 : i*state_num) = f_blk;
end
f = [f1; zeros(N*controller_num,1)];

A_eq1 = zeros(bar_states*N, bar_states*N);
A_eq1(1:bar_states, 1:bar_states) = eye(bar_states);
A_Xrot = [-At, eye(bar_states), zeros(bar_states, bar_states*(N-2))];
% Shift for each
for ind = 1:N-1
    A_eq1(ind*bar_states + 1:ind*bar_states + bar_states, 1:bar_states*N) = A_Xrot;
    A_Xrot = circshift(A_Xrot,bar_states, 2);
end
A_eq2 = zeros(bar_states*N, 3*legs*N);
for ind = 1:N
    temp_vec = zeros(Xmpc_elem - 3*legs*N, 3*legs);
    start = 1 + bar_states*(ind - 1);
    temp_vec(start:start+bar_states-1, :) = -Bt;
    A_eq2(:, ind*3*legs - 3*legs + 1: ind*3*legs) = temp_vec;
end
Aeq = [A_eq1, A_eq2];


Beq = [At*[X;9.81]; zeros((N-1)*state_num,1)];

mu = 0.5;
% Inequality constraints A_iq
Fiq_mat = [1, 0, -mu; -1, 0, -mu; 0, 1, -mu; 0, -1, -mu; 0, 0, 1; 0, 0, -1];
A_ineq = [zeros(6*legs*N, bar_states*N), kron(eye(legs*N),Fiq_mat)];

% Inequality constraints b_iq
b_ineq = zeros(6*N*legs, 1);
for ind = 1:6:6*N*legs
    alpha = ftcontacts(floor(ind/6) + 1);
    b_ineq(ind:ind + 5, 1) = [0; 0; 0; 0; alpha*F_max; -alpha*F_min];
end

options = optimoptions('quadprog','Display','off');
%X_star = quadprog(H,f,A_ineq,b_ineq,Aeq,Beq,[],[],[],options);
X_star = quadprog(H,f,A_ineq,b_ineq,Aeq,Beq);
grf_legs = X_star(N*state_num+1:N*state_num+controller_num);

grf_legs_fixed = zeros(12, 1);
rrf_legs = zeros(3*legs, 1);
for ind = 0:legs-1
    % grf_legs_fixed(ind*3 + 1, 1) = grf_legs(ind*3+1, 1);
    % grf_legs_fixed(ind*3 + 2, 1) = grf_legs(ind*3+2, 1) * -1;
    % grf_legs_fixed(ind*3 + 3, 1) = grf_legs(ind*3+3, 1);
    rrf_legs(ind*3 + 1: ind*3 + 3,1) = -1 * R'*grf_legs(ind*3 + 1: ind*3 + 3,1);
end

end


function mpcTable = gait(t,N,dt,gaitname)
if isequal(gaitname, 'standing')
    offsets = [0,0,0,0];
    duration = [N,N,N,N];
elseif isequal(gaitname, 'trotting')
    offsets = [0,N/2,N/2,0];
    duration = [N/2,N/2,N/2,N/2];
elseif isequal(gaitname, 'bounding')
    offsets = [N/2,N/2,0,0];
    duration = [N/2,N/2,N/2,N/2];
end
mpcTable = zeros(N*4,1);
iteration = floor(mod(t/dt,N));
for i = 0:N-1
    iter = mod((i +1 + iteration),N);
    progress = iter - offsets;
    for j = 1:4
        if progress(j) < 0
            progress(j) = progress(j) + N;
        end
        if progress(j) < duration(j)
            mpcTable(i*4+j) = 1;
        else
            mpcTable(i*4+j) = 0;
        end
    end
end
end

function S = Vec2Skewmat(a)
S = [0 -a(3) a(2);
    a(3) 0 -a(1);
    -a(2) a(1) 0];
end
