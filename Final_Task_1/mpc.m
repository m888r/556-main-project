    %{
        X : states [P; yaw; pitch; roll; dP; dw]
        pf : foot positions [pf1; pf2; pf3; pf4]
        Xd : desired states [Pd; yawd; pitchd; rolld; dPd; dwd]
        gait : contact vector 0/1
    %}

function grf_forces = mpc(X, pf, Xd, gait) 
    N = 3;
    dt = [];
    Q = [];
    R = [];
    F_max = [];
    F_min = [];
    I_b = [];

    P = X(1:3);
    ypr = X(4:6);
    X_bard = [Xd; 9.81];
    X_bar = [X; 9.81];
    forces = 4;
    bar_states = 13;
    Xmpc_elem = bar_states*N + forces*3*N;
        
    % Cost function parameters
    temp_fQs = [];
    for ind = 1:N
        temp_Qs = [temp_Qs; diag(Q)];
        temp_Rs = [temp_Rs; diag(R)];
        temp_fQs = [temp_fQs; -Q'*X_bard];
    end
    H = diag([temp_Qs; temp_Rs]);
    f = [temp_fQs; zeros(3*forces*N, 1)];

    % Inequality constraints A_iq
    Fiq_mat = [-1, 0, -mu; 1, 0, -mu; 0, -1, -mu; 0, 1, -mu; 0, 0, 1; 0, 0, -1];
    A_iq = [zeros(6*forces*N, bar_states*N), kron(eye(forces*N),Fiq_mat)];


    % Inequality constraints b_iq
    b_iq = [];
    for ind = 1:forces*N
        alpha = gait(ind);
        b_iq = [b_iq; 0; 0; 0; 0; alpha*F_max; -alpha*F_min];
    end

    % Linearization
    % [Yaw, Pitch, Roll]
    psi = ypr(1);
    theta = ypr(2);
    phi  = ypr(3);

    Rz_T = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1];
    I_w = Rz_T'*params.I_b*Rz_T;

    A_bar = zeros(states);
    A_bar(1:3, 3*2+1:3*3) = eye(3);
    A_bar(3+1:3*2, 3*3+1:3*4) = Rz_T;
    A_bar(3*3, states) = -1;

    % Position of feet in world frame
    r_f1 = pf(1:3) - P;
    r_f2 = pf(4:6) - P;
    r_f3 = pf(7:9) - P;
    r_f4 = pf(10:12) - P;
    r_fs = [r_f1, r_f2, r_f3, r_f4];
    temp_mat = [];
    for ind = 1:forces
        temp_r = r_fs(:, ind);
        temp = [0, -temp_r(3), temp_r(2); temp_r(3), 0, -temp_r(1); -temp_r(2), temp_r(1), 0];
        temp_term = inv(I_w)*temp;
        temp_mat = [temp_mat, temp_term];
    end
    B_bar = [zeros(3*2, 3*forces); [eye(3)/m, eye(3)/m, eye(3)/m, eye(3)/m]; temp_mat; zeros(1, 3*forces)];

    % Discretization
    A_k = A_bar*dt + eye(states);
    B_k = B_bar*dt;

    % MPC Dynamic Constraints
    A_eq = zeros(states, Xmpc_elem - 3*forces*N);
    A_eq(1:states, 1:states) = eye(states);
    A_Xrot = [-A_k, eye(states)];
    A_Xrot = [A_Xrot, zeros(states, states*(N-2))];
    % Shift for each 
    for ind = 1:N-1
        A_eq = [A_eq; A_Xrot];
        A_Xrot = circshift(A_Xrot,states, 2);
    end
    for ind = 1:N
        temp_vec = zeros(Xmpc_elem - 3*forces*N, 1);
        start = 1 + states*(ind - 1);
        temp_vec(start:start+states-1, 1:3*forces) = -B_k; 
        A_eq = [A_eq, temp_vec];
    end

    b_eq = zeros(Xmpc_elem - 3*forces*N, 1);
    b_eq(1:states) = A_k*X_bar;


    X_mpc = quadprog(H, f, A_iq, b_iq, A_eq, b_eq);
    grf_forces = X_mpc(N*states + 1:N*states + 3*forces, 1);
end