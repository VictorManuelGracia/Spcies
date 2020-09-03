%% This function computes the ingredients of the EADMM algorithm for MPCT
% We generate three pairs of structures.


function str = Spcies_compute_MPCT_EADMM_ingredients(sys, param, options)

    %% Variable extraction
    n = sys.n_x;
    m = sys.n_u;
    A = sys.A;
    B = sys.Bu;  
    N = param.N;
    rho = param.beta;
    Q = param.Q;
    R = param.R;
    T = param.T;
    S = param.S;
    if isfield(param, 'epsilon_x')
        epsilon_x = param.epsilon_x;
    else
        epsilon_x = 1e-6;
    end
    if isfield(param, 'epsilon_u')
        epsilon_u = param.epsilon_u;
    else
        epsilon_u = 1e-6;
    end
    if isfield(param, 'inf_bound_value')
        inf_bound_value = param.inf_bound_value;
    else
        inf_bound_value =  100000;
    end
    if  isfield(param, 'x0_bound_value')
        x0_bound_value = param.x0_bound_value;
    else
        x0_bound_value = 100000;
    end
    
    %% Decision variables
    z1_0 = zeros((N+1)*(n+m), 1); % z1 = (xi, ui)
    z2_0 = zeros(n+m, 1); % z2 = (xs, us)
    z3_0 = zeros((N+1)*(n+m), 1); % z3 = (hat_xi, hat_ui)
    lambda_0 = zeros((N+1)*(n+m) + n + 1*(n+m), 1); % Dual variables
    
    %% Matrices A1, A2 and A3
    
    % A1 : z2 = (xi, ui)
    A1 = -[ [-eye(n) zeros(n, N*(n+m)+m)]; eye((N+1)*(n+m)); [zeros(n+m, N*(n+m)) eye(n+m)]];
    
    % A2 : z1 = (xs, us)
    A2 = [zeros(n, n+m); kron(ones(N,1), eye(n+m)); kron(ones(2,1), eye(n+m))];

    % A3 : z3 = (hat_xi, hat_ui)
    A3 = [zeros(n, (N+1)*(n+m)); eye((N+1)*(n+m)); zeros(n+m, (N+1)*(n+m))];
    
    % b
    b = zeros((N+1)*(n+m) + n + 1*(n+m), 1);
    
    %% Problem 1: z1 = (xi, ui)
    
    % QP1 matrices
    H1 = zeros((N+1)*(n+m)) + (rho.*A1)'*A1;
    q1 = zeros((N+1)*(n+m), 1); % This has to be updated using A1, A2, A3, b, and the values of z1 and z3
    Az1 = [];
    bz1 = [];
    LB1 = [kron(ones(N+1,1), [sys.LBx; sys.LBu])]; 
    UB1 = [kron(ones(N+1,1), [sys.UBx; sys.UBu])];
    
    % Ingredients with minimal memory consumption
    H1i = 1./diag(H1);
    
    %% Problem 2: z2 = (xs, us)
    H2 = blkdiag(T, S) + (rho.*A2)'*A2;
    q2 = zeros(n+m, 1); % This has to be updated using A1, A2, A3, b, and the values of z2 and z3
    Az2 = [(A - eye(n)) B];
    bz2 = zeros(n,1);
    LB2 = [sys.LBx; sys.LBu];
    UB2 = [sys.UBx; sys.UBu];
    
    % Matrices for explicit solution
    H2i = inv(H2);
    W2 = H2i*Az2'*inv(Az2*H2i*Az2')*Az2*H2i - H2i;
    
    %% Problem 3: z3 = (hat_xi, hat_ui)
    H3 = kron(eye(N+1), blkdiag(Q, R)) + (rho.*A3)'*A3;
    q3 = zeros((N+1)*(n+m), 1);
    LB3 = [];
    UB3 = [];

    Az3 = kron(eye(N), [A B]); % Diagonal de la matriz
    j = 0;
    for i=1:n:n*N-n % Inserto las matrices -I en Az
        j = j+1;
        Az3(i:i+n-1,((j-1)*(n+m)+(m+n+1)):((j-1)*(n+m)+(n+m+1)+n-1)) = -eye(n);
    end
    Az3 = [Az3 [zeros((N-1)*n, n); -eye(n)] zeros(N*n, m)];
    bz3 = zeros(N*n, 1); % This needs to be updated for the initial state

    % Matrices for explicit solution
    W3 = Az3*inv(H3)*Az3';
    W3c = chol(W3);
    
    % Ingredients with minimal memory consumption
    H3i = 1./diag(H3);
       
    %% Warmstart
    
    % Dimensions
    dim_z1 = size(H1, 1);
    dim_z2 = size(H2, 1);
    dim_z3 = size(H3, 1);
    dim_l = size(A1, 1);
    dim_w = dim_z1 + dim_z2 + dim_z3 + dim_l;
    
    % Equality constraints
    Aeq = [A1 A2 A3];
    beq = b;
    
    % The cost function that would result from grouping z = [z1; z2; z3; lambda] only due to the \theta_i functions
    H = blkdiag(zeros(dim_z1), blkdiag(T, S), kron(eye(N+1), blkdiag(Q, R)), zeros(dim_l));
    %q = [zeros(dim_z1, 1); [-T*xr; -S*ur]; zeros(dim_z3 + dim_l)];
    
    S_l = zeros(dim_l, dim_w); % Selects lambda from w, i.e. lambda = S_l*w
    S_l(:, dim_z1 + dim_z2 + dim_z3 + (1:dim_l)) = eye(dim_l);
    
    S_z = zeros(dim_z1 + dim_z2 + dim_z3, dim_w);
    S_z(:, 1:dim_z1 + dim_z2 + dim_z3) = eye(dim_z1 + dim_z2 + dim_z3);
    
    % Matrix B such that b = B*x
    Bz = zeros(dim_l, n);
    Bz(1:n, 1:n) = eye(n);
    
    % Matrices of the partial derivatives
    H_hat = 0.5*H - S_l'*Aeq*S_z;
    H_hat_inv = inv(H_hat + H_hat');
    
    % Prediction step matrix. w_{k+1} = w - Ln*(x_{k+1} - x_k)
    L = H_hat_inv*S_l'*Bz;
    
    % Extract from the matrix Ln
    L_z1 = L(1:dim_z1, :);
    L_z2 = L(dim_z1 + (1:dim_z2),:);
    L_z3 = L(dim_z1 + dim_z2 + (1:dim_z3), :);
    L_l = L(dim_z1 + dim_z2 + dim_z3 + (1:dim_l), :);
    
    %% Construct str
    str.N = N; % Prediction horizon
    str.n = n; % Dimension of state space
    str.m = m; % Dimension of input space
    str.H1i = reshape(H1i, m+n, N+1); % Inverse of diagonal of H1 in matrix form
    str.H3i = reshape(H3i, m+n, N+1); % Inverse of 
    str.AB = [A B]; % Matrices of the system model
    str.W2 = W2;
    str.T = -T;
    str.S = -S;
    str.LB = [sys.LBx; sys.LBu];
    str.LB(isinf(str.LB)) = -inf_bound_value;
    str.UB = [sys.UBx; sys.UBu];
    str.UB(isinf(str.UB)) = inf_bound_value;
    str.LBs = [sys.LBx + epsilon_x*ones(n, 1); sys.LBu + epsilon_u*ones(m, 1)];
    str.UBs = [sys.UBx - epsilon_x*ones(n, 1); sys.UBu - epsilon_u*ones(m, 1)];
    str.LB0 = [-x0_bound_value*ones(n,1); sys.LBu];
    str.UB0 = [x0_bound_value*ones(n,1); sys.UBu];
    
    % Penalty parameter
%         rho_b = zeros((N+1)*(n+m), 1);
%         rho_b(1:n) = rho(1:n);
%         rho_b(n+(1:m)) = rho(2*n+(1:m));
%         rho_b(n+m+(1:(N-1)*(n+m))) = rho(2*n+m+(1:(N-1)*(n+m)));
%         rho_b(N*(n+m) + (1:n+m)) = rho(end-n-m+1:end);
%     str.rho = reshape(rho_b, m+n, N+1);

    str.beta = rho(n+1:end-n-m);
    str.beta_0 = rho(1:n);
    str.beta_s = rho(end-n-m+1:end);
    
    % Warmstart
    str.L_z2 = L_z2;
    str.L_z3 = L_z3(1:n, :);
    str.L_l = L_l(1:2*n, :); 
    
    % Scaling vectors
    str.scaling = [sys.Nx; sys.Nu];
    str.scaling_inv_u = 1./sys.Nu;
    str.OpPoint = [sys.x0; sys.u0];
    
    % Alpha and Beta
    str.Beta = zeros(n,n,N);
    str.Alpha = zeros(n,n,N-1);
    for i = 1:N
        str.Beta(:,:,i) = W3c((i-1)*n+(1:n),(i-1)*n+(1:n));
        for j = 1:n
            str.Beta(j,j,i) = 1/str.Beta(j,j,i);
        end
    end
    for i = 1:N-1
        str.Alpha(:,:,i) = W3c((i-1)*n+(1:n),i*n+(1:n));
    end
    
end
