%% compute_MPCT_x1_free_ADMM_ingredients
%
% Computes the ingredients for the double-first predicted-state MPCT
% formulation, solved using ADMM exploiting the semi-banded structure of
% the problem using the Woodbury Matrix Identity.
% 
% TODO: PONER NOMBRE DEL ARTÍCULO SI LO ACEPTAN 
% 
% INPUTS:
%   - controller: Contains the information of the controller
%   - opt: Structure containing optins of the solver.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredints required by the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [vars] = compute_MPCT_x1_free_ADMM_ingredients(controller, opt)

    %% Extract from controller
    if isa(controller, 'TrackingMPC')
        A = controller.model.A;
        B = controller.model.Bu;
        n = controller.model.n_x;
        m = controller.model.n_u;
        N = controller.N;
        Q = controller.Q;
        R = controller.R;
        T = controller.T;
        S = controller.S;
        LBx = controller.model.LBx;
        LBu = controller.model.LBu;
        UBx = controller.model.UBx;
        UBu = controller.model.UBu;    
    else
        A = controller.sys.A;
        if isa(controller.sys, 'ssModel')
            B = controller.sys.Bu;
        else
            B = controller.sys.B;
        end
        n = size(A, 1);
        m = size(B, 2);
        N = controller.param.N;
        Q = controller.param.Q;
        R = controller.param.R;
        T = controller.param.T;
        S = controller.param.S;
        try
            LBx = controller.sys.LBx;
        catch
            LBx = -options.inf_bound*ones(n, 1);
        end
        try
            UBx = controller.sys.UBx;
        catch
            UBx = opt.inf_value*ones(n, 1);
        end
        try
            LBu = controller.sys.LBu;
        catch
            LBu = -opt.inf_value*ones(m, 1);
        end
        try
            UBu = controller.sys.UBu;
        catch
            UBu = opt.inf_value*ones(m, 1);
        end
    end

    %% Dimension of the problem
    n_z = (N+2)*n+(N+1)*m; % Number of decision variables
    m_z = (N+2)*n; % Number of equality constraints

    %% Turn rho into a vector
    if isscalar(opt.solver.rho) && opt.solver.force_vector_rho
        rho = opt.solver.rho*ones(n_z);
    else
        rho = opt.solver.rho;
    end
    if isscalar(rho)
        vars.rho_is_scalar = true;
    else
        vars.rho_is_scalar = false;
    end

    %% Get alpha and beta
    alpha = opt.solver.alpha;
    beta = opt.solver.beta;
    
    %% Compute the Hessian
    Gamma_hat = blkdiag(Q,R,Q);
    for i = 2:N
        Gamma_hat = blkdiag(Gamma_hat,blkdiag(Q,R));
    end
    Gamma_hat = blkdiag(Gamma_hat,blkdiag((N+1)*Q+T,N*R+S));
    Gamma_hat = Gamma_hat + diag(rho)*eye(size(Gamma_hat,1)); % Band of the Hessian of the problem of z^{k+1} constructed

    Gamma_hat_inv = inv(Gamma_hat);
    
    Y = [-blkdiag(Q,R), [-Q ; zeros(m,n)], kron(-ones(1,N-1),blkdiag(Q,R))];
    
    U_hat = [Y', zeros(size(Y,2),n+m) ; zeros(n+m,n+m), eye(n+m)];
    
    V_hat = [zeros(n+m,size(Y,2)), eye(n+m); Y, zeros(n+m,n+m)];

    % Note that P = Gamma_hat + U_hat*V_hat = H + rho*I (P is semi-banded)

    %% Compute equality constraints (G*dec_var = beq)
    G = zeros(m_z,n_z);

    G(1:n,1:n) = eye(n);
    
    G(n+1:2*n,1:n+m+n) = [A, B, -eye(n)]; % Constraints of system dynamics: \hat{x}_{1} = A*x_{0} + B*u_{0}
    
    k=1;
    for i = 2*n+1:n:m_z-n
        G(i:i+n-1,i+k*m:i+k*m+n+m+n-1) = [A B -eye(n)]; % Constraints of system dynamics: x_{k+1} = A*x_{k} + B*u_{k}
        k=k+1;
    end
    
    G(m_z-n+1:m_z,n_z-n-m+1:n_z) = [(A-eye(n)) B];% Condition of (x_s,u_s) as an equilibrium point
    
    Gamma_tilde = G*Gamma_hat_inv*G';

    U_tilde_full = -G*Gamma_hat_inv*U_hat*inv(eye(2*(n+m))+V_hat*Gamma_hat_inv*U_hat);

    if vars.rho_is_scalar
        U_tilde_ini = [U_tilde_full(1:n,1:n), U_tilde_full(1:n,n+m+1:2*n+m)];
        U_tilde_mid = U_tilde_full(n+1:2*n,:);
        U_tilde_final = U_tilde_full(N*n+1:(N+2)*n,:);
    else
        U_tilde = U_tilde_full;
    end

    V_tilde = V_hat*Gamma_hat_inv*G';

    % Verification: Gamma_tilde + U_tilde*V_tilde == G*inv(P)*G'

    % Computation of M_hat = inv(I+V_hat*inv(Gamma_hat)*U_hat)*V_hat (step 2 of algorithm for semi-banded linear systems)
    M_hat = inv(eye(size(V_hat,1),size(U_hat,2))+V_hat*Gamma_hat_inv*U_hat)*V_hat;

    % For C, we only save once the repeated part of the matrix
    M_hat_x1 = [M_hat(1:n,1:n) ; M_hat(n+m+1:2*n+m,1:n)];
    M_hat_x2 = [M_hat(1:n,N*(n+m)+n+1:N*(n+m)+2*n) ; M_hat(n+m+1:2*n+m,N*(n+m)+n+1:N*(n+m)+2*n)];

    M_hat_u1 = [M_hat(n+1:n+m,n+1:n+m) ; M_hat(2*n+m+1:2*(n+m),n+1:n+m)];
    M_hat_u2 = [M_hat(n+1:n+m,N*(n+m)+2*n+1:(N+1)*(n+m)+n) ; M_hat(2*n+m+1:2*(n+m),N*(n+m)+2*n+1:(N+1)*(n+m)+n)];

    M_tilde_full = inv(eye(size(V_tilde,1),size(U_tilde_full,2))+V_tilde*inv(Gamma_tilde)*U_tilde_full)*V_tilde;

    % Only for C version of the solver. In Matlab, we use M_tilde_full
    if vars.rho_is_scalar
        M_tilde = [M_tilde_full(:,1:2*n), M_tilde_full(:,N*n+1:(N+2)*n)]; % For C, we only save once the repeated part of the matrix
    else
        M_tilde = M_tilde_full;
    end

    
    %% Compute upper and lower bounds
    LB = [LBx;LBu]; % Lower bounds for predicted states and inputs
    UB = [UBx;UBu]; % Upper bounds for predicted states and inputs
    % Note that the bounds for \hat{x}_1 will be implemented using soft constraints

    %% Create variables used in the ADMM_semiband solver for MPCT
    vars.N = N;  % Prediction horizon
    vars.n = n; % Dimension of state space
    vars.m = m; % Dimension of input space

    vars.A = A;
    vars.B = B;
    vars.Q = Q;
    vars.R = R;
    vars.T = T;
    vars.S = S;
    vars.G = G;
    vars.U_hat = U_hat;
    vars.Gamma_tilde = Gamma_tilde; % Only needed for solver in Matlab. In C, Alpha's and Beta's are used
    vars.U_tilde_full = U_tilde_full;
    if vars.rho_is_scalar
        vars.U_tilde_ini = U_tilde_ini;
        vars.U_tilde_mid = U_tilde_mid;
        vars.U_tilde_final = U_tilde_final;
    else
        vars.U_tilde = U_tilde;
    end
    vars.M_hat = M_hat;
    vars.M_hat_x1 = M_hat_x1;
    vars.M_hat_x2 = M_hat_x2;
    vars.M_hat_u1 = M_hat_u1;
    vars.M_hat_u2 = M_hat_u2;
    vars.M_tilde_full = M_tilde_full;
    vars.M_tilde = M_tilde;
    vars.LB = LB;
    vars.UB = UB;
    vars.rho = rho;

    if vars.rho_is_scalar
        vars.rho_i = 1/rho;
        vars.Q_rho_i = inv(Q + rho*diag(ones(n,1)));
        vars.R_rho_i = inv(R + rho*diag(ones(m,1)));
        vars.S_rho_i = inv(N*R + S + rho*diag(ones(m,1)));
        vars.T_rho_i = inv((N+1)*Q + T + rho*diag(ones(n,1)));
        vars.alpha_rho_i = alpha/(2*rho);
        vars.beta_rho_i = beta/(2*rho);
    else
        vars.rho_i = 1./rho;
        vars.Q_rho_i = zeros(n,n,N+1);
        vars.R_rho_i = zeros(m,m,N);

        vars.Q_rho_i(:,:,1) = Gamma_hat_inv(1:n,1:n);
        vars.R_rho_i(:,:,1) = Gamma_hat_inv(n+1:n+m,n+1:n+m);
        vars.Q_rho_i(:,:,2) = Gamma_hat_inv(n+m+1:n+m+n,n+m+1:n+m+n);

        for i = 3:N+1
            vars.Q_rho_i(:,:,i) = Gamma_hat_inv(n+(i-2)*(n+m)+1:n+(i-2)*(n+m)+n,n+(i-2)*(n+m)+1:n+(i-2)*(n+m)+n);
            vars.R_rho_i(:,:,i-1) = Gamma_hat_inv(n+(i-2)*(n+m)+n+1:n+(i-2)*(n+m)+n+m,n+(i-2)*(n+m)+n+1:n+(i-2)*(n+m)+n+m);
        end

        vars.T_rho_i = Gamma_hat_inv(n+N*(n+m)+1:n+N*(n+m)+n,n+N*(n+m)+1:n+N*(n+m)+n);
        vars.S_rho_i = Gamma_hat_inv(n+N*(n+m)+n+1:n+N*(n+m)+n+m,n+N*(n+m)+n+1:n+N*(n+m)+n+m);

        vars.alpha_rho_i = alpha./(2*rho);
        vars.beta_rho_i = beta./(2*rho);
    end

    % Scaling vectors and operating point
    if isa(controller, 'TrackingMPC')
        vars.scaling_x = controller.model.Nx;
        vars.scaling_u = controller.model.Nu;
        vars.scaling_i_u = 1./controller.model.Nu;
        vars.OpPoint_x = controller.model.x0;
        vars.OpPoint_u = controller.model.u0;
    else
        if isfield(controller.sys, 'Nx')
            vars.scaling_x = controller.sys.Nx;
        else
            vars.scaling_x = ones(n, 1);
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling_u = controller.sys.Nu;
        else
            vars.scaling_u = ones(m, 1);
        end
        if isfield(controller.sys, 'Nu')
            vars.scaling_i_u = 1./controller.sys.Nu;
        else
            vars.scaling_i_u = ones(m, 1);
        end
        if isfield(controller.sys, 'x0')
            vars.OpPoint_x = controller.sys.x0;
        else
            vars.OpPoint_x = zeros(n, 1);
        end
        if isfield(controller.sys, 'u0')
            vars.OpPoint_u = controller.sys.u0;
        else
            vars.OpPoint_u = zeros(m, 1);
        end
    end

    % Alpha and Beta from Cholesky Decomposition
    n_Beta = m_z/n; % Number of Beta's
    n_Alpha = n_Beta-1; % Number of Alpha's

    vars.Beta = zeros(n,n,n_Beta);
    vars.Alpha = zeros(n,n,n_Alpha);

    % Extract Alpha's and Beta's from Gamma_tilde_c
    Gamma_tilde_c = chol(Gamma_tilde);

    for i = 1 : n : m_z
        vars.Beta(:,:,(i-1)/n+1) = Gamma_tilde_c(i:i+n-1,i:i+n-1);
        if((i-1)/n+1 <= n_Alpha) % If we are in the last column, we do not add a new Alpha
            vars.Alpha(:,:,(i-1)/n+1) = Gamma_tilde_c(i:i+n-1,i+n:i+2*n-1);
        end
    end
    
    % Deleting Alpha(:,:,2), which is always zero
    for i = 2 : n_Alpha-1
        vars.Alpha(:,:,i) = vars.Alpha(:,:,i+1);
    end

    vars.Alpha = vars.Alpha(:,:,1:end-1);

    % Passing the inverse of the diagonal of Beta's so that we multiply by them instead
    % of dividing. Used in solve_banded_Chol() function in C
    for i = 1 : n 
        vars.Beta(i,i,:) = 1/vars.Beta(i,i,:);
    end
    


end

