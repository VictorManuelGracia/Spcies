%% sp_test_MPCT
% 
% This function tests the behaviour of MPCT when implemented with the
% ADMM_semiband solver
% 
% @result: Struct containing '1' if the solution vector provided by the 
% Spcies solver resembles the solution provided by Yalmip, '0' otherwise
% 
% @solvers_options: Structure containing options for both Spcies and Yalmip solvers

function [correct, results] = sp_test_MPCT(solvers_options)

    % Create the system in which the solvers are tested
    p = 3; % Number of objects
    M = [1; 0.5; 1]; % Mass of each object
    K = 2*ones(p+1, 1); % Spring constant of each spring
    F = [1; zeros(p-2, 1); 1]; % Objects in which an external force is applied
    
    sysC = sp_utils.gen_oscillating_masses(M, K, F); % Generate the continuous-time model
    
    n = size(sysC.A, 1); % For convenience, we save the size of the state dimension
    m = size(sysC.B, 2); % and of the input dimension
    
    % Obtain a discrete-time state space model
    Ts = 0.2; % Select the sample time
    sysD = c2d(sysC, Ts); % Discrete-time state space model
    
    % Set some bounds for the state and input
    LBx = -[ones(p, 1); 0.5*ones(p, 1)]; % Lower bound for the state
    UBx = [0.3; 0.3; 0.3; 0.5*ones(p, 1)]; % Upper bound for the state
    LBu = -[0.8; 0.8]; % Lower bound for the input
    UBu = [0.8; 0.8]; % Upper bound for the input

    % Create sys structure
    sys = struct('A', sysD.A, 'B', sysD.B, 'LBx', LBx, 'UBx', UBx, 'LBu', LBu, 'UBu', UBu);
    
    % Cost function matrices
    Q = 10*ones(n,n) + diag(ones(n,1));
    R = 0.1*ones(m,m) + diag(ones(m,1));

    T = 10*ones(n,n) + diag(ones(n,1));
    S = 0.1*ones(m,m) + diag(ones(m,1));

    % Prediction horizon
    N = 10;

    % Create parameters structure9
    param.N = N;
    param.Q = Q;
    param.R = R;
    param.S = S;
    param.T = T;

    % Compute an admissible reference
    ur(1,1) = 0.1;
    ur(2,1) = 0.2;
    xr = -(sysD.A-eye(n))\(sysD.B*ur);    

    % Yalmip options
    external_opt.tol = solvers_options.tol;

    % Test the solvers
    % Simulate the solver with n_sim random initial states per value of rho

    x0 = [-0.05; 0.1 ; -0.10 ; -0.01 ; -0.03 ; -0.04]; %Initial state inside the set of constraints

    [sol,ok] = sp_solve_MPCT_external(sys,param,x0,xr,ur,external_opt); % Take the optimal solution from "sol" structure

    if (ok == false)
        error('External solver did not converge')
    end

    % Group the solution of the external solver in a proper format
    l=0;
    
    for k = 1:n+m:N*(n+m)
        l = l+1;
        external_sol.x(:,l) = sol{1,1}(:,l);
        external_sol.u(:,l) = sol{1,2}(:,l);
    end

    external_sol.xs = sol{1,3};
    external_sol.us = sol{1,4};

    % Call the specific formulation/method/submethod Spcies solver
    [correct(1), results(1)] = sp_test_MPCT_ADMM_semiband(sys,param,x0,xr,ur,external_sol,solvers_options);
    % [correct(2), results(2)] = sp_test_MPCT_ADMM_cs(x0,xr,ur,external_sol,options);
    
end

