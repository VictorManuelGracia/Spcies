%% sp_gen_MPCT_YALMIP
%
% external_MPCT_solver = sp_gen_MPCT_ADMM_semiband_YALMIP(sys, param, opt, solver_name)
%
% Returns a solver of the MPCT_ADMM_semiband formulation based on Yalmip
% 
% @sys: Structure containing the system model
% @param: Structure containing parameters of the MPCT controller
% @opt: Structure containing Yalmip options
% @solver_name: String containing the solver name (supported by Yalmip, e.g., OSQP)
% 
% @external_MPCT_solver: yalmip::optimizer function that contains the solver for MPCT

function external_MPCT_solver = sp_gen_MPCT_YALMIP(sys, param, opt, solver_name)

    yalmip('clear')

    % Get system parameters
    n = size(sys.A,1);
    m = size(sys.B,2);
    p = size(sys.C,1);

    % Get MPCT parameters
    N = param.N;

    %% Create yalmip decision variables
    x = sdpvar(repmat(n,1,N),repmat(1,1,N)); % States
    u = sdpvar(repmat(m,1,N),repmat(1,1,N)); % Inputs
    xs = sdpvar(repmat(n,1,1),repmat(1,1,1)); % Artificial reference for the states
    us = sdpvar(repmat(m,1,1),repmat(1,1,1)); % Artificial reference for the inputs
    xr = sdpvar(repmat(n,1,1),repmat(1,1,1)); % Reference for the states
    ur = sdpvar(repmat(m,1,1),repmat(1,1,1)); % Reference for the inputs

    constraints = []; % Set of all contraints
    objective = 0; % Objective function

    %% Construct contraints and objective function
    for k = 1:N
        objective = objective + (x{k}-xs)'*param.Q*(x{k}-xs);
        objective = objective + (u{k}-us)'*param.R*(u{k}-us);
    end
    
    objective = objective + (xs-xr)'*param.T*(xs-xr) + (us-ur)'*param.S*(us-ur);

    for k = 1:N-1
        constraints = [constraints , x{k+1} == sys.A*x{k} + sys.B*u{k}];
        constraints = [constraints , sys.LBx <= x{k+1} <= sys.UBx];
        constraints = [constraints , sys.LBu <= u{k} <= sys.UBu];
        % constraints = [constraints , sys.LBy <= sys.C*x{k}+sys.D*u{k} <= sys.UBy];
    end

    constraints = [constraints , sys.LBu <= u{N} <= sys.UBu];

    constraints = [constraints , xs == sys.A*xs + sys.B*us];

    constraints = [constraints , xs == sys.A*x{N} + sys.B*u{N}];

    constraints = [constraints , (sys.LBx+1e-6*ones(n,1)) <= xs <= (sys.UBx-1e-6*ones(n,1))]; % Smaller constraint for xs
    constraints = [constraints , (sys.LBu+1e-6*ones(m,1)) <= us <= (sys.UBu-1e-6*ones(m,1))]; % Smaller constraint for us
    % constraints = [constraints , (sys.LBy+1e-6*ones(p,1)) <= sys.C*xs+sys.D*us <= (sys.UBy-1e-6*ones(p,1))]; % Smaller constraint for us

    %% Solver options
    if solver_name == 'osqp'
        yalmip_options = sdpsettings('solver' , solver_name ,'savesolveroutput' , 1 ,'verbose', 0);
        yalmip_options.osqp.eps_abs = opt.tol;
        yalmip_options.osqp.eps_rel = opt.tol;
        yalmip_options.osqp.eps_prim_inf = opt.tol;
        yalmip_options.osqp.eps_dual_inf = opt.tol;
    else
        error('The solver to be used should be OSQP');
    end

    %% Construct the optimizer object
    external_MPCT_solver = optimizer(constraints,objective,yalmip_options,{x{1},xr,ur},{[x{:}],[u{:}],xs,us});

end