%% sp_test_MPCT_ADMM_semiband
% 
% This function generates the Spcies solver for MPCT using ADMM_semiband
% and compares the solution given by the external solver with the solution
% obtained by Spcies
% 
% @correct: '1' if the MPCT_ADMM_semiband test passed, '0' otherwise
% @result: Structure containing the result of the MPCT_ADMM_semiband test
%   - formulation
%   - method
%   - submethod
%   - gap -> Measures used to determine that the solution is correct. Check: gap{i} <= max_gap
%   - error -> String that indicates the type of error. ='' if there is no error.
%   - sol -> Solution obtained from solver (sol.x = [x0, x1], sol.u, etc.)
%   - sol_external -> Solution obtained with the external solver
%   - opt -> To reproduce test
%   - version -> Tag of Spcies version used when the test was run
% 
% @sys: Structure containing the system model
% @param: Structure containing parameters of the MPCT controller
% @x0: Current state of the system
% @xr: Reference for the states
% @ur: Reference for the inputs
% @external_sol: Solution obtained by the external solver (yalmip using osqp)
% @sp_opt: Options for Spcies solver

function [correct,result] = sp_test_MPCT_ADMM_semiband(sys,param,x0,xr,ur,external_sol,sp_opt)

    % Get n, m and N
    n = size(xr,1);
    m = size(ur,1);
    N = param.N;

    % Spcies options
    sp_opt.tol = sp_opt.tol;
    sp_opt.debug = true;
    sp_opt.save_name = 'mpc_solver';
    sp_opt.directory = '';
    sp_opt.time = false;
    sp_opt.rho = 0.1;
    sp_opt.k_max = 3000;

    % Fill the result structure
    result.formulation = 'MPCT';
    result.method = 'ADMM';
    result.submethod = 'semiband';
    result.error = ''; % This value changes if there is any error along the test
    result.version = '';
    result.opt = '';
    result.sol_external = external_sol;

    %% Spcies solver

    spcies('clear');
        
    % Generate the Spcies solver
    spcies('gen','sys', sys, 'param', param, 'options', sp_opt, ...
         'platform', 'Matlab', 'formulation', 'MPCT', 'method','ADMM','submethod','semiband');

    [~, hK, ~, info] = mpc_solver(x0, xr, ur); % Take the optimal solution from "info" structure

    % Group the solutions Spcies solver in the same format as the one in the external solver
    
    l = 0;
    
    for k = 1:n+m:N*(n+m)
    
        l = l+1;
        result.sol.x(:,l) = info.z(k:k+n-1);
        result.sol.u(:,l) = info.z(k+n:k+n+m-1);
    
    end
    
    result.sol.xs = info.z(N*(n+m)+1:N*(n+m)+n);
    result.sol.us = info.z(N*(n+m)+n+1:(N+1)*(n+m));
    
    % Compare solution vectors
    for l = 1 : N

        result.gap_x(l,1) = norm(result.sol.x(:,l)-external_sol.x(:,l),'Inf');
        result.gap_u(l,1) = norm(result.sol.u(:,l)-external_sol.u(:,l),'Inf');
        
        if(result.gap_x(l,1)>sp_opt.max_gap || result.gap_u(l,1)>sp_opt.max_gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
            result.error = 'Maximum gap exceeded';
        end

    end

    result.gap_xs = norm(result.sol.xs-external_sol.xs,'Inf');
    result.gap_us = norm(result.sol.us-external_sol.us,'Inf');
    
    if(result.gap_xs>sp_opt.max_gap || result.gap_us>sp_opt.max_gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
        result.error = 'Maximum gap exceeded';
    end


    if (result.error == "")
        correct = true;
    else
        correct = false;
    end

    % result = correct; %TODO: Need to change this, just provisional

end

