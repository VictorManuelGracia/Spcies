%% sp_test_MPCT_ADMM_semiband
% 
% This function generates the Spcies solver for MPCT using ADMM_semiband
% and compares the solution given by the external solver with the solution
% obtained by Spcies
% 
% @correct: '1' if the MPCT_ADMM_semiband test passed, '0' otherwise
% @result: Structure containing the result of the MPCT_ADMM_semiband test
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

    correct = true;

    spcies('clear');
        
    % Generate the Spcies solver
    spcies('gen','sys', sys, 'param', param, 'options', sp_opt, ...
         'platform', 'Matlab', 'formulation', 'MPCT', 'method','ADMM','submethod','semiband');

    [~, hK, ~, info] = mpc_solver(x0, xr, ur); % Take the optimal solution from "info" structure

    % Group the solutions Spcies solver in the same format as the one in the external solver
    
    l = 0;
    
    for k = 1:n+m:N*(n+m)
    
        l = l+1;
        spcies_sol.x(:,l) = info.z(k:k+n-1);
        spcies_sol.u(:,l) = info.z(k+n:k+n+m-1);
    
    end
    
    spcies_sol.xs = info.z(N*(n+m)+1:N*(n+m)+n);
    spcies_sol.us = info.z(N*(n+m)+n+1:(N+1)*(n+m));
    
    % Compare solution vectors
    for l = 1 : N
        if(norm(spcies_sol.x(:,l)-external_sol.x(:,l),'Inf')>sp_opt.gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
            correct = false;
        end
        if(norm(spcies_sol.u(:,l)-external_sol.u(:,l),'Inf')>sp_opt.gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
            correct = false;
        end
    end
    
    if(norm(spcies_sol.xs-external_sol.xs,'Inf')>sp_opt.gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
        correct = false;
    end
    if(norm(spcies_sol.us-external_sol.us,'Inf')>sp_opt.gap) % The infinity norm is of the order of magnitude of options.tol, so we give some margin
        correct = false;
    end
        

    result = correct; %TODO: Need to change this, just provisional

end

