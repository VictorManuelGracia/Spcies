%% sp_solve_MPCT_external
%
% This function calls sp_gen_MPCT_YALMIP() to generate the
% yalmip solver, and solves the optimization problem provided by sp_test_MPCT using yalmip
% 
% @sys: Structure containing the system model
% @param: Structure containing parameters of the MPCT controller
% @x0: Current state of the system
% @xr: Reference for the states
% @ur: Reference for the inputs
% @external_opt: Sructure containing options for yalmip
%
% @sol: Structure containing the solution vectors of the problem
% @e_flag: Its value is set to '0' if the problem has been solved
% successfully, '1' otherwise

function [sol, ok] = sp_solve_MPCT_external(sys,param,x0,xr,ur,external_opt)

    controller = sp_gen_MPCT_YALMIP(sys,param,external_opt,'osqp');

    [sol,e_flag] = controller(x0,xr,ur);

    ok = true;

    if (e_flag~=0)
        ok = false;
    end

end

