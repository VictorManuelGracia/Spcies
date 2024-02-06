%% spcies_test()
%
% This function calls the individual test functions
%
% @result: returns '1' if the test passed successfully, '0' otherwise
% 
% @solvers_options: Structure containing options for both Spcies and Yalmip solvers

function [correct,results] = spcies_test(varargin)

    % Default values
    def_tol = 1e-8;
    def_max_gap = 1e-6;

    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_test';

    % Optional
    addOptional(par, 'tol', def_tol, @(x)isnumeric(x) && x>0);
    addOptional(par, 'max_gap', def_max_gap, @(x)isnumeric(x) && x>0);

    % Parse
    parse(par, varargin{:})

    % Rename
    solvers_options.tol = par.Results.tol;
    solvers_options.max_gap = par.Results.max_gap;

    [correct.MPCT,results.MPCT] = sp_test_MPCT(solvers_options);

end