%% spcies_test()
%
% This function calls the individual test functions
%
% @correct: Returns '1' if the tests passed successfully, '0' otherwise
% @results: Structure containing the result structure of all tests
% 
% @solvers_options: Structure containing options for both Spcies and Yalmip solvers

function [correct,results] = spcies_test(varargin)

    % Default options values
    def_tol = 1e-8;
    def_max_gap = 1e-6;
    def_verbose = 1;
    def_stop_on_error = false;

    % Default type, method, submethod
    def_formulation = '';
    def_method = '';
    def_submethod = '';

    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_test';

    % Options
    addOptional(par, 'tol', def_tol, @(x)isnumeric(x) && x>0);
    addOptional(par, 'max_gap', def_max_gap, @(x)isnumeric(x) && x>0);
    addOptional(par, 'verbose', def_verbose, @(x)isnumeric(x) && x>=0 && x<=3);
    addOptional(par, 'stop_on_error', def_stop_on_error, @(x)isnumeric(x) && x>=0 && x<=1);

    % Type, method, submethod
    addOptional(par, 'formulation', def_formulation, @(x)isstring(x));
    addOptional(par, 'method', def_method, @(x)isstring(x));
    addOptional(par, 'submethod', def_submethod, @(x)isstring(x));

    % Parse
    parse(par, varargin{:})

    % Rename
    solvers_options.tol = par.Results.tol;
    solvers_options.max_gap = par.Results.max_gap;
    solvers_options.verbose = par.Results.verbose;
    solvers_options.stop_on_error = par.Results.stop_on_error;
    solvers_options.method = par.Results.method;
    solvers_options.submethod = par.Results.submethod;

    switch par.Results.formulation
        case "MPCT"
            [correct.MPCT,results.MPCT] = sp_test_MPCT(solvers_options);
        case ""
            [correct.MPCT,results.MPCT] = sp_test_MPCT(solvers_options);
            % Add here the rest of formulations when available
        otherwise
            error('Unrecognized formulation or not supported')
    end
    

end