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
    def_tol = 1e-9;
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
    solvers_options.tol_p = par.Results.tol;
    solvers_options.tol_d = par.Results.tol;
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
            % TODO: Add here the rest of formulations when available
        otherwise
            error('Unrecognized formulation or not supported')
    end

    fprintf("Tests results:\n");

    correct_tests = true; % Sets to false if any test fails

    cell_formulation_names = fieldnames(correct);
            
    for i = 1:length(cell_formulation_names)

        cell_method_names = fieldnames(correct.(cell_formulation_names{i}));

        for j = 1:length(cell_method_names)

            if isstruct(correct.(cell_formulation_names{i}).(cell_method_names{j}))

                cell_submethod_names = fieldnames(correct.(cell_formulation_names{i}).(cell_method_names{j}));
                
                for k = 1:length(cell_submethod_names)

                    if correct.(cell_formulation_names{i}).(cell_method_names{j}).(cell_submethod_names{k}) == false

                        correct_tests = false;

                    end

                end

            else

                if correct.(cell_formulation_names{i}).(cell_method_names{j}) == false

                    correct_tests = false;

                end

            end

        end

    end

    for i = 0:par.Results.verbose

        switch i
    
            case 0
                
                disp("============================================")

                if correct_tests == true
                    fprintf("Tests passed successfully\n");
                else
                    fprintf("Tests failed\n");
                end

                disp("============================================")
                
            case 1
                
                for i = 1:length(cell_formulation_names)
    
                    fprintf("* %s: \n",cell_formulation_names{i});
    
                    for j = 1:length(cell_method_names)
    
                        if isstruct(correct.(cell_formulation_names{i}).(cell_method_names{j}))
    
                            fprintf("\n  > %s: \n\n",cell_method_names{j});
        
                            for k = 1:length(cell_submethod_names)

                                if correct.(cell_formulation_names{i}).(cell_method_names{j}).(cell_submethod_names{k})

                                    passed = "passed";

                                else

                                    passed = "failed";

                                end
            
                                fprintf("    - %s: %s\n",cell_submethod_names{k},passed);
            
                            end
    
                        else

                            if correct.(cell_formulation_names{i}).(cell_method_names{j})
                                passed = "passed";
                            else
                                passed = "failed";
                            end
    
                            fprintf("\n  > %s: %s\n\n",cell_method_names{j},passed);
    
                        end
    
                    end
    
                end

                disp("============================================")
    
            case 2 % TODO: Select properly the data we want to show in this level of verbose
                
                fprintf("Details: \n\n");
                
                for i = 1:length(cell_formulation_names)

                    for j = 1:length(cell_method_names)

                        if isstruct(correct.(cell_formulation_names{i}).(cell_method_names{j}))

                            for k = 1:length(cell_submethod_names)
    
                                fprintf("%s/%s/%s:\n",cell_formulation_names{i},cell_method_names{j},cell_submethod_names{k});

                                disp(results.(cell_formulation_names{i}).(cell_method_names{j}).(cell_submethod_names{k}))
    
                            end

                        else

                            fprintf("%s/%s:\n",cell_formulation_names{i},cell_method_names{j});

                            disp(results.(cell_formulation_names{i}).(cell_method_names{j}))

                        end

                    end

                end

            disp("============================================")
        
        end

    end



end