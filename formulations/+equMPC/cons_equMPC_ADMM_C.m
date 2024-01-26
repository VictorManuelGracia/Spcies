%% cons_equMPC_ADMM_C
%
% Generates the constructor for C of the ADMM-based solver for the equality MPC formulation
%
% Information about this formulation and the solver can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .N: Prediction horizon.
%       - options: Instance of Spcies_options. Solver specific options are:
%              - .rho: Penalty parameter. Scalar of vector.
%                      If a vector is provided, it must have the same dimensions as the decision variables.
%              - .tol: Exit tolerance of the solver.
%              - .k_max: Maximum number of iterations of the solver.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_equMPC_ADMM_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Compute the ingredients of the controller
    vars = equMPC.compute_equMPC_ADMM_ingredients(recipe.controller, recipe.options);

    % Check that the options are allowed
    if recipe.options.time_varying && size(vars.LB, 2) > 1
        error("EquMPC ADMM time varying solver only allows fixed bounds along the prediction horizon");
    end
    if recipe.options.time_varying && ~vars.rho_is_scalar
        error("EquMPC ADMM time varying solver only allows the use of a scalar rho");
    end
    
    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    N = vars.N;
    
    % Determine if constant variables are defined as static
    if recipe.options.const_are_static
        var_options = {'static', 'constant', 'array'};
    else
        var_options = {'constant', 'array'};
    end
    
    % Determine if float or double variables are used
    precision = recipe.options.precision;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)

    % Defines
    defCell = recipe.options.default_defCell();
    defCell = add_line(defCell, 'nn_', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm_', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm_', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN_', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', recipe.options.solver.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol', recipe.options.solver.tol, 1, 'float', 'define');

    % Constants
    constCell = [];
    if size(vars.LB, 2) > 1
        % Different constraints for each prediction step
        constCell = add_line(constCell, 'LB0', vars.LB(n+1:end, 1), 1, precision, var_options);
        constCell = add_line(constCell, 'UB0', vars.UB(n+1:end, 1), 1, precision, var_options);
        constCell = add_line(constCell, 'LB', vars.LB(:, 2:end)', 1, precision, var_options);
        constCell = add_line(constCell, 'UB', vars.UB(:, 2:end)', 1, precision, var_options);
        defCell = add_line(defCell, 'VAR_BOUNDS', 1, 1, 'int', 'define');
    else
        if ~recipe.options.time_varying
            constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
            constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
        end
    end
    if ~recipe.options.time_varying
        constCell = add_line(constCell, 'Hi', vars.Hi, 1, precision, var_options);
        constCell = add_line(constCell, 'Hi_0', vars.Hi_0, 1, precision, var_options);
        constCell = add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
        constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
        constCell = add_line(constCell, 'AB', vars.AB, 1, precision, var_options);
        constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, precision, var_options);
        constCell = add_line(constCell, 'Beta', vars.Beta, 1, precision, var_options);
    end
    if recipe.options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end
    
    % rho
    if vars.rho_is_scalar
        defCell = add_line(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
        defCell = add_line(defCell, 'rho', vars.rho, 1, precision, 'define');
        defCell = add_line(defCell, 'rho_i', vars.rho_i, 1, precision, 'define');
    else
        constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_0', vars.rho_0, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i_0', vars.rho_i_0, 1, precision, var_options);
    end

    %% Declare an empty constructor object
    constructor = Spcies_constructor;
    
    %% Fill in the files
    
    % .c file
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_equMPC_ADMM_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_equMPC_ADMM_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end

