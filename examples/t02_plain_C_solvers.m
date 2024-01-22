%% Tutorial: Understanding and working with the C generated solvers
% 
% This tutorial provide a basic example of working with the plain C solvers
% generated by Spcies.
% In this tutorial we will generate a solver for the LaxMPC formulation in plain C
% and we will show how that solver can be called from a main() function to perform
% a closed-loop simulation of the system.
% 
% This Matlab script will generate the solver in the folder /examples/cl_in_C/, which
% contains the file "main_cl_in_C.c" that declared the main() function that will use the 
% generated solver.
% Therefore, this tutorial is divided into two parts: first, read through and run this
% script, which will generate the MPC solver. Next, read /examples/cl_in_C/main_cl_in_C.c
% and follow the instructions therein to compile and run the closed-loop simulation in C.
clear; clc;

%% STEP 1: Construct the system
% We will use the same oscillating masses system used in the basic_tutorial, which, for 
% convenience, we can easily create using the utility function sp_utils.example_OscMass().
[sys, paramMPC] = sp_utils.example_OscMass();

%% STEP 2: Generate the solver in plain C

% Select the solver options
solver_options.rho = 15; % Value of the penalty parameter of the ADMM algorithm
solver_options.k_max = 5000; % Maximum number of iterations of the solver
solver_options.tol = 1e-3; % Exit tolerance of the solver

options.save_name = 'myMPCsolver';
options.directory = './cl_in_C/'; % We save the solver in the directory were we have the main()
options.time = true;

% Generate the solver (note that we select the 'platform' as 'C')
spcies('gen', 'sys', sys, 'param', paramMPC, 'solver_options', solver_options,...
       'options', options, 'platform', 'C', 'formulation', 'laxMPC');

%% STEP 3: Use the generated solver
% Go to /examples/cl_in_C/
% The files "myMPCsolver.c" and "myMPCsolver.h" should now be in the folder
% You should also see the file "main_cl_in_C.c", which is included in the Spcies toolbox.
% Open the file and read through it, following any instructions within it to compile and run
% the closed-loop simulation that it contains.

