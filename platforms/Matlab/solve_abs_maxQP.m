%% solve_abs_maxQP - Solves a two-dimensional QP with abs() and max() terms in the functional
% 
% INPUTS:
% 
%   - b: Linear term of x in the functional
%   - c: Linear term of y in the functional
%   - alpha: Weight for the penalization of the difference between x and y
%   - beta: Weight for the soft-constraints penalization in x
%   - d: Lower bound of the (soft) constraint in x
%   - e: Upper bound of the (soft) constraint in x
%
% OUTPUTS:
% 
%   - x_opt: Optimal x of the optimization problem
%   - y_opt: Optimal y of the optimization problem
% 
% This function solves an optimization problem of the form:
% min_{x,y} (1/2)*(x^2+y^2) - bx - cy + alpha*|x-y| + beta*max(d-x,x-e,0)
% s.t. LBy <= y <= UBy,
% where LBy < UBy and d < e.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies

function [x_opt,y_opt] = solve_abs_maxQP(b,c,alpha,beta,d,e,LBy,UBy)

x_a_opt = min(max(solve_maxQP((b+c)/2,beta/2,d,e),LBy),UBy);
y_a_opt = x_a_opt;

x_b_opt = solve_maxQP(b-alpha,beta,d,e);
y_b_opt = min(max(c+alpha,LBy),UBy);

if x_b_opt <= y_b_opt
    x_b_opt = x_a_opt;
    y_b_opt = x_a_opt;
end

x_c_opt = solve_maxQP(b+alpha,beta,d,e);
y_c_opt = min(max(c-alpha,LBy),UBy);

if x_c_opt >= y_c_opt
    x_c_opt = x_a_opt;
    y_c_opt = x_a_opt;
end

% Functional evaluation
J_a_opt = (1/2) * (x_a_opt^(2)+y_a_opt^(2)) - b*x_a_opt - c*y_a_opt + beta*max([d-x_a_opt,x_a_opt-e,0]);
J_b_opt = (1/2) * (x_b_opt^(2)+y_b_opt^(2)) - b*x_b_opt - c*y_b_opt + alpha*abs(x_b_opt-y_b_opt) + beta*max([d-x_b_opt,x_b_opt-e,0]);
J_c_opt = (1/2) * (x_c_opt^(2)+y_c_opt^(2)) - b*x_c_opt - c*y_c_opt + alpha*abs(y_c_opt-x_c_opt) + beta*max([d-x_c_opt,x_c_opt-e,0]);

J_opt(1,1) = J_a_opt; % Save the functional values in the first row
J_opt(1,2) = J_b_opt;
J_opt(1,3) = J_c_opt;

J_opt(2,1) = x_a_opt; % Save "x" values in the second row
J_opt(2,2) = x_b_opt;
J_opt(2,3) = x_c_opt;

J_opt (3,1) = y_a_opt; % Save "y" values in the third row
J_opt (3,2) = y_b_opt;
J_opt (3,3) = y_c_opt;

index = find(J_opt(1,:) == min(J_opt(1,:))); % Finds the index of the best functional evaluation (J_a_opt, J_b_opt, J_c_opt)

x_opt = J_opt(2,index(1)); % Chooses x_opt. I set (1) because more than one functional can have the same value.
y_opt = J_opt(3,index(1)); % Chooses y_opt


end

