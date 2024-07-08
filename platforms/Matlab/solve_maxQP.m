%% solve_maxQP - Solves a soft-constrained version of a box-constrained QP
% 
% INPUTS:
% 
%   - b: Linear term of the functional
%   - beta: Weight for the soft-constraints penalization
%   - d: Lower bound of the (soft) constraint
%   - e: Upper bound of the (soft) constraint
%
% OUTPUTS:
% 
%   - x_opt: Optimal solution of the optimization problem
% 
% This function solves a scalar optimization problem of the form:
% min_{x} (1/2)*x^2 - b*x + beta*max(d-x,x-e,0),
% where d < e.
% The QP above is a softened version of the (hard) box-constrained QP:
% min_{x} (1/2)*x^2 - b*x s.t. d<=x<=e.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies

function x_opt = solve_maxQP(b,beta,d,e)

x_1 = b+beta;
x_2 = b;
x_3 = b-beta;

if x_1 <= d
    x_opt = x_1;
elseif x_1 > d && x_2 < d
    x_opt = d;
elseif x_2 >= d && x_2 <= e
    x_opt = x_2;
elseif x_2 > e && x_3 < e
    x_opt = e;
elseif x_3 >= e
    x_opt = x_3;
end

end

