%% Test for the equMPC formulation using FISTA

function [gap, exit] = test_equMPC_FISTA(sys, status)

    % Solver options
    solver_options.k_max = 5000;
    solver_options.tol = 1e-7;
    solver_options.debug = true;
    
    % Parameters of the MPC formulation
    param.Q = blkdiag(15*eye(sys.p), 1*eye(sys.p));
    param.R = 0.1*eye(sys.m);
    param.N = 10;
    
%     sys.LBx = kron(ones(1, param.N), sys.LBx);
%     sys.LBu = kron(ones(1, param.N), sys.LBu);
%     sys.UBx = kron(ones(1, param.N), sys.UBx);
%     sys.UBu = kron(ones(1, param.N), sys.UBu);
    
    % Construct solver
    spcies_gen_controller('sys', sys, 'param', param, 'solver_options', solver_options,...
    'platform', 'Matlab', 'type', 'equMPC', 'method', 'FISTA');
    
    % Solve using the sparse solver
    [u_s, k_s, e_s, sol_s] = equMPC(status.x, status.xr, status.ur);
    
    % Solse using the non-sparse solver
    [u_ns, k_ns, e_ns, sol_ns] = spcies_equMPC_FISTA_solver(status.x, status.xr, status.ur, 'sys', sys,...
                                                 'param', param, 'options', solver_options, 'genHist', 1);
                                            
    % Compare solutions
    z_opt = [0.799999999996694;0.799999999996694;0.0389465698414548;0.0243980535436783;0.0389465698414548;0.167221459601478;0.0278930770748502;0.167221459601478;0.631745583010538;0.631745583010557;0.0819576903040167;0.0339682900247973;0.0819576903039969;0.257767221215772;0.0768023718295581;0.257767221215886;-0.383234442676579;-0.383234442676814;0.119715592543692;0.0583486609591897;0.119715592543669;0.116388800170057;0.170606279636900;0.116388800170171;-0.163187673394386;-0.163187673394685;0.132522566251594;0.101182743087651;0.132522566251617;0.0128428304429917;0.249678003080135;0.0128428304432191;0.330595043643664;0.330595043643340;0.135763259124978;0.153525518757655;0.135763259124989;0.0226474003635531;0.260568864084917;0.0226474003636667;0.617710857410554;0.617710857410055;0.148367030267606;0.200583930682023;0.148367030267604;0.104863342254589;0.200770018255525;0.104863342254703;0.574795982455467;0.574795982454877;0.176806140605590;0.231796663016577;0.176806140605595;0.177813128269122;0.110622920809988;0.177813128269236;0.296595829866468;0.296595829865853;0.212711037770729;0.246077086136719;0.212711037770753;0.177382468293331;0.0379995787703820;0.177382468293445;0.00467755502702261;0.00467755502631229;0.240333539411684;0.249742929491069;0.240333539411684;0.0953860809354410;0.00510024768539097;0.0953860809354410;0.0102156758795957;0.0102156758787522];
    
    gap.spcies = [norm(sol_s.z - sol_ns.sol.z, Inf);
                  norm(sol_s.lambda - sol_ns.sol.lambda, Inf)];
                         
    gap.opt = norm(sol_s.z - z_opt, Inf);                   
    
     % Exit flags
    exit = [e_s, e_ns];
    
end
