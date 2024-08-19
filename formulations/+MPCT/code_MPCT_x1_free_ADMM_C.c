/**
 * ADMM solver for the MPCT formulation modified by including two first-predicted states:
 *  - The first x_1 (namely hat{x}_1) is a prediction obtained by applying u_0 to the linear
 *    prediction model in the current state of the system (hat{x}).
 *  - The second x_1 (namely x_1) is the starting point for the rest of the sequence of predicted
 *    states.
 * The formulation penalizes with an exact penalty function (l1-norm) the difference between both "x_1". 
 * Moreover, hat{x}_1 is subject to soft inequality box constraints, while x_1 is subject to hard 
 * inequality box constraints.
 *
 * ARGUMENTS:
 * The current system state is given in "x0_in". Pointer to array of size nn_.
 * The state reference is given in "xr_in". Pointer to array of size nn_.
 * The input reference is given in "ur_in". Pointer to array of size mm_.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm_.
 * The number of iterations is returned in "k_in". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged succesfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 * Computation times are also returned in the structure sol.
 * 
 */

 void MPCT_x1_free_ADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

    #if MEASURE_TIME == 1

    #if WIN32
    static LARGE_INTEGER start, post_update, post_solve, post_polish;
    #else // If Linux
    struct timespec start, post_update, post_solve, post_polish;
    #endif

    read_time(&start);

    #endif

    // Initialize solver variables
    int done = 0;
    int k = 0; // Number of iterations
    double x0[nn_] = {0.0}; // Current system state
    double xr[nn_] = {0.0}; // State reference
    double ur[mm_] = {0.0}; // Input reference
    double z[(NN_+1)*nm_+nn_] = {0.0}; // Decision variable z
    double v[(NN_+1)*nm_+nn_] = {0.0}; // Decision variable v
    double v_old[(NN_+1)*nm_+nn_] = {0.0}; // Decision variable v in the previous iteration
    double lambda[(NN_+1)*nm_+nn_] = {0.0}; // Decision variable lambda
    double q[nm_] = {0.0}; // Linear term vector in the functional. Only non-zero elements are considered.
    double xi[(NN_+1)*nm_+nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double mu[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z3_ac[(NN_+1)*nm_+nn_] = {0.0}; // Used to solve the equality-constrained QP step. Stores z3_a and z3_c.
    double z2[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step. This one is used as z2_a, z2_b and z2_c.
    double p[(NN_+1)*nm_+nn_] = {0.0}; // Used to solve the equality-constrained QP
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied

    // TODO: Make that only the diagonals of weight matrices are declared when they are diagonal, instead of the whole weight matrices. Same for Q_rho_i, R_rho, S_rho and T_rho_i.

    // Constant variables
    $INSERT_CONSTANTS$

    // Obtain variables in scaled units
    #if in_engineering == 1
    for(unsigned int i = 0; i < nn_; i++){
        x0[i] = scaling_x[i]*( x0_in[i] - OpPoint_x[i] );
        xr[i] = scaling_x[i]*( xr_in[i] - OpPoint_x[i] );
    }
    for(unsigned int i = 0; i < mm_; i++){
        ur[i] = scaling_u[i]*( ur_in[i] - OpPoint_u[i] );
    }
    #endif
    #if in_engineering == 0
    for(unsigned int i = 0; i < nn_; i++){
        x0[i] = x0_in[i];
        xr[i] = xr_in[i];
    }
    for(unsigned int i = 0; i < mm_; i++){
        ur[i] = ur_in[i];
    }
    #endif

    // Compute q

    for(unsigned int i = 0 ; i < nn_ ; i++){
        
        for(unsigned int j = 0 ; j < nn_ ; j++){

            q[i] -= T[i][j] * xr[j];

        }

    }

    for(unsigned int i = 0 ; i < mm_ ; i++){
        
        for(unsigned int j = 0 ; j < mm_ ; j++){

            q[nn_ + i] -= S[i][j] * ur[j];

        }

    }

    // Measure time
    #if MEASURE_TIME == 1
    read_time(&post_update);
    get_elapsed_time(&sol->update_time, &post_update, &start);
    #endif

    // Algorithm
    while(done==0){

        k += 1;

        // Save the value of v
        memcpy(v_old, v, sizeof(double)*((NN_+1)*nm_+nn_));
        // Reset acumulator variables
        memset(xi, 0, sizeof(double)*((NN_+1)*nm_+nn_));
        memset(z3_ac, 0, sizeof(double)*((NN_+1)*nm_+nn_));
        memset(z2, 0, sizeof(double)*2*nm_);

        //********** Equality-constrained QP solve **********//
        // This problem updates z

        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){

            #ifdef SCALAR_RHO
            p[i] = lambda[i] - rho * v[i];
            #else
            p[i] = lambda[i] - rho[i] * v[i];
            #endif    

        }
        
        for (unsigned int i = 0 ; i < nm_ ; i++){

            p[i+NN_*nm_+nn_] += q[i];

        }

        // Compute xi from P*xi=p using semi-band algorithm, where P = H + rho*I = Gamma_hat + U_hat*V_hat 
        
        // Obtains z1_a, stored in xi to save memory
        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, xi, p);

        // z2_a = M_hat * z1_a computed sparsely

        // First nn_ rows
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i][j]*xi[j];
                
            }
            
        }

        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i][j]*xi[nm_+j];
                
            }
            
        }


        for (unsigned int l = 1 ; l < NN_ ; l++){
            
            for (unsigned int i = 0 ; i < nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i][j]*xi[l*nm_+nn_+j];
                    
                }
                
            }

        }

        for (unsigned int i = 0 ; i < nn_ ; i++){
            
            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i][j]*xi[NN_*nm_+nn_+j];
                
            }

        }

        // From nn_+1 to nm_ rows
        for (unsigned int i = nn_ ; i < nm_ ; i++){
        
            for (unsigned int j = 0 ; j < mm_ ; j++){
            
                z2[i] += M_hat_u1[i-nn_][j]*xi[nn_+j];
            
            }

        }

        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = nn_ ; i < nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u1[i-nn_][j]*xi[l*nm_+nn_+nn_+j];
                
                }

            }
        
        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u2[i-nn_][j]*xi[NN_*nm_+nn_+nn_+j];
            
            }

        }

        // From nm_+1 to nm_+nn_ rows
        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i-mm_][j]*xi[j];
                
            }
            
        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i-mm_][j]*xi[nm_+j];
                
            }
            
        }


        for (unsigned int l = 1 ; l < NN_ ; l++){
            
            for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i-mm_][j]*xi[l*nm_+nn_+j];
                    
                }
                
            }

        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){
            
            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i-mm_][j]*xi[NN_*nm_+nn_+j];
                
            }

        }

        // From nm_+nn_+1 to nm_+nm_ rows
        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){
        
            for (unsigned int j = 0 ; j < mm_ ; j++){
            
                z2[i] += M_hat_u1[i-2*nn_][j]*xi[nn_+j];
            
            }

        }

        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u1[i-2*nn_][j]*xi[l*nm_+nn_+nn_+j];
                
                }

            }
        
        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u2[i-2*nn_][j]*xi[NN_*nm_+nn_+nn_+j];
            
            }

        } // End of computation of z2_a

        memset(v, 0, sizeof(double)*((NN_+1)*nm_+nn_));

        // (U_hat * z2_a) computed sparsely, stored in v to save memory

        // TODO: Make a different case for IS_DIAG == true, meaning that weights are diagonal. Many operations can be avoided in that case.

        // First nn_ rows
        for(unsigned int i = 0 ; i < nn_ ; i++){
            
            for(unsigned int j = 0 ; j < nn_ ; j++){

                v[i] -= Q[i][j] * z2[j];

            }

        }

        // From nn_+1 to nm_ rows
        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for(unsigned int j = nn_ ; j < nm_ ; j++){

                v[i] -= R[i-nn_][j-nn_] * z2[j];

            }

        }

        // From nm_+1 rows to nm_+nn_ rows
        for(unsigned int i = nm_ ; i < nm_+nn_ ; i++){
            
            for(unsigned int j = 0 ; j < nn_ ; j++){

                v[i] -= Q[i-nm_][j] * z2[j];

            }

        }

        // Rest of rows except last nm_ rows
        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_+nn_ ; i < l*nm_+nn_+nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    v[i] -= Q[i-l*nm_-nn_][j] * z2[j];

                }

            }

            for (unsigned int i = l*nm_+2*nn_ ; i < l*nm_+2*nn_+mm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    v[i] -= R[i-l*nm_-2*nn_][j] * z2[j+nn_];

                }

            }

        }

        // Last nm_ rows
        for (unsigned int i = NN_*nm_+nn_ ; i < (NN_+1)*nm_+nn_ ; i++){

            v[i] = z2[i-(NN_-1)*nm_-nn_];

        }
        // End of computation of (U_hat * z2_a), stored in v

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, v); // Obtains z3_a, stored in z3_ac

        // Computation of xi = z1_a - z3_a
        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){

            xi[i] -= z3_ac[i]; // xi[i] = z1_a[i] - z3_a[i];

        }
        // End of computation of xi


        /****** Compute mu using semi-band algorithm******/

        // Sparse computation of -(G*xi+b), stored in mu to save memory
        memset(mu, 0, sizeof(double)*(NN_+2)*nn_);

        // First nn_ rows
        for (unsigned int i = 0 ; i < nn_ ; i++){

            mu[i] = -(x0[i] + xi[i]); // -(b[i]+xi[i]);

        }

        // From nn_+1 rows to 2*nn_ rows
        for (unsigned int i = nn_ ; i < 2*nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                mu[i] -= A[i-nn_][j] * xi[j];

            }

            mu[i] += xi[i+mm_];

            for (unsigned int j = 0 ; j < mm_ ; j++){

                mu[i] -= B[i-nn_][j] * xi[j+nn_];

            }

        }

        // Rest of rows except last nn_ rows
        for (unsigned int l = 2 ; l <= NN_ ; l++){

            for (unsigned int i = l*nn_ ; i < (l+1)*nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    mu[i] -= A[i-l*nn_][j] * xi[j+l*nn_+(l-1)*mm_];

                }

                mu[i] += xi[i+l*mm_+nn_];

                for (unsigned int j = 0 ; j < mm_; j++){

                    mu[i] -= B[i-l*nn_][j] * xi[j+(l+1)*nn_+(l-1)*mm_];

                }

            }

        }

        // Last nn_ rows
        for (unsigned int i = (NN_+1)*nn_ ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                mu[i] -= A[i-(NN_+1)*nn_][j] * xi[j+NN_*nm_+nn_];

            }

            mu[i] += xi[i-(NN_+1)*nn_+NN_*nm_+nn_];

            for (unsigned int j = 0 ; j < mm_ ; j++){

                mu[i] -= B[i-(NN_+1)*nn_][j] * xi[j+NN_*nm_+2*nn_];

            }

        }
        // End of computation of -(G*xi+b), stored in mu.

        solve_banded_Chol(Alpha, Beta, mu); // Obtains z1_b. We use mu to store the result. Note that mu contained the independent term vector -(G*xi+b) before calling this function.

        memset(z2, 0, sizeof(double)*2*nm_);

        // Computation of z2_b = M_tilde*z1_b
        #ifdef SCALAR_RHO
        for (unsigned int i = 0 ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[j]; // z2[i] += M_tilde[i][j] * z1_b[j]. M_tilde is dense, but it presents repetitions inside if rho is scalar, so we use a shortened version of it instead

            }

            for (unsigned int l = 0 ; l < NN_-1 ; l++){

                for (unsigned int j = nn_ ; j < 2*nn_ ; j++){

                    z2[i] += M_tilde[i][j] * mu[l*nn_+j]; // z2[i] += M_tilde[i][j] * z1_b[l*nn_+j];

                }

            }

            for (unsigned int j = 2*nn_ ; j < 4*nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[(NN_-2)*nn_+j]; // z2[i] += M_tilde[i][j] * z1_b[(NN_-2)*nn_+j];

            }

        }

        #else

        for  (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < (NN_+2)*nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[j]; // z2[i] += M_tilde[i][j] * z1_b[j]. M_tilde is dense. It does not have repetitions inside when rho is a vector, so we use the full matrix

            }

        }

        #endif
        // End of computation of z2_b

        memset(v, 0, sizeof(double)*((NN_+1)*nm_+nn_));

        // Computation of (U_tilde*z2_b), stored in v to save memory (needed to compute z3_b later)
        #ifdef SCALAR_RHO

        // First nn_ rows of U_tilde have some zero part, so we avoid it
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                v[i] += U_tilde_ini[i][j] * z2[j];

            }

            for (unsigned int j = nm_ ; j < nm_+nn_ ; j++){

                v[i] += U_tilde_ini[i][j-mm_] * z2[j];

            }

        }

        // When rho is scalar, from rows nn_+1 to (N+1)*nn_, we have repetitions, so we avoid storing that part
        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = 0 ; i < nn_ ; i++){

                for (unsigned int j = 0 ; j < 2*nm_ ; j++){

                    v[l*nn_+i] += U_tilde_mid[i][j] * z2[j];

                }

            }

        }

        // Last NN_*nn_+1 to (NN_+2)*nn_ rows are also different
        for (unsigned int i = 0 ; i < 2*nn_ ; i++){

            for (unsigned int j = 0 ; j < 2*nm_ ; j++){

                v[NN_*nn_+i] += U_tilde_final[i][j] * z2[j];

            }

        }

        #else

        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ; j < 2*nm_ ; j++){
                
                v[i] += U_tilde[i][j] * z2[j];

            }

        }

        #endif
        // End of computation of (U_tilde*z2_b)

        solve_banded_Chol(Alpha, Beta, v); // Obtains z3_b, which is stored in v to save memory.

        // Computation of mu
        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){ 

            mu[i] -= v[i]; // mu[i] = z1_b[i] - z3_b[i];

        }
        // End of computation of mu


        /****  Compute z^{k+1} ****/

        // Sparse computation of -(G'*mu+p), used later to compute z1_c. We store it in variable p.

        for (unsigned int i = 0 ; i < nn_ ; i++){

            p[i] = -(p[i] + mu[i]);

            for (unsigned int j = 0; j < nn_ ; j++){

                p[i] -= A[j][i] * mu[nn_+j];

            }

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += B[j][i-nn_] * mu[nn_+j];

            }

            p[i] = -p[i];

        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            p[i] = -(p[i]-mu[i-mm_]);

        }

        for (unsigned int i = nm_+nn_ ; i < 3*nn_+mm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += A[j][i-nm_-nn_] * mu[2*nn_+j];

            }

            p[i] = -p[i];

        }

        for (unsigned int i = 2*nm_+nn_-mm_ ; i < 2*nm_ + nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += B[j][i-2*nm_-nn_+mm_] * mu[2*nn_+j];

            }

            p[i] = -p[i];

        }

        for (unsigned int l = 3 ; l <= NN_ ; l++){

            for (unsigned int i = (l-1)*nm_+nn_ ; i < (l+1)*nn_+(l-1)*mm_ ; i++){

                p[i] = -(p[i] - mu[i-(l-1)*mm_-nn_]);

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    p[i] -= A[j][i-(l-1)*nm_-nn_] * mu[l*nn_+j];

                }

            }

            for (unsigned int i = l*nm_+nn_-mm_ ; i < l*nm_ + nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    p[i] += B[j][i-l*nm_-nn_+mm_] * mu[l*nn_+j];

                }

                p[i] = -p[i];

            }

        }

        for (unsigned int i = NN_*nm_+nn_; i < NN_*nm_ + 2*nn_ ; i++){

            p[i] = -p[i] + (mu[i+nn_-NN_*mm_-nn_] + mu[i-NN_*mm_-nn_]);

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] -= A[j][i-NN_*nm_-nn_] * mu[(NN_+1)*nn_+j];

            }

        }

        for (unsigned int i = NN_*nm_+2*nn_ ; i<(NN_+1)*nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += B[j][i-NN_*nm_-2*nn_] * mu[(NN_+1)*nn_+j];

            }

            p[i] = -p[i];

        }
        
        // End of computation of -(G'*mu+p), which is stored in p.

        memset(z, 0, sizeof(double)*((NN_+1)*nm_+nn_));

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z, p); // Obtains z1_c, stored in z to save memory

        memset(z2, 0, sizeof(double)*2*nm_);

        // z2_c = M_hat * z1_c computed sparsely, stored in z2 variable

        // First nn_ rows
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i][j]*z[j];
                
            }
            
        }

        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i][j]*z[nm_+j];
                
            }
            
        }


        for (unsigned int l = 1 ; l < NN_ ; l++){
            
            for (unsigned int i = 0 ; i < nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i][j]*z[l*nm_+nn_+j];
                    
                }
                
            }

        }

        for (unsigned int i = 0 ; i < nn_ ; i++){
            
            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i][j]*z[NN_*nm_+nn_+j];
                
            }

        }

        // From nn_+1 to nm_ rows
        for (unsigned int i = nn_ ; i < nm_ ; i++){
        
            for (unsigned int j = 0 ; j < mm_ ; j++){
            
                z2[i] += M_hat_u1[i-nn_][j]*z[nn_+j];
            
            }

        }

        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = nn_ ; i < nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u1[i-nn_][j]*z[l*nm_+nn_+nn_+j];
                
                }

            }
        
        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u2[i-nn_][j]*z[NN_*nm_+nn_+nn_+j];
            
            }

        }

        // From nm_+1 to nm_+nn_ rows
        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i-mm_][j]*z[j];
                
            }
            
        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x1[i-mm_][j]*z[nm_+j];
                
            }
            
        }


        for (unsigned int l = 1 ; l < NN_ ; l++){
            
            for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i-mm_][j]*z[l*nm_+nn_+j];
                    
                }
                
            }

        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){
            
            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i-mm_][j]*z[NN_*nm_+nn_+j];
                
            }

        }

        // From nm_+nn_+1 to nm_+nm_ rows
        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){
        
            for (unsigned int j = 0 ; j < mm_ ; j++){
            
                z2[i] += M_hat_u1[i-2*nn_][j]*z[nn_+j];
            
            }

        }

        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u1[i-2*nn_][j]*z[l*nm_+nn_+nn_+j];
                
                }

            }
        
        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u2[i-2*nn_][j]*z[NN_*nm_+nn_+nn_+j];
            
            }

        } // End of computation of z2_c

        memset(v, 0, sizeof(double)*((NN_+1)*nm_+nn_));

        // (U_hat * z2_c) computed sparsely, stored in v to save memory

        // First nn_ rows
        for(unsigned int i = 0 ; i < nn_ ; i++){
            
            for(unsigned int j = 0 ; j < nn_ ; j++){

                v[i] -= Q[i][j] * z2[j];

            }

        }

        // From nn_+1 to nm_ rows
        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for(unsigned int j = nn_ ; j < nm_ ; j++){

                v[i] -= R[i-nn_][j-nn_] * z2[j];

            }

        }

        // From nm_+1 rows to nm_+nn_ rows
        for(unsigned int i = nm_ ; i < nm_+nn_ ; i++){
            
            for(unsigned int j = 0 ; j < nn_ ; j++){

                v[i] -= Q[i-nm_][j] * z2[j];

            }

        }

        // Rest of rows except last nm_ rows
        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_+nn_ ; i < l*nm_+nn_+nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    v[i] -= Q[i-l*nm_-nn_][j] * z2[j];

                }

            }

            for (unsigned int i = l*nm_+2*nn_ ; i < l*nm_+2*nn_+mm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    v[i] -= R[i-l*nm_-2*nn_][j] * z2[j+nn_];

                }

            }

        }

        // Last nm_ rows
        for (unsigned int i = NN_*nm_+nn_ ; i < (NN_+1)*nm_+nn_ ; i++){

            v[i] = z2[i-(NN_-1)*nm_-nn_];

        }
        // End of computation of (U_hat * z2_c), stored in v

        memset(z3_ac, 0, sizeof(double)*((NN_+1)*nm_+nn_));

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, v); // Obtains z3_c

        // Computation of  z^{k+1}
        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){

            z[i] -= z3_ac[i]; // z^{k+1} = z1_c - z3_c

        }
        // End of computation of z^{k+1}

        //********** Inequality-constrained QP solve **********//
        // This problem updates v

        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){

            #ifdef SCALAR_RHO
            v[i] = rho_i * lambda[i] + z[i];
            #else
            v[i] = rho_i[i] * lambda[i] + z[i];
            #endif

        }

        // x_0 unconstrained
        for (unsigned int i = 0 ; i < nn_ ; i++){

            v[i] = (v[i] > -inf) ? v[i] : -inf;
            v[i] = (v[i] < inf) ? v[i] : inf;
        
        }

        // u_0 hard-constrained
        for (unsigned int i = nn_ ; i < nm_ ; i++){

            v[i] = (v[i] > LB[i]) ? v[i] : LB[i];
            v[i] = (v[i] < UB[i]) ? v[i] : UB[i];
        
        }

        // Elements related to (hat{x}_1,x_1)
        #ifdef SCALAR_RHO
        for(unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            solve_abs_max_QP(&v[i], &v[i+nn_], v[i], v[i+nn_], alpha_rho_i, beta_rho_i, LB[i-nm_], UB[i-nm_], LB[i-nm_], UB[i-nm_]);

        }
        #else
        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            solve_abs_max_QP(&v[i], &v[i+nn_], v[i], v[i+nn_], alpha_rho_i[i], beta_rho_i[i], LB[i-nm_], UB[i-nm_], LB[i-nm_], UB[i-nm_]);

        }
        #endif

        // Elements related to the rest of the predicted sequence:
        // (u_1, x_2, u_2, ..., x_{N-1}, u_{N-1}, x_s, u_s)

        for (unsigned int i = nm_+2*nn_ ; i < nm_+2*nn_+mm_ ; i++){ // Box constraints for u_1

            v[i] = (v[i] > LB[i-nm_-nn_]) ? v[i] : LB[i-nm_-nn_];
            v[i] = (v[i] < UB[i-nm_-nn_]) ? v[i] : UB[i-nm_-nn_];

        }

        // Box constraints for the rest of elements except (x_s,u_s)
        for (unsigned int l = 2 ; l < NN_ ; l++){

            for (unsigned int i = 0 ; i < nm_ ; i++){

                v[i+l*nm_+nn_] = (v[i+l*nm_+nn_] > LB[i]) ? v[i+l*nm_+nn_] : LB[i];
                v[i+l*nm_+nn_] = (v[i+l*nm_+nn_] < UB[i]) ? v[i+l*nm_+nn_] : UB[i];

            }

        }

        // Box constraints for the elements related to (x_s,u_s)
        for (unsigned int i = 0 ; i < nn_ ; i++){

            v[i+NN_*nm_+nn_] = (v[i+NN_*nm_+nn_] > LB[i]+eps_x) ? v[i+NN_*nm_+nn_] : LB[i]+eps_x;
            v[i+NN_*nm_+nn_] = (v[i+NN_*nm_+nn_] < UB[i]-eps_x) ? v[i+NN_*nm_+nn_] : UB[i]-eps_x;

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            v[i+NN_*nm_+nn_] = (v[i+NN_*nm_+nn_] > LB[i]+eps_u) ? v[i+NN_*nm_+nn_] : LB[i]+eps_u;
            v[i+NN_*nm_+nn_] = (v[i+NN_*nm_+nn_] < UB[i]-eps_u) ? v[i+NN_*nm_+nn_] : UB[i]-eps_u;

        }

        //********** Update dual variables **********//

        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){ // Computation of lambda^{k+1}

            #ifdef SCALAR_RHO
            lambda[i] += rho * (z[i] - v[i]);
            #else
            lambda[i] += rho[i] * (z[i] - v[i]);
            #endif

        }

        // Compute the residuals

        res_flag = 0; // Reset the residual flag

        for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){
            
            res_fixed_point = v[i] - v_old[i];
            res_primal_feas = z[i] - v[i];
            // Obtain absolute values
            res_fixed_point = (res_fixed_point > 0.0) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = (res_primal_feas > 0.0) ? res_primal_feas : -res_primal_feas;
            
            if (res_fixed_point > tol_d || res_primal_feas > tol_p){

                res_flag = 1;
                break;

            }

        }

        // Exit condition
        if (res_flag == 0){
            done = 1;
            *e_flag = 1;
        }
        else if (k >= k_max){
            done = 1;
            *e_flag = -1;        
        }

    }

    // Measure time
    #if MEASURE_TIME == 1
    read_time(&post_solve);
    get_elapsed_time(&sol->solve_time, &post_solve, &post_update);
    #endif

    // Control action
    #if in_engineering == 1
    for (unsigned int i = nn_ ; i < nm_ ; i++){

        u_opt[i-nn_] = v[i] * scaling_i_u[i-nn_] + OpPoint_u[i-nn];

    }
    #else
    for(unsigned int i = nn_ ; i < nm_ ; i++){
        
        u_opt[i-nn_] = v[i];
    
    }
    #endif

    // Return number of iterations
    *k_in = k;

    // Save solution into structure
    #ifdef DEBUG
    for (unsigned int i = 0 ; i < (NN_+1)*nm_+nn_ ; i++){
        
        sol->z[i] = z[i];
        sol->v[i] = v[i];
        sol->lambda[i] = lambda[i];
    
    }
    #endif

    // Measure time
    #if MEASURE_TIME == 1
    read_time(&post_polish);
    get_elapsed_time(&sol->polish_time, &post_polish, &post_solve);
    get_elapsed_time(&sol->run_time, &post_polish, &start);
    #endif

 }

 void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *d){

    // We are using the independent term vector "d" to return the solution vector "z" so as to save memory

    // Forward substitution

    for (unsigned int i = 0 ; i < nn_ ; i++){

        for(unsigned int p = 0 ; p < i ; p++){

            d[i] -= Beta[0][p][i] * d[p]; 
        
        }
        
        d[i] *= Beta[0][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    for (unsigned int i = 0 ; i < nn_ ; i++){

        for(unsigned int p = 0 ; p < i ; p++){

            d[nn_+i] -= Beta[1][p][i] * d[nn_+p]; 
        
        }

        for(unsigned int p = 0; p < nn_ ; p++){

            d[nn_+i] -= Alpha[0][p][i] * d[p];

        }
        
        d[nn_+i] *= Beta[1][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    // Skipping the Alpha which is equal to 0
    for (unsigned int i = 0 ; i < nn_ ; i++){

        for(unsigned int p = 0 ; p < i ; p++){

            d[(2)*nn_+i] -= Beta[2][p][i] * d[(2)*nn_+p]; 
        
        }
        
        d[(2)*nn_+i] *= Beta[2][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    // Continue with the rest of Alpha's and Beta's once the Alpha equal to 0 is skipped
    for (unsigned int k=3 ; k < NN_+2 ; k++){
        
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for(unsigned int p = 0 ; p < i ; p++){

                d[(k)*nn_+i] -= Beta[k][p][i] * d[(k)*nn_+p]; 
            
            }

            for(unsigned int p = 0; p < nn_ ; p++){

                d[(k)*nn_+i] -= Alpha[k-2][p][i] * d[(k-1)*nn_+p];

            }
            
            d[(k)*nn_+i] *= Beta[k][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

        }

    }

    // Backward substitution

    for(unsigned int i = nn_ ; i > 0 ; i--){

        for(unsigned int p = i+1 ; p <= nn_ ; p++){

            d[(NN_+1)*nn_+i-1] -= Beta[NN_+1][i-1][p-1] * d[(NN_+1)*nn_+(p-1)];

        }

        d[(NN_+1)*nn_+i-1] *= Beta[NN_+1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    for (unsigned int k = NN_+1 ; k > 2 ; k--){

        for(unsigned int i = nn_ ; i > 0 ; i--){

            for(unsigned int p = i+1 ; p <= nn_ ; p++){

                d[(k-1)*nn_+i-1] -= Beta[k-1][i-1][p-1] * d[(k-1)*nn_+(p-1)];

            }
                
            for(unsigned int p = 0 ; p < nn_ ; p++){
            
                d[(k-1)*nn_+i-1] -= Alpha[k-2][i-1][p] * d[(k)*nn_+p];
            
            }

            d[(k-1)*nn_+i-1] *= Beta[k-1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

        }

    }

    // Skipping the Alpha which is equal to 0
    for(unsigned int i = nn_ ; i > 0 ; i--){

        for(unsigned int p = i+1 ; p <= nn_ ; p++){

            d[nn_+i-1] -= Beta[1][i-1][p-1] * d[nn_+(p-1)];

        }

        d[nn_+i-1] *= Beta[1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    // Continue with the rest of Alpha's and Beta's once the Alpha equal to 0 is skipped
    for(unsigned int i = nn_ ; i > 0 ; i--){

        for(unsigned int p = i+1 ; p <= nn_ ; p++){

            d[i-1] -= Beta[0][i-1][p-1] * d[(p-1)];

        }
            
        for(unsigned int p = 0 ; p < nn_ ; p++){
        
            d[i-1] -= Alpha[0][i-1][p] * d[nn_+p];
        
        }

        d[i-1] *= Beta[0][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    

}

#ifdef SCALAR_RHO
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d){

    // First nn components
    for(unsigned int j = 0 ; j < nn_ ; j++){
        
        for (unsigned int k = 0 ; k < nn_ ; k++){
                
            z[j] += Q_rho_i[j][k] * d[k];

        }

    }
    
    // From first nn+1 components to nn+mm components
    for(unsigned int j = nn_ ; j < nm_ ; j++){

        for (unsigned int k = 0 ; k < mm_ ; k++){
                
            z[j] += R_rho_i[j-nn_][k] * d[nn_+k];

        }

    }

    // From nn+mm+1 components to nn+mm+nn components
    for(unsigned int j = nm_ ; j < nm_+nn_ ; j++){

        for (unsigned int k = 0 ; k < nn_ ; k++){

            z[j] += Q_rho_i[j-nm_][k] * d[nm_+k];

        }

    }
    
    // Rest of elements
    for (unsigned int i = 1 ; i < NN_ ; i++){ // Moving in groups of nn+mm components

        for (unsigned int j = 0 ; j < nn_ ; j++){ // Moving inside the groups
            
            for (unsigned int k = 0 ; k < nn_ ; k++){ // Multiplying the rows by the corresponding part of the independent term vector
                
                z[i*nm_+nn_+j] += Q_rho_i[j][k] * d[i*nm_+nn_+k];

            }

        }

        for (unsigned int j = nn_ ; j < nm_ ; j++){

            for (unsigned int k = 0 ; k < mm_ ; k++){

                z[i*nm_+nn_+j] += R_rho_i[j-nn_][k] * d[i*nm_+nn_+nn_+k];

            }

        }

    }

    for (unsigned int j = 0 ; j < nn_ ; j++){

        for (unsigned int k = 0 ; k < nn_ ; k++){

            z[NN_*nm_ + nn_ + j] += T_rho_i[j][k] * d[NN_*nm_+nn_+k];

        }
        
    }

    for (unsigned int j = nn_ ; j < nm_ ; j++){

        for (unsigned int k = 0 ; k < mm_ ; k++){

            z[NN_*nm_ + nn_ + j] += S_rho_i[j-nn_][k] * d[NN_*nm_+nn_+nn_+k];

        }

    }

}
#else
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_][nn_], const double (*R_rho_i)[mm_][mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d){

    // First nn components
    for(unsigned int j = 0 ; j < nn_ ; j++){
        
        for (unsigned int k = 0 ; k < nn_ ; k++){
                
            z[j] += Q_rho_i[0][j][k] * d[k];

        }

    }
    
    // From first nn+1 components to nn+mm components
    for(unsigned int j = nn_ ; j < nm_ ; j++){

        for (unsigned int k = 0 ; k < mm_ ; k++){
                
            z[j] += R_rho_i[0][j-nn_][k] * d[nn_+k];

        }

    }

    // From nn+mm+1 components to nn+mm+nn components
    for(unsigned int j = nm_ ; j < nm_+nn_ ; j++){

        for (unsigned int k = 0 ; k < nn_ ; k++){

            z[j] += Q_rho_i[1][j-nm_][k] * d[nm_+k];

        }

    }
    
    // Rest of elements
    for (unsigned int i = 1 ; i < NN_ ; i++){ // Moving in groups of nn+mm components

        for (unsigned int j = 0 ; j < nn_ ; j++){ // Moving inside the groups
            
            for (unsigned int k = 0 ; k < nn_ ; k++){ // Multiplying the rows by the corresponding part of the independent term vector
                
                z[i*nm_+nn_+j] += Q_rho_i[i+1][j][k] * d[i*nm_+nn_+k];

            }

        }

        for (unsigned int j = nn_ ; j < nm_ ; j++){

            for (unsigned int k = 0 ; k < mm_ ; k++){

                z[i*nm_+nn_+j] += R_rho_i[i][j-nn_][k] * d[i*nm_+nn_+nn_+k];

            }

        }

    }

    for (unsigned int j = 0 ; j < nn_ ; j++){

        for (unsigned int k = 0 ; k < nn_ ; k++){

            z[NN_*nm_ + nn_ + j] += T_rho_i[j][k] * d[NN_*nm_ + nn_ + k];

        }
        
    }

    for (unsigned int j = nn_ ; j < nm_ ; j++){

        for (unsigned int k = 0 ; k < mm_ ; k++){

            z[NN_*nm_ + nn_ + j] += S_rho_i[j-nn_][k] * d[NN_*nm_ + nn_ + nn_ + k];

        }

    }
}

#endif

double solve_max_QP(double b, double beta, double d, double e){

    // Solves a scalar problem of the form: min_{x} (1/2)*x^2 - b*x + beta*max(d-x,x-e,0), where d < e.
    double x1 = 0.0;
    double x2 = 0.0;
    double x3 = 0.0;

    double x = 0.0; // Solution

    x1 = b + beta;
    x2 = b;
    x3 = b-beta;

    if (x1 <= d){

        x = x1;
    
    }

    else if (x1 > d && x2 < d){

        x = d;

    }

    else if (x2 >= d && x2 <= e){

        x = x2;

    }

    else if (x2 > e && x3 < e){

        x = e;

    }

    else if (x3 >= e){

        x = x3;

    }

    return x;
    
}

double functional_eval(double x_opt, double y_opt, double b, double c, double alpha, double beta, double d, double e){
    
    // Evaluates the functional J = (1/2) * (x_opt^(2)+y_opt^(2)) - b*x_opt - c*y_opt + alpha*|x_opt-y_opt| + beta*max(d-x_opt, x_opt-e, 0).
    double J = 0.0;

    J = 0.5*(x_opt*x_opt + y_opt*y_opt) - b*x_opt - c*y_opt;

    // Adding part of alpha*|x-y|
    if (x_opt > y_opt){
        J = J + alpha*(x_opt-y_opt);
    }
    else if (x_opt < y_opt){
        J = J + alpha*(y_opt-x_opt);
    }

    // Adding part of beta*max(d-x_opt, x_opt-e, 0)
    if (d > x_opt){

        J = J + beta*(d-x_opt);

    }
    else if(x_opt > e){

        J = J + beta*(x_opt-e);

    }
    // If none of the two previous conditions are satisfied, then max reamins 0, meaning that d < x_opt < e.

    return J;

}

void solve_abs_max_QP(double *x, double *y, double b, double c, double alpha, double beta, double d, double e, double LB, double UB){
    // Solves a two-dimensional problem of the form: min_{x,y} (1/2)*(x^2+y^2) - bx - cy + alpha*|x-y| + beta*max(d-x,x-e,0) s.t. LB <= y <= UB, 
    // where LB < UB and d < e. Solutions are returned in x and y.

    double x_a_opt = 0.0;
    double x_b_opt = 0.0;
    double x_c_opt = 0.0;
    double y_a_opt = 0.0;
    double y_b_opt = 0.0;
    double y_c_opt = 0.0;

    double J_a_opt = 0.0;
    double J_b_opt = 0.0;
    double J_c_opt = 0.0;

    // Case a: Suppose x==y
    x_a_opt = solve_max_QP((b+c)*0.5, beta*0.5, d, e);
    // Saturate x_a_opt between LB and UB
    if (x_a_opt > UB){
        
        x_a_opt = UB;

    }
    else if (x_a_opt < LB){

        x_a_opt = LB;
    
    }

    y_a_opt = x_a_opt;

    // Case b: Suppose x>y
    x_b_opt = solve_max_QP(b-alpha, beta, d, e);
    y_b_opt = c + alpha;
    // Saturate y_b_opt between LB and UB
    if (y_b_opt > UB){

        y_b_opt = UB;

    }
    else if(y_b_opt < LB){

        y_b_opt = LB;

    }
    // Check if x_b_opt > y_b_opt, as we assummed x>y. If not, then x=y, and we get x_a_opt and y_a_opt.
    if (x_b_opt <= y_b_opt){

        x_b_opt = x_a_opt;
        y_b_opt = y_a_opt;

    }

    // Case c: Suppose x<y
    x_c_opt = solve_max_QP(b+alpha, beta, d, e);
    y_c_opt = c - alpha;
    // Saturate y_c_opt between LB and UB
    if (y_c_opt > UB){
        
        y_c_opt = UB;

    }
    else if(y_c_opt < LB){

        y_c_opt = LB;

    }
    // Check if x_c_opt < y_c_opt, as we assummed x<y. If not, then x=y, and we get x_a_opt and y_a_opt
    if (x_c_opt >= y_c_opt){

        x_c_opt = x_a_opt;
        y_c_opt = y_a_opt;

    }

    // Evaluate the functional with the three pairs of candidates
    J_a_opt = functional_eval(x_a_opt, y_a_opt, b, c, alpha, beta, d, e);
    J_b_opt = functional_eval(x_b_opt, y_b_opt, b, c, alpha, beta, d, e);
    J_c_opt = functional_eval(x_c_opt, y_c_opt, b, c, alpha, beta, d, e);

    // Get the pair of candidates which minimizes the functional
    if (J_a_opt < J_b_opt){

        if (J_a_opt < J_c_opt){

            *x = x_a_opt;
            *y = y_a_opt;

        }
        else{ // J_a_opt >= J_c_opt

            *x = x_c_opt;
            *y = y_c_opt;

        }

    }

    else{ // J_a_opt >= J_b_opt

        if (J_b_opt < J_c_opt){

            *x = x_b_opt;
            *y = y_b_opt;

        }

        else{ // J_b_opt >= J_c_opt

            *x = x_c_opt;
            *y = y_c_opt;

        }

    }

}

#if MEASURE_TIME == 1

spcies_snippet_get_elapsed_time();

spcies_snippet_read_time();

#endif