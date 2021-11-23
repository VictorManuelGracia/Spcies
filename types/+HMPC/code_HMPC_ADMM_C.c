/**
 * Sparse ADMM solver for the HMPC formulation with box constraints.
 *
 * ARGUMENTS:
 * The current system state is given in "pointer_x0". Pointer to array of size nn.
 * The state reference is given in "pointer_xr". Pointer to array of size nn.
 * The input reference is given in "pointer_ur". Pointer to array of size mm.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm.
 * The number of iterations is returned in "pointer_k". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 *
 * If CONF_MATLAB is defined, them the solver uses slightly different arguments for the mex file.
 *
 */

#ifdef CONF_MATLAB

void HMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *s_opt, double *z_hat_opt, double *s_hat_opt, double *lambda_opt, double *mu_opt){

#else

void HMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution *sol){

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
#ifdef CONF_MATLAB
double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
#else
int k = 0; // Number of iterations
#endif
double x0[nn]; // Current system state
double xr[nn] = {0.0}; // State reference
double ur[mm] = {0.0}; // Control input reference

// For solving the system of equations
double q[dim] = {0.0}; // Cost function vector

#ifdef NON_SPARSE
double primal_hat[dim+n_s]; // Vector of primal decision variables primal = (z_hat, s_hat)
double q_hat[dim+n_s]; // Vector q_hat, which is the first part of rhs
double *z_hat = &primal_hat[0]; // Primal decision variable z_hat
double *s_hat = &primal_hat[dim]; // Primal decision variable s_hat
#else
double rhs[nrow_M]; // Vector for storing the right-hand-side of the W system of equations
double *z_hat = &rhs[0]; // Primal decision variable z_hat
double *s_hat = &rhs[dim]; // Primal decision variable s_hat
#endif
// double bh[n_eq+n_s] = {0.0}; // Vector of the equality constraints

double primal[dim+n_s] = {0.0}; // Vector of primal decision variables primal = (z, s)
double primal_ant[dim+n_s]; // Vector containing the value of primal at iteration k-1
// double primal_hat[dim+n_s] = {0.0}; // Vector of primal decision variables primal_hat = (z_hat, s_hat)
double dual[dim+n_s] = {0.0}; // Vector of dual variables dual = (lambda, mu)
double *z = &primal[0]; // Primal decision variable z
double *s = &primal[dim]; // Primal decision variable s
double *lambda = &dual[0]; // Dual variable lambda
double *mu = &dual[dim]; // Dual variable mu
double s_norm; // Norm of the second-to-the-last elements of s
double s_proj_step; // Variable used to project s onto the SOCs
#ifdef COUPLED_CONSTRAINTS
double *s_cone = &s[NN*n_y]; // Pointer to the first component of s with cone constraints
#endif

unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
double res_fixed_point; // Variable used to determine if a fixed point has been reached
double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
$INSERT_VARIABLES$

// Constant variables
$INSERT_CONSTANTS$

// Obtain variables in scaled units
#if in_engineering == 1
for(unsigned int i = 0; i < nn; i++){
    x0[i] = scaling_x[i]*( pointer_x0[i] - OpPoint_x[i] );
    xr[i] = scaling_x[i]*( pointer_xr[i] - OpPoint_x[i] );
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = scaling_u[i]*( pointer_ur[i] - OpPoint_u[i] );
}
#endif
#if in_engineering == 0
for(unsigned int i = 0; i < nn; i++){
    x0[i] = pointer_x0[i];
    xr[i] = pointer_xr[i];
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = pointer_ur[i];
}
#endif

// Update the elements of rhs corresponding to the current system state
#ifdef NON_SPARSE
for(unsigned int j = 0; j < nn; j++){
    bh[j] = 0.0;
    for(unsigned int i = 0; i < nn; i++){
        bh[j] -= A[j][i]*x0[i];
    }
}
#else
for(unsigned int j = 0; j < nn; j++){
    bh[idx_x0[j]] = 0.0;
    for(unsigned int i = 0; i < nn; i++){
        bh[idx_x0[j]] -= A[j][i]*x0[i];
    }
}
#endif

// Update the elements of q corresponding to the reference
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[(NN-1)*nm+mm+j] -= Te[j][i]*xr[i];
    }
}
for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[(NN-1)*nm+mm+3*nn+j] -= Se[j][i]*ur[i];
    }
}

// Algorithm
while(done == 0){

    k += 1; // Increment iteration counter

    // Save the value of primal into variable primal_ant
    memcpy(primal_ant, primal, sizeof(double)*(dim+n_s));

    //********** Update primal_hat **********//

    // Compute q_hat = -[q + lambda - sigma*z; mu - rho*s], which is the first part of rhs
    #ifdef NON_SPARSE
    for(unsigned int j = 0; j < dim; j++){
        q_hat[j] = sigma*z[j] - q[j] - lambda[j];
    }
    for(unsigned int j = 0; j < n_s; j++){
        q_hat[j+dim] = rho*s[j] - mu[j];
    }
    #else
    for(unsigned int j = 0; j < dim; j++){
        rhs[j] = sigma*z[j] - q[j] - lambda[j];
    }
    for(unsigned int j = 0; j < n_s; j++){
        rhs[j+dim] = rho*s[j] - mu[j];
    }
    // Compute the last part of rhs, which corresponds to bh
    for(unsigned int j = 0; j < n_s+n_eq; j++){
        rhs[dim+n_s+j] = bh[j];
    }
    #endif

    // Solve the system of equations
    // We have two ways of solving it
    // If NON_SPARSE is defined, then we use aux = M2*bh - M1*q_hat;
    // If it is not defined then we solve the system of equations W*[primal; v]=rhs
    // using sparse LDL representation using the method from the QDLDL solver ( https://github.com/oxfordcontrol/qdldl)

    #ifdef NON_SPARSE

    for(unsigned int i = 0; i < dim+n_s; i++){
        primal_hat[i] = 0.0;
    }
    for(unsigned int i = 0; i < dim+n_s; i++){
        for(unsigned int j = 0; j < dim_M2; j++){
            primal_hat[i] += M2[i][j]*bh[j];
        }
    }
    for(unsigned int i = 0; i < dim+n_s; i++){
        for(unsigned int j = 0; j < dim+n_s; j++){
            primal_hat[i] -= M1[i][j]*q_hat[j];
        }
    }

    #else

    // Forward substitution
    for(unsigned int i = 0; i < nrow_M; i++){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            rhs[L_row[j]] -= L_val[j]*rhs[i];
        }
    }

    // Divide by the diagonal matrix D
    for(unsigned int j = 0; j < nrow_M; j++){
        rhs[j] *= Dinv[j];
    }

    // Backward substitution
    for(unsigned int i = nrow_M-1; i != -1; i--){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            rhs[i] -= L_val[j]*rhs[L_row[j]];
        }
    }

    #endif

    //********** Update dual in symmetric ADMM **********//

    #ifdef IS_SYMMETRIC
    // Update lambda
    for(unsigned int j = 0; j < dim; j++){
        lambda[j] += alpha_SADMM*sigma*(z_hat[j] - z[j]);
    }

    // Update mu
    for(unsigned int j = 0; j < n_s; j++){
        mu[j] += alpha_SADMM*rho*(s_hat[j] - s[j]);
    }
    #endif


    //********** Update primal **********//

    // Compute z
    for(unsigned int j = 0; j < dim; j++){
        z[j] = z_hat[j] + sigma_i*lambda[j];
    }
    #ifndef COUPLED_CONSTRAINTS
    // Upper and lower bounds
    for(unsigned int j = 0; j < dim-3*nn-3*mm; j++){
        z[j] = (z[j] > LB[j]) ? z[j] : LB[j]; // maximum between z and the lower bound
        z[j] = (z[j] > UB[j]) ? UB[j] : z[j]; // minimum between z and the upper bound
    }
    #endif

    // Compute s
    for(unsigned int j = 0; j < n_s; j++){
        s[j] = s_hat[j] + rho_i*mu[j];
    }

    #ifndef COUPLED_CONSTRAINTS

    #ifdef USE_SOC
    // Project onto the SOC
    for(unsigned int j = 0; j < n_soc; j++){
        proj_SOC3( &s[3*j], 1.0, 0.0);
    }
    #else
    // Project onto the SOC
    for(unsigned int j = 0; j < nm; j++){
        proj_SOC3( &s[3*j], 1.0, LBy[j]);
        proj_SOC3( &s[3*j], -1.0, UBy[j]);
    }
    #endif

    #else

    // Project onto the box constraints
    for(unsigned int j = 0; j < NN; j++){
        for(unsigned int i = 0; i < n_y; i++){
            s[j*n_y+i] = (s[j*n_y+i] > LBy[i]) ? s[j*n_y+i] : LBy[i]; // maximum between s and the lower bound
            s[j*n_y+i] = (s[j*n_y+i] > UBy[i]) ? UBy[i] : s[j*n_y+i]; // minimum between s and the upper bound
        }
    }

    #ifdef USE_SOC
    // Project onto the SOC
    for(unsigned int j = 0; j < n_soc; j++){
        proj_SOC3( &s_cone[3*j], 1.0, 0.0);
    }
    #else
    // Project onto the SOC
    for(unsigned int j = 0; j < n_y; j++){
        proj_SOC3( &s_cone[3*j], 1.0, LBy[j]);
        proj_SOC3( &s_cone[3*j], -1.0, UBy[j]);
    }
    #endif

    #endif

    //********** Update dual **********//

    #ifdef IS_SYMMETRIC

    // Update lambda
    for(unsigned int j = 0; j < dim; j++){
        lambda[j] += alpha_SADMM*sigma*(z_hat[j] - z[j]);
    }

    // Update mu
    for(unsigned int j = 0; j < n_s; j++){
        mu[j] += alpha_SADMM*rho*(s_hat[j] - s[j]);
    }

    #else

    // Update lambda
    for(unsigned int j = 0; j < dim; j++){
        lambda[j] += sigma*(z_hat[j] - z[j]);
    }

    // Update mu
    for(unsigned int j = 0; j < n_s; j++){
        mu[j] += rho*(s_hat[j] - s[j]);
    }

    #endif

    //********** Update residuals **********//

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < dim + n_s; j++){
    // for(unsigned int j = dim+n_s-1; j !=-1; j--){
        res_fixed_point = primal_ant[j] - primal[j];
        #ifdef NON_SPARSE
        res_primal_feas = primal[j] - primal_hat[j];
        #else
        res_primal_feas = primal[j] - rhs[j];
        #endif
        // Obtain absolute values
        res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
        res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
        if( res_fixed_point > tol_d || res_primal_feas > tol_p){
            res_flag = 1;
            break;
        }
    }


    //********** Exit condition **********//
    
    if(res_flag == 0){
        done = 1;
        #ifdef CONF_MATLAB
        e_flag[0] = 1.0;
        #else
        *e_flag = 1;
        #endif
    }
    else if( k >= k_max ){
        done = 1;
        #ifdef CONF_MATLAB
        e_flag[0] = -1.0;
        #else
        *e_flag = -1;
        #endif
    }

}

// Control action
#if in_engineering == 1
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = z[j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = z[j];
}
#endif

// Return number of iterations
#ifdef CONF_MATLAB
pointer_k[0] = k;
#else
*pointer_k = k;
#endif

// Save solution into structure
#ifdef DEBUG

#ifdef CONF_MATLAB

for(unsigned int j = 0; j < dim; j++){
    z_opt[j] = z[j];
    z_hat_opt[j] = z_hat[j];
    lambda_opt[j] = lambda[j];
}
// for(unsigned int j = 0; j < n_eq+n_s; j++){
    // z_opt[j] = rhs[j];
// }
for(unsigned int j = 0; j < n_s; j++){
    s_opt[j] = s[j];
    s_hat_opt[j] = s_hat[j];
    mu_opt[j] = mu[j];
}

#else

for(unsigned int j = 0; j < dim; j++){
    sol->z[j] = z[j];
    sol->z_hat[j] = z_hat[j];
    sol->lambda[j] = lambda[j];
}
for(unsigned int j = 0; j < n_s; j++){
    sol->s[j] = s[j];
    sol->s_hat[j] = s_hat[j];
    sol->mu[j] = mu[j];
}

#endif

#endif

}

//********** Projection to SOC **********//
// Returns, in x, the projection of x onto the shifted-SOC given by alpha and d
// The function assumes that the dimension of x is 3, which is the case in the HMPC solver

void proj_SOC3(double *x, double alpha, double d){

    double x_0 = x[0];
    double x_norm = 0.0;
    double x_proj_step;
    double corrected_x_0;

    // Compute x_norm (the norm of the 2nd-to-last elements of x)
    for(unsigned int j = 1; j < 3; j++){
        x_norm += x[j]*x[j];
    }
    x_norm = sqrt(x_norm);

    // Compute auxliary term
    corrected_x_0 = alpha*(x_0 - d);

    // Project onto the shifted-SOC
    if(x_norm <= corrected_x_0){
    } else if(x_norm <= -corrected_x_0){
        x[0] = d; 
        for(unsigned int j = 1; j < 3; j++){
            x[j] = 0.0;
        }
    } else{
        x_proj_step = (corrected_x_0 + x_norm)/(2*x_norm);
        x[0] = x_proj_step*x_norm*alpha + d;
        for(unsigned int j = 1; j < 3; j++){
            x[j] = x_proj_step*x[j];
        }
    }

}

