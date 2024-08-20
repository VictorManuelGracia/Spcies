#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

#if MEASURE_TIME == 1
#if WIN32
#include <Windows.h>
#else // If Linux
#include <time.h>
#endif
#endif

typedef struct {
    double z[(NN_+1)*nm_+nn_]; // Optimal z
    double v[(NN_+1)*nm_+nn_]; // Optimal v
    double lambda[(NN_+1)*nm_+nn_]; // Optimal lambda
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} sol_$INSERT_NAME$;

void MPCT_x1_free_ADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);

// TODO: Create the case when IS_DIAG is equal to 1, meaning that Q, R, S and T are diagonal matrices. Many operations can be avoided in that case.
#ifdef SCALAR_RHO
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d);
#else
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_][nn_], const double (*R_rho_i)[mm_][mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d);
#endif

void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *d);

double solve_max_QP(double b, double beta, double d, double e); // Solves a scalar problem of the form: min_{x} (1/2)*x^2 - b*x + beta*max(d-x,x-e,0), where d < e.

double functional_eval(double x_opt, double y_opt, double b, double c, double alpha, double beta, double d, double e); 
// Evaluates the functional J = (1/2) * (x_opt^(2)+y_opt^(2)) - b*x_opt - c*y_opt + alpha*|x_opt-y_opt| + beta*max(d-x_opt, x_opt-e, 0).

void solve_abs_max_QP(double *x, double *y, double b, double c, double alpha, double beta, double d, double e, double LB, double UB); 
// Solves a two-dimensional problem of the form: min_{x,y} (1/2)*(x^2+y^2) - bx - cy + alpha*|x-y| + beta*max(d-x,x-e,0) s.t. LB <= y <= UB, 
// where LB < UB and d < e. Solutions are returned in x and y.

#if MEASURE_TIME == 1
spcies_snippet_get_elapsed_time();
spcies_snippet_read_time();
#endif

#endif