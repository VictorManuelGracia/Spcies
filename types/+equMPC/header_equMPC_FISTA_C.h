#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

typedef struct {
    double z[(NN-1)*nm+mm]; // Optimal z
    double lambda[NN*nn]; // Optimal lambda
} sol_equMPC_FISTA;

#ifdef CONF_MATLAB

void equMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *lambda_opt);

#else

void equMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, sol_equMPC_FISTA *sol);

#endif

void compute_z_lambda_equMPC_FISTA(double *z_0, double z[][nm], double lambda[][nn], double *q);

void compute_residual_vector_equMPC_FISTA(double res_vec[][nn], double *z_0, double z[][nm], double *b, double *xr);

void solve_W_matrix_form(double mu[][nn]);

#endif

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies
