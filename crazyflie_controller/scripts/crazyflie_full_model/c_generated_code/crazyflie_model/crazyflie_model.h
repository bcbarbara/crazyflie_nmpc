#ifndef crazyflie_MODEL
#define crazyflie_MODEL

#ifdef __cplusplus
extern "C" {
#endif


/* explicit ODE */

// explicit ODE
int crazyflie_expl_ode_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int crazyflie_expl_ode_fun_work(int *, int *, int *, int *);
const int *crazyflie_expl_ode_fun_sparsity_in(int);
const int *crazyflie_expl_ode_fun_sparsity_out(int);
int crazyflie_expl_ode_fun_n_in();
int crazyflie_expl_ode_fun_n_out();

// explicit forward VDE
int crazyflie_expl_vde_forw(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int crazyflie_expl_vde_forw_work(int *, int *, int *, int *);
const int *crazyflie_expl_vde_forw_sparsity_in(int);
const int *crazyflie_expl_vde_forw_sparsity_out(int);
int crazyflie_expl_vde_forw_n_in();
int crazyflie_expl_vde_forw_n_out();

// explicit adjoint VDE
int crazyflie_expl_vde_adj(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int crazyflie_expl_vde_adj_work(int *, int *, int *, int *);
const int *crazyflie_expl_vde_adj_sparsity_in(int);
const int *crazyflie_expl_vde_adj_sparsity_out(int);
int crazyflie_expl_vde_adj_n_in();
int crazyflie_expl_vde_adj_n_out();

// explicit adjoint ODE jac
int crazyflie_expl_ode_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int crazyflie_expl_ode_hess_work(int *, int *, int *, int *);
const int *crazyflie_expl_ode_hess_sparsity_in(int);
const int *crazyflie_expl_ode_hess_sparsity_out(int);
int crazyflie_expl_ode_hess_n_in();
int crazyflie_expl_ode_hess_n_out();



#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // crazyflie_MODEL