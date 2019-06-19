#ifndef ACADOS_SOLVER_crazyflie_pos_H_
#define ACADOS_SOLVER_crazyflie_pos_H_

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
extern "C" {
int acados_create();
int acados_solve();
int acados_free();

ocp_nlp_in * acados_get_nlp_in();
ocp_nlp_out * acados_get_nlp_out();
ocp_nlp_solver * acados_get_nlp_solver();
ocp_nlp_config * acados_get_nlp_config();
void * acados_get_nlp_opts();
ocp_nlp_dims * acados_get_nlp_dims();
}
// ** global data **
extern ocp_nlp_in * nlp_in;
extern ocp_nlp_out * nlp_out;
extern ocp_nlp_solver * nlp_solver;
extern void * nlp_opts;
extern ocp_nlp_plan * nlp_solver_plan;
extern ocp_nlp_config * nlp_config;
extern ocp_nlp_dims * nlp_dims;


extern external_function_casadi * forw_vde_casadi;









#endif  // ACADOS_SOLVER_crazyflie_pos_H_
