#ifndef ACADOS_SIM_crazyflie_H_
#define ACADOS_SIM_crazyflie_H_

#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

int crazyflie_acados_sim_create();
int crazyflie_acados_sim_solve();
int crazyflie_acados_sim_free();

sim_config  * crazyflie_acados_get_sim_config();
sim_in      * crazyflie_acados_get_sim_in();
sim_out     * crazyflie_acados_get_sim_out();
void        * crazyflie_acados_get_sim_dims();
sim_opts    * crazyflie_acados_get_sim_opts();
sim_solver  * crazyflie_acados_get_sim_solver();

// ** global data **
extern sim_config  * crazyflie_sim_config;
extern sim_in      * crazyflie_sim_in;
extern sim_out     * crazyflie_sim_out; 
extern void        * crazyflie_sim_dims;
extern sim_opts    * crazyflie_sim_opts;
extern sim_solver  * crazyflie_sim_solver; 

#ifdef __cplusplus
}
#endif



extern external_function_casadi * sim_forw_vde_casadi;
extern external_function_casadi * sim_expl_ode_fun_casadi;




#endif  // ACADOS_SIM_crazyflie_H_