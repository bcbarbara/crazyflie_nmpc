/*
 *    This file is part of acados.
 *
 *    acados is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    acados is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with acados; if not, write to the Free Software Foundation,
 *    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// example specific
#include "crazyflie_model/crazyflie_model.h"



#include "acados_solver_crazyflie.h"
#define NX_    13
#define NZ_    0
#define NU_    4
#define NP_    0
#define NBX_   0
#define NBU_   4
#define NSBX_  0
#define NSBU_  0
#define NSH_  0
#define NSHN_  0
#define NSBXN_ 0
#define NS_    0
#define NSN_   0
#define NG_    0
#define NBXN_  0
#define NGN_   0
#define NY_    17
#define NYN_   13
#define N_     50
#define NPD_   0
#define NPDN_  0
#define NH_    0
#define NHN_   0

#if NX_ < 1
#define NX   1
#else
#define NX   NX_
#endif

#if NZ_ < 1
#define NZ   1
#else
#define NZ   NZ_
#endif

#if NU_ < 1
#define NU   1
#else
#define NU   NU_
#endif

#if NP_ < 1
#define NP   1
#else
#define NP   NP_
#endif

#if NBX_ < 1
#define NBX   1
#else
#define NBX   NBX_
#endif

#if NBU_ < 1
#define NBU   1
#else
#define NBU   NBU_
#endif

#if NSBX_ < 1
#define NSBX   1
#else
#define NSBX   NSBX_
#endif

#if NSBU_ < 1
#define NSBU   1
#else
#define NSBU   NSBU_
#endif

#if NSH_ < 1
#define NSH   1
#else
#define NSH   NSH_
#endif

#if NSHN_ < 1
#define NSHN   1
#else
#define NSHN   NSHN_
#endif

#if NS_ < 1
#define NS   1
#else
#define NS   NS_
#endif

#if NSBXN_ < 1
#define NSBXN   1
#else
#define NSBXN   NSBXN_
#endif

#if NSN_ < 1
#define NSN   1
#else
#define NSN   NSN_
#endif

#if NG_ < 1
#define NG   1
#else
#define NG   NG_
#endif

#if NBXN_ < 1
#define NBXN   1
#else
#define NBXN   NBXN_
#endif

#if NGN_ < 1
#define NGN   1
#else
#define NGN  NGN_
#endif

#if NY_ < 1
#define NY   1
#else
#define NY   NY_
#endif

#if NYN_ < 1
#define NYN   1
#else
#define NYN   NYN_
#endif

#if N_ < 1
#define N   1
#else
#define N   N_
#endif

#if NPD_ < 1
#define NPD   1
#else
#define NPD   NPD_
#endif

#if NPDN_ < 1
#define NPDN   1
#else
#define NPDN   NPDN_
#endif

#if NH_ < 1
#define NH   1
#else
#define NH   NH_
#endif

#if NHN_ < 1
#define NHN   1
#else
#define NHN   NHN_
#endif

int acados_create() {

    int status = 0;

    double Tf = 1;

    // initial state x0
    int idxbx0[NX];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;

    double lbx0[NX];
    lbx0[0] = 0.2;
    lbx0[1] = -0.4;
    lbx0[2] = 0.0;
    lbx0[3] = 1.0;
    lbx0[4] = 0.0;
    lbx0[5] = 0.0;
    lbx0[6] = 0.0;
    lbx0[7] = 0.0;
    lbx0[8] = 0.0;
    lbx0[9] = 0.0;
    lbx0[10] = 0.0;
    lbx0[11] = 0.0;
    lbx0[12] = 0.0;

    double ubx0[NX];
    ubx0[0] = 0.2;
    ubx0[1] = -0.4;
    ubx0[2] = 0.0;
    ubx0[3] = 1.0;
    ubx0[4] = 0.0;
    ubx0[5] = 0.0;
    ubx0[6] = 0.0;
    ubx0[7] = 0.0;
    ubx0[8] = 0.0;
    ubx0[9] = 0.0;
    ubx0[10] = 0.0;
    ubx0[11] = 0.0;
    ubx0[12] = 0.0;

    // set up bounds for u
    int idxbu[NBU];
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;

    double lbu[NBU];
    lbu[0] = 0;
    lbu[1] = 0;
    lbu[2] = 0;
    lbu[3] = 0;

    double ubu[NBU];
    ubu[0] = 48000.02374769851;
    ubu[1] = 48000.02374769851;
    ubu[2] = 48000.02374769851;
    ubu[3] = 48000.02374769851;
    
    // set up soft bounds for u
    int idxsbu[NSBU];

    double lsbu[NSBU];

    double usbu[NSBU];
    
    // set up soft bounds for nonlinear constraints
    int idxsh[NSH];

    double lsh[NSH];

    double ush[NSH];
    
    // bounds on x
    int idxbx[NBX];

    double lbx[NBX];

    double ubx[NBX];
    
    // soft bounds on x
    int idxsbx[NSBX];

    double lsbx[NSBX];

    double usbx[NSBX];

    // set up general constraints for stage 0 to N-1 
    double D[NG*NU];
    double C[NG*NX];
    double lg[NG];
    double ug[NG];

    // set up nonlinear constraints for stage 0 to N-1 
    double lh[NH];
    double uh[NH];
    
    // set up bounds for last stage
    // x
    int idxbx_e[NBXN];

    double lbx_e[NBXN];
    
    double ubx_e[NBXN];
    
    // soft bounds on x
    int idxsbx_e[NSBXN];
    
    double lsbx_e[NSBXN];
    
    double usbx_e[NBXN];

    // set up soft bounds for nonlinear constraints
    int idxsh_e[NSHN];

    double lsh_e[NSHN];

    double ush_e[NSHN];
    
    // set up general constraints for last stage 
    double C_e[NGN*NX];
    double lg_e[NGN];
    double ug_e[NGN];

    // set up nonlinear constraints for last stage 
    double lh_e[NHN];
    double uh_e[NHN];

    double yref[NY];
    double W[NY*NY];

    double Vx[NY*NX];
    double Vu[NY*NU];
    double Vz[NY*NZ];
    double Zl[NS];
    double Zu[NS];
    double zl[NS];
    double zu[NS];

    double yref_e[NYN];
    double W_e[NYN*NYN];
    double Zl_e[NSN];
    double Zu_e[NSN];
    double zl_e[NSN];
    double zu_e[NSN];

    double Vx_e[NYN*NX];
    
    for (int ii = 0; ii < NU + NX; ii++)
        yref[ii] = 0.0;
    
    W[0 + (NY) * 0] = 4.0;
    W[0 + (NY) * 1] = 0.0;
    W[0 + (NY) * 2] = 0.0;
    W[0 + (NY) * 3] = 0.0;
    W[0 + (NY) * 4] = 0.0;
    W[0 + (NY) * 5] = 0.0;
    W[0 + (NY) * 6] = 0.0;
    W[0 + (NY) * 7] = 0.0;
    W[0 + (NY) * 8] = 0.0;
    W[0 + (NY) * 9] = 0.0;
    W[0 + (NY) * 10] = 0.0;
    W[0 + (NY) * 11] = 0.0;
    W[0 + (NY) * 12] = 0.0;
    W[0 + (NY) * 13] = 0.0;
    W[0 + (NY) * 14] = 0.0;
    W[0 + (NY) * 15] = 0.0;
    W[0 + (NY) * 16] = 0.0;
    
    W[1 + (NY) * 0] = 0.0;
    W[1 + (NY) * 1] = 4.0;
    W[1 + (NY) * 2] = 0.0;
    W[1 + (NY) * 3] = 0.0;
    W[1 + (NY) * 4] = 0.0;
    W[1 + (NY) * 5] = 0.0;
    W[1 + (NY) * 6] = 0.0;
    W[1 + (NY) * 7] = 0.0;
    W[1 + (NY) * 8] = 0.0;
    W[1 + (NY) * 9] = 0.0;
    W[1 + (NY) * 10] = 0.0;
    W[1 + (NY) * 11] = 0.0;
    W[1 + (NY) * 12] = 0.0;
    W[1 + (NY) * 13] = 0.0;
    W[1 + (NY) * 14] = 0.0;
    W[1 + (NY) * 15] = 0.0;
    W[1 + (NY) * 16] = 0.0;
    
    W[2 + (NY) * 0] = 0.0;
    W[2 + (NY) * 1] = 0.0;
    W[2 + (NY) * 2] = 6.0;
    W[2 + (NY) * 3] = 0.0;
    W[2 + (NY) * 4] = 0.0;
    W[2 + (NY) * 5] = 0.0;
    W[2 + (NY) * 6] = 0.0;
    W[2 + (NY) * 7] = 0.0;
    W[2 + (NY) * 8] = 0.0;
    W[2 + (NY) * 9] = 0.0;
    W[2 + (NY) * 10] = 0.0;
    W[2 + (NY) * 11] = 0.0;
    W[2 + (NY) * 12] = 0.0;
    W[2 + (NY) * 13] = 0.0;
    W[2 + (NY) * 14] = 0.0;
    W[2 + (NY) * 15] = 0.0;
    W[2 + (NY) * 16] = 0.0;
    
    W[3 + (NY) * 0] = 0.0;
    W[3 + (NY) * 1] = 0.0;
    W[3 + (NY) * 2] = 0.0;
    W[3 + (NY) * 3] = 0.1;
    W[3 + (NY) * 4] = 0.0;
    W[3 + (NY) * 5] = 0.0;
    W[3 + (NY) * 6] = 0.0;
    W[3 + (NY) * 7] = 0.0;
    W[3 + (NY) * 8] = 0.0;
    W[3 + (NY) * 9] = 0.0;
    W[3 + (NY) * 10] = 0.0;
    W[3 + (NY) * 11] = 0.0;
    W[3 + (NY) * 12] = 0.0;
    W[3 + (NY) * 13] = 0.0;
    W[3 + (NY) * 14] = 0.0;
    W[3 + (NY) * 15] = 0.0;
    W[3 + (NY) * 16] = 0.0;
    
    W[4 + (NY) * 0] = 0.0;
    W[4 + (NY) * 1] = 0.0;
    W[4 + (NY) * 2] = 0.0;
    W[4 + (NY) * 3] = 0.0;
    W[4 + (NY) * 4] = 0.1;
    W[4 + (NY) * 5] = 0.0;
    W[4 + (NY) * 6] = 0.0;
    W[4 + (NY) * 7] = 0.0;
    W[4 + (NY) * 8] = 0.0;
    W[4 + (NY) * 9] = 0.0;
    W[4 + (NY) * 10] = 0.0;
    W[4 + (NY) * 11] = 0.0;
    W[4 + (NY) * 12] = 0.0;
    W[4 + (NY) * 13] = 0.0;
    W[4 + (NY) * 14] = 0.0;
    W[4 + (NY) * 15] = 0.0;
    W[4 + (NY) * 16] = 0.0;
    
    W[5 + (NY) * 0] = 0.0;
    W[5 + (NY) * 1] = 0.0;
    W[5 + (NY) * 2] = 0.0;
    W[5 + (NY) * 3] = 0.0;
    W[5 + (NY) * 4] = 0.0;
    W[5 + (NY) * 5] = 0.1;
    W[5 + (NY) * 6] = 0.0;
    W[5 + (NY) * 7] = 0.0;
    W[5 + (NY) * 8] = 0.0;
    W[5 + (NY) * 9] = 0.0;
    W[5 + (NY) * 10] = 0.0;
    W[5 + (NY) * 11] = 0.0;
    W[5 + (NY) * 12] = 0.0;
    W[5 + (NY) * 13] = 0.0;
    W[5 + (NY) * 14] = 0.0;
    W[5 + (NY) * 15] = 0.0;
    W[5 + (NY) * 16] = 0.0;
    
    W[6 + (NY) * 0] = 0.0;
    W[6 + (NY) * 1] = 0.0;
    W[6 + (NY) * 2] = 0.0;
    W[6 + (NY) * 3] = 0.0;
    W[6 + (NY) * 4] = 0.0;
    W[6 + (NY) * 5] = 0.0;
    W[6 + (NY) * 6] = 0.1;
    W[6 + (NY) * 7] = 0.0;
    W[6 + (NY) * 8] = 0.0;
    W[6 + (NY) * 9] = 0.0;
    W[6 + (NY) * 10] = 0.0;
    W[6 + (NY) * 11] = 0.0;
    W[6 + (NY) * 12] = 0.0;
    W[6 + (NY) * 13] = 0.0;
    W[6 + (NY) * 14] = 0.0;
    W[6 + (NY) * 15] = 0.0;
    W[6 + (NY) * 16] = 0.0;
    
    W[7 + (NY) * 0] = 0.0;
    W[7 + (NY) * 1] = 0.0;
    W[7 + (NY) * 2] = 0.0;
    W[7 + (NY) * 3] = 0.0;
    W[7 + (NY) * 4] = 0.0;
    W[7 + (NY) * 5] = 0.0;
    W[7 + (NY) * 6] = 0.0;
    W[7 + (NY) * 7] = 1.0;
    W[7 + (NY) * 8] = 0.0;
    W[7 + (NY) * 9] = 0.0;
    W[7 + (NY) * 10] = 0.0;
    W[7 + (NY) * 11] = 0.0;
    W[7 + (NY) * 12] = 0.0;
    W[7 + (NY) * 13] = 0.0;
    W[7 + (NY) * 14] = 0.0;
    W[7 + (NY) * 15] = 0.0;
    W[7 + (NY) * 16] = 0.0;
    
    W[8 + (NY) * 0] = 0.0;
    W[8 + (NY) * 1] = 0.0;
    W[8 + (NY) * 2] = 0.0;
    W[8 + (NY) * 3] = 0.0;
    W[8 + (NY) * 4] = 0.0;
    W[8 + (NY) * 5] = 0.0;
    W[8 + (NY) * 6] = 0.0;
    W[8 + (NY) * 7] = 0.0;
    W[8 + (NY) * 8] = 1.0;
    W[8 + (NY) * 9] = 0.0;
    W[8 + (NY) * 10] = 0.0;
    W[8 + (NY) * 11] = 0.0;
    W[8 + (NY) * 12] = 0.0;
    W[8 + (NY) * 13] = 0.0;
    W[8 + (NY) * 14] = 0.0;
    W[8 + (NY) * 15] = 0.0;
    W[8 + (NY) * 16] = 0.0;
    
    W[9 + (NY) * 0] = 0.0;
    W[9 + (NY) * 1] = 0.0;
    W[9 + (NY) * 2] = 0.0;
    W[9 + (NY) * 3] = 0.0;
    W[9 + (NY) * 4] = 0.0;
    W[9 + (NY) * 5] = 0.0;
    W[9 + (NY) * 6] = 0.0;
    W[9 + (NY) * 7] = 0.0;
    W[9 + (NY) * 8] = 0.0;
    W[9 + (NY) * 9] = 2.0;
    W[9 + (NY) * 10] = 0.0;
    W[9 + (NY) * 11] = 0.0;
    W[9 + (NY) * 12] = 0.0;
    W[9 + (NY) * 13] = 0.0;
    W[9 + (NY) * 14] = 0.0;
    W[9 + (NY) * 15] = 0.0;
    W[9 + (NY) * 16] = 0.0;
    
    W[10 + (NY) * 0] = 0.0;
    W[10 + (NY) * 1] = 0.0;
    W[10 + (NY) * 2] = 0.0;
    W[10 + (NY) * 3] = 0.0;
    W[10 + (NY) * 4] = 0.0;
    W[10 + (NY) * 5] = 0.0;
    W[10 + (NY) * 6] = 0.0;
    W[10 + (NY) * 7] = 0.0;
    W[10 + (NY) * 8] = 0.0;
    W[10 + (NY) * 9] = 0.0;
    W[10 + (NY) * 10] = 1.5;
    W[10 + (NY) * 11] = 0.0;
    W[10 + (NY) * 12] = 0.0;
    W[10 + (NY) * 13] = 0.0;
    W[10 + (NY) * 14] = 0.0;
    W[10 + (NY) * 15] = 0.0;
    W[10 + (NY) * 16] = 0.0;
    
    W[11 + (NY) * 0] = 0.0;
    W[11 + (NY) * 1] = 0.0;
    W[11 + (NY) * 2] = 0.0;
    W[11 + (NY) * 3] = 0.0;
    W[11 + (NY) * 4] = 0.0;
    W[11 + (NY) * 5] = 0.0;
    W[11 + (NY) * 6] = 0.0;
    W[11 + (NY) * 7] = 0.0;
    W[11 + (NY) * 8] = 0.0;
    W[11 + (NY) * 9] = 0.0;
    W[11 + (NY) * 10] = 0.0;
    W[11 + (NY) * 11] = 1.5;
    W[11 + (NY) * 12] = 0.0;
    W[11 + (NY) * 13] = 0.0;
    W[11 + (NY) * 14] = 0.0;
    W[11 + (NY) * 15] = 0.0;
    W[11 + (NY) * 16] = 0.0;
    
    W[12 + (NY) * 0] = 0.0;
    W[12 + (NY) * 1] = 0.0;
    W[12 + (NY) * 2] = 0.0;
    W[12 + (NY) * 3] = 0.0;
    W[12 + (NY) * 4] = 0.0;
    W[12 + (NY) * 5] = 0.0;
    W[12 + (NY) * 6] = 0.0;
    W[12 + (NY) * 7] = 0.0;
    W[12 + (NY) * 8] = 0.0;
    W[12 + (NY) * 9] = 0.0;
    W[12 + (NY) * 10] = 0.0;
    W[12 + (NY) * 11] = 0.0;
    W[12 + (NY) * 12] = 1.5;
    W[12 + (NY) * 13] = 0.0;
    W[12 + (NY) * 14] = 0.0;
    W[12 + (NY) * 15] = 0.0;
    W[12 + (NY) * 16] = 0.0;
    
    W[13 + (NY) * 0] = 0.0;
    W[13 + (NY) * 1] = 0.0;
    W[13 + (NY) * 2] = 0.0;
    W[13 + (NY) * 3] = 0.0;
    W[13 + (NY) * 4] = 0.0;
    W[13 + (NY) * 5] = 0.0;
    W[13 + (NY) * 6] = 0.0;
    W[13 + (NY) * 7] = 0.0;
    W[13 + (NY) * 8] = 0.0;
    W[13 + (NY) * 9] = 0.0;
    W[13 + (NY) * 10] = 0.0;
    W[13 + (NY) * 11] = 0.0;
    W[13 + (NY) * 12] = 0.0;
    W[13 + (NY) * 13] = 0.01;
    W[13 + (NY) * 14] = 0.0;
    W[13 + (NY) * 15] = 0.0;
    W[13 + (NY) * 16] = 0.0;
    
    W[14 + (NY) * 0] = 0.0;
    W[14 + (NY) * 1] = 0.0;
    W[14 + (NY) * 2] = 0.0;
    W[14 + (NY) * 3] = 0.0;
    W[14 + (NY) * 4] = 0.0;
    W[14 + (NY) * 5] = 0.0;
    W[14 + (NY) * 6] = 0.0;
    W[14 + (NY) * 7] = 0.0;
    W[14 + (NY) * 8] = 0.0;
    W[14 + (NY) * 9] = 0.0;
    W[14 + (NY) * 10] = 0.0;
    W[14 + (NY) * 11] = 0.0;
    W[14 + (NY) * 12] = 0.0;
    W[14 + (NY) * 13] = 0.0;
    W[14 + (NY) * 14] = 0.01;
    W[14 + (NY) * 15] = 0.0;
    W[14 + (NY) * 16] = 0.0;
    
    W[15 + (NY) * 0] = 0.0;
    W[15 + (NY) * 1] = 0.0;
    W[15 + (NY) * 2] = 0.0;
    W[15 + (NY) * 3] = 0.0;
    W[15 + (NY) * 4] = 0.0;
    W[15 + (NY) * 5] = 0.0;
    W[15 + (NY) * 6] = 0.0;
    W[15 + (NY) * 7] = 0.0;
    W[15 + (NY) * 8] = 0.0;
    W[15 + (NY) * 9] = 0.0;
    W[15 + (NY) * 10] = 0.0;
    W[15 + (NY) * 11] = 0.0;
    W[15 + (NY) * 12] = 0.0;
    W[15 + (NY) * 13] = 0.0;
    W[15 + (NY) * 14] = 0.0;
    W[15 + (NY) * 15] = 0.01;
    W[15 + (NY) * 16] = 0.0;
    
    W[16 + (NY) * 0] = 0.0;
    W[16 + (NY) * 1] = 0.0;
    W[16 + (NY) * 2] = 0.0;
    W[16 + (NY) * 3] = 0.0;
    W[16 + (NY) * 4] = 0.0;
    W[16 + (NY) * 5] = 0.0;
    W[16 + (NY) * 6] = 0.0;
    W[16 + (NY) * 7] = 0.0;
    W[16 + (NY) * 8] = 0.0;
    W[16 + (NY) * 9] = 0.0;
    W[16 + (NY) * 10] = 0.0;
    W[16 + (NY) * 11] = 0.0;
    W[16 + (NY) * 12] = 0.0;
    W[16 + (NY) * 13] = 0.0;
    W[16 + (NY) * 14] = 0.0;
    W[16 + (NY) * 15] = 0.0;
    W[16 + (NY) * 16] = 0.01;
    
    Vx[0 + (NY) * 0] = 1.0;
    Vx[0 + (NY) * 1] = 0.0;
    Vx[0 + (NY) * 2] = 0.0;
    Vx[0 + (NY) * 3] = 0.0;
    Vx[0 + (NY) * 4] = 0.0;
    Vx[0 + (NY) * 5] = 0.0;
    Vx[0 + (NY) * 6] = 0.0;
    Vx[0 + (NY) * 7] = 0.0;
    Vx[0 + (NY) * 8] = 0.0;
    Vx[0 + (NY) * 9] = 0.0;
    Vx[0 + (NY) * 10] = 0.0;
    Vx[0 + (NY) * 11] = 0.0;
    Vx[0 + (NY) * 12] = 0.0;
    
    Vx[1 + (NY) * 0] = 0.0;
    Vx[1 + (NY) * 1] = 1.0;
    Vx[1 + (NY) * 2] = 0.0;
    Vx[1 + (NY) * 3] = 0.0;
    Vx[1 + (NY) * 4] = 0.0;
    Vx[1 + (NY) * 5] = 0.0;
    Vx[1 + (NY) * 6] = 0.0;
    Vx[1 + (NY) * 7] = 0.0;
    Vx[1 + (NY) * 8] = 0.0;
    Vx[1 + (NY) * 9] = 0.0;
    Vx[1 + (NY) * 10] = 0.0;
    Vx[1 + (NY) * 11] = 0.0;
    Vx[1 + (NY) * 12] = 0.0;
    
    Vx[2 + (NY) * 0] = 0.0;
    Vx[2 + (NY) * 1] = 0.0;
    Vx[2 + (NY) * 2] = 1.0;
    Vx[2 + (NY) * 3] = 0.0;
    Vx[2 + (NY) * 4] = 0.0;
    Vx[2 + (NY) * 5] = 0.0;
    Vx[2 + (NY) * 6] = 0.0;
    Vx[2 + (NY) * 7] = 0.0;
    Vx[2 + (NY) * 8] = 0.0;
    Vx[2 + (NY) * 9] = 0.0;
    Vx[2 + (NY) * 10] = 0.0;
    Vx[2 + (NY) * 11] = 0.0;
    Vx[2 + (NY) * 12] = 0.0;
    
    Vx[3 + (NY) * 0] = 0.0;
    Vx[3 + (NY) * 1] = 0.0;
    Vx[3 + (NY) * 2] = 0.0;
    Vx[3 + (NY) * 3] = 1.0;
    Vx[3 + (NY) * 4] = 0.0;
    Vx[3 + (NY) * 5] = 0.0;
    Vx[3 + (NY) * 6] = 0.0;
    Vx[3 + (NY) * 7] = 0.0;
    Vx[3 + (NY) * 8] = 0.0;
    Vx[3 + (NY) * 9] = 0.0;
    Vx[3 + (NY) * 10] = 0.0;
    Vx[3 + (NY) * 11] = 0.0;
    Vx[3 + (NY) * 12] = 0.0;
    
    Vx[4 + (NY) * 0] = 0.0;
    Vx[4 + (NY) * 1] = 0.0;
    Vx[4 + (NY) * 2] = 0.0;
    Vx[4 + (NY) * 3] = 0.0;
    Vx[4 + (NY) * 4] = 1.0;
    Vx[4 + (NY) * 5] = 0.0;
    Vx[4 + (NY) * 6] = 0.0;
    Vx[4 + (NY) * 7] = 0.0;
    Vx[4 + (NY) * 8] = 0.0;
    Vx[4 + (NY) * 9] = 0.0;
    Vx[4 + (NY) * 10] = 0.0;
    Vx[4 + (NY) * 11] = 0.0;
    Vx[4 + (NY) * 12] = 0.0;
    
    Vx[5 + (NY) * 0] = 0.0;
    Vx[5 + (NY) * 1] = 0.0;
    Vx[5 + (NY) * 2] = 0.0;
    Vx[5 + (NY) * 3] = 0.0;
    Vx[5 + (NY) * 4] = 0.0;
    Vx[5 + (NY) * 5] = 1.0;
    Vx[5 + (NY) * 6] = 0.0;
    Vx[5 + (NY) * 7] = 0.0;
    Vx[5 + (NY) * 8] = 0.0;
    Vx[5 + (NY) * 9] = 0.0;
    Vx[5 + (NY) * 10] = 0.0;
    Vx[5 + (NY) * 11] = 0.0;
    Vx[5 + (NY) * 12] = 0.0;
    
    Vx[6 + (NY) * 0] = 0.0;
    Vx[6 + (NY) * 1] = 0.0;
    Vx[6 + (NY) * 2] = 0.0;
    Vx[6 + (NY) * 3] = 0.0;
    Vx[6 + (NY) * 4] = 0.0;
    Vx[6 + (NY) * 5] = 0.0;
    Vx[6 + (NY) * 6] = 1.0;
    Vx[6 + (NY) * 7] = 0.0;
    Vx[6 + (NY) * 8] = 0.0;
    Vx[6 + (NY) * 9] = 0.0;
    Vx[6 + (NY) * 10] = 0.0;
    Vx[6 + (NY) * 11] = 0.0;
    Vx[6 + (NY) * 12] = 0.0;
    
    Vx[7 + (NY) * 0] = 0.0;
    Vx[7 + (NY) * 1] = 0.0;
    Vx[7 + (NY) * 2] = 0.0;
    Vx[7 + (NY) * 3] = 0.0;
    Vx[7 + (NY) * 4] = 0.0;
    Vx[7 + (NY) * 5] = 0.0;
    Vx[7 + (NY) * 6] = 0.0;
    Vx[7 + (NY) * 7] = 1.0;
    Vx[7 + (NY) * 8] = 0.0;
    Vx[7 + (NY) * 9] = 0.0;
    Vx[7 + (NY) * 10] = 0.0;
    Vx[7 + (NY) * 11] = 0.0;
    Vx[7 + (NY) * 12] = 0.0;
    
    Vx[8 + (NY) * 0] = 0.0;
    Vx[8 + (NY) * 1] = 0.0;
    Vx[8 + (NY) * 2] = 0.0;
    Vx[8 + (NY) * 3] = 0.0;
    Vx[8 + (NY) * 4] = 0.0;
    Vx[8 + (NY) * 5] = 0.0;
    Vx[8 + (NY) * 6] = 0.0;
    Vx[8 + (NY) * 7] = 0.0;
    Vx[8 + (NY) * 8] = 1.0;
    Vx[8 + (NY) * 9] = 0.0;
    Vx[8 + (NY) * 10] = 0.0;
    Vx[8 + (NY) * 11] = 0.0;
    Vx[8 + (NY) * 12] = 0.0;
    
    Vx[9 + (NY) * 0] = 0.0;
    Vx[9 + (NY) * 1] = 0.0;
    Vx[9 + (NY) * 2] = 0.0;
    Vx[9 + (NY) * 3] = 0.0;
    Vx[9 + (NY) * 4] = 0.0;
    Vx[9 + (NY) * 5] = 0.0;
    Vx[9 + (NY) * 6] = 0.0;
    Vx[9 + (NY) * 7] = 0.0;
    Vx[9 + (NY) * 8] = 0.0;
    Vx[9 + (NY) * 9] = 1.0;
    Vx[9 + (NY) * 10] = 0.0;
    Vx[9 + (NY) * 11] = 0.0;
    Vx[9 + (NY) * 12] = 0.0;
    
    Vx[10 + (NY) * 0] = 0.0;
    Vx[10 + (NY) * 1] = 0.0;
    Vx[10 + (NY) * 2] = 0.0;
    Vx[10 + (NY) * 3] = 0.0;
    Vx[10 + (NY) * 4] = 0.0;
    Vx[10 + (NY) * 5] = 0.0;
    Vx[10 + (NY) * 6] = 0.0;
    Vx[10 + (NY) * 7] = 0.0;
    Vx[10 + (NY) * 8] = 0.0;
    Vx[10 + (NY) * 9] = 0.0;
    Vx[10 + (NY) * 10] = 1.0;
    Vx[10 + (NY) * 11] = 0.0;
    Vx[10 + (NY) * 12] = 0.0;
    
    Vx[11 + (NY) * 0] = 0.0;
    Vx[11 + (NY) * 1] = 0.0;
    Vx[11 + (NY) * 2] = 0.0;
    Vx[11 + (NY) * 3] = 0.0;
    Vx[11 + (NY) * 4] = 0.0;
    Vx[11 + (NY) * 5] = 0.0;
    Vx[11 + (NY) * 6] = 0.0;
    Vx[11 + (NY) * 7] = 0.0;
    Vx[11 + (NY) * 8] = 0.0;
    Vx[11 + (NY) * 9] = 0.0;
    Vx[11 + (NY) * 10] = 0.0;
    Vx[11 + (NY) * 11] = 1.0;
    Vx[11 + (NY) * 12] = 0.0;
    
    Vx[12 + (NY) * 0] = 0.0;
    Vx[12 + (NY) * 1] = 0.0;
    Vx[12 + (NY) * 2] = 0.0;
    Vx[12 + (NY) * 3] = 0.0;
    Vx[12 + (NY) * 4] = 0.0;
    Vx[12 + (NY) * 5] = 0.0;
    Vx[12 + (NY) * 6] = 0.0;
    Vx[12 + (NY) * 7] = 0.0;
    Vx[12 + (NY) * 8] = 0.0;
    Vx[12 + (NY) * 9] = 0.0;
    Vx[12 + (NY) * 10] = 0.0;
    Vx[12 + (NY) * 11] = 0.0;
    Vx[12 + (NY) * 12] = 1.0;
    
    Vx[13 + (NY) * 0] = 0.0;
    Vx[13 + (NY) * 1] = 0.0;
    Vx[13 + (NY) * 2] = 0.0;
    Vx[13 + (NY) * 3] = 0.0;
    Vx[13 + (NY) * 4] = 0.0;
    Vx[13 + (NY) * 5] = 0.0;
    Vx[13 + (NY) * 6] = 0.0;
    Vx[13 + (NY) * 7] = 0.0;
    Vx[13 + (NY) * 8] = 0.0;
    Vx[13 + (NY) * 9] = 0.0;
    Vx[13 + (NY) * 10] = 0.0;
    Vx[13 + (NY) * 11] = 0.0;
    Vx[13 + (NY) * 12] = 0.0;
    
    Vx[14 + (NY) * 0] = 0.0;
    Vx[14 + (NY) * 1] = 0.0;
    Vx[14 + (NY) * 2] = 0.0;
    Vx[14 + (NY) * 3] = 0.0;
    Vx[14 + (NY) * 4] = 0.0;
    Vx[14 + (NY) * 5] = 0.0;
    Vx[14 + (NY) * 6] = 0.0;
    Vx[14 + (NY) * 7] = 0.0;
    Vx[14 + (NY) * 8] = 0.0;
    Vx[14 + (NY) * 9] = 0.0;
    Vx[14 + (NY) * 10] = 0.0;
    Vx[14 + (NY) * 11] = 0.0;
    Vx[14 + (NY) * 12] = 0.0;
    
    Vx[15 + (NY) * 0] = 0.0;
    Vx[15 + (NY) * 1] = 0.0;
    Vx[15 + (NY) * 2] = 0.0;
    Vx[15 + (NY) * 3] = 0.0;
    Vx[15 + (NY) * 4] = 0.0;
    Vx[15 + (NY) * 5] = 0.0;
    Vx[15 + (NY) * 6] = 0.0;
    Vx[15 + (NY) * 7] = 0.0;
    Vx[15 + (NY) * 8] = 0.0;
    Vx[15 + (NY) * 9] = 0.0;
    Vx[15 + (NY) * 10] = 0.0;
    Vx[15 + (NY) * 11] = 0.0;
    Vx[15 + (NY) * 12] = 0.0;
    
    Vx[16 + (NY) * 0] = 0.0;
    Vx[16 + (NY) * 1] = 0.0;
    Vx[16 + (NY) * 2] = 0.0;
    Vx[16 + (NY) * 3] = 0.0;
    Vx[16 + (NY) * 4] = 0.0;
    Vx[16 + (NY) * 5] = 0.0;
    Vx[16 + (NY) * 6] = 0.0;
    Vx[16 + (NY) * 7] = 0.0;
    Vx[16 + (NY) * 8] = 0.0;
    Vx[16 + (NY) * 9] = 0.0;
    Vx[16 + (NY) * 10] = 0.0;
    Vx[16 + (NY) * 11] = 0.0;
    Vx[16 + (NY) * 12] = 0.0;
    
    Vu[0 + (NY) * 0] = 0.0;
    Vu[0 + (NY) * 1] = 0.0;
    Vu[0 + (NY) * 2] = 0.0;
    Vu[0 + (NY) * 3] = 0.0;
    
    Vu[1 + (NY) * 0] = 0.0;
    Vu[1 + (NY) * 1] = 0.0;
    Vu[1 + (NY) * 2] = 0.0;
    Vu[1 + (NY) * 3] = 0.0;
    
    Vu[2 + (NY) * 0] = 0.0;
    Vu[2 + (NY) * 1] = 0.0;
    Vu[2 + (NY) * 2] = 0.0;
    Vu[2 + (NY) * 3] = 0.0;
    
    Vu[3 + (NY) * 0] = 0.0;
    Vu[3 + (NY) * 1] = 0.0;
    Vu[3 + (NY) * 2] = 0.0;
    Vu[3 + (NY) * 3] = 0.0;
    
    Vu[4 + (NY) * 0] = 0.0;
    Vu[4 + (NY) * 1] = 0.0;
    Vu[4 + (NY) * 2] = 0.0;
    Vu[4 + (NY) * 3] = 0.0;
    
    Vu[5 + (NY) * 0] = 0.0;
    Vu[5 + (NY) * 1] = 0.0;
    Vu[5 + (NY) * 2] = 0.0;
    Vu[5 + (NY) * 3] = 0.0;
    
    Vu[6 + (NY) * 0] = 0.0;
    Vu[6 + (NY) * 1] = 0.0;
    Vu[6 + (NY) * 2] = 0.0;
    Vu[6 + (NY) * 3] = 0.0;
    
    Vu[7 + (NY) * 0] = 0.0;
    Vu[7 + (NY) * 1] = 0.0;
    Vu[7 + (NY) * 2] = 0.0;
    Vu[7 + (NY) * 3] = 0.0;
    
    Vu[8 + (NY) * 0] = 0.0;
    Vu[8 + (NY) * 1] = 0.0;
    Vu[8 + (NY) * 2] = 0.0;
    Vu[8 + (NY) * 3] = 0.0;
    
    Vu[9 + (NY) * 0] = 0.0;
    Vu[9 + (NY) * 1] = 0.0;
    Vu[9 + (NY) * 2] = 0.0;
    Vu[9 + (NY) * 3] = 0.0;
    
    Vu[10 + (NY) * 0] = 0.0;
    Vu[10 + (NY) * 1] = 0.0;
    Vu[10 + (NY) * 2] = 0.0;
    Vu[10 + (NY) * 3] = 0.0;
    
    Vu[11 + (NY) * 0] = 0.0;
    Vu[11 + (NY) * 1] = 0.0;
    Vu[11 + (NY) * 2] = 0.0;
    Vu[11 + (NY) * 3] = 0.0;
    
    Vu[12 + (NY) * 0] = 0.0;
    Vu[12 + (NY) * 1] = 0.0;
    Vu[12 + (NY) * 2] = 0.0;
    Vu[12 + (NY) * 3] = 0.0;
    
    Vu[13 + (NY) * 0] = 0.0001;
    Vu[13 + (NY) * 1] = 0.0;
    Vu[13 + (NY) * 2] = 0.0;
    Vu[13 + (NY) * 3] = 0.0;
    
    Vu[14 + (NY) * 0] = 0.0;
    Vu[14 + (NY) * 1] = 0.0001;
    Vu[14 + (NY) * 2] = 0.0;
    Vu[14 + (NY) * 3] = 0.0;
    
    Vu[15 + (NY) * 0] = 0.0;
    Vu[15 + (NY) * 1] = 0.0;
    Vu[15 + (NY) * 2] = 0.0001;
    Vu[15 + (NY) * 3] = 0.0;
    
    Vu[16 + (NY) * 0] = 0.0;
    Vu[16 + (NY) * 1] = 0.0;
    Vu[16 + (NY) * 2] = 0.0;
    Vu[16 + (NY) * 3] = 0.0001;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    yref[0] = 0.0;
    yref[1] = 0.0;
    yref[2] = 0.1;
    yref[3] = 1.0;
    yref[4] = 0.0;
    yref[5] = 0.0;
    yref[6] = 0.0;
    yref[7] = 0.0;
    yref[8] = 0.0;
    yref[9] = 0.0;
    yref[10] = 0.0;
    yref[11] = 0.0;
    yref[12] = 0.0;
    yref[13] = 0.0;
    yref[14] = 0.0;
    yref[15] = 0.0;
    yref[16] = 0.0;
    
    W_e[0 + (NYN) * 0] = 4.0;
    W_e[0 + (NYN) * 1] = 0.0;
    W_e[0 + (NYN) * 2] = 0.0;
    W_e[0 + (NYN) * 3] = 0.0;
    W_e[0 + (NYN) * 4] = 0.0;
    W_e[0 + (NYN) * 5] = 0.0;
    W_e[0 + (NYN) * 6] = 0.0;
    W_e[0 + (NYN) * 7] = 0.0;
    W_e[0 + (NYN) * 8] = 0.0;
    W_e[0 + (NYN) * 9] = 0.0;
    W_e[0 + (NYN) * 10] = 0.0;
    W_e[0 + (NYN) * 11] = 0.0;
    W_e[0 + (NYN) * 12] = 0.0;
    
    W_e[1 + (NYN) * 0] = 0.0;
    W_e[1 + (NYN) * 1] = 4.0;
    W_e[1 + (NYN) * 2] = 0.0;
    W_e[1 + (NYN) * 3] = 0.0;
    W_e[1 + (NYN) * 4] = 0.0;
    W_e[1 + (NYN) * 5] = 0.0;
    W_e[1 + (NYN) * 6] = 0.0;
    W_e[1 + (NYN) * 7] = 0.0;
    W_e[1 + (NYN) * 8] = 0.0;
    W_e[1 + (NYN) * 9] = 0.0;
    W_e[1 + (NYN) * 10] = 0.0;
    W_e[1 + (NYN) * 11] = 0.0;
    W_e[1 + (NYN) * 12] = 0.0;
    
    W_e[2 + (NYN) * 0] = 0.0;
    W_e[2 + (NYN) * 1] = 0.0;
    W_e[2 + (NYN) * 2] = 6.0;
    W_e[2 + (NYN) * 3] = 0.0;
    W_e[2 + (NYN) * 4] = 0.0;
    W_e[2 + (NYN) * 5] = 0.0;
    W_e[2 + (NYN) * 6] = 0.0;
    W_e[2 + (NYN) * 7] = 0.0;
    W_e[2 + (NYN) * 8] = 0.0;
    W_e[2 + (NYN) * 9] = 0.0;
    W_e[2 + (NYN) * 10] = 0.0;
    W_e[2 + (NYN) * 11] = 0.0;
    W_e[2 + (NYN) * 12] = 0.0;
    
    W_e[3 + (NYN) * 0] = 0.0;
    W_e[3 + (NYN) * 1] = 0.0;
    W_e[3 + (NYN) * 2] = 0.0;
    W_e[3 + (NYN) * 3] = 0.1;
    W_e[3 + (NYN) * 4] = 0.0;
    W_e[3 + (NYN) * 5] = 0.0;
    W_e[3 + (NYN) * 6] = 0.0;
    W_e[3 + (NYN) * 7] = 0.0;
    W_e[3 + (NYN) * 8] = 0.0;
    W_e[3 + (NYN) * 9] = 0.0;
    W_e[3 + (NYN) * 10] = 0.0;
    W_e[3 + (NYN) * 11] = 0.0;
    W_e[3 + (NYN) * 12] = 0.0;
    
    W_e[4 + (NYN) * 0] = 0.0;
    W_e[4 + (NYN) * 1] = 0.0;
    W_e[4 + (NYN) * 2] = 0.0;
    W_e[4 + (NYN) * 3] = 0.0;
    W_e[4 + (NYN) * 4] = 0.1;
    W_e[4 + (NYN) * 5] = 0.0;
    W_e[4 + (NYN) * 6] = 0.0;
    W_e[4 + (NYN) * 7] = 0.0;
    W_e[4 + (NYN) * 8] = 0.0;
    W_e[4 + (NYN) * 9] = 0.0;
    W_e[4 + (NYN) * 10] = 0.0;
    W_e[4 + (NYN) * 11] = 0.0;
    W_e[4 + (NYN) * 12] = 0.0;
    
    W_e[5 + (NYN) * 0] = 0.0;
    W_e[5 + (NYN) * 1] = 0.0;
    W_e[5 + (NYN) * 2] = 0.0;
    W_e[5 + (NYN) * 3] = 0.0;
    W_e[5 + (NYN) * 4] = 0.0;
    W_e[5 + (NYN) * 5] = 0.1;
    W_e[5 + (NYN) * 6] = 0.0;
    W_e[5 + (NYN) * 7] = 0.0;
    W_e[5 + (NYN) * 8] = 0.0;
    W_e[5 + (NYN) * 9] = 0.0;
    W_e[5 + (NYN) * 10] = 0.0;
    W_e[5 + (NYN) * 11] = 0.0;
    W_e[5 + (NYN) * 12] = 0.0;
    
    W_e[6 + (NYN) * 0] = 0.0;
    W_e[6 + (NYN) * 1] = 0.0;
    W_e[6 + (NYN) * 2] = 0.0;
    W_e[6 + (NYN) * 3] = 0.0;
    W_e[6 + (NYN) * 4] = 0.0;
    W_e[6 + (NYN) * 5] = 0.0;
    W_e[6 + (NYN) * 6] = 0.1;
    W_e[6 + (NYN) * 7] = 0.0;
    W_e[6 + (NYN) * 8] = 0.0;
    W_e[6 + (NYN) * 9] = 0.0;
    W_e[6 + (NYN) * 10] = 0.0;
    W_e[6 + (NYN) * 11] = 0.0;
    W_e[6 + (NYN) * 12] = 0.0;
    
    W_e[7 + (NYN) * 0] = 0.0;
    W_e[7 + (NYN) * 1] = 0.0;
    W_e[7 + (NYN) * 2] = 0.0;
    W_e[7 + (NYN) * 3] = 0.0;
    W_e[7 + (NYN) * 4] = 0.0;
    W_e[7 + (NYN) * 5] = 0.0;
    W_e[7 + (NYN) * 6] = 0.0;
    W_e[7 + (NYN) * 7] = 1.0;
    W_e[7 + (NYN) * 8] = 0.0;
    W_e[7 + (NYN) * 9] = 0.0;
    W_e[7 + (NYN) * 10] = 0.0;
    W_e[7 + (NYN) * 11] = 0.0;
    W_e[7 + (NYN) * 12] = 0.0;
    
    W_e[8 + (NYN) * 0] = 0.0;
    W_e[8 + (NYN) * 1] = 0.0;
    W_e[8 + (NYN) * 2] = 0.0;
    W_e[8 + (NYN) * 3] = 0.0;
    W_e[8 + (NYN) * 4] = 0.0;
    W_e[8 + (NYN) * 5] = 0.0;
    W_e[8 + (NYN) * 6] = 0.0;
    W_e[8 + (NYN) * 7] = 0.0;
    W_e[8 + (NYN) * 8] = 1.0;
    W_e[8 + (NYN) * 9] = 0.0;
    W_e[8 + (NYN) * 10] = 0.0;
    W_e[8 + (NYN) * 11] = 0.0;
    W_e[8 + (NYN) * 12] = 0.0;
    
    W_e[9 + (NYN) * 0] = 0.0;
    W_e[9 + (NYN) * 1] = 0.0;
    W_e[9 + (NYN) * 2] = 0.0;
    W_e[9 + (NYN) * 3] = 0.0;
    W_e[9 + (NYN) * 4] = 0.0;
    W_e[9 + (NYN) * 5] = 0.0;
    W_e[9 + (NYN) * 6] = 0.0;
    W_e[9 + (NYN) * 7] = 0.0;
    W_e[9 + (NYN) * 8] = 0.0;
    W_e[9 + (NYN) * 9] = 2.0;
    W_e[9 + (NYN) * 10] = 0.0;
    W_e[9 + (NYN) * 11] = 0.0;
    W_e[9 + (NYN) * 12] = 0.0;
    
    W_e[10 + (NYN) * 0] = 0.0;
    W_e[10 + (NYN) * 1] = 0.0;
    W_e[10 + (NYN) * 2] = 0.0;
    W_e[10 + (NYN) * 3] = 0.0;
    W_e[10 + (NYN) * 4] = 0.0;
    W_e[10 + (NYN) * 5] = 0.0;
    W_e[10 + (NYN) * 6] = 0.0;
    W_e[10 + (NYN) * 7] = 0.0;
    W_e[10 + (NYN) * 8] = 0.0;
    W_e[10 + (NYN) * 9] = 0.0;
    W_e[10 + (NYN) * 10] = 1.5;
    W_e[10 + (NYN) * 11] = 0.0;
    W_e[10 + (NYN) * 12] = 0.0;
    
    W_e[11 + (NYN) * 0] = 0.0;
    W_e[11 + (NYN) * 1] = 0.0;
    W_e[11 + (NYN) * 2] = 0.0;
    W_e[11 + (NYN) * 3] = 0.0;
    W_e[11 + (NYN) * 4] = 0.0;
    W_e[11 + (NYN) * 5] = 0.0;
    W_e[11 + (NYN) * 6] = 0.0;
    W_e[11 + (NYN) * 7] = 0.0;
    W_e[11 + (NYN) * 8] = 0.0;
    W_e[11 + (NYN) * 9] = 0.0;
    W_e[11 + (NYN) * 10] = 0.0;
    W_e[11 + (NYN) * 11] = 1.5;
    W_e[11 + (NYN) * 12] = 0.0;
    
    W_e[12 + (NYN) * 0] = 0.0;
    W_e[12 + (NYN) * 1] = 0.0;
    W_e[12 + (NYN) * 2] = 0.0;
    W_e[12 + (NYN) * 3] = 0.0;
    W_e[12 + (NYN) * 4] = 0.0;
    W_e[12 + (NYN) * 5] = 0.0;
    W_e[12 + (NYN) * 6] = 0.0;
    W_e[12 + (NYN) * 7] = 0.0;
    W_e[12 + (NYN) * 8] = 0.0;
    W_e[12 + (NYN) * 9] = 0.0;
    W_e[12 + (NYN) * 10] = 0.0;
    W_e[12 + (NYN) * 11] = 0.0;
    W_e[12 + (NYN) * 12] = 1.5;
    
    Vx_e[0 + (NYN) * 0] = 1.0;
    Vx_e[0 + (NYN) * 1] = 0.0;
    Vx_e[0 + (NYN) * 2] = 0.0;
    Vx_e[0 + (NYN) * 3] = 0.0;
    Vx_e[0 + (NYN) * 4] = 0.0;
    Vx_e[0 + (NYN) * 5] = 0.0;
    Vx_e[0 + (NYN) * 6] = 0.0;
    Vx_e[0 + (NYN) * 7] = 0.0;
    Vx_e[0 + (NYN) * 8] = 0.0;
    Vx_e[0 + (NYN) * 9] = 0.0;
    Vx_e[0 + (NYN) * 10] = 0.0;
    Vx_e[0 + (NYN) * 11] = 0.0;
    Vx_e[0 + (NYN) * 12] = 0.0;
    
    Vx_e[1 + (NYN) * 0] = 0.0;
    Vx_e[1 + (NYN) * 1] = 1.0;
    Vx_e[1 + (NYN) * 2] = 0.0;
    Vx_e[1 + (NYN) * 3] = 0.0;
    Vx_e[1 + (NYN) * 4] = 0.0;
    Vx_e[1 + (NYN) * 5] = 0.0;
    Vx_e[1 + (NYN) * 6] = 0.0;
    Vx_e[1 + (NYN) * 7] = 0.0;
    Vx_e[1 + (NYN) * 8] = 0.0;
    Vx_e[1 + (NYN) * 9] = 0.0;
    Vx_e[1 + (NYN) * 10] = 0.0;
    Vx_e[1 + (NYN) * 11] = 0.0;
    Vx_e[1 + (NYN) * 12] = 0.0;
    
    Vx_e[2 + (NYN) * 0] = 0.0;
    Vx_e[2 + (NYN) * 1] = 0.0;
    Vx_e[2 + (NYN) * 2] = 1.0;
    Vx_e[2 + (NYN) * 3] = 0.0;
    Vx_e[2 + (NYN) * 4] = 0.0;
    Vx_e[2 + (NYN) * 5] = 0.0;
    Vx_e[2 + (NYN) * 6] = 0.0;
    Vx_e[2 + (NYN) * 7] = 0.0;
    Vx_e[2 + (NYN) * 8] = 0.0;
    Vx_e[2 + (NYN) * 9] = 0.0;
    Vx_e[2 + (NYN) * 10] = 0.0;
    Vx_e[2 + (NYN) * 11] = 0.0;
    Vx_e[2 + (NYN) * 12] = 0.0;
    
    Vx_e[3 + (NYN) * 0] = 0.0;
    Vx_e[3 + (NYN) * 1] = 0.0;
    Vx_e[3 + (NYN) * 2] = 0.0;
    Vx_e[3 + (NYN) * 3] = 1.0;
    Vx_e[3 + (NYN) * 4] = 0.0;
    Vx_e[3 + (NYN) * 5] = 0.0;
    Vx_e[3 + (NYN) * 6] = 0.0;
    Vx_e[3 + (NYN) * 7] = 0.0;
    Vx_e[3 + (NYN) * 8] = 0.0;
    Vx_e[3 + (NYN) * 9] = 0.0;
    Vx_e[3 + (NYN) * 10] = 0.0;
    Vx_e[3 + (NYN) * 11] = 0.0;
    Vx_e[3 + (NYN) * 12] = 0.0;
    
    Vx_e[4 + (NYN) * 0] = 0.0;
    Vx_e[4 + (NYN) * 1] = 0.0;
    Vx_e[4 + (NYN) * 2] = 0.0;
    Vx_e[4 + (NYN) * 3] = 0.0;
    Vx_e[4 + (NYN) * 4] = 1.0;
    Vx_e[4 + (NYN) * 5] = 0.0;
    Vx_e[4 + (NYN) * 6] = 0.0;
    Vx_e[4 + (NYN) * 7] = 0.0;
    Vx_e[4 + (NYN) * 8] = 0.0;
    Vx_e[4 + (NYN) * 9] = 0.0;
    Vx_e[4 + (NYN) * 10] = 0.0;
    Vx_e[4 + (NYN) * 11] = 0.0;
    Vx_e[4 + (NYN) * 12] = 0.0;
    
    Vx_e[5 + (NYN) * 0] = 0.0;
    Vx_e[5 + (NYN) * 1] = 0.0;
    Vx_e[5 + (NYN) * 2] = 0.0;
    Vx_e[5 + (NYN) * 3] = 0.0;
    Vx_e[5 + (NYN) * 4] = 0.0;
    Vx_e[5 + (NYN) * 5] = 1.0;
    Vx_e[5 + (NYN) * 6] = 0.0;
    Vx_e[5 + (NYN) * 7] = 0.0;
    Vx_e[5 + (NYN) * 8] = 0.0;
    Vx_e[5 + (NYN) * 9] = 0.0;
    Vx_e[5 + (NYN) * 10] = 0.0;
    Vx_e[5 + (NYN) * 11] = 0.0;
    Vx_e[5 + (NYN) * 12] = 0.0;
    
    Vx_e[6 + (NYN) * 0] = 0.0;
    Vx_e[6 + (NYN) * 1] = 0.0;
    Vx_e[6 + (NYN) * 2] = 0.0;
    Vx_e[6 + (NYN) * 3] = 0.0;
    Vx_e[6 + (NYN) * 4] = 0.0;
    Vx_e[6 + (NYN) * 5] = 0.0;
    Vx_e[6 + (NYN) * 6] = 1.0;
    Vx_e[6 + (NYN) * 7] = 0.0;
    Vx_e[6 + (NYN) * 8] = 0.0;
    Vx_e[6 + (NYN) * 9] = 0.0;
    Vx_e[6 + (NYN) * 10] = 0.0;
    Vx_e[6 + (NYN) * 11] = 0.0;
    Vx_e[6 + (NYN) * 12] = 0.0;
    
    Vx_e[7 + (NYN) * 0] = 0.0;
    Vx_e[7 + (NYN) * 1] = 0.0;
    Vx_e[7 + (NYN) * 2] = 0.0;
    Vx_e[7 + (NYN) * 3] = 0.0;
    Vx_e[7 + (NYN) * 4] = 0.0;
    Vx_e[7 + (NYN) * 5] = 0.0;
    Vx_e[7 + (NYN) * 6] = 0.0;
    Vx_e[7 + (NYN) * 7] = 1.0;
    Vx_e[7 + (NYN) * 8] = 0.0;
    Vx_e[7 + (NYN) * 9] = 0.0;
    Vx_e[7 + (NYN) * 10] = 0.0;
    Vx_e[7 + (NYN) * 11] = 0.0;
    Vx_e[7 + (NYN) * 12] = 0.0;
    
    Vx_e[8 + (NYN) * 0] = 0.0;
    Vx_e[8 + (NYN) * 1] = 0.0;
    Vx_e[8 + (NYN) * 2] = 0.0;
    Vx_e[8 + (NYN) * 3] = 0.0;
    Vx_e[8 + (NYN) * 4] = 0.0;
    Vx_e[8 + (NYN) * 5] = 0.0;
    Vx_e[8 + (NYN) * 6] = 0.0;
    Vx_e[8 + (NYN) * 7] = 0.0;
    Vx_e[8 + (NYN) * 8] = 1.0;
    Vx_e[8 + (NYN) * 9] = 0.0;
    Vx_e[8 + (NYN) * 10] = 0.0;
    Vx_e[8 + (NYN) * 11] = 0.0;
    Vx_e[8 + (NYN) * 12] = 0.0;
    
    Vx_e[9 + (NYN) * 0] = 0.0;
    Vx_e[9 + (NYN) * 1] = 0.0;
    Vx_e[9 + (NYN) * 2] = 0.0;
    Vx_e[9 + (NYN) * 3] = 0.0;
    Vx_e[9 + (NYN) * 4] = 0.0;
    Vx_e[9 + (NYN) * 5] = 0.0;
    Vx_e[9 + (NYN) * 6] = 0.0;
    Vx_e[9 + (NYN) * 7] = 0.0;
    Vx_e[9 + (NYN) * 8] = 0.0;
    Vx_e[9 + (NYN) * 9] = 1.0;
    Vx_e[9 + (NYN) * 10] = 0.0;
    Vx_e[9 + (NYN) * 11] = 0.0;
    Vx_e[9 + (NYN) * 12] = 0.0;
    
    Vx_e[10 + (NYN) * 0] = 0.0;
    Vx_e[10 + (NYN) * 1] = 0.0;
    Vx_e[10 + (NYN) * 2] = 0.0;
    Vx_e[10 + (NYN) * 3] = 0.0;
    Vx_e[10 + (NYN) * 4] = 0.0;
    Vx_e[10 + (NYN) * 5] = 0.0;
    Vx_e[10 + (NYN) * 6] = 0.0;
    Vx_e[10 + (NYN) * 7] = 0.0;
    Vx_e[10 + (NYN) * 8] = 0.0;
    Vx_e[10 + (NYN) * 9] = 0.0;
    Vx_e[10 + (NYN) * 10] = 1.0;
    Vx_e[10 + (NYN) * 11] = 0.0;
    Vx_e[10 + (NYN) * 12] = 0.0;
    
    Vx_e[11 + (NYN) * 0] = 0.0;
    Vx_e[11 + (NYN) * 1] = 0.0;
    Vx_e[11 + (NYN) * 2] = 0.0;
    Vx_e[11 + (NYN) * 3] = 0.0;
    Vx_e[11 + (NYN) * 4] = 0.0;
    Vx_e[11 + (NYN) * 5] = 0.0;
    Vx_e[11 + (NYN) * 6] = 0.0;
    Vx_e[11 + (NYN) * 7] = 0.0;
    Vx_e[11 + (NYN) * 8] = 0.0;
    Vx_e[11 + (NYN) * 9] = 0.0;
    Vx_e[11 + (NYN) * 10] = 0.0;
    Vx_e[11 + (NYN) * 11] = 1.0;
    Vx_e[11 + (NYN) * 12] = 0.0;
    
    Vx_e[12 + (NYN) * 0] = 0.0;
    Vx_e[12 + (NYN) * 1] = 0.0;
    Vx_e[12 + (NYN) * 2] = 0.0;
    Vx_e[12 + (NYN) * 3] = 0.0;
    Vx_e[12 + (NYN) * 4] = 0.0;
    Vx_e[12 + (NYN) * 5] = 0.0;
    Vx_e[12 + (NYN) * 6] = 0.0;
    Vx_e[12 + (NYN) * 7] = 0.0;
    Vx_e[12 + (NYN) * 8] = 0.0;
    Vx_e[12 + (NYN) * 9] = 0.0;
    Vx_e[12 + (NYN) * 10] = 0.0;
    Vx_e[12 + (NYN) * 11] = 0.0;
    Vx_e[12 + (NYN) * 12] = 1.0;
    yref_e[0] = 0.0;
    yref_e[1] = 0.0;
    yref_e[2] = 0.1;
    yref_e[3] = 1.0;
    yref_e[4] = 0.0;
    yref_e[5] = 0.0;
    yref_e[6] = 0.0;
    yref_e[7] = 0.0;
    yref_e[8] = 0.0;
    yref_e[9] = 0.0;
    yref_e[10] = 0.0;
    yref_e[11] = 0.0;
    yref_e[12] = 0.0;

    int max_num_sqp_iterations = 100;

    int nx[N+1];
    int nu[N+1];
    int nbx[N+1];
    int nbu[N+1];
    int nsbx[N+1];
    int nsbu[N+1];
    int nsh[N+1];
    int ns[N+1];
    int nb[N+1];
    int ng[N+1];
    int nh[N+1];
    int nz[N+1];
    int nv[N+1];
    int ny[N+1];
    int npd[N+1];
    int npd_e[N+1];

    for(int i = 0; i < N+1; i++) {
        nx[i]  = NX_;
        nu[i]  = NU_;
        nbx[i] = NBX_;
        nbu[i] = NBU_;
        nb[i]  = NBU_ + NBX_;
        nsbx[i]  = NSBX_;
        nsbu[i]  = NSBU_;
        nsh[i]  = NSH_;
        ns[i]  = NS_;
        ng[i]  = NG_;
        nh[i]  = NH_;
        npd[i] = NPD_;
        nz[i]  = NZ_;
        nv[i]  = NX_ + NU_;
        ny[i]  = NY_;
    }

    nbx[0] = NX_;
    nbu[0] = NBU_;
    nb[0]  = NX_ + NBU_;
    nsbx[N]  = NSBXN_;
    nsbu[N]  = 0;
    nsh[N]  = NSHN_;
    ns[N]  = NSN_;

    nu[N]  = 0;
    nx[N]  = NX_;
    nz[N]  = 0;
    nh[N]  = NHN_;
    npd[N]  = NPDN_;
    nv[N]  = NX_; 
    ny[N]  = NYN_;
    nbu[N] = 0;
    nbx[N] = NBXN_;
    ng[N]  = NGN_;
    nb[N]  = NBXN_;

    // Make plan
    nlp_solver_plan = ocp_nlp_plan_create(N);
    
    nlp_solver_plan->nlp_solver = SQP_RTI;
    
    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    for (int i = 0; i <= N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;
    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++) {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    
    nlp_config = ocp_nlp_config_create(*nlp_solver_plan);

    /* create and set ocp_nlp_dims */
	nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++) {
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }

    
    // explicit ode
    
    forw_vde_casadi = (external_function_casadi *) malloc(sizeof(external_function_casadi)*N);
    

    for (int i = 0; i < N; ++i) {
        forw_vde_casadi[i].casadi_fun = &crazyflie_expl_vde_forw;
        forw_vde_casadi[i].casadi_n_in = &crazyflie_expl_vde_forw_n_in;
        forw_vde_casadi[i].casadi_n_out = &crazyflie_expl_vde_forw_n_out;
        forw_vde_casadi[i].casadi_sparsity_in = &crazyflie_expl_vde_forw_sparsity_in;
        forw_vde_casadi[i].casadi_sparsity_out = &crazyflie_expl_vde_forw_sparsity_out;
        forw_vde_casadi[i].casadi_work = &crazyflie_expl_vde_forw_work;
        external_function_casadi_create(&forw_vde_casadi[i]);
    }

    
    

    nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);

    for (int i = 0; i < N; ++i)
        nlp_in->Ts[i] = Tf/N;

    // NLP cost linear least squares
    // C  // TODO(oj) this can be done using
    // // ocp_nlp_cost_set_model(nlp_config, nlp_dims, nlp_in, i, "Cyt", Cyt);
    // ocp_nlp_cost_ls_model **cost_ls = (ocp_nlp_cost_ls_model **) nlp_in->cost;
    // for (int i = 0; i <= N; ++i) {
    //     blasfeo_dgese(nv[i], ny[i], 0.0, &cost_ls[i]->Cyt, 0, 0);
    //     for (int j = 0; j < nu[i]; j++)
    //         BLASFEO_DMATEL(&cost_ls[i]->Cyt, j, nx[i]+j) = 1.0;
    //     for (int j = 0; j < nx[i]; j++)
    //         BLASFEO_DMATEL(&cost_ls[i]->Cyt, nu[i]+j, j) = 1.0;
    // }
    // W
    for (int i = 0; i < N; ++i) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    // W_e
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);


	for (int i = 0; i < N; ++i) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vz", Vz);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
	}

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Zl", Zl_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Zu", Zu_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "zl", zl_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "zu", zu_e);

    // NLP dynamics
    int set_fun_status;
    for (int i = 0; i < N; ++i) {
     
        set_fun_status = ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_for", &forw_vde_casadi[i]);
        if (set_fun_status != 0) { printf("Error while setting expl_vde_for[%i]\n", i);  exit(1); }
        
    
    }

    // NLP constraints
    // TODO(oj) remove this when idxb setter available

    // bounds for stage 0
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbu", idxbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", lbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", ubu);

    // bounds for intermediate stages
    for (int i = 1; i < N; ++i)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);

    }

    nlp_opts = ocp_nlp_opts_create(nlp_config, nlp_dims);
    
    
    int ns_val = 1; 
    for (int i = 0; i < N; i++) ocp_nlp_dynamics_opts_set(nlp_config, nlp_opts, i, "ns", &ns_val);
    bool jac_reuse_val = true;
    for (int i = 0; i < N; i++) ocp_nlp_dynamics_opts_set(nlp_config, nlp_opts, i, "jac_reuse", &jac_reuse_val);

    
    // ocp_nlp_sqp_rti_opts *sqp_opts = (ocp_nlp_sqp_rti_opts *) nlp_opts;
    
    

    nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    for (int i = 0; i <= N; ++i) {
        blasfeo_dvecse(nu[i]+nx[i], 0.0, nlp_out->ux+i, 0);
    }
    
    nlp_solver = ocp_nlp_solver_create(nlp_config, nlp_dims, nlp_opts);

    // initialize parameters to nominal value
    

    return status;
}

int acados_solve() {

    // solve NLP 
    int solver_status = ocp_nlp_solve(nlp_solver, nlp_in, nlp_out);

    return solver_status;
}

int acados_free() {

    // free memory
    ocp_nlp_opts_destroy(nlp_opts);
    ocp_nlp_in_destroy(nlp_in);
    ocp_nlp_out_destroy(nlp_out);
    ocp_nlp_solver_destroy(nlp_solver);
    ocp_nlp_dims_destroy(nlp_dims);
    ocp_nlp_config_destroy(nlp_config);
    ocp_nlp_plan_destroy(nlp_solver_plan);

    // free external function 
    
    for(int i = 0; i < 50; i++) {
        
        external_function_casadi_free(&forw_vde_casadi[i]);
        
    
    }
    
    
    return 0;
}

ocp_nlp_in * acados_get_nlp_in() { return  nlp_in; }
ocp_nlp_out * acados_get_nlp_out() { return  nlp_out; }
ocp_nlp_solver * acados_get_nlp_solver() { return  nlp_solver; }
ocp_nlp_config * acados_get_nlp_config() { return  nlp_config; }
void * acados_get_nlp_opts() { return  nlp_opts; }
ocp_nlp_dims * acados_get_nlp_dims() { return  nlp_dims; }