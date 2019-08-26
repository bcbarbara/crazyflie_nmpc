#define S_FUNCTION_NAME   acados_solver_sfunction_crazyflie_simple_model
#define S_FUNCTION_LEVEL  2

#define MDL_START

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// TODO(oj) remove, when setters for Cyt,idxb available
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// example specific
#include "crazyflie_simple_model_model/crazyflie_simple_model_model.h"
#include "acados_solver_crazyflie_simple_model.h"

#include "simstruc.h"

#define SAMPLINGTIME -1
// ** global data **
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;


external_function_casadi * forw_vde_casadi;









static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states 
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    // specify the number of input ports 
    
    if ( !ssSetNumInputPorts(S, 3) )
    
        return;

    // specify the number of output ports 
    if ( !ssSetNumOutputPorts(S, 5) )
        return;

    // specify dimension information for the input ports 
    ssSetInputPortVectorDimension(S, 0, 9);
    ssSetInputPortVectorDimension(S, 1, 650);
    ssSetInputPortVectorDimension(S, 2, 9);
    

    // specify dimension information for the output ports 
    ssSetOutputPortVectorDimension(S, 0, 4 ); // optimal input
    ssSetOutputPortVectorDimension(S, 1, 1 );                 // solver status
    ssSetOutputPortVectorDimension(S, 2, 1 );                 // KKT residuals
    ssSetOutputPortVectorDimension(S, 3, 9 ); // first state
    ssSetOutputPortVectorDimension(S, 4, 1);                  // computation times

    // specify the direct feedthrough status 
    ssSetInputPortDirectFeedThrough(S, 0, 1); // current state x0
    ssSetInputPortDirectFeedThrough(S, 1, 1); // y_ref
    ssSetInputPortDirectFeedThrough(S, 2, 1); // y_ref__e
    

    // one sample time 
    ssSetNumSampleTimes(S, 1);
    }


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

    #endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    acados_create();
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // get input signals
    InputRealPtrsType in_x0_sign;
    InputRealPtrsType in_y_ref_sign;
    InputRealPtrsType in_y_ref_e_sign;
    
    
    // local buffers
    real_t in_x0[9];
    real_t in_y_ref[650];
    real_t in_y_ref_e[9];
    

    in_x0_sign = ssGetInputPortRealSignalPtrs(S, 0);
    in_y_ref_sign = ssGetInputPortRealSignalPtrs(S, 1);
    in_y_ref_e_sign = ssGetInputPortRealSignalPtrs(S, 2);
    

    // copy signals into local buffers
    for (int i = 0; i < 9; i++) in_x0[i] = (double)(*in_x0_sign[i]);
    for (int i = 0; i < 650; i++) in_y_ref[i] = (double)(*in_y_ref_sign[i]);
    for (int i = 0; i < 9; i++) in_y_ref_e[i] = (double)(*in_y_ref_e_sign[i]);
    

    // for (int i = 0; i < 4; i++) ssPrintf("x0[%d] = %f\n", i, in_x0[i]);
    // ssPrintf("\n");

    // set initial condition
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", in_x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", in_x0);

    // update reference
    for (int ii = 0; ii < 50; ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, 
                nlp_in, ii, "yref", (void *) (in_y_ref + ii*13);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 
        50, "yref", (void *) in_y_ref_e);

    // update value of parameters
    
    
    // assign pointers to output signals 
    real_t *out_u0, *out_status, *out_KKT_res, *out_x1, *out_cpu_time;

    out_u0          = ssGetOutputPortRealSignal(S, 0);
    out_status      = ssGetOutputPortRealSignal(S, 1);
    out_KKT_res     = ssGetOutputPortRealSignal(S, 2);
    out_x1          = ssGetOutputPortRealSignal(S, 3);
    out_cpu_time    = ssGetOutputPortRealSignal(S, 4);
    
    // call acados_solve()
    int acados_status = acados_solve();

    *out_status = (real_t) acados_status;
    *out_KKT_res = (real_t) nlp_out->inf_norm_res;
    *out_cpu_time = (real_t) nlp_out->total_time;
    
    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_u0);

    // get next state
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *) out_x1);

}

static void mdlTerminate(SimStruct *S)
{
    acados_free();
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif