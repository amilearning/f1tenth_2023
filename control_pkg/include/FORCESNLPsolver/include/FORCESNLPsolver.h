#ifndef FORCESNLPsolver_H
#define FORCESNLPsolver_H
/* Generated by FORCESPRO v6.0.1 on Tuesday, March 28, 2023 at 12:41:03 PM */

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#include <stddef.h>
#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double FORCESNLPsolver_float;
typedef double FORCESNLPsolver_callback_float;
typedef double FORCESNLPsolverinterface_float;
typedef int FORCESNLPsolver_int;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_FORCESNLPsolver
#define MISRA_C_FORCESNLPsolver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_FORCESNLPsolver
#define RESTRICT_CODE_FORCESNLPsolver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_FORCESNLPsolver
#define SET_PRINTLEVEL_FORCESNLPsolver (0)
#endif

/* timing */
#ifndef SET_TIMING_FORCESNLPsolver
#define SET_TIMING_FORCESNLPsolver (0)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_FORCESNLPsolver (200)	 


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_FORCESNLPsolver (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_FORCESNLPsolver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_FORCESNLPsolver (2)

/* solver stopped externally */
#define EXIT_EXTERNAL_FORCESNLPsolver (3)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_FORCESNLPsolver (-6)

/* no progress in method possible */
#define NOPROGRESS_FORCESNLPsolver (-7)

/* regularization error */
#define REGULARIZATION_ERROR_FORCESNLPsolver (-9)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_FORCESNLPsolver (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_FORCESNLPsolver (-12)

/* error in linesearch */
#define LINESEARCH_ERROR_FORCESNLPsolver (-13)

/* thread error */
#define THREAD_FAILURE_FORCESNLPsolver (-98)

/* locking mechanism error */
#define LOCK_FAILURE_FORCESNLPsolver (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_FORCESNLPsolver (-100)

/* Insufficient number of internal memory instances.
 * Increase codeoptions.max_num_mem. */
#define MEMORY_INVALID_FORCESNLPsolver (-101)
/* Number of threads larger than specified.
 * Increase codeoptions.nlp.max_num_threads. */
#define NUMTHREADS_INVALID_FORCESNLPsolver (-102)

/* qp solver error */
#define QP_SOLVER_FAILURE_FORCESNLPsolver (-8)


/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 5 */
    FORCESNLPsolver_float xinit[5];

    /* vector of size 70 */
    FORCESNLPsolver_float x0[70];

    /* vector of size 20 */
    FORCESNLPsolver_float all_parameters[20];

    /* scalar */
    FORCESNLPsolver_int reinitialize;


} FORCESNLPsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* column vector of length 7 */
    FORCESNLPsolver_float x01[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x02[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x03[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x04[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x05[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x06[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x07[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x08[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x09[7];

    /* column vector of length 7 */
    FORCESNLPsolver_float x10[7];


} FORCESNLPsolver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* scalar: iteration number */
    solver_int32_default it;

    /* scalar: inf-norm of equality constraint residuals */
    FORCESNLPsolver_float res_eq;

    /* scalar: norm of stationarity condition */
    FORCESNLPsolver_float rsnorm;

    /* scalar: primal objective */
    FORCESNLPsolver_float pobj;

    /* scalar: total solve time */
    FORCESNLPsolver_float solvetime;

    /* scalar: time spent in function evaluations */
    FORCESNLPsolver_float fevalstime;

    /* scalar: time spent solving inner QPs */
    FORCESNLPsolver_float QPtime;

    /* scalar: iterations spent in solving inner QPs */
    solver_int32_default QPit;

    /* scalar: last exitflag of inner QP solver */
    solver_int32_default QPexitflag;

    /* column vector of length 8: solver ID of FORCESPRO solver */
    solver_int32_default solver_id[8];


} FORCESNLPsolver_info;

/* MEMORY STRUCT --------------------------------------------------------*/
typedef struct FORCESNLPsolver_mem FORCESNLPsolver_mem;
#ifdef __cplusplus
extern "C" {
#endif
/* MEMORY STRUCT --------------------------------------------------------*/
extern FORCESNLPsolver_mem * FORCESNLPsolver_external_mem(void * mem_ptr, solver_int32_unsigned i_mem, size_t mem_size);
extern size_t FORCESNLPsolver_get_mem_size( void );
extern size_t FORCESNLPsolver_get_const_size( void );
#ifdef __cplusplus
}
#endif

/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Tuesday, March 28, 2023 12:41:04 PM */
/* User License expires on: (UTC) Monday, August 7, 2023 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Monday, August 7, 2023 10:00:00 PM (approx.) */
/* Solver Id: bf0c8b9d-e1bf-49d2-8534-075ef0d48d47 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef solver_int32_default(*FORCESNLPsolver_extfunc)(FORCESNLPsolver_float* x, FORCESNLPsolver_float* y, FORCESNLPsolver_float* lambda, FORCESNLPsolver_float* params, FORCESNLPsolver_float* pobj, FORCESNLPsolver_float* g, FORCESNLPsolver_float* c, FORCESNLPsolver_float* Jeq, FORCESNLPsolver_float* h, FORCESNLPsolver_float* Jineq, FORCESNLPsolver_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default FORCESNLPsolver_solve(FORCESNLPsolver_params *params, FORCESNLPsolver_output *output, FORCESNLPsolver_info *info, FORCESNLPsolver_mem *mem, FILE *fs, FORCESNLPsolver_extfunc evalextfunctions_FORCESNLPsolver);


extern solver_int32_default FORCESNLPsolver_adtool2forces(FORCESNLPsolver_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */);



#ifdef __cplusplus
}
#endif

#endif
