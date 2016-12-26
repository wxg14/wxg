//
// MATLAB Compiler: 4.14 (R2010b)
// Date: Fri Dec 16 15:25:27 2016
// Arguments: "-B" "macro_default" "-W" "cpplib:calibration" "-T" "link:lib"
// "-d" "D:\project\Calibration\calibration\src" "-w"
// "enable:specified_file_mismatch" "-w" "enable:repeated_file" "-w"
// "enable:switch_ignored" "-w" "enable:missing_lib_sentinel" "-w"
// "enable:demo_license" "-v" "E:\motion_4u\Software\matlab_cali\Correct.m"
// "E:\motion_4u\Software\matlab_cali\INV_GET.m" 
//

#ifndef __calibration_h
#define __calibration_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_calibration
#define PUBLIC_calibration_C_API __global
#else
#define PUBLIC_calibration_C_API /* No import statement needed. */
#endif

#define LIB_calibration_C_API PUBLIC_calibration_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_calibration
#define PUBLIC_calibration_C_API __declspec(dllexport)
#else
#define PUBLIC_calibration_C_API __declspec(dllimport)
#endif

#define LIB_calibration_C_API PUBLIC_calibration_C_API


#else

#define LIB_calibration_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_calibration_C_API 
#define LIB_calibration_C_API /* No special import/export declaration */
#endif

extern LIB_calibration_C_API 
bool MW_CALL_CONV calibrationInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_calibration_C_API 
bool MW_CALL_CONV calibrationInitialize(void);

extern LIB_calibration_C_API 
void MW_CALL_CONV calibrationTerminate(void);



extern LIB_calibration_C_API 
void MW_CALL_CONV calibrationPrintStackTrace(void);

extern LIB_calibration_C_API 
bool MW_CALL_CONV mlxCorrect(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

extern LIB_calibration_C_API 
bool MW_CALL_CONV mlxINV_GET(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

extern LIB_calibration_C_API 
long MW_CALL_CONV calibrationGetMcrID();


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_calibration
#define PUBLIC_calibration_CPP_API __declspec(dllexport)
#else
#define PUBLIC_calibration_CPP_API __declspec(dllimport)
#endif

#define LIB_calibration_CPP_API PUBLIC_calibration_CPP_API

#else

#if !defined(LIB_calibration_CPP_API)
#if defined(LIB_calibration_C_API)
#define LIB_calibration_CPP_API LIB_calibration_C_API
#else
#define LIB_calibration_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_calibration_CPP_API void MW_CALL_CONV Correct(int nargout, mwArray& H, mwArray& B, const mwArray& Matrix, const mwArray& Sample, const mwArray& MagRefStre);

extern LIB_calibration_CPP_API void MW_CALL_CONV INV_GET(int nargout, mwArray& qiuni, const mwArray& a);

#endif
#endif
