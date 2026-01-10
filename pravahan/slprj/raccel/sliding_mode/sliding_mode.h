#ifndef sliding_mode_h_
#define sliding_mode_h_
#ifndef sliding_mode_COMMON_INCLUDES_
#define sliding_mode_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "sliding_mode_types.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#include "rtGetInf.h"
#define MODEL_NAME sliding_mode
#define NSAMPLE_TIMES (4) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (12) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (4)   
#elif NCSTATES != 4
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T cvsh2wppvp [ 4 ] ; real_T hxxlqejlqe [ 2 ] ; real_T
frltof5w1d [ 2 ] ; real_T b5mch31ag1 [ 4 ] ; real_T f05ndgvcfi [ 4 ] ; real_T
lyqikrix1a [ 4 ] ; real_T mgthgo0xnm [ 4 ] ; real_T exm13eufw3 [ 4 ] ; real_T
cr1f4lq33r [ 4 ] ; real_T luz5pmgfey [ 8 ] ; real_T en3labafh0 [ 8 ] ; real_T
jbmxj2cv5i [ 16 ] ; } B ; typedef struct { real_T jizccaxfkn [ 4 ] ; struct {
void * LoggedData ; } iogt3txuqe ; struct { void * LoggedData ; } c1nf0gk2e3
; int32_T oslah5f3b3 ; int32_T djuz2hac2i ; boolean_T hd5lfda2lr ; boolean_T
dl0lhrufxm ; } DW ; typedef struct { real_T ocjgzpkjel [ 4 ] ; } X ; typedef
struct { real_T ocjgzpkjel [ 4 ] ; } XDot ; typedef struct { boolean_T
ocjgzpkjel [ 4 ] ; } XDis ; typedef struct { real_T ocjgzpkjel [ 4 ] ; }
CStateAbsTol ; typedef struct { real_T ocjgzpkjel [ 4 ] ; } CXPtMin ; typedef
struct { real_T ocjgzpkjel [ 4 ] ; } CXPtMax ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T
VaryingStateSpace_InitialCondition ; real_T
SlidingModeControllerReachingLaw_Phi ; real_T
SlidingModeControllerReachingLaw_ReachingRate ; real_T
SlidingModeControllerReachingLaw_SlidingMatrix [ 8 ] ; real_T
Delay_InitialCondition ; real_T Constant2_Value [ 4 ] ; real_T
Constant4_Value ; real_T Constant5_Value [ 16 ] ; real_T Constant6_Value [ 8
] ; } ; extern const char_T * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ;
extern X rtX ; extern DW rtDW ; extern P rtP ; extern mxArray *
mr_sliding_mode_GetDWork ( ) ; extern void mr_sliding_mode_SetDWork ( const
mxArray * ssDW ) ; extern mxArray *
mr_sliding_mode_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * sliding_mode_GetCAPIStaticMap ( void ) ;
extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ;
extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T
tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
