#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "sliding_mode_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 17
#endif
#ifndef SS_INT64
#define SS_INT64 18
#endif
#else
#include "builtin_typeid_types.h"
#include "sliding_mode.h"
#include "sliding_mode_capi.h"
#include "sliding_mode_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 1 , TARGET_STRING ( "sliding_mode/f from u1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 2 , TARGET_STRING ( "sliding_mode/g from u1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 2 , 0 , TARGET_STRING ( "sliding_mode/Delay" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 1 } , { 3 , 0 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)/Transpose" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 2 } , { 4 , 0 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)/Divide" ) , TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 } , { 5 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/State" ) , TARGET_STRING ( "x" ) , 0 , 0 , 5 , 0 , 3 } , { 6 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/ProductA" ) , TARGET_STRING ( "Ax" ) , 0 , 0 , 5 , 0 , 3 } , { 7 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/ProductB" ) , TARGET_STRING ( "Bu" ) , 0 , 0 , 5 , 0 , 0 } , { 8 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/ProductD" ) , TARGET_STRING ( "Du" ) , 0 , 0 , 5 , 0 , 0 } , { 9 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/RSy" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 3 } , { 10 , 0 , TARGET_STRING ( "sliding_mode/Varying State Space/dxSum" ) , TARGET_STRING ( "dx" ) , 0 , 0 , 5 , 0 , 3 } , { 11 , 0 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)/Reaching Law/Constant Rate/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 1 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 12 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)" ) , TARGET_STRING ( "SlidingMatrix" ) , 0 , 1 , 0 } , { 13 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)" ) , TARGET_STRING ( "ReachingRate" ) , 0 , 6 , 0 } , { 14 , TARGET_STRING ( "sliding_mode/Sliding Mode Controller (Reaching Law)" ) , TARGET_STRING ( "Phi" ) , 0 , 6 , 0 } , { 15 , TARGET_STRING ( "sliding_mode/Varying State Space" ) , TARGET_STRING ( "InitialCondition" ) , 0 , 6 , 0 } , { 16 , TARGET_STRING ( "sliding_mode/Constant2" ) , TARGET_STRING ( "Value" ) , 0 , 2 , 0 } , { 17 , TARGET_STRING ( "sliding_mode/Constant4" ) , TARGET_STRING ( "Value" ) , 0 , 6 , 0 } , { 18 , TARGET_STRING ( "sliding_mode/Constant5" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 19 , TARGET_STRING ( "sliding_mode/Constant6" ) , TARGET_STRING ( "Value" ) , 0 , 1 , 0 } , { 20 , TARGET_STRING ( "sliding_mode/Delay" ) , TARGET_STRING ( "InitialCondition" ) , 0 , 6 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1 } ; static const rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . jbmxj2cv5i [ 0 ] , & rtB .
en3labafh0 [ 0 ] , & rtB . cvsh2wppvp [ 0 ] , & rtB . luz5pmgfey [ 0 ] , &
rtB . frltof5w1d [ 0 ] , & rtB . b5mch31ag1 [ 0 ] , & rtB . mgthgo0xnm [ 0 ]
, & rtB . exm13eufw3 [ 0 ] , & rtB . f05ndgvcfi [ 0 ] , & rtB . lyqikrix1a [
0 ] , & rtB . cr1f4lq33r [ 0 ] , & rtB . hxxlqejlqe [ 0 ] , & rtP .
SlidingModeControllerReachingLaw_SlidingMatrix [ 0 ] , & rtP .
SlidingModeControllerReachingLaw_ReachingRate , & rtP .
SlidingModeControllerReachingLaw_Phi , & rtP .
VaryingStateSpace_InitialCondition , & rtP . Constant2_Value [ 0 ] , & rtP .
Constant4_Value , & rtP . Constant5_Value [ 0 ] , & rtP . Constant6_Value [ 0
] , & rtP . Delay_InitialCondition , } ; static int32_T * rtVarDimsAddrMap [
] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_MATRIX_COL_MAJOR , 0 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 2 , 2 ,
0 } , { rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 6 , 2 , 0
} , { rtwCAPI_MATRIX_COL_MAJOR , 8 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 4
, 2 , 0 } , { rtwCAPI_SCALAR , 10 , 2 , 0 } } ; static const uint_T
rtDimensionArray [ ] = { 4 , 4 , 4 , 2 , 4 , 1 , 2 , 4 , 2 , 1 , 1 , 1 } ;
static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 } ; static const
rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static const
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 1 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 2 , ( uint8_T ) 0 } , { ( NULL ) , ( NULL ) , 3 , 0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 12 , rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 9 , rtModelParameters , 0 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 2096759775U , 2439401993U , 1016586332U , 2257312720U } , ( NULL ) , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * sliding_mode_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void sliding_mode_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void sliding_mode_host_InitializeDataMapInfo ( sliding_mode_host_DataMapInfo_T
* dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
