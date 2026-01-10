#include "sliding_mode_capi_host.h"
static sliding_mode_host_DataMapInfo_T root;
static int initialized = 0;
rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        sliding_mode_host_InitializeDataMapInfo(&(root), "sliding_mode");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction(){return(getRootMappingInfo());}
