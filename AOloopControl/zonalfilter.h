#ifndef AOLOOPCONTROL_ZONALFILTER_H
#define AOLOOPCONTROL_ZONALFILTER_H

#include "fps.h"
#include "processinfo.h"

errno_t CLIADDCMD_AOloopControl__zonalfilter();

#define ZONALFILTER_PARAMS(X) \
    X(CLIARG_UINT64,  FPTYPE_UINT64,     uint64_t, ".AOloopindex", "AO loop index",           "0",             0,      \&AOloopindex,   (void*)&val, CLIARG_VISIBLE_DEFAULT) \
    X(CLIARG_STREAM,  FPTYPE_STREAMNAME, char*,    ".inzval",      "input DM zonal values",   "aol0_actvalDM", "aol0_actvalDM", \&inzval,       (void*)val,  CLIARG_VISIBLE_DEFAULT) \
    X(CLIARG_STREAM,  FPTYPE_STREAMNAME, char*,    ".outzval",     "output DM zonal values",  "aol0_actvalDMf","aol0_actvalDMf",\&outzval,      (void*)val,  CLIARG_VISIBLE_DEFAULT) \
    X(CLIARG_ONOFF,   FPTYPE_ONOFF,      int64_t,  ".loopON",      "loop on/off (off=freeze)","ON",            1,      \&loopON,        (void*)&val, CLIARG_HIDDEN_DEFAULT) \
    X(CLIARG_INT64,   FPTYPE_INT64,      int64_t,  ".loopNBstep",  "loop nb steps (-1 = inf)","-1",           -1,      \&loopNBstep,    (void*)&val, CLIARG_HIDDEN_DEFAULT) \
    X(CLIARG_ONOFF,   FPTYPE_ONOFF,      int64_t,  ".loopZERO",    "loop zero",               "OFF",           0,      \&loopZERO,      (void*)&val, CLIARG_HIDDEN_DEFAULT) \
    X(CLIARG_FLOAT32, FPTYPE_FLOAT32,    float,    ".loopgain",    "loop gain (speed)",       "0.01",          0.01,   \&loopgain,      (void*)&val, CLIARG_HIDDEN_DEFAULT) \
    X(CLIARG_FLOAT32, FPTYPE_FLOAT32,    float,    ".loopmult",    "loop mult (attenuation)", "0.95",          0.95,   \&loopmult,      (void*)&val, CLIARG_HIDDEN_DEFAULT) \
    X(CLIARG_FLOAT32, FPTYPE_FLOAT32,    float,    ".looplimit",   "loop limit",              "1.0",           1.0,    \&looplimit,     (void*)&val, CLIARG_HIDDEN_DEFAULT)

extern uint64_t *AOloopindex;
extern char     *inzval;
extern char     *outzval;
extern int64_t  *loopON;
extern int64_t  *loopNBstep;
extern int64_t  *loopZERO;
extern float    *loopgain;
extern float    *loopmult;
extern float    *looplimit;

#define ZONALFILTER_HELPTEXT \
    "zonalfilter: zonal filtering in DM space\n" \
    "========================================\n"

#endif
