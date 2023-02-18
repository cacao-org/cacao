
#ifndef AOLOOPCONTROL_COMPUTECALIB_HADAMARD_H
#define AOLOOPCONTROL_COMPUTECALIB_HADAMARD_H

imageID AOloopControl_computeCalib_mkHadamardModes(
    const char *DMmask_name,
    const char *outname
);

imageID AOloopControl_computeCalib_Hadamard_decodeRM(
    const char *inname,
    const char *Hmatname,
    const char *indexname,
    const char *outname
);


errno_t CLIADDCMD_AOloopControl_computeCalib__mkHadamard();


#endif
