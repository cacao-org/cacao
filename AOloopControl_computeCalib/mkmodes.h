
#ifndef AOLOOPCONTROL_COMPUTECALIB_MKMODES_H
#define AOLOOPCONTROL_COMPUTECALIB_MKMODES_H

imageID AOloopControl_computeCalib_mkModes(const char *ID_name,
                                           uint32_t    msizex,
                                           uint32_t    msizey,
                                           float       CPAmax,
                                           float       deltaCPA,
                                           double      xc,
                                           double      yc,
                                           double      r0,
                                           double      r1,
                                           int         MaskMode,
                                           int         BlockNB,
                                           float       SVDlim,
                                           char       *outdir);

imageID AOloopControl_computeCalib_mkModes_new(const char *ID_name,
                                               uint32_t    msizex,
                                               uint32_t    msizey,
                                               float       CPAmax,
                                               float       deltaCPA,
                                               double      xc,
                                               double      yc,
                                               double      r0,
                                               double      r1,
                                               int         MaskMode,
                                               int         BlockNB,
                                               float       SVDlim,
                                               char       *outdir);

#endif
