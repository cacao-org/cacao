/**
 * @file modes_spatial_extrapolate.h
 * @brief Modes spatial extrapolate module
 */

#ifndef CACAO_COMPUTECALIB_MODES_SPATIAL_EXTRAPOLATE_H
#define CACAO_COMPUTECALIB_MODES_SPATIAL_EXTRAPOLATE_H

errno_t modes_spatial_extrapolate(IMGID imgmodes,
                                  IMGID imgmask,
                                  IMGID imgcpa,
                                  IMGID *imgoutmodes);

#endif
