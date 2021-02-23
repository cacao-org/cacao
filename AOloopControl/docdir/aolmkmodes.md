% Making control modes (aolmkmodes)
% Olivier Guyon
% July 9, 2016


# Making AO control modes

## Overview

Script ``aolmkmodes`` creates control modes from system response matrix.


~~~ {.bash}
!INCLUDE "../scripts/auxscripts/aolmkmodes"
~~~


## Algorithm

![Control Modes](AOloopControl_figs-3.jpg "Control Modes")

Takes response matrix (zrespM)

The first steps are done in DM space:

* computes Zernike / Fourier mode basis for dm mask -> ./mkmodestmp/fmodes0all.fits
* compute WFS response to modes -> ./mkmodestmp/fmodesWFS00all.fits
* separate DM modes into blocks
* REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim0 FOR CUTOFF -> ./mkmodestmp/fmodes1_xx.fits ./mkmodestmp/fmodes1all.fits
* REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes2all.fits
* STEP 5: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim0 FOR CUTOFF -> fmodes2ball.fits  (DM space)


Followed by WFS space computations:

* STEP 6: COMPUTE WFS RESPONSE TO MODES:  fmodes2ball -> fmodesWFS0all.fits
* STEP 7: REMOVE WFS MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE WFS-SPACE ORTHOGONALITY BETWEEN BLOCKS: fmodesWFS0all (corresponding to fmodes2ball) -> fmodesWFS1all / fmodes3all
* STEP 8: SVD WFS SPACE IN EACH BLOCK : fmodesWFS1all, fmodes3 -> fmodes4all
* STEP 9: REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes5all.fits  (DM space)
* STEP 10: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim0 FOR CUTOFF -> fmodes5ball.fits  (DM space)








