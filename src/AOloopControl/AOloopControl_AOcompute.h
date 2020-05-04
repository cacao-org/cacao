#ifndef _AOLOOPCONTROL_AOcompute_H
#define _AOLOOPCONTROL_AOcompute_H



typedef struct
{
    int ComputeWFSsol_FLAG;
    int ComputeFLAG0;
    int ComputeFLAG1;
    int ComputeFLAG2;
    int ComputeFLAG3;

    int GPU0;                     // NB of GPU devices in set 0. 1+ if matrix multiplication done by GPU (set 0)
    int GPU1;                     // NB of GPU devices in set 1. 1+ if matrix multiplication done by GPU (set 1)
    int GPU2;                     // NB of GPU devices in set 2. 1+ if matrix multiplication done by GPU (set 2)
    int GPU3;                     // NB of GPU devices in set 3. 1+ if matrix multiplication done by GPU (set 3)
    int GPU4;                     // NB of GPU devices in set 4. 1+ if matrix multiplication done by GPU (set 4)
    int GPU5;                     // NB of GPU devices in set 5. 1+ if matrix multiplication done by GPU (set 5)
    int GPU6;                     // NB of GPU devices in set 6. 1+ if matrix multiplication done by GPU (set 6)
    int GPU7;                     // NB of GPU devices in set 7. 1+ if matrix multiplication done by GPU (set 7)

    int GPUall;                   // 1 if scaling computations done by GPU
    int GPUusesem;                // 1 if using semaphores to control GPU
    int AOLCOMPUTE_TOTAL_ASYNC;   // 1 if performing image total in separate thread (runs faster, but image total dates from last frame)

} AOLOOPCONF_AOcompute;






int AOloopControl_AOcompute_GUI(
    long loop,
    double frequ
);


#endif
