#ifndef _AOLOOPCONTROL_AOcompute_H
#define _AOLOOPCONTROL_AOcompute_H



typedef struct
{
	int_fast8_t ComputeWFSsol_FLAG; 
	int_fast8_t ComputeFLAG0;
	int_fast8_t ComputeFLAG1;
	int_fast8_t ComputeFLAG2;
	int_fast8_t ComputeFLAG3;

    int_fast8_t GPU0;                     // NB of GPU devices in set 0. 1+ if matrix multiplication done by GPU (set 0)
    int_fast8_t GPU1;                     // NB of GPU devices in set 1. 1+ if matrix multiplication done by GPU (set 1)
    int_fast8_t GPU2;                     // NB of GPU devices in set 2. 1+ if matrix multiplication done by GPU (set 2)
    int_fast8_t GPU3;                     // NB of GPU devices in set 3. 1+ if matrix multiplication done by GPU (set 3)    
    int_fast8_t GPU4;                     // NB of GPU devices in set 4. 1+ if matrix multiplication done by GPU (set 4)
    int_fast8_t GPU5;                     // NB of GPU devices in set 5. 1+ if matrix multiplication done by GPU (set 5)
    int_fast8_t GPU6;                     // NB of GPU devices in set 6. 1+ if matrix multiplication done by GPU (set 6)
    int_fast8_t GPU7;                     // NB of GPU devices in set 7. 1+ if matrix multiplication done by GPU (set 7)
            
    int_fast8_t GPUall;                   // 1 if scaling computations done by GPU
    int_fast8_t GPUusesem;                // 1 if using semaphores to control GPU
    int_fast8_t AOLCOMPUTE_TOTAL_ASYNC;   // 1 if performing image total in separate thread (runs faster, but image total dates from last frame)

} AOLOOPCONF_AOcompute;



int AOloopControl_AOcompute_GUI(
	long loop, 
	double frequ
	);


#endif
