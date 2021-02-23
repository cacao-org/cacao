# Running the loop: setting mode (CPU/GPU) {#page_cacao_running_loop_mode}

There are multiple ways to perform the computations on CPU and/or GPUs. The main 3 parameters are:

- **GPU**     : 0 if matrix multiplication(s) done on CPU, >0 for GPU use. This is the number GPUs to use for matrix mult.

- **CMmode**  : 1 if using a combined matrix between WFS pixels and DM actuators, skipping intermediate computation of modes

- **GPUall**  : if using GPUall, then the WFS reference subtraction is wrapped inside the GPU matrix multiplication


------- --------- --------- ------------ ---------- --------------------------------------------------------------------------------
GPU     CMmode    GPUall    Matrix       Features   Description
------- --------- --------- ------------ ---------- --------------------------------------------------------------------------------
>0      ON         ON       contrMcact   fastest    dark-subtracted WFS frame imWFS0 is multiplited by collapsed control matrix (only active pixels).
                                         no mcoeff  normalization and WFS reference subtraction are wrapped in this GPU operation as subtraction of pre-computed vector output.
                                                    This is the fastest mode.
                            
>0      ON         OFF      contrMcact              WFS reference is subtracted from imWFS0 in CPU, yielding imWFS2.                         
                                                    imWFS2 is multiplied by control matrix (only active pixels) in GPU.
                            
>0      OFF        OFF      contrM                  MWFS reference is subtracted from imWFS0 in CPU, yiedling imWFS2.
                                                    imWFS2 is multiplied (GPU) by control matrix to yield mode values.
                                                    Mode coefficients then multiplied (GPU) by modes.

0       ON         -        contrMcact              imWFS2 is multiplied by control matrix (only active pixels) in CPU

0       OFF        -        contrM                  imWFS2 multiplied by modal control matrix
------- --------- --------- ------------ ----------- ---------------------------------------------------------------------
