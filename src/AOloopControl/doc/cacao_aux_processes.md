


# Auxilliary processes {#page_cacao_aux_processes}

A number of auxilliary processes can be running in addition to the main loop operation.


## Extract WFS modes

Launches script `./auxscripts/modesextractwfs` :

~~~ {.numberLines}
!INCLUDE "../scripts/auxscripts/modesextractwfs"
~~~

Converts WFS residuals into modes.


## Extract open loop modes

Launches script C function (CPU-based):

~~~
key       :    aolcompolm
module    :    AOloopControl.c
info      :    compute open loop mode values
syntax    :    <loop #>
example   :    aolcompolm 2
C call    :    long AOloopControl_ComputeOpenLoopModes(long loop)
~~~

This function is entirely modal, and assumes that the WFS modes (see section above) are computed. The key input to the function is `aolN_modeval`, the WFS residual mode values. The function uses this telemetry and knowledge of loop gain and mult factor to track open loop mode values.

Optionally, it also includes `aolN_modeval_pC`, the predictive control mode values that are added to the correction in predictive mode.


## Running average of dmC


Launches script `./auxscripts/aol_dmCave 0.0005` :

~~~ {.numberLines}
!INCLUDE "../scripts/auxscripts/aol_dmCave"
~~~


## Compute and average wfsres


Launches script `./auxscripts/aolmkWFSres 0.0005` :

~~~ {.numberLines}
!INCLUDE "../scripts/auxscripts/aolmkWFSres"
~~~
