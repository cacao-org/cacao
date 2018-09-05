cmake_minimum_required(VERSION 3.6)

# Global option #
#################

set (CMAKE_CXX_STANDARD 14)

project(pyCacao LANGUAGES CXX)

if(NOT TARGET wyrm)
    add_subdirectory(wyrm wyrm)
endif()

add_library(pyCacao MODULE
pyCacao.cpp
)

add_definitions(${CLIcore_DEFINITIONS})

# target_compile_options(pyCacao PRIVATE -fopenmp)

target_link_libraries(pyCacao PRIVATE wyrm CLIcore 
00CORE 
COREMOD_arith 
COREMOD_memory 
ImageStreamIO 
COREMOD_tools 
info 
fft 
image_basic
COREMOD_iofits 
ZernikePolyn
image_filter
image_gen
statistic
image_format 
img_reduce 
psf
cudacomp
kdtree 
linARfilterPred
AOloopControl
AOloopControl_acquireCalib
AOloopControl_compTools
AOloopControl_computeCalib
AOloopControl_DM
AOloopControl_IOtools
AOloopControl_perfTest
AOloopControl_PredictiveControl
FPAOloopControl
linopt_imtools
m
readline
ncurses
cfitsio
dl
rt
${GSL_LIBRARIES} 
${FFTW_LIBRARIES}
${FFTWF_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
-fopenmp
)

set_target_properties(pyCacao PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")
