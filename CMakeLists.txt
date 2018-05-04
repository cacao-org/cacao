# run from _build directory :
# cmake .. -DCMAKE_INSTALL_PREFIX=../_install

cmake_minimum_required (VERSION 3.1)
project (cacao C)

# options
option(USE_CUDA  "Use CUDA library"  OFF)
option(USE_MAGMA "Use MAGMA library" OFF)

if(USE_MAGMA)
set(USE_CUDA ON)
endif(USE_MAGMA)

message("====================================================")
message("PROJECT_SOURCE_DIR  = ${PROJECT_SOURCE_DIR}")
message("USE_CUDA            = ${USE_CUDA}")
message("USE_MAGMA           = ${USE_MAGMA}")
message("====================================================")



if(USE_CUDA)
find_package( CUDA REQUIRED )
endif(USE_CUDA)

if(USE_MAGMA)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAGMA REQUIRED magma)
message(STATUS ${MAGMA_LIBRARY_DIRS})
link_directories( ${MAGMA_LIBRARY_DIRS} )
endif(USE_MAGMA)


#                               
# recursively add libraries to be compiled
# each library compile as SHARED
#

add_subdirectory ( "src/ImageStreamIO" )
add_subdirectory ( "src/CommandLineInterface" )
add_subdirectory ( "src/00CORE" )
add_subdirectory ( "src/COREMOD_arith" )
add_subdirectory ( "src/COREMOD_iofits" )
add_subdirectory ( "src/COREMOD_memory" )
add_subdirectory ( "src/COREMOD_tools" )
add_subdirectory ( "src/info" )
add_subdirectory ( "src/fft" )
add_subdirectory ( "src/statistic" )
add_subdirectory ( "src/linopt_imtools" )
add_subdirectory ( "src/image_gen" )
add_subdirectory ( "src/image_filter" )
add_subdirectory ( "src/image_basic" )
add_subdirectory ( "src/ZernikePolyn" )
add_subdirectory ( "src/image_format" )
add_subdirectory ( "src/img_reduce" )
add_subdirectory ( "src/psf" )
add_subdirectory ( "src/cudacomp" )
add_subdirectory ( "src/kdtree" )
add_subdirectory ( "src/linARfilterPred" ) 

add_subdirectory ( "src/AOloopControl" )
add_subdirectory ( "src/AOloopControl_acquireCalib" )
add_subdirectory ( "src/AOloopControl_compTools" )
add_subdirectory ( "src/AOloopControl_computeCalib" )
add_subdirectory ( "src/AOloopControl_DM" )
add_subdirectory ( "src/AOloopControl_IOtools" )
add_subdirectory ( "src/AOloopControl_perfTest" )
add_subdirectory ( "src/AOloopControl_PredictiveControl" )
add_subdirectory ( "src/FPAOloopControl" )


# main
add_executable(cacao 
src/CLImain.c)

target_include_directories(cacao PUBLIC
${PROJECT_SOURCE_DIR}/src
${GSL_INCLUDE_DIRS}
${FFTW_INCLUDE_DIRS}
${FFTWF_INCLUDE_DIRS}
${CURSES_INCLUDE_DIR}
)

target_compile_options(cacao PUBLIC 
${FFTW_CFLAGS_OTHER}
${FFTWF_CFLAGS_OTHER}
-fopenmp)



set_target_properties(cacao PROPERTIES COMPILE_FLAGS "-DPACKAGE_NAME=\\\"cacao\\\" -DPACKAGE_VERSION=\\\"0.0.0\\\" -DCONFIGDIR=\\\"${PROJECT_SOURCE_DIR}/config\\\" -DSOURCEDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DABSSRCTOPDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DPACKAGE_BUGREPORT=\\\"https://github.com/cacao-org/cacao/issues\\\"" )


target_link_libraries (cacao PUBLIC
CLIcore 
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
cfitsio
dl
rt
gomp
${GSL_LIBRARIES} 
${FFTW_LIBRARIES}
${FFTWF_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
${CURSES_LIBRARIES})


if(USE_MAGMA)
target_link_libraries (cacao PUBLIC ${MAGMA_LIBRARIES})
set_target_properties(cacao PROPERTIES COMPILE_FLAGS "-DHAVE_CUDA -DHAVE_MAGMA")
endif(USE_MAGMA)

install(TARGETS cacao DESTINATION bin)
