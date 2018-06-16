# Example, run from _build directory :
#
# cmake .. -DCMAKE_INSTALL_PREFIX=../_install
# cmake .. -DUSE_MAGMA=ON
# cmake .. -DUSE_QDWHPARTIAL=ON
#
# make clean
# make
# sudo make install
#

cmake_minimum_required (VERSION 3.1)
project (cacao C)



# PERFORMANCE ORIENTED COMPILING OPTIONS

# Adds options to the compiler command line for targets in the current directory and below that are added after this command is invoked
add_compile_options(-std=gnu11)
add_compile_options(-Ofast)
add_compile_options(-march=native)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
message("Found OpenMP")
add_compile_options(-fopenmp)
#CMAKE_EXE_LINKER_FLAGS
endif()

message("COMPILE_OPTIONS:  ${COMPILE_OPTIONS}")

# runs the standard link-time optimizer
add_compile_options(-flto)

# A common flag is -pipe. This flag has no effect on the generated code, but it makes the compilation process faster. It tells the compiler to use pipes instead of temporary files during the different stages of compilation, which uses more memory. On systems with low memory, GCC might get killed. In those cases do not use this flag. 
add_compile_options(-pipe)




# options
option(USE_CUDA  "Use CUDA        library"  OFF)
option(USE_MAGMA "Use MAGMA       library" OFF)
option(USE_MAGMA "Use QDWHpartial library" OFF)


# MAGMA (optional)
if(USE_MAGMA)
set(USE_CUDA ON)
endif(USE_MAGMA)


# QDWH (optional)
if(USE_QDWHPARTIAL)
set(USE_QDWHPARTIAL ON)
set(USE_CUDA ON)
set(USE_MAGMA ON)
endif(USE_QDWHPARTIAL)
set( QDWHPARTIAL_INCLUDE_PATH "/home/guyon/src/subaru/qdwhpartial-svd/magma" )
set( QDWHPARTIAL_LIBRARIES "psinv" )




if(USE_CUDA)
find_package( CUDA REQUIRED )
endif(USE_CUDA)

if(USE_MAGMA)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAGMA REQUIRED magma)
message(STATUS ${MAGMA_LIBRARY_DIRS})
link_directories( ${MAGMA_LIBRARY_DIRS} )
endif(USE_MAGMA)


# ncurses
#set( CURSES_INCLUDE_PATH "/usr/include" )
#message("CURSES_INCLUDE_PATH = ${CURSES_INCLUDE_PATH}")
#set( CMAKE_INCLUDE_PATH "/usr/include")
#set( CMAKE_LIBRARY_PATH "/usr/lib/x86_64-linux-gnu/libncurses.so")
find_package(Curses REQUIRED)
message("CURSES_INCLUDE_PATH = ${CURSES_INCLUDE_PATH}")
message("CURSES_LIBRARIES    = ${CURSES_LIBRARIES}")
SET(CURSES_USE_NCURSES TRUE)




message("====================================================")
message("PROJECT_SOURCE_DIR       = ${PROJECT_SOURCE_DIR}")

message("USE_CUDA                 = ${USE_CUDA}")

message("USE_MAGMA                = ${USE_MAGMA}")
if(USE_MAGMA)
message("   MAGMA_LIBRARIES       = ${MAGMA_LIBRARIES}")
endif(USE_MAGMA)

message("USE QDWHPARTIAL          = ${USE_QDWHPARTIAL}")
if(USE_QDWHPARTIAL)
message("   QDWHPARTIAL_LIBRARIES = ${QDWHPARTIAL_LIBRARIES}")
endif(USE_QDWHPARTIAL)

message("====================================================")




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

# adds the options to all targets within the directory and its sub-directories
target_compile_options(cacao PUBLIC 
${FFTW_CFLAGS_OTHER}
${FFTWF_CFLAGS_OTHER}
-fopenmp
)

#add_definitions(

set(BUILD_FLAGS "-DPACKAGE_NAME=\\\"cacao\\\" -DPACKAGE_VERSION=\\\"0.0.0\\\" -DCONFIGDIR=\\\"${PROJECT_SOURCE_DIR}/config\\\" -DSOURCEDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DABSSRCTOPDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DPACKAGE_BUGREPORT=\\\"https://github.com/cacao-org/cacao/issues\\\"" )





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



if(USE_MAGMA)
target_link_libraries (cacao PUBLIC ${MAGMA_LIBRARIES})
set(BUILD_FLAGS "${BUILD_FLAGS} -DHAVE_CUDA -DHAVE_MAGMA" )
endif(USE_MAGMA)



if(USE_QDWHPARTIAL)
target_link_libraries (cacao PUBLIC ${QDWHPARTIAL_LIBRARIES})
target_include_directories(cacao PUBLIC ${QDWHPARTIAL_INCLUDE_PATH} )
set(BUILD_FLAGS "${BUILD_FLAGS} -DHAVE_QDWHpartial" )
endif(USE_QDWHPARTIAL)


set_target_properties(cacao PROPERTIES COMPILE_FLAGS "${BUILD_FLAGS}" )



message("====================================================")
get_target_property(cacaocompopts cacao COMPILE_OPTIONS)

message("BUILD_FLAGS    :  ${BUILD_FLAGS}")
message("COMPILE_OPTIONS:  ${cacaocompopts}")
message("CMAKE_EXE_LINKER_FLAGS  :  ${CMAKE_EXE_LINKER_FLAGS}")
message("====================================================")


install(TARGETS cacao DESTINATION bin)













