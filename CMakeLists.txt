# Example, run from _build directory :
#
#
# 
# cmake ..
# cmake .. -DCMAKE_INSTALL_PREFIX=../_install
#
#
#
# USING GPU libraries is inherited from milk
# ===============================
# cmake .. -DUSE_MAGMA=ON
#
#
#
# BUILD TYPE
# ===============================
# Debug build type turns off optimization
# cmake -DCMAKE_BUILD_TYPE=Debug ..
# cmake -DCMAKE_BUILD_TYPE=Debug .. -DUSE_MAGMA=ON
#
# Release build type turns on optimization flags
# cmake -DCMAKE_BUILD_TYPE=Release ..
#
#
#
# COMPILING
# ================================
# make clean
# make
# sudo make install
#
#
#

cmake_minimum_required (VERSION 3.5)

project (cacao C)

# Version number
set ( VERSION_MAJOR 1 )
set ( VERSION_MINOR 01 )
set ( VERSION_PATCH 02 )
#set ( VERSION_OPTION "-dev" )

# turn VERBOSE ON
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_INSTALL_PREFIX "/usr/local/${PROJECT_NAME}-${VERSION_MAJOR}.${VERSION_MINOR}")


# Configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/cacao_config.h.in"
  "${PROJECT_SOURCE_DIR}/src/cacao_config.h"
  )

configure_file (
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_SOURCE_DIR}/src/config.h"
  )





# =======================================
# PERFORMANCE ORIENTED COMPILING OPTIONS
# =======================================

# Set Release build flags
set(CMAKE_C_FLAGS_DEBUG     "-O0 -g -Wall -Wextra")

# Set Release build flags
set(CMAKE_C_FLAGS_RELEASE     "-Ofast -DNDEBUG")


# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}")
endif()

message("CMAKE_BUILD_TYPE : ${CMAKE_BUILD_TYPE}")

# Adds options to the compiler command line for targets in the current directory 
# and below that are added after this command is invoked

add_compile_options(-std=gnu11)
add_compile_options(-march=native)


# NOTE: -fstack-usage is incompatible with link-time optimizer
#add_compile_options(-fstack-usage)

# link-time optimizer
add_compile_options(-flto)
add_compile_options(-fwhole-program)


# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
message("Found OpenMP")
add_compile_options(-fopenmp)
endif()

message("COMPILE_OPTIONS:  ${COMPILE_OPTIONS}")


# A common flag is -pipe. This flag has no effect on the generated code, 
# but it makes the compilation process faster. It tells the compiler 
# to use pipes instead of temporary files during the different stages 
# of compilation, which uses more memory. On systems with low memory, 
# GCC might get killed. In those cases do not use this flag. 
add_compile_options(-pipe)



# =======================================
# GPU ACCELERATION
# =======================================




# =======================================
# GPU ACCELERATION
# =======================================

# MAGMA (optional)
if(USE_MAGMA)
set(USE_CUDA ON)
endif(USE_MAGMA)


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
SET(CURSES_USE_NCURSES TRUE)

find_package( Threads )


# FIND MILK
find_package(PkgConfig REQUIRED)
pkg_check_modules(MILK REQUIRED milk>=1.01)
link_directories(${MILK_LIBRARY_DIRS})
include_directories(${MILK_INCLUDE_DIRS})
message("MILK_VERSION = ${MILK_VERSION}")


string(FIND "${MILK_CFLAGS}" "MILK_USE_CUDA=ON" sresult)
if(${sresult} GREATER 0)
message("milk was compiled with cuda")
message("milk will use cuda")
set(USE_CUDA ON)
else()
message("milk was NOT compiled with cuda")
message(WARNING "cacao will NOT use cuda")
set(USE_CUDA OFF)
endif()


string(FIND "${MILK_CFLAGS}" "MILK_USE_MAGMA=ON" sresult)
if(${sresult} GREATER 0)
message("milk was compiled with magma")
message("cacao will use magma")
set(USE_MAGMA ON)
else()
message("milk was NOT compiled with magma")
message(WARNING "cacao will NOT use magma")
set(USE_MAGMA OFF)
endif()



message("====================================================")
message("VERSION  ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
message("====================================================")

message("PROJECT_SOURCE_DIR       = ${PROJECT_SOURCE_DIR}")
message("PROJECT_BINARY_DIR       = ${PROJECT_BINARY_DIR}")


message("CMAKE_INSTALL_PREFIX     = ${CMAKE_INSTALL_PREFIX}")

message("USE_CUDA                 = ${USE_CUDA}")

message("USE_MAGMA                = ${USE_MAGMA}")
if(USE_MAGMA)
message("   MAGMA_LIBRARIES       = ${MAGMA_LIBRARIES}")
endif(USE_MAGMA)

message("EXTRAMODULES             = ${EXTRAMODULES}")

message("====================================================")




#                               
# recursively add libraries to be compiled
# each library compile as SHARED
#

add_subdirectory ( "src/AOloopControl" )
add_subdirectory ( "src/AOloopControl_acquireCalib" )
add_subdirectory ( "src/AOloopControl_compTools" )
add_subdirectory ( "src/AOloopControl_computeCalib" )
add_subdirectory ( "src/AOloopControl_DM" )
add_subdirectory ( "src/AOloopControl_IOtools" )
add_subdirectory ( "src/AOloopControl_perfTest" )
add_subdirectory ( "src/AOloopControl_PredictiveControl" )



#
# Add extra optional modules (list provided by user)
# separator character ";"
#

FOREACH(extramodule ${EXTRAMODULES})
add_subdirectory ( "src/${extramodule}" )
ENDFOREACH()




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

set(BUILD_FLAGS "-DPACKAGE_NAME=\\\"cacao\\\" -DCONFIGDIR=\\\"${PROJECT_SOURCE_DIR}/config\\\" -DSOURCEDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DABSSRCTOPDIR=\\\"${PROJECT_SOURCE_DIR}\\\" -DPACKAGE_BUGREPORT=\\\"https://github.com/cacao-org/cacao/issues\\\"" )


target_link_libraries (cacao PUBLIC
cacaoAOloopControl
cacaoAOloopControlacquireCalib
cacaoAOloopControlcompTools
cacaoAOloopControlcomputeCalib
cacaoAOloopControlDM
cacaoAOloopControlIOtools
cacaoAOloopControlperfTest
cacaoAOloopControlPredictiveControl
${EXTRAMODULES}
${MILK_LIBRARIES}
${GSL_LIBRARIES} 
${CMAKE_THREAD_LIBS_INIT}
-fopenmp
)


if(USE_CUDA)
target_link_libraries (cacao PUBLIC ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})
set(BUILD_FLAGS "${BUILD_FLAGS} -DHAVE_CUDA" )
endif(USE_CUDA)


if(USE_MAGMA)
target_link_libraries (cacao PUBLIC ${MAGMA_LIBRARIES})
set(BUILD_FLAGS "${BUILD_FLAGS} -DHAVE_CUDA -DHAVE_MAGMA" )
endif(USE_MAGMA)




set_target_properties(cacao PROPERTIES COMPILE_FLAGS "${BUILD_FLAGS}" )



message("====================================================")
get_target_property(cacaocompopts cacao COMPILE_OPTIONS)

message("CMAKE_THREAD_LIBS_INIT  :  ${CMAKE_THREAD_LIBS_INIT}")
message("BUILD_FLAGS             :  ${BUILD_FLAGS}")
message("COMPILE_FLAGS           :  ${COMPILE_FLAGS}")
message("COMPILE_OPTIONS         :  ${cacaocompopts}")
message("CMAKE_EXE_LINKER_FLAGS  :  ${CMAKE_EXE_LINKER_FLAGS}")

message("CMAKE_C_FLAGS           :  ${CMAKE_C_FLAGS}")
message("CMAKE_CXX_FLAGS         :  ${CMAKE_CXX_FLAGS}")

message("CMAKE_C_FLAGS_DEBUG           : ${CMAKE_C_FLAGS_DEBUG}")
message("CMAKE_C_FLAGS_RELEASE         : ${CMAKE_C_FLAGS_RELEASE}")
message("CMAKE_C_FLAGS_RELWITHDEBINFO  : ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
message("CMAKE_C_FLAGS_MINSIZEREL      : ${CMAKE_C_FLAGS_MINSIZEREL}")

message("====================================================")


set(PROGRAM_PERMISSIONS_DEFAULT
    OWNER_WRITE OWNER_READ OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE)

install(TARGETS cacao DESTINATION bin PERMISSIONS ${PROGRAM_PERMISSIONS_DEFAULT} SETUID)


set(CMAKE_INSTALL_PREFIX_LINK "/usr/local/${PROJECT_NAME}")
if(INSTALLMAKEDEFAULT)
    install(CODE "
    EXECUTE_PROCESS(COMMAND ln -sf ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_PREFIX_LINK})
    ")
endif(INSTALLMAKEDEFAULT)

# Note: may need to write "/usr/local/magma/lib" to:
# /etc/ld.so.conf.d/magma.conf










