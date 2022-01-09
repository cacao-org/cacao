set(LIBNAME "cacaoAOloopControlcomputeCalib")
set(SRCNAME "AOloopControl_computeCalib")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED
${SRCNAME}.c
AOloopControl_computeCalib.h
AOloopControl_computeCalib_CM.c
AOloopControl_computeCalib_dm.c
AOloopControl_computeCalib_Hadamard.c
AOloopControl_computeCalib_loDMmodes.c
AOloopControl_computeCalib_modes.c
AOloopControl_computeCalib_processRM.c
compute_control_modes.c
compute_control_modes.h
)



if(USE_CUDA)
find_package(CUDA 9.0 REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
message("---- CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")
message("---- CUDA_LIBRARIES    = ${CUDA_LIBRARIES}" )
target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA)
target_link_libraries(${LIBNAME} PUBLIC ${CUDA_LIBRARIES})

if(USE_MAGMA)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAGMA REQUIRED magma)
message("---- MAGMA_LIBRARY_DIRS =  ${MAGMA_LIBRARY_DIRS}")
message("---- MAGMA_LIBRARIES    =  ${MAGMA_LIBRARIES}" )
message("---- MAGMA_CFLAGS_OTHER =  ${MAGMA_CFLAGS_OTHER}" )
target_include_directories(${LIBNAME} PUBLIC ${MAGMA_INCLUDE_DIRS})
target_link_directories(${LIBNAME} PUBLIC ${MAGMA_LIBRARY_DIRS})
target_link_libraries(${LIBNAME} PUBLIC ${MAGMA_LIBRARIES})
target_compile_options(${LIBNAME} PUBLIC -DHAVE_MAGMA ${MAGMA_CFLAGS_OTHER})
endif(USE_MAGMA)

endif(USE_CUDA)

target_include_directories(${LIBNAME} PUBLIC ${CUDA_INCLUDE_DIRS} ${MAGMA_INCLUDE_DIRS})
target_link_libraries(${LIBNAME} PUBLIC CLIcore milkinfo cacaoAOloopControl)



install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
