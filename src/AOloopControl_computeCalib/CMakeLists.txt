set(LIBNAME "cacaoAOloopControlcomputeCalib")
set(SRCNAME "AOloopControl_computeCalib")

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(${LIBNAME} SHARED 
${SRCNAME}.c  
AOloopControl_computeCalib.h 
AOloopControl_computeCalib_CM.c  
AOloopControl_computeCalib_dm.c  
AOloopControl_computeCalib_Hadamard.c  
AOloopControl_computeCalib_loDMmodes.c  
AOloopControl_computeCalib_modes.c  
AOloopControl_computeCalib_processRM.c
)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

