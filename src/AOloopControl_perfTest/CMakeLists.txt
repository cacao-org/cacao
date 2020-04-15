set(LIBNAME "cacaoAOloopControlperfTest")
set(SRCNAME "AOloopControl_perfTest")


project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(${LIBNAME} SHARED 
${SRCNAME}.c 
${SRCNAME}.h 
AOloopControl_perfTest_DM.c  
AOloopControl_perfTest_status.c 
AOloopControl_perfTest_LinSim.c
)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

