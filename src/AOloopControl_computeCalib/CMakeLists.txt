project(lib_AOloopControl_computeCalib_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl_computeCalib SHARED AOloopControl_computeCalib.c  AOloopControl_computeCalib.h AOloopControl_computeCalib_CM.c  AOloopControl_computeCalib_dm.c  AOloopControl_computeCalib_Hadamard.c  AOloopControl_computeCalib_loDMmodes.c  AOloopControl_computeCalib_modes.c  AOloopControl_computeCalib_processRM.c)

install(TARGETS AOloopControl_computeCalib DESTINATION lib)
install(FILES AOloopControl_computeCalib.h DESTINATION include)

