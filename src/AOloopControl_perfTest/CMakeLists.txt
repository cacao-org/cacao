project(lib_AOloopControl_perfTest_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_perfTest SHARED AOloopControl_perfTest.c AOloopControl_perfTest.h AOloopControl_perfTest_DM.c  AOloopControl_perfTest_status.c AOloopControl_perfTest_LinSim.c)

install(TARGETS AOloopControl_perfTest DESTINATION lib)
install(FILES AOloopControl_perfTest.h DESTINATION include)

