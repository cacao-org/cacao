project(lib_AOloopControl_computeCalib_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_computeCalib SHARED AOloopControl_computeCalib.c)

install(TARGETS AOloopControl_computeCalib DESTINATION lib)
install(FILES AOloopControl_computeCalib.h DESTINATION include)

