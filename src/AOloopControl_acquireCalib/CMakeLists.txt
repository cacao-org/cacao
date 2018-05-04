project(lib_AOloopControl_acquireCalib_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_acquireCalib SHARED AOloopControl_acquireCalib.c)

install(TARGETS AOloopControl_acquireCalib DESTINATION lib)
install(FILES AOloopControl_acquireCalib.h DESTINATION include)

