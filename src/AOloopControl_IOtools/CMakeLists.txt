project(lib_AOloopControl_IOtools_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_IOtools SHARED AOloopControl_IOtools.c)

install(TARGETS AOloopControl_IOtools DESTINATION lib)
install(FILES AOloopControl_IOtools.h DESTINATION include)

