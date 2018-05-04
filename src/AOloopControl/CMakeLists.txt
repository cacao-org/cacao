project(lib_AOloopControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl SHARED AOloopControl.c)

install(TARGETS AOloopControl DESTINATION lib)
install(FILES AOloopControl.h DESTINATION include)

