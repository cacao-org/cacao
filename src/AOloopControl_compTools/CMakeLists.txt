project(lib_AOloopControl_compTools_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_compTools SHARED AOloopControl_compTools.c)

install(TARGETS AOloopControl_compTools DESTINATION lib)
install(FILES AOloopControl_compTools.h DESTINATION include)

