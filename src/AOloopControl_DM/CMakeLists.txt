project(lib_AOloopControl_DM_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_DM SHARED AOloopControl_DM.c)

install(TARGETS AOloopControl_DM DESTINATION lib)
install(FILES AOloopControl_DM.h DESTINATION include)

