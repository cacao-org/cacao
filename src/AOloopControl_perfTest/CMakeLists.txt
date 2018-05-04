project(lib_AOloopControl_perfTest_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_perfTest SHARED AOloopControl_perfTest.c)

install(TARGETS AOloopControl_perfTest DESTINATION lib)
install(FILES AOloopControl_perfTest.h DESTINATION include)

