project(lib_FPAOloopControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(FPAOloopControl SHARED FPAOloopControl.c)

install(TARGETS FPAOloopControl DESTINATION lib)
install(FILES FPAOloopControl.h DESTINATION include)

