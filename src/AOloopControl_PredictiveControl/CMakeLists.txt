project(lib_AOloopControl_PredictiveControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(AOloopControl_PredictiveControl SHARED AOloopControl_PredictiveControl.c)

install(TARGETS AOloopControl_PredictiveControl DESTINATION lib)
install(FILES AOloopControl_PredictiveControl.h DESTINATION include)

