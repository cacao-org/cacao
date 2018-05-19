project(lib_AOloopControl_PredictiveControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl_PredictiveControl SHARED 
AOloopControl_PredictiveControl.c 
AOloopControl_PredictiveControl.h 
AOloopControl_PredictiveControl_builPFloop_WatchInput.c 
AOloopControl_PredictiveControl_testPredictiveFilter.c 
AOloopControl_PredictiveControl_setPFsimpleAve.c
AOloopControl_PredictiveControl_mapPredictiveFilter.c
)

install(TARGETS AOloopControl_PredictiveControl DESTINATION lib)
install(FILES AOloopControl_PredictiveControl.h DESTINATION include)

