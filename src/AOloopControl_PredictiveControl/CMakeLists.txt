set(LIBNAME "cacaoAOloopControlPredictiveControl")
set(SRCNAME "AOloopControl_PredictiveControl")


project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(${LIBNAME} SHARED 
/${SRCNAME}).c 
/${SRCNAME}).h 
AOloopControl_PredictiveControl_builPFloop_WatchInput.c 
AOloopControl_PredictiveControl_testPredictiveFilter.c 
AOloopControl_PredictiveControl_setPFsimpleAve.c
AOloopControl_PredictiveControl_mapPredictiveFilter.c
)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES /${SRCNAME}).h DESTINATION include/${SRCNAME})

