set(LIBNAME "cacaoAOloopControlPredictiveControl")
set(SRCNAME "AOloopControl_PredictiveControl")


project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED
${SRCNAME}.c
${SRCNAME}.h
AOloopControl_PredictiveControl_builPFloop_WatchInput.c
AOloopControl_PredictiveControl_testPredictiveFilter.c
AOloopControl_PredictiveControl_setPFsimpleAve.c
AOloopControl_PredictiveControl_mapPredictiveFilter.c
)

target_link_libraries(${LIBNAME} PUBLIC CLIcore milklinoptimtools cacaoAOloopControl)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
