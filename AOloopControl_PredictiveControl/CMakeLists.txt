set(LIBNAME "cacaoAOloopControlPredictiveControl")
set(SRCNAME "AOloopControl_PredictiveControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

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


install(PROGRAMS
    scripts/cacao-fpslistadd-APPLYPF
    scripts/cacao-fpslistadd-MKPF
    DESTINATION bin)


install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
