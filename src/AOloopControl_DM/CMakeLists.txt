set(LIBNAME "cacaoAOloopControlDM")
set(SRCNAME "AOloopControl_DM")

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(${LIBNAME} SHARED
${SRCNAME}.c
${SRCNAME}.h
AOloopControl_DM_config.c
AOloopControl_DM_init.c
AOloopControl_DM_runtimecomp.c
AOloopControl_DM_toolbox.c
AOloopControl_DM_TTcircle_astrogrid.c
AOloopControl_DM_turbusimul.c
)

target_link_libraries(${LIBNAME} PUBLIC CLIcore)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
