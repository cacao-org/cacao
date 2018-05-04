project(lib_AOloopControl_DM_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl_DM SHARED AOloopControl_DM.c AOloopControl_DM.h AOloopControl_DM_config.c  AOloopControl_DM_init.c  AOloopControl_DM_runtimecomp.c  AOloopControl_DM_toolbox.c  AOloopControl_DM_TTcircle_astrogrid.c  AOloopControl_DM_turbusimul.c )

install(TARGETS AOloopControl_DM DESTINATION lib)
install(FILES AOloopControl_DM.h DESTINATION include)

