project(lib_AOloopControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl SHARED AOloopControl.c AOloopControl.h AOloopControl_wfs.c AOloopControl_dm.c AOloopControl_aorun.c AOloopControl_loop_ctr.c AOloopControl_loop_onoff.c AOloopControl_dmwrite.c AOloopControl_autotune.c AOloopControl_arpf_onoff.c AOloopControl_time_param.c AOloopControl_loop_param.c AOloopControl_fpspeckle_mod.c AOloopControl_process_files.c AOloopControl_loadconfigure.c AOloopControl_initmem.c AOloopControl_read_param.c)

install(TARGETS AOloopControl DESTINATION lib)
install(FILES AOloopControl.h DESTINATION include)

