project(lib_FPAOloopControl_project)

add_library(FPAOloopControl SHARED FPAOloopControl.c)

target_link_libraries(FPAOloopControl PUBLIC CLIcore milklinoptimtools cacaoAOloopControl)

install(TARGETS FPAOloopControl DESTINATION lib)
install(FILES FPAOloopControl.h DESTINATION include)
