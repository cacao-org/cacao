set(LIBNAME "cacaoAOloopControlacquireCalib")
set(SRCNAME "AOloopControl_acquireCalib")

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SRCNAME}.c)

target_link_libraries(${LIBNAME} PUBLIC CLIcore milkinfo cacaoAOloopControl)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
