set(LIBNAME "cacaoAOloopControlcompTools")
set(SRCNAME "AOloopControl_compTools")

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SRCNAME}.c)

target_link_libraries(${LIBNAME} PUBLIC CLIcore cacaoAOloopControl)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
