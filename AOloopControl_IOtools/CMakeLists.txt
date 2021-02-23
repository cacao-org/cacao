set(LIBNAME "cacaoAOloopControlIOtools")
set(SRCNAME "AOloopControl_IOtools")


project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SRCNAME}.c
${SRCNAME}.h
AOloopControl_IOtools_camerainput.c
AOloopControl_IOtools_datastream_processing.c
AOloopControl_IOtools_load_image_sharedmem.c
AOloopControl_IOtools_RTLOGsave.c
)

target_link_libraries(${LIBNAME} PUBLIC CLIcore milkinfo cacaoAOloopControl)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
