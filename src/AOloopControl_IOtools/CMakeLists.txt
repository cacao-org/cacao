set(LIBNAME "cacaoAOloopControlIOtools")
set(SRCNAME "AOloopControl_IOtools")


project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(${LIBNAME} SHARED ${SRCNAME}.c 
${SRCNAME}.h  
AOloopControl_IOtools_camerainput.c  
AOloopControl_IOtools_datastream_processing.c  
AOloopControl_IOtools_load_image_sharedmem.c
AOloopControl_IOtools_RTLOGsave.c
)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

