project(lib_AOloopControl_IOtools_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl_IOtools SHARED AOloopControl_IOtools.c AOloopControl_IOtools.h  AOloopControl_IOtools_camerainput.c  AOloopControl_IOtools_datastream_processing.c  AOloopControl_IOtools_load_image_sharedmem.c)

install(TARGETS AOloopControl_IOtools DESTINATION lib)
install(FILES AOloopControl_IOtools.h DESTINATION include)

