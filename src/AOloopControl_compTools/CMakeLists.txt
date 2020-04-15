set(LIBNAME "cacaoAOloopControlcompTools")
set(SRCNAME "AOloopControl_compTools")

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(${LIBNAME} SHARED ${SRCNAME}.c)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

