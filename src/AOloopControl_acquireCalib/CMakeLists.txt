set(LIBNAME "cacaoAOloopControlacquireCalib")
set(SRCNAME "AOloopControl_acquireCalib")

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")
add_library(${LIBNAME} SHARED ${SRCNAME}.c)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

