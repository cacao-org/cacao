set(LIBNAME "cacaopyramidWFStools")
set(SRCNAME "pyramidWFStools")

message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")

set(LINKLIBS
	CLIcore
	milkinfo
	cacaoAOloopControl
)

# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SOURCEFILES})

# note: set to PUBLIC if other modules link to this one
target_include_directories(${LIBNAME} PRIVATE ${PROJECT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
