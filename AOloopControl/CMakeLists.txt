set(LIBNAME "cacaoAOloopControl")
set(SRCNAME "AOloopControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")

file(GLOB_RECURSE SCRIPTS LIST_DIRECTORIES false "scripts/*")
list(FILTER SCRIPTS EXCLUDE REGEX "/\\..+")

project(lib_${LIBNAME}_project)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/..")

milk_add_runnable_module(${LIBNAME} SOURCES ${SOURCEFILES} LINK_LIBS milklinoptimtools_static)
target_include_directories(${LIBNAME}_objs PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC milklinoptimtools)
# ==================

install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)
