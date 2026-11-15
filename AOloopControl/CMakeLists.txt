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

milk_add_runnable_module(${LIBNAME} SOURCES ${SOURCEFILES} LINK_LIBS milklinoptimtools_static
                         milkstatistic_static)
target_include_directories(${LIBNAME}_objs PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC milklinoptimtools milkstatistic)
# ==================

install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

# Standalone engine-tier ANSI TUI (no CLIcore / ncurses linkage)
add_executable(cacao-modalstatsTUI-bin tui/modalstatsTUI.c)
target_include_directories(cacao-modalstatsTUI-bin PRIVATE
    ${CMAKE_SOURCE_DIR}/src/tui
    ${CMAKE_SOURCE_DIR}/src/engine/libfps
    ${PROJECT_SOURCE_DIR}/tui)
target_link_libraries(cacao-modalstatsTUI-bin PRIVATE ImageStreamIO m)
milk_pgo_target(cacao-modalstatsTUI-bin)
milk_lto_target(cacao-modalstatsTUI-bin)
install(TARGETS cacao-modalstatsTUI-bin DESTINATION bin)
