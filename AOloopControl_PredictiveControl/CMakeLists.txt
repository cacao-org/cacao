set(LIBNAME "cacaoAOloopControlPredictiveControl")
set(SRCNAME "AOloopControl_PredictiveControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

project(lib_${LIBNAME}_project)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/..")

milk_add_runnable_module(${LIBNAME} SOURCES ${SOURCEFILES}
                         LINK_LIBS milklinoptimtools_static cacaoAOloopControl_static)
# sources #include cross-plugin headers (e.g. linopt_imtools/*.h) directly
target_include_directories(${LIBNAME}_objs PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC milklinoptimtools cacaoAOloopControl)
# ==================

install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)
