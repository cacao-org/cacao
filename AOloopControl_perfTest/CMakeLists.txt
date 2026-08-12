set(LIBNAME "cacaoAOloopControlperfTest")
set(SRCNAME "AOloopControl_perfTest")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SOURCEFILES})
target_include_directories(
  ${LIBNAME} PRIVATE $<TARGET_PROPERTY:CLIcore,INTERFACE_INCLUDE_DIRECTORIES>)
# sources #include cross-plugin headers (e.g. statistic/statistic.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/cacao-src)

target_link_libraries(${LIBNAME} PUBLIC CLIcore milkstatistic cacaoAOloopControl)

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(cacaopt-compRMsens compRMsensitivity.c)
add_cacao_standalone(cacaopt-mlat mlat.c)
add_cacao_standalone(cacaopt-zoptsearch zoptsearch.c)
add_cacao_standalone(cacaopt-wfsroptsel wfsrefoptimselect.c)
add_cacao_standalone_plugins(cacaopt-slogtsample streamlogtimesample.c)
add_cacao_standalone(cacaopt-mlatdecode mlat_decode.c)

# ---- specialized standalone overrides (TODO: automate) ----
# "_compute" library variants no longer exist; link the regular libs.
target_link_libraries(cacao-fpsexec-cacaopt-mlat PUBLIC milkstatistic)
target_link_libraries(cacao-fpsexec-cacaopt-zoptsearch PUBLIC milkstatistic)
target_link_libraries(cacao-fpsexec-cacaopt-slogtsample PUBLIC milkstatistic)
target_link_libraries(cacao-fpsexec-cacaopt-mlatdecode PUBLIC milkstatistic)
