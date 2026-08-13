set(LIBNAME "cacaoAOloopControlacquireCalib")
set(SRCNAME "AOloopControl_acquireCalib")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")




file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

set(LINKLIBS
	CLIcore
    milkinfo
    cacaoAOloopControl
)




# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
#
add_library(${LIBNAME} SHARED ${SOURCEFILES})
target_include_directories(
  ${LIBNAME} PRIVATE $<TARGET_PROPERTY:CLIcore,INTERFACE_INCLUDE_DIRECTORIES>)
# sources #include cross-plugin headers (e.g. image_gen/*.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)

target_include_directories(${LIBNAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone_plugins(cacaoac-measlinresp measure_linear_resp.c)

# ---- specialized standalone overrides (TODO: automate) ----
# "_compute" library variants no longer exist; link the regular libs.
target_link_libraries(cacao-fpsexec-cacaoac-measlinresp PUBLIC milkCOREMODiofits)
