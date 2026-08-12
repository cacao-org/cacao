set(LIBNAME "cacaoAOloopControlDM")
set(SRCNAME "AOloopControl_DM")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

set(LINKLIBS
	CLIcore
    milkfft
    milkimagegen
    milkimagefilter
    cacaoAOloopControl
)




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

target_include_directories(${LIBNAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(dmcomb AOloopControl_DM_comb.c)
add_cacao_standalone_plugins(dmturb DMturbulence.c fft imagegen statistic)
