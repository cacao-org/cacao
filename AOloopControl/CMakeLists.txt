set(LIBNAME "cacaoAOloopControl")
set(SRCNAME "AOloopControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")

file(GLOB_RECURSE SCRIPTS LIST_DIRECTORIES false "scripts/*")
list(FILTER SCRIPTS EXCLUDE REGEX "/\\..+")

set(LINKLIBS
	CLIcore
	milklinoptimtools
)

# DEFAULT SETTINGS Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SOURCEFILES})
target_include_directories(
  ${LIBNAME} PRIVATE $<TARGET_PROPERTY:CLIcore,INTERFACE_INCLUDE_DIRECTORIES>)
# sources #include cross-plugin headers (e.g. statistic/statistic.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)

target_link_libraries(${LIBNAME} PUBLIC ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(cacao-zonalfilter zonalfilter.c)
add_cacao_standalone(mfilt modalfilter.c)
