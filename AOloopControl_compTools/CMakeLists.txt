set(LIBNAME "cacaoAOloopControlcompTools")
set(SRCNAME "AOloopControl_compTools")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SOURCEFILES})

target_include_directories(
  ${LIBNAME} PRIVATE $<TARGET_PROPERTY:CLIcore,INTERFACE_INCLUDE_DIRECTORIES>)
# sources may #include cross-plugin headers (e.g. statistic/statistic.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)

target_link_libraries(${LIBNAME} PUBLIC CLIcore cacaoAOloopControl)

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
