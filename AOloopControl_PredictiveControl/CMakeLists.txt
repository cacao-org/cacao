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
# Keep IEEE finiteness checks intact in selflearn_core.c even under -Ofast
set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/selflearn_core.c"
    PROPERTIES COMPILE_OPTIONS "-fno-finite-math-only"
)

# Optional libgric integration for online GRIC state-space clustering (selflearn)
find_path(GRIC_INCLUDE_DIR gric/gric.h
          HINTS "${INSTALLDIR}/include" "${CMAKE_INSTALL_PREFIX}/include"
                "$ENV{MILK_INSTALL_ROOT}/include" "/usr/local/include")
find_library(GRIC_LIBRARY NAMES gric
             HINTS "${INSTALLDIR}/lib" "${CMAKE_INSTALL_PREFIX}/lib"
                   "$ENV{MILK_INSTALL_ROOT}/lib" "/usr/local/lib")
if(GRIC_INCLUDE_DIR AND GRIC_LIBRARY)
    message(STATUS " ${LIBNAME}: linking libgric (${GRIC_LIBRARY})")
    milk_runnable_module_usage(${LIBNAME} DEFINITIONS HAVE_LIBGRIC=1
                               INCLUDES "${GRIC_INCLUDE_DIR}")
    target_link_libraries(${LIBNAME} PUBLIC "${GRIC_LIBRARY}")
else()
    message(STATUS " ${LIBNAME}: libgric not found; selflearn will use built-in leader clustering")
endif()

milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC milklinoptimtools cacaoAOloopControl)
# ==================

install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)
