set(LIBNAME "cacaopyramidWFStools")
project(lib_${LIBNAME}_project)

# list source files (.c) other than modulename.c
set(SOURCEFILES
  pyWFSgridmatch.c
)


# SRCNAME is current directory (last subdir)
set(NAME0 "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "/" " " NAME1 ${NAME0})
string(REGEX MATCH "[a-zA-Z0-9_]+$" SRCNAME "${NAME0}")
message(" SRCNAME = ${SRCNAME}")

# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
#
add_library(${LIBNAME} SHARED ${SRCNAME}.c ${SOURCEFILES})



# list include files (.h) that should be installed on system
set(INCLUDEFILES
)

# list scripts that should be installed on system
set(SCRIPTFILES
)

set(LINKLIBS
	CLIcore
	milkinfo
	cacaoAOloopControl
)

#If not using MKL, check if OpenBLAS has lapacke symbols.  Include lapacke if not.
if(NOT MKL_FOUND)

    check_library_exists(openblas LAPACKE_sgemlq "" DONT_NEED_LAPACKE)

    if(DONT_NEED_LAPACKE)
       message("-- MKL NOT FOUND")
       message("   LINKING OpenBLAS")
    else()
       message("-- MKL NOT FOUND")
       message("   LINKING LAPACKE")
       list(APPEND LINKLIBS lapacke)
    endif()

endif()




# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================



# LIBNAME constructed by removing spaces
#string(REPLACE "_" "" LIBNAME "${SRCNAME}")
#message(" LIBNAME = ${LIBNAME}")


# note: set to PUBLIC if other modules link to this one
target_include_directories(${LIBNAME} PRIVATE ${PROJECT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTFILES} DESTINATION bin)
