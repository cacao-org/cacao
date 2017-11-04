#
# Process this file with autoconf to produce a configure script.
#

m4_define([PROGNAME], CACAO)
m4_define([PVERSION], 3.4.00)

AC_INIT([PROGNAME], [PVERSION], [oliv.guyon@gmail.com], [PROGNAME])

AC_PREREQ(2.61)
#AC_CONFIG_HEADER([config.h])

# sanity check a file to see if we're really in the build directory
AC_CONFIG_SRCDIR([src/CLImain.c])

AC_CONFIG_MACRO_DIR([m4])




# Init Automake
AM_INIT_AUTOMAKE([subdir-objects foreign -Wall])
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])
# Init Libtool
LT_INIT

# find the c-complier
#AC_PROG_CC
AM_PROG_CC_C_O

# check for openMP
AC_OPENMP

# Support for Bison and Flex
AC_PROG_YACC
AM_PROG_LEX

# Support for pthreads
ACX_PTHREAD

#LIBS="$PTHREAD_LIBS $LIBS"
#CFLAGS="$CFLAGS $PTHREAD_CFLAGS"
#CC="$PTHREAD_CC"


#
# Coerce @prefix@ default from /usr/local/ to fullpath of dir configure is run in
AC_PREFIX_DEFAULT(`readlink -f .`)




#PKG_CHECK_MODULES([CHECK], [check >= 0.9.10])





CPPFLAGS="$CPPFLAGS -I/usr/local/include/"

#LDFLAGS="$LDFLAGS -L/usr/lib/"

AC_ARG_ENABLE(fftwmt,
 [AS_HELP_STRING(  --enable-fftwmt         Enable FFTW multi-thread )],
 [ fftwmt=yes
 ])




#
# Look for libraries 
#


AC_CHECK_LIB([m],[main],
		[],
		[ echo "Cannot find required library:math";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:math"]);
		  exit 0;
		])


AC_CHECK_LIB([dl],[main],
		[],
		[ echo "Cannot find required library:dl";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:dl"]);
		  exit 0;
		])


case "$host_os" in
*darwin* | *rhapsody* | *macosx*)
     AC_DEFINE_UNQUOTED(OSX,1,[This is an OS X system])
     ;;
*)
AC_CHECK_LIB([rt], [clock],
		[],
		[ echo "Cannot find required library:rt";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:rt"]);
		  exit 0;
		])
;;
esac


AC_CHECK_LIB([pthread], [pthread_create].
		[],
		[ echo "Cannot find required library:pthread";
 		cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:pthread"]);
		  exit 0;
		])

AC_CHECK_LIB([gslcblas],[main],
		[],
		[ echo "Cannot find required library:gslcblas";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:gslcblas"]);
		  exit 0;
		])


AC_CHECK_LIB([gsl],[main],
		[],
		[ echo "Cannot find required library:gsl";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:gsl"]);
		  exit 0;
		])


AC_CHECK_LIB([cfitsio],[main],
		[],
		[ echo "Cannot find required static library:fitsio";
		  echo "did you install libcfitsio3-dev package ?";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:fitsio"]);
		  exit 0;
		])
		
		
AC_SEARCH_LIBS( [readline], [readline], 
                [AC_DEFINE([HAVE_READLINE_H], 1, "Define to 1 if you have <readline.h>") ], 
                [ echo "Cannot find required static library:readline";
		  echo "did you install libreadline<#>-dev package ?";
                  cat doc/configure.howto;
                  AC_MSG_FAILURE(["FATAL...Cannot find required library:readlline"]); 
                  exit 0;
                ])

AC_CHECK_LIB( [ncurses], [initscr], 
		[],
		[ echo "Cannot find required library:ncurses";
 		  echo "did you install ncurses package ?";
                  cat doc/configure.howto;
                  AC_MSG_FAILURE(["FATAL...Cannot find required library:ncurses"]); 
                ]
)

AC_CHECK_LIB( [fftw3f], [main], 
                [], 
                [ echo "Cannot find required library:fftw3f";
 		  echo "did you install fftw3f-dev package ?";
                  cat doc/configure.howto;
                  AC_MSG_FAILURE(["FATAL...Cannot find required library:fftw3f"]); 
                ]
)

AC_CHECK_LIB( [fftw3], [main], 
                [], 
                [ echo "Cannot find required library:fftw3";
		  echo "did you install fftw3-dev package ?";
                  cat doc/configure.howto;
                  AC_MSG_FAILURE(["FATAL...Cannot find required library:fftw3"]); 
                ]
)

AC_CHECK_LIB([gomp], [omp_get_num_threads])

#
# fftw multi-thread
#	
if   test "x$fftwmt" = "xyes" ; then
	AC_CHECK_LIB( [fftw3f_threads], [main],
        	        [], 
        	        [ echo "Cannot find required library:fftw3f_threads";
        	          cat doc/configure.howto;
        	          AC_MSG_FAILURE(["FATAL...Cannot find required library:fftw3f_threads"]); 
        	        ]
	)	
	AC_CHECK_LIB( [fftw3_threads], [main],
        	        [], 
        	        [ echo "Cannot find required library:fftw3_threads";
        	          cat doc/configure.howto;
        	          AC_MSG_FAILURE(["FATAL...Cannot find required library:fftw3f_threads"]); 
        	        ]
	)	
	
	AC_CHECK_LIB([pthread], [pthread_create],
		[],
		[ echo "Cannot find required library:pthread";
		  cat doc/configure.howto;
		  AC_MSG_FAILURE(["FATAL...Cannot find required library:pthread"]);
		  exit 0;
		])
	
	AC_DEFINE(FFTWMT)
fi


#AC_CHECK_LIB([usb], [usb_close], ,[AC_MSG_ERROR(
# [libusb library not found.  Check with vendor or see http://libusb.sourceforge.net])],
# [$USBLDFLAGS])


#AC_SEARCH_LIBS( [bob],      
#                [tom], 
#                [AC_DEFINE([HAVE_BOB_H], 1,
#                           "Define to 1 if you have <readline.h>")] , 
#                [AC_MSG_ERROR([$str]); exit 0] )

#AC_CHECK_HEADERS( [bob.h], 
#                  [], 
#                  [ echo "Cannot find required header:bob.h";
#                    cat doc/configure.howto;
#                    AC_MSG_FAILURE(["Cannot find required header:bob.h"]);
#                  ]
#)  

AC_CHECK_HEADERS( [fftw3.h], 
                  [], 
                  [ echo "Cannot find required header:fftw3.h";
                    cat doc/configure.howto;
                    AC_MSG_FAILURE(["Cannot find required header:fftw3.h"]);
                  ]
)  

AC_CHECK_HEADERS( [fitsio.h], 
                  [], 
                  [ echo "Cannot find required header:fitsio.h";
                    cat doc/configure.howto;
                    AC_MSG_FAILURE(["Cannot find required header:fitsio.h"]);
                  ]
)  




#AC_MSG_NOTICE([Looking for special libraries])
#AC_INCLUDES_DEFAULT
#AC_CHECK_LIB(gsl,gsl_ran_gaussian_ziggurat, , echo "FATAL..."; exit 0; )
#AC_PROG_LEX
#AC_PROG_YACC
#AC_PROG_INSTALL

#AC_MSG_CHECKING (feature-description)
#AC_MSG_NOTICE([eg:checking if stack overflow is detectable])
#AC_MSG_FAILURE (error-description, [exit-status])












#=================================================
# Setup default CUDA paths
#CUDA_INSTALL_PATH="/usr/local/cuda"
#CUDA_SDK_INSTALL_PATH="/usr/local/NVIDIA_GPU_Computing_SDK"

# Check: enable CUDA?
AC_MSG_CHECKING([whether to use CUDA library])
AC_ARG_ENABLE(cuda,
    [AS_HELP_STRING(--enable-cuda,enable CUDA support)],
    cudaval="yes",
    cudaval="no")
AC_MSG_RESULT([$cudaval])

# check to see if CUDA been selected
if test "$cudaval" == "no"; then
    AC_MSG_WARN([Not using CUDA!])
fi

# Setup custom CUDA paths
AC_ARG_WITH([cuda],
   [AS_HELP_STRING(
        [--with-cuda=PATH],
        [prefix where CUDA is installed @<:@default=auto@:>@])],
   [CUDA_INSTALL_PATH=$withval],
   [with_cuda=auto])
AC_ARG_WITH([cuda-sdk],
   [AS_HELP_STRING(
        [--with-cuda-sdk=PATH],
        [prefix where CUDA SDK is installed @<:@default=auto@:>@])],
   [CUDA_SDK_INSTALL_PATH=$withval],
   [with_cuda_sdk=auto])

# Setup nvcc flags
if test  "$cudaval" = "yes"; then
    NVCCFLAGS="$NVCCFLAGS"
    CUDA_CFLAGS="$CUDA_CFLAGS"
    CUDA_CFLAGS="$CUDA_CFLAGS -I$CUDA_SDK_INSTALL_PATH/C/common/inc/"
    CUDA_CFLAGS="$CUDA_CFLAGS -I$CUDA_INSTALL_PATH/include -I/usr/local/cuda/include/"
    CUDA_LDFLAGS="-L$CUDA_INSTALL_PATH/lib64"
    CUDA_LIBS="-lcuda -lcudart -lcufft -lcublas -lcusolver"
    NVCCFLAGS="$NVCCFLAGS $CUDA_CFLAGS $CUDA_LDFLAGS $CUDA_LIBS"
    NVCC="nvcc"
    AC_SUBST(NVCC)
    AC_SUBST(NVCCFLAGS)
    AC_DEFINE([HAVE_CUDA],[1],[Defined if CUDA should be used])
    eval "ARRAY${N}='-D_CUDA'"
    N=`expr $N + 1`
    LIBS="$LIBS $CUDA_LIBS"
    LDFLAGS="$LDFLAGS $CUDA_LDFLAGS -L/usr/local/cuda/lib64/"
    CFLAGS="$CFLAGS $CUDA_CFLAGS"
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH -L/usr/local/cuda/lib64/"
fi




#=================================================
# Setup MAGMA


# Check: enable MAGMA?
AC_MSG_CHECKING([whether to use MAGMA library])
AC_ARG_ENABLE(magma,
    [AS_HELP_STRING(--enable-magma,enable MAGMA support)],
    magmaval="yes",
    magmaval="no")
AC_MSG_RESULT([$magmaval])

# check to see if CUDA been selected
if test "$magmaval" == "no"; then
    AC_MSG_WARN([Not using MAGMA!])
fi

if test  "$magmaval" = "yes"; then
	MAGMA_LIBS="-lmagma "
	MAGMA_CFLAGS="-DADD_ -I/usr/local/magma/include/"
	MAGMA_LDFLAGS="-L/usr/local/magma/lib/"
	AC_DEFINE([HAVE_MAGMA],[1],[Defined if MAGMA should be used])
	eval "ARRAY${N}='-D_MAGMA'"
    N=`expr $N + 1`
    LIBS="$LIBS $MAGMA_LIBS"
    LDFLAGS="$LDFLAGS $MAGMA_LDFLAGS"
    CFLAGS="$CFLAGS $MAGMA_CFLAGS"
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH -L/usr/local/magma/lib/"
fi




# Create our own a symbol for execname until we can find the right one
EXECNAME=PROGNAME
AC_SUBST(EXECNAME)

LIBS="$LIBS -lpthread"

#if test  "$cudaval" = "yes"; then
#	LIBS="$LIBS -lcuda -lcudart -lcufft"
#fi

AC_CONFIG_FILES(Makefile \
src/Makefile \
src/ImageStreamIO/Makefile \
src/00CORE/Makefile \
src/COREMOD_arith/Makefile \
src/COREMOD_iofits/Makefile \
src/COREMOD_memory/Makefile \
src/COREMOD_tools/Makefile \
src/info/Makefile \
src/fft/Makefile \
src/statistic/Makefile \
src/linopt_imtools/Makefile \
src/image_gen/Makefile \
src/image_filter/Makefile \
src/image_basic/Makefile \
src/ZernikePolyn/Makefile \
src/image_format/Makefile \
src/img_reduce/Makefile \
src/psf/Makefile \
src/cudacomp/Makefile \
src/kdtree/Makefile \
src/linARfilterPred/Makefile \
src/AOloopControl/Makefile \
src/AOloopControl_acquireCalib/Makefile \
src/AOloopControl_compTools/Makefile \
src/AOloopControl_computeCalib/Makefile \
src/AOloopControl_DM/Makefile \
src/AOloopControl_IOtools/Makefile \
src/AOloopControl_perfTest/Makefile \
src/AOloopControl_PredictiveControl/Makefile \
src/FPAOloopControl/Makefile \

AC_OUTPUT

echo ""
echo "                       Configure Summary"
echo "                       ================="
echo ""
echo "Compiling Options:"
echo "------------------"
echo "                     C Compiler: $CC"
echo "                         CFLAGS: $CFLAGS"

echo "                        LDFLAGS: $LDFLAGS"
echo "                           LIBS: $LIBS"

