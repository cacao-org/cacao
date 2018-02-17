# cacao Initial Setup {#page_cacao_Initial_Setup}



## Installing and configuring the cacao package

Include recommended high performance compile flags for faster execution speed:

	./configure CFLAGS='-Ofast -march=native'

If you have installed CUDA and MAGMA libraries:

	./configure CFLAGS='-Ofast -march=native' --enable-cuda --enable-magma

A pre-written high performance configure script is available:

	./configure_highperf


The executable is built with:

	make
	make install
	
The executable is `./bin/cacao`



## System configuration (to be added to .bashrc file)

Add cacao to PATH

	export PATH=<srcdir>/bin:$PATH

Add milk scripts`to PATH 

	export PATH=<srcdir>/src/CommandLineInterface/scripts:$PATH

If milk is not installed, create sym link milk->cacao

	sudo ln -s /usr/local/bin/cacao /usr/local/bin/milk



## Setting up the work directory

Conventions:

- `<srcdir>` is the source code directory, usually `.../cacao/`
- `<workdir>` is the work directory where the program and scripts will be executed. Note that the full 
path should end with `.../AOloop<#>` where `<#>` ranges from 0 to 9. For example, `AOloop2`.


The work directory is where all scripts and high level commands should be run from. You will first need to create the work directory and then load scripts from the source directory to the work directory.

First, execute from the source directory the 'syncscript -e' command to export key install scripts:

	mkdir /<workdir>
	cd <srcdir>/src/AOloopControl/scripts
	./syncscripts -e /<workdir>

The 'syncscript' script should now be in the work directory. Execute it to copy all scripts to the work directory:

	cd /<workdir>
	./syncscripts

Symbolic links to the source scripts and executable are now installed in the work directory (exact links / directories may vary) :

	scexao@scexao:~/AOloop/AOloop0$ ls -l
	total 40
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 aocscripts
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 aohardsim
	lrwxrwxrwx 1 scexao scexao   56 Jan  3 23:11 aolconf -> /home/scexao/src/cacao/src/AOloopControl/scripts/aolconf
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 aolconfscripts
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 aolfuncs
	lrwxrwxrwx 1 scexao scexao   67 Jan  3 23:10 AOloopControl -> /home/scexao/src/cacao/src/AOloopControl/scripts/../../../bin/cacao
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 aosetup
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 23:11 auxscripts
	-rw-rw-r-- 1 scexao scexao   39 Jan  3 22:47 shmimmonstartup
	drwxrwxr-x 2 scexao scexao 4096 Jan  3 18:31 status
	lrwxrwxrwx 1 scexao scexao   60 Jan  3 23:10 syncscripts -> /home/scexao/src/cacao/src/AOloopControl/scripts/syncscripts

If new scripts are added in the source directory, running `./syncscripts` again from the work directory will add them to the work directory.

