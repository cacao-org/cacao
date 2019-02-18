# cacao Initial Setup {#page_cacao_Initial_Setup}

@note This file: ./src/AOloopControl/doc/cacao_Initial_Setup.md

[TOC]

---

@note <srcdir> is the directory where cacao should be installed. For example: `/home/myname/src`. 


# Installing cacao {#page_cacaoSetup_install}


## Download and install package {#page_cacaoSetup_downloadinstall}

See @ref page_installation for detailed instructions.


After installing the required packages, the main steps will be:

	git clone --recursive https://github.com/cacao-org/cacao cacao

	cd <srcdir>/cacao
	mkdir _build
	cd _build
	cmake .. -DUSE_MAGMA=ON
	make
	sudo make install




## Post installation configuration {#page_cacaoSetup_postinstall}


If milk is not installed, create sym link milk->cacao

	sudo ln -s /usr/local/bin/cacao /usr/local/bin/milk



---

# Setting up AOloop work directory {#page_cacaoSetup_AOloopdir}



Conventions:

- `<srcdir>` is the source code directory, for example `/home/CACAOuser/src`
- `<workdir>` is the work directory where the program and scripts will be executed. Note that the full 
path should end with `/AOloop<#>` where `<#>` ranges from 0 to 9. For example, `<workdir>` could be `/home/CACAOuser/AOloop/AOloop2`.


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

