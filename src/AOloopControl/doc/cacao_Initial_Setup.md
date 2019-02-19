# cacao Initial Setup {#page_cacao_Initial_Setup}

@note This file: ./src/AOloopControl/doc/cacao_Initial_Setup.md

[TOC]

---

@note <srcdir> is the directory where cacao should be installed. For example: `/home/cacaouser/src`. 


# Installing cacao {#page_cacaoSetup_install}


## Download and install package {#page_cacaoSetup_downloadinstall}

See @ref page_installation for detailed instructions.


After installing the required packages, the main steps will be:

	cd <srcdir>
	git clone --recursive https://github.com/cacao-org/cacao cacao
	cd cacao
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

- `<srcdir>` is the source code directory, for example `/home/cacaouser/src`
- `<workdir>` is the work directory where the program and scripts will be executed. Note that the full 
path should end with `/AOloop<#>` where `<#>` ranges from 0 to 9. For example, `<workdir>` could be `/home/cacaouser/AOloop/AOloop2`.


The work directory is where all scripts and high level commands should be run from. You will first need to create the work directory and then load scripts from the source directory to the work directory.

First, execute from the source directory the 'syncscript -e' command to export key install scripts:

	mkdir /<workdir>
	cd <srcdir>/src/AOloopControl/scripts
	./syncscripts -e /<workdir>

The 'syncscript' script should now be in the work directory. Execute it to copy all scripts to the work directory:

	cd /<workdir>
	./syncscripts

Symbolic links to the source scripts and executable are now installed in the work directory (exact links / directories may vary) :

	cacaouser@cacaortc:~/AOloop/AOloop0$ ls -l
	total 16
	drwxrwxr-x 1 cacaouser cacaouser  620 Feb 18 22:08 aocscripts
	drwxrwxr-x 1 cacaouser cacaouser  332 Feb 18 22:08 aohardsim
	lrwxrwxrwx 1 cacaouser cacaouser   56 Feb 18 22:08 aolconf -> /home/cacaouser/src/cacao/src/AOloopControl/scripts/aolconf
	drwxrwxr-x 1 cacaouser cacaouser  796 Feb 18 22:08 aolconfscripts
	drwxrwxr-x 1 cacaouser cacaouser  338 Feb 18 22:08 aolfuncs
	lrwxrwxrwx 1 cacaouser cacaouser   20 Feb 18 21:52 AOloopControl -> /usr/local/bin/cacao
	drwxrwxr-x 1 cacaouser cacaouser  120 Feb 18 22:08 aosetup
	drwxrwxr-x 1 cacaouser cacaouser 1756 Feb 18 22:08 auxscripts
	lrwxrwxrwx 1 cacaouser cacaouser   59 Feb 18 22:08 cacao.tmux -> /home/cacaouser/src/cacao/src/AOloopControl/scripts/cacao.tmux
	drwxrwxr-x 1 cacaouser cacaouser   20 Feb 18 22:08 dmutils
	lrwxrwxrwx 1 cacaouser cacaouser   60 Feb 18 21:52 syncscripts -> /home/cacaouser/src/cacao/src/AOloopControl/scripts/syncscripts


If new scripts are added in the source directory, running `./syncscripts` again from the work directory will add them to the work directory.

