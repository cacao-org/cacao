# 1. Running example


## 1.1. Set up configuration files on disk

copy cacaoloop-ex01-conf directory to local work directory


## 1.2. Deploy tasks


	cacao-task-manager -X 3 cacaoloop-ex01


Launches all conf-processes
launches DMcomb run-process

Interactive FPS control GUI runs in tmux session cacaoloop01_fpsCTRL



## 1.3. Run execution script


    cp cacaoloop-ex01-conf/aorunscript .
	./aorunscript

The execution script will run one step forward each time it is called, so call it multiple time to advance through the example.



# 2. Notes


Three directories are associated to each fps:

- datadir: where files are written, acts as a staging/working directory
- confdir: where configuration is stored. This can be automatically read/imported by other FPSs
- archivedir
These directories are set in <FPSname>.conf.datadir, <FPSname>.conf.confdir, and <FPSname>.conf.archivedir


FPS actions include:

- ADOPT   : copy from datadir to confdir
- SYNC    : load/update from confdir to FPS memory content
- ARCHIVE : copy from datadir to archive directory
- PULL    : copy from archive to datadir


Communication between FPSs can be achieved in multiple ways:

- A FPS entry can point to another FPS. This allows real-time exchange of parameter values.
 


After each step, to archive results:

cacao-fpsarchive

To adopt results :

cacao-fpsconfadopt


