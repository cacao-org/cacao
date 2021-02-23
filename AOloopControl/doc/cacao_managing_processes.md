
# MANAGING MULTIPLE PROCESSES {#page_cacao_managing_processes}

Note: Here, a process can consist of any operation or set of operations, as defined by the user. In this appendix, a process does not strictly correspond to a CPU process.

cacao's bash script includes tools to manage logical links between processes. A process can be **queued**, waiting to start until some condition is met, and its status (running, waiting to be launched, or completed) can be checked. 

The command `l` under the GUI top menu will launch a window displaying processes status. 

## Queued process

A queued process is a process that has been sent to a tmux session but is awaiting previous operations, within the same tmux session, to be completed. The process is logged as queued when the corresponding command is sent to the tmux session. When the tmux session reaches the command, it will move it from queued to active.


To log queued processes as such, follow this bash template:

	logRunningProcessQ "<process_tag>" <tmux_session_name>
	tmux send-keys -t <tmux_session_name> "process_command" C-m

or

	logRunningProcessQ0 "<process_tag>" <tmux_session_name> "<comments>"
	tmux send-keys -t <tmux_session_name> "mv runproc/<process_tag>.runprocQ runproc/<process_tag>.runproc" C-m
	tmux send-keys -t <tmux_session_name> "process_command" C-m

`process_tag` is a name chosen to represent the operations performed. The second option can be used when writing to a script, so it is more flexible.


## Active process

To mark a process as active, it can either be queued as described above, or the user can include at the time the process starts:

	touch runproc/<process_tag>.runproc

Once the process is completed, it needs to be removed from the active process list :

	rm runproc/<process_tag>.runproc

## Locking/unlocking processes

The command `./auxscripts/waitforfilek` in `<srcdir>/scripts` locks the bash script execution until a file `<tagname>.unlock` appears. The command will remove the unlock file once it appears, and continue execution. It can take a timeout option. Typical use:

	tmux send-keys -t <tmux_session_name> "command_that_needs_to_be_completed_before_non-tmux_script_commands" C-m
	tmux send-keys -t <tmux_session_name> "touch <tagname>.unlock" C-m  # command will unlock the script once the unlock file appears
	./auxscripts/waitforfilek -t <timeout_sec> <tagname>
	<non-tmux_script_commands_to_be_executed_AFTER_tmux_commands>

The command creates a "<tagname>.lock" file, and waits for a "<tagname>.unlock" to appear. When the unlock file is detected, both lock and unlock files are removed. The lock file indicates that a process is currently locked. This scheme can be employed to synchronize multiple scripts. 

The lower level `./auxscripts/waitonfile` command allows users to manually setup locks between proceses, by waiting for a file to disappear. Typical use:

	touch fname.lock
	<some other process here, will remove fname.lock file when done>
	./auxscripts/waitonfile fname.lock
	<more bash instructions here>
