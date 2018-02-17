# cacao Control: Setup and Overview {#page_cacao_Control_Setup_Overview}





## GUI description

The script `aolconf` starts the main GUI, from which all setup and control can be done. The GUI consists of several main screens, as shown below.

![aolconf GUI screens](./figures/aolconfGUIscreens.jpg "GUI screens")


## Commands log


ALL commands are logged by the script : 

	./aolconfscripts/aollog




### Automatically generated internal command log (very detailed)

`aolconf` uses the script `aolconfscripts/aollog` to log commands and status into file `./logdir/<UTDATE>/logging/<LOOPNAME>.log`. A sym link to `aolconf.log` is created for convenience, so the log content can be viewed with:

~~~~
tail -f aolconf.log
~~~~

Inside the bash script, function `aoconflog` is used to call `aolconfscripts/aollog` with the proper loop name, as defined in the main `aolconf` script:

~~~
# internal log - logs EVERYTHING
function aoconflog {
./aolconfscripts/aollog "$LOOPNAME" "$@"
}
~~~




### User-provided UsExternal log (less verbose)


More important commands are logged simultaneously to the internal log and an external log. The user provides the command to externally log commands. The executable should be in the path, and named `aollogext`. The syntax is:

~~~
./aollogext <string>
~~~

The string usually consists of the loop name followed by comments.

Inside the bash script, function `aoconflogext` is used to call `aolconfscripts/aollog` with the proper loop name, as defined in the main `aolconf` script:

~~~
# external log, use for less verbose log
function aoconflogext {
./aolconfscripts/aollog -e "$LOOPNAME" "$@"
}
~~~






### Interactive user log

To start the interactive log script:

~~~
./aolconfscripts/aollog -i <LOOPNAME> NULL
~~~

Entries will be logged in the `./logdir/<UTDATE>/logging/<LOOPNAME>.log` file (with sym link to `aolconf.log`).

It is also common practice to start a `MISC` log for misc comments, also to be included in the external log:

~~~
./aolconfscripts/aollog -ie MISC NULL
~~~

The corresponding log can be viewed by:

~~~
tail -f logdit/<UTDATE>/logging/MISC.log
~~~

