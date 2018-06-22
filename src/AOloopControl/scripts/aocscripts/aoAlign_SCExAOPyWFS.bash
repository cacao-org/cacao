#!/bin/bash

execname="cacao"
echo "execname = $execname"
mesgfile="/dev/null"

tempfile=`tempfile 2>/dev/null` || tempfile=/tmp/test$$
trap "rm -f $tempfile" 0 1 2 5 15


LINES=$( tput lines )
COLUMNS=$( tput cols )
let " nbwlines = $LINES - 10 "
let " nbwcols = $COLUMNS - 10 "
echo "$COLUMNS -> $nbwcols"


# short loop name (for status log)
if [ -f SLOOPNAME ]; then
SLOOPNAME=$( head -1 SLOOPNAME )
else
SLOOPNAME="null"
fi



LOOPNUMBER_file="LOOPNUMBER"


##############################################
# FUNCTION: READ INSTRUMENT CONG VAL FROM FILE
##############################################
# arg 1: parameter name
# arg 2: default value
# 
# param value is stored in variable "instconfvalue"
function ConfReadInstConf {
file="./conf/instconf_$1.txt"
if [ -f "$file" ]; then
instconfvalue=$( head -1 $file )
else
instconfvalue="$2"
echo "$2" > $file
fi
}






mkdir -p conf
mkdir -p status
mkdir -p tmp


LOOPNUMBER_default=2  # loop number

# LOOPNUMBER (loop number)
if [ ! -f $LOOPNUMBER_file ]
then
	echo "creating loop number"
	echo "$LOOPNUMBER_default" > $LOOPNUMBER_file
else
	LOOPNUMBER=$( head -1 $LOOPNUMBER_file)
	echo "LOOPNUMBER = $LOOPNUMBER"
fi




# ======================= LOGGING =================================
LOOPNAME=$( head -1 LOOPNAME )
echo "LOOPNAME = $LOOPNAME"
#
 internal log - logs EVERYTHING
function aoconflog {
#echo "$@" >> aolconf.log
#dolog "$LOOPNAME" "$@"
./aolconfscripts/aollog "$LOOPNAME" "$@"
}

# external log, less verbose
function aoconflogext {
#echo "$@" >> aolconf.log
#dolog "$LOOPNAME" "$@"
./aolconfscripts/aollog "$LOOPNAME" "$@"
dologext "$LOOPNAME $@"
}


# status update
# uses SLOOPNAME (short loop name)
function aoconfLogStatusUpdate {
/home/scexao/bin/scexaostatusext "$@"
}



function stringcenter {
line=$1
    let " col1 = $nbwcols-35"
    columns="$col1"
    string=$(printf "%*s%*s\n" $(( (${#line} + columns) / 2)) "$line" $(( (columns - ${#line}) / 2)) " ")
}




function recomputeLatency {
frHz=$1
# read latencies 
hardwlatency1=$( head -1 conf/param_hardwlatency1.txt )
wfsmextrlatency=$( head -1 conf/param_wfsmextrlatency.txt )
complatency=$( head -1 conf/param_complatency.txt )
echo "$hardwlatency1 $wfsmextrlatency $complatency $frHz" > tmpfile.txt
hardwlatency=$( awk '{ printf("%.6f\n", ($1*$4+0.5)/$4) }' tmpfile.txt )
hardwlatency_frame=$( awk '{ printf("%05.3f\n", $1*$4+0.5) }' tmpfile.txt )
wfsmextrlatency_frame=$( awk '{ printf("%05.3f\n", $2*$4) }' tmpfile.txt )
complatency_frame=$( awk '{ printf("%05.3f\n", $3*$4) }' tmpfile.txt )

echo "$hardwlatency" > ./conf/param_hardwlatency.txt
echo "$hardwlatency_frame" > ./conf/param_hardwlatency_frame.txt
echo "$wfsmextrlatency_frame" > ./conf/param_wfsmextrlatency_frame.txt
echo "$complatency_frame" > ./conf/param_complatency_frame.txt
echo "$frHz" > ./conf/param_loopfrequ.txt
$execname << EOF
aolsethlat $hardwlatency_frame
aolsetwlat $wfsmextrlatency_frame
aolsetclat $complatency_frame
aolsetloopfrequ $frHz
exit
EOF
}


function setpywfsfrequ {
	
echo "${pyfreq}" > ./conf/instconf_pywfs_freq.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_freq ${pyfreq}"

cp ./conf/param_mloopfrequ.${pyfreq}.txt ./conf/param_mloopfrequ.txt
cp ./conf/param_hardwlatency1.${pyfreq}.txt ./conf/param_hardwlatency1.txt
cp ./conf/param_hardwlatency_frame.${pyfreq}.txt ./conf/param_hardwlatency_frame.txt

recomputeLatency ${pyfreq} &> $mesgfile &
aoconflogext "Set pyfreq = $pyfreq Hz" &> $mesgfile &
fi
}



function func_set_pcampospickoff {
pfilename="./status/pcampos_ref${pypickoff}.txt"
if [ -f "$pfilename" ]; then
cp ${pfilename} ./status/pcamposREF.txt
if [ -f ./status/pcamposREF.txt ]; then
pywfsreimagexposref0=$( awk '{print $1}' ./status/pcamposREF.txt )
pywfsreimageyposref0=$( awk '{print $2}' ./status/pcamposREF.txt )
pywfs_pup x goto $pywfsreimagexposref0
pywfs_pup y goto $pywfsreimageyposref0
pywfsreimagexposref="$pywfsreimagexposref0"
pywfsreimageyposref="$pywfsreimageyposref0"
fi
fi
}



# Filter wheels
echo "declaring filter wheels"
pyfwlist=("0" "          ")
pyfwlist+=( "1" "  OPEN    " )
pyfwlist+=( "2" "700nm 50nm" )
pyfwlist+=( "3" "  BLOCK   " )
pyfwlist+=( "4" "750nm 50nm" )
pyfwlist+=( "5" "850nm 25nm" )
pyfwlist+=( "6" "850nm 40nm" )


















state="menualign"


while true; do

stateok=0

mkdir -p status

statfile="./status/status_alignTT.txt"
TTloopstat=$( head -1 $statfile)
if [[ -f "$statfile" && ( "$TTloopstat" = " ON" || "$TTloopstat" = "OFF" || "$TTloopstat" = "PAU" ) ]]; then
echo "OK" &> $mesgfile
else
echo "OFF" > $statfile
TTloopstat="OFF"
fi

statfile="./status/status_alignPcam.txt"
Pcamloopstat=$( head -1 $statfile)
if [[ -f "$statfile" && ( "$Pcamloopstat" = " ON" || "$Pcamloopstat" = "OFF" || "$Pcamloopstat" = "PAU" ) ]]; then
echo "OK" &> $mesgfile
else
echo "OFF" > $statfile
Pcamloopstat="OFF"
fi


PyrFilter=$( head -1 ./conf/instconf_pywfs_filter.txt)

if [ "$TTloopstat" = " ON" ]; then
TTloopstat_C="\Zr\Z2 ON\Zn"
else
TTloopstat_C="OFF"
fi

if [ "$Pcamloopstat" = " ON" ]; then
Pcamloopstat_C="\Zr\Z2 ON\Zn"
else
Pcamloopstat_C="OFF"
fi


if [ $state = "menualign" ]; then
stateok=1
menuname="ALIGNMENT - LOOP ${LOOPNAME} ($LOOPNUMBER})\n
\n
   TT   loop is : $TTloopstat_C\n
   Pcam loop is : $Pcamloopstat_C\n
   Pyr Filter   : $PyrFilter\n"


pyTTloopgain=$( head -1 ./status/gain_PyAlignTT.txt)
Pcamloopgain=$( head -1 ./status/gain_PyAlignCam.txt)


if [ -f ./status/pcampos.txt ]; then
pywfsreimagexposref=$( awk '{print $1}' ./status/pcampos.txt )
pywfsreimageyposref=$( awk '{print $2}' ./status/pcampos.txt )
fi


stringcenter "Pyramid modulation"
menuitems=( "1 ->" "\Zb\Zr$string\Zn" )


menuitems+=( "" "" )

PyWFSmodOK=1

ConfReadInstConf "pywfs_freq" "?"
pyfreq=$instconfvalue


pmodscale="0"

if [ "$pyfreq" = "?" ]; then
menuitems+=( " " "\Zr\Z1 freq = UNKNOWN\Zn" )
PyWFSmodOK=0
fi


if [ "$pyfreq" = "0500" ]; then
pmodscale="250"
menuitems+=( "pyfr05" "\Zr\Z2 freq = 0.5 kHz\Zn" )
else
menuitems+=( "pyfr05" " freq = 0.5 kHz" )
fi

if [ "$pyfreq" = "1000" ]; then
pmodscale="250"
menuitems+=( "pyfr10" "\Zr\Z2 freq = 1.0 kHz\Zn" )
else
menuitems+=( "pyfr10" " freq = 1.0 kHz" )
fi

if [ "$pyfreq" = "1500" ]; then
pmodscale="250"
menuitems+=( "pyfr15" "\Zr\Z2 freq = 1.5 kHz\Zn" )
else
menuitems+=( "pyfr15" " freq = 1.5 kHz" )
fi

if [ "$pyfreq" = "2000" ]; then
pmodscale="250"
menuitems+=( "pyfr20" "\Zr\Z2 freq = 2.0 kHz\Zn" )
else
menuitems+=( "pyfr20" " freq = 2.0 kHz" )
fi

if [ "$pyfreq" = "2500" ]; then
pmodscale="215"
menuitems+=( "pyfr25" "\Zr\Z2 freq = 2.5 kHz\Zn" )
else
menuitems+=( "pyfr25" " freq = 2.5 kHz" )
fi

if [ "$pyfreq" = "3000" ]; then
pmodscale="180"
menuitems+=( "pyfr30" "\Zr\Z2 freq = 3.0 kHz\Zn" )
else
menuitems+=( "pyfr30" " freq = 3.0 kHz" )
fi

if [ "$pyfreq" = "3500" ]; then
pmodscale="145"
menuitems+=( "pyfr35" "\Zr\Z2 freq = 3.5 kHz\Zn" )
else
menuitems+=( "pyfr35" " freq = 3.5 kHz" )
fi


menuitems+=( "" "" )

ConfReadInstConf "pywfs_modampl" "?"
pymodampl=$instconfvalue


echo "$pmodscale $pymodampl" > tmpfile.txt
#pmodradmas=$( awk '{ printf("%5.1f\n", $1*$2) }' tmpfile.txt )
pmodradmas05=$( awk '{ printf("%5.1f\n", $1*0.05) }' tmpfile.txt )
pmodradmas10=$( awk '{ printf("%5.1f\n", $1*0.10) }' tmpfile.txt )
pmodradmas15=$( awk '{ printf("%5.1f\n", $1*0.15) }' tmpfile.txt )
pmodradmas20=$( awk '{ printf("%5.1f\n", $1*0.20) }' tmpfile.txt )
pmodradmas25=$( awk '{ printf("%5.1f\n", $1*0.25) }' tmpfile.txt )
pmodradmas30=$( awk '{ printf("%5.1f\n", $1*0.30) }' tmpfile.txt )
pmodradmas35=$( awk '{ printf("%5.1f\n", $1*0.35) }' tmpfile.txt )
pmodradmas40=$( awk '{ printf("%5.1f\n", $1*0.40) }' tmpfile.txt )
pmodradmas45=$( awk '{ printf("%5.1f\n", $1*0.45) }' tmpfile.txt )
pmodradmas50=$( awk '{ printf("%5.1f\n", $1*0.50) }' tmpfile.txt )
pmodradmas55=$( awk '{ printf("%5.1f\n", $1*0.55) }' tmpfile.txt )
pmodradmas60=$( awk '{ printf("%5.1f\n", $1*0.60) }' tmpfile.txt )
pmodradmas65=$( awk '{ printf("%5.1f\n", $1*0.65) }' tmpfile.txt )
pmodradmas70=$( awk '{ printf("%5.1f\n", $1*0.70) }' tmpfile.txt )
pmodradmas75=$( awk '{ printf("%5.1f\n", $1*0.75) }' tmpfile.txt )
pmodradmas80=$( awk '{ printf("%5.1f\n", $1*0.80) }' tmpfile.txt )
pmodradmas85=$( awk '{ printf("%5.1f\n", $1*0.85) }' tmpfile.txt )
pmodradmas90=$( awk '{ printf("%5.1f\n", $1*0.90) }' tmpfile.txt )
pmodradmas95=$( awk '{ printf("%5.1f\n", $1*0.95) }' tmpfile.txt )
pmodradmas00=$( awk '{ printf("%5.1f\n", $1*1.00) }' tmpfile.txt )
rm tmpfile.txt


if [ "$pymodampl" = "?" ]; then
menuitems+=( " " "\Zr\Z1 amplitude = UNKNOWN\Zn" )
PyWFSmodOK=0
fi


if [ "$pymodampl" = "0.05" ]; then
menuitems+=( "pymoda005" "\Zr\Z2 modulation amplitude = 0.05\Zn (modulation radius = ${pmodradmas05} mas)" )
else
menuitems+=( "pymoda005" " modulation amplitude = 0.05 (modulation radius = ${pmodradmas05} mas)" )
fi

if [ "$pymodampl" = "0.10" ]; then
menuitems+=( "pymoda010" "\Zr\Z2 modulation amplitude = 0.10\Zn  (modulation radius = ${pmodradmas10} mas)" )
else
menuitems+=( "pymoda010" " modulation amplitude = 0.10  (modulation radius = ${pmodradmas10} mas)" )
fi

if [ "$pymodampl" = "0.15" ]; then
menuitems+=( "pymoda015" "\Zr\Z2 modulation amplitude = 0.15\Zn  (modulation radius = ${pmodradmas15} mas)" )
else
menuitems+=( "pymoda015" " modulation amplitude = 0.15  (modulation radius = ${pmodradmas15} mas)" )
fi

if [ "$pymodampl" = "0.20" ]; then
menuitems+=( "pymoda020" "\Zr\Z2 modulation amplitude = 0.20\Zn  (modulation radius = ${pmodradmas20} mas)" )
else
menuitems+=( "pymoda020" " modulation amplitude = 0.20  (modulation radius = ${pmodradmas20} mas)" )
fi

if [ "$pymodampl" = "0.25" ]; then
menuitems+=( "pymoda025" "\Zr\Z2 modulation amplitude = 0.25\Zn  (modulation radius = ${pmodradmas25} mas)" )
else
menuitems+=( "pymoda025" " modulation amplitude = 0.25  (modulation radius = ${pmodradmas25} mas)" )
fi

if [ "$pymodampl" = "0.30" ]; then
menuitems+=( "pymoda030" "\Zr\Z2 modulation amplitude = 0.30\Zn  (modulation radius = ${pmodradmas30} mas)" )
else
menuitems+=( "pymoda030" " modulation amplitude = 0.30  (modulation radius = ${pmodradmas30} mas)" )
fi

if [ "$pymodampl" = "0.35" ]; then
menuitems+=( "pymoda035" "\Zr\Z2 modulation amplitude = 0.35\Zn  (modulation radius = ${pmodradmas35} mas)" )
else
menuitems+=( "pymoda035" " modulation amplitude = 0.35  (modulation radius = ${pmodradmas35} mas)" )
fi

if [ "$pymodampl" = "0.40" ]; then
menuitems+=( "pymoda040" "\Zr\Z2 modulation amplitude = 0.40\Zn  (modulation radius = ${pmodradmas40} mas)" )
else
menuitems+=( "pymoda040" " modulation amplitude = 0.40  (modulation radius = ${pmodradmas40} mas)" )
fi

if [ "$pymodampl" = "0.45" ]; then
menuitems+=( "pymoda045" "\Zr\Z2 modulation amplitude = 0.45\Zn  (modulation radius = ${pmodradmas45} mas)" )
else
menuitems+=( "pymoda045" " modulation amplitude = 0.45  (modulation radius = ${pmodradmas45} mas)" )
fi

if [ "$pymodampl" = "0.50" ]; then
menuitems+=( "pymoda050" "\Zr\Z2 modulation amplitude = 0.50\Zn  (modulation radius = ${pmodradmas50} mas)" )
else
menuitems+=( "pymoda050" " modulation amplitude = 0.50  (modulation radius = ${pmodradmas50} mas)" )
fi

if [ "$pymodampl" = "0.55" ]; then
menuitems+=( "pymoda055" "\Zr\Z2 modulation amplitude = 0.55\Zn  (modulation radius = ${pmodradmas55} mas)" )
else
menuitems+=( "pymoda055" " modulation amplitude = 0.55  (modulation radius = ${pmodradmas55} mas)" )
fi

if [ "$pymodampl" = "0.60" ]; then
menuitems+=( "pymoda060" "\Zr\Z2 modulation amplitude = 0.60\Zn  (modulation radius = ${pmodradmas60} mas)" )
else
menuitems+=( "pymoda060" " modulation amplitude = 0.60  (modulation radius = ${pmodradmas60} mas)" )
fi

if [ "$pymodampl" = "0.65" ]; then
menuitems+=( "pymoda065" "\Zr\Z2 modulation amplitude = 0.65\Zn  (modulation radius = ${pmodradmas65} mas)" )
else
menuitems+=( "pymoda065" " modulation amplitude = 0.65  (modulation radius = ${pmodradmas65} mas)" )
fi

if [ "$pymodampl" = "0.70" ]; then
menuitems+=( "pymoda070" "\Zr\Z2 modulation amplitude = 0.70\Zn  (modulation radius = ${pmodradmas70} mas)" )
else
menuitems+=( "pymoda070" " modulation amplitude = 0.70  (modulation radius = ${pmodradmas70} mas)" )
fi

if [ "$pymodampl" = "0.75" ]; then
menuitems+=( "pymoda075" "\Zr\Z2 modulation amplitude = 0.75\Zn  (modulation radius = ${pmodradmas75} mas)" )
else
menuitems+=( "pymoda075" " modulation amplitude = 0.75  (modulation radius = ${pmodradmas75} mas)" )
fi

if [ "$pymodampl" = "0.80" ]; then
menuitems+=( "pymoda080" "\Zr\Z2 modulation amplitude = 0.80\Zn  (modulation radius = ${pmodradmas80} mas)" )
else
menuitems+=( "pymoda080" " modulation amplitude = 0.80  (modulation radius = ${pmodradmas80} mas)" )
fi

if [ "$pymodampl" = "0.85" ]; then
menuitems+=( "pymoda085" "\Zr\Z2 modulation amplitude = 0.85\Zn  (modulation radius = ${pmodradmas85} mas)" )
else
menuitems+=( "pymoda085" " modulation amplitude = 0.85  (modulation radius = ${pmodradmas85} mas)" )
fi

if [ "$pymodampl" = "0.90" ]; then
menuitems+=( "pymoda090" "\Zr\Z2 modulation amplitude = 0.90\Zn  (modulation radius = ${pmodradmas90} mas)" )
else
menuitems+=( "pymoda090" " modulation amplitude = 0.90  (modulation radius = ${pmodradmas90} mas)" )
fi

if [ "$pymodampl" = "0.95" ]; then
menuitems+=( "pymoda095" "\Zr\Z2 modulation amplitude = 0.95\Zn  (modulation radius = ${pmodradmas95} mas)" )
else
menuitems+=( "pymoda095" " modulation amplitude = 0.95  (modulation radius = ${pmodradmas95} mas)" )
fi

if [ "$pymodampl" = "1.00" ]; then
menuitems+=( "pymoda100" "\Zr\Z2 modulation amplitude = 1.00\Zn  (modulation radius = ${pmodradmas00} mas)" )
else
menuitems+=( "pymoda100" " modulation amplitude = 1.00  (modulation radius = ${pmodradmas00} mas)" )
fi


menuitems+=( "" "" )

ConfReadInstConf "pywfs_filter" "?"
pyfilter=$instconfvalue

#file="./conf/instconf_pywfs_filter.txt"
#if [ -f $file ]; then
#pyfilter=$( head -1 $file)
#else
#pyfilter="1"
#echo "$pyfilter" > $file
#fi


if [ "$pyfilter" = "?" ]; then
menuitems+=( " " "\Zr\Z1 PyWFS filter = UNKNOWN\Zn" )
fi

if [ "$pyfilter" = "1" ]; then
menuitems+=( "pyfilt1" "\Zr\Z2 PyWFS filter 1  (Open)\Zn" )
else
menuitems+=( "pyfilt1" " PyWFS filter 1  (Open)" )
fi


if [ "$pyfilter" = "2" ]; then
menuitems+=( "pyfilt2" "\Zr\Z2 PyWFS filter 2  (700 nm, 50 nm BW)\Zn" )
else
menuitems+=( "pyfilt2" " PyWFS filter 2  (700 nm, 50 nm BW)" )
fi


if [ "$pyfilter" = "3" ]; then
menuitems+=( "pyfilt3" "\Zr\Z2 PyWFS filter 3  (BLOCK)\Zn" )
else
menuitems+=( "pyfilt3" " PyWFS filter 3  (BLOCK)" )
fi


if [ "$pyfilter" = "4" ]; then
menuitems+=( "pyfilt4" "\Zr\Z2 PyWFS filter 4  (750 nm, 50 nm BW)\Zn" )
else
menuitems+=( "pyfilt4" " PyWFS filter 4  (750 nm, 50 nm BW)" )
fi


if [ "$pyfilter" = "5" ]; then
menuitems+=( "pyfilt5" "\Zr\Z2 PyWFS filter 5  (850 nm, 25 nm BW)\Zn" )
else
menuitems+=( "pyfilt5" " PyWFS filter 5  (850 nm, 25 nm BW)" )
fi


if [ "$pyfilter" = "6" ]; then
menuitems+=( "pyfilt6" "\Zr\Z2 PyWFS filter 6  (850 nm, 40 nm BW)\Zn" )
else
menuitems+=( "pyfilt6" " PyWFS filter 6  (850 nm, 40 nm BW)" )
fi




menuitems+=( "" "" )

ConfReadInstConf "pywfs_pickoff" "?"
pypickoff=$instconfvalue

#file="./conf/instconf_pywfs_pickoff.txt"
#if [ -f $file ]; then
#pypickoff=$( head -1 $file)
#else
#pypickoff="01"
#echo "$pypickoff" > $file
#fi

if [ "$pypickoff" = "?" ]; then
menuitems+=( " " "\Zr\Z1 PyWFS pickoff = UNKNOWN\Zn" )
fi


if [ "$pypickoff" = "01" ]; then
menuitems+=( "pypick01" "\Zr\Z2 PyWFS pickoff 01  (Open)\Zn" )
else
menuitems+=( "pypick01" " PyWFS pickoff 01  (Open)" )
fi

if [ "$pypickoff" = "02" ]; then
menuitems+=( "pypick02" "\Zr\Z2 PyWFS pickoff 02  (Silver mirror)\Zn" )
else
menuitems+=( "pypick02" " PyWFS pickoff 02  (Silver mirror)" )
fi

if [ "$pypickoff" = "03" ]; then
menuitems+=( "pypick03" "\Zr\Z2 PyWFS pickoff 03  (50/50 splitter)\Zn" )
else
menuitems+=( "pypick03" " PyWFS pickoff 03  (50/50 splitter)" )
fi

if [ "$pypickoff" = "04" ]; then
menuitems+=( "pypick04" "\Zr\Z2 PyWFS pickoff 04  (650 nm SP)\Zn" )
else
menuitems+=( "pypick04" " PyWFS pickoff 04  (650 nm SP)" )
fi

if [ "$pypickoff" = "05" ]; then
menuitems+=( "pypick05" "\Zr\Z2 PyWFS pickoff 05  (700 nm SP)\Zn" )
else
menuitems+=( "pypick05" " PyWFS pickoff 05  (700 nm SP)" )
fi

if [ "$pypickoff" = "06" ]; then
menuitems+=( "pypick06" "\Zr\Z2 PyWFS pickoff 06  (750 nm SP)\Zn" )
else
menuitems+=( "pypick06" " PyWFS pickoff 06  (750 nm SP)" )
fi

if [ "$pypickoff" = "07" ]; then
menuitems+=( "pypick07" "\Zr\Z2 PyWFS pickoff 07  (800 nm SP)\Zn" )
else
menuitems+=( "pypick07" " PyWFS pickoff 07  (800 nm SP)" )
fi

if [ "$pypickoff" = "08" ]; then
menuitems+=( "pypick08" "\Zr\Z2 PyWFS pickoff 08  (850 nm SP)\Zn" )
else
menuitems+=( "pypick08" " PyWFS pickoff 08  (850 nm SP)" )
fi

if [ "$pypickoff" = "09" ]; then
menuitems+=( "pypick09" "\Zr\Z2 PyWFS pickoff 09  (750 nm LP)\Zn" )
else
menuitems+=( "pypick09" " PyWFS pickoff 09  (750 nm LP)" )
fi

if [ "$pypickoff" = "10" ]; then
menuitems+=( "pypick10" "\Zr\Z2 PyWFS pickoff 10  (800 nm LP)\Zn" )
else
menuitems+=( "pypick10" " PyWFS pickoff 10  (800 nm LP)" )
fi

if [ "$pypickoff" = "11" ]; then
menuitems+=( "pypick11" "\Zr\Z2 PyWFS pickoff 11  (850 nm LP)\Zn" )
else
menuitems+=( "pypick11" " PyWFS pickoff 11  (850 nm LP)" )
fi

if [ "$pypickoff" = "12" ]; then
menuitems+=( "pypick12" "\Zr\Z2 PyWFS pickoff 12  (Open)\Zn" )
else
menuitems+=( "pypick12" " PyWFS pickoff 12  (Open)" )
fi


pymodampl10=$(sed 's/\.//' ./conf/instconf_pywfs_modampl.txt)
loopconfname="fr${pyfreq}_mod${pymodampl10}_pf${pyfilter}_pp${pypickoff}_"
echo "${loopconfname}" > ./conf/conf_loopconfname.txt


menuitems+=( "" "" )







TTposX=$( head -1 status/stat_AnalogVoltage_D.txt )
TTposY=$( head -1 status/stat_AnalogVoltage_C.txt )

TTposXref=$( head -1 status/stat_AnalogVoltage_Dref.txt )
TTposYref=$( head -1 status/stat_AnalogVoltage_Cref.txt )

stringcenter "Pyramid TT align ( 90.3 mas/V )"
menuitems+=( "2 ->" "\Zb\Zr$string\Zn" )

menuitems+=( " " "Current position ( scale = 90.3 mas/V ) = $TTposX  $TTposY" )

menuitems+=( "tz" "Zero TT align (-5.0 -5.0)" )
menuitems+=( "ttr" "Move to TT reference position [ $TTposXref  $TTposYref ]" )
menuitems+=( "tts" "Store current position as reference" )

menuitems+=( "tst0" "alignment step = 0.05" )
menuitems+=( "tst1" "alignment step = 0.1" )
menuitems+=( "tst2" "alignment step = 0.2" )
menuitems+=( "tst3" "alignment step = 0.5" )
menuitems+=( "txm" "TT x -$TTstep (PyWFS bottom left )" )
menuitems+=( "txp" "TT x +$TTstep (PyWFS top    right)" )
menuitems+=( "tym" "TT y -$TTstep (PyWFS top    left )" )
menuitems+=( "typ" "TT y +$TTstep (PyWFS bottom right)" )



if [ "$TTloopstat" = "OFF" ]; then
menuitems+=( "ts" "\Zb ====== Start TT align ====== \Zn" )
menuitems+=( "" "" )
fi

if [ "$TTloopstat" = " ON" ]; then
menuitems+=( "tp" "\Zr\Z2 TT loop running \Zn  \Z1\Zr Press to PAUSE \Zn" )
menuitems+=( "tk" "\Zr\Z2 TT loop running \Zn  \Z1\Zr Press to STOP \Zn" )
fi

if [ "$TTloopstat" = "PAU" ]; then
menuitems+=( "tr" "\Zb ====== Resume TT align (after pause) ====== \Zn" )
menuitems+=( "" "" )
fi

menuitems+=( "tg" "py TT loop gain = ${pyTTloopgain}")
menuitems+=( "tm" "Monitor TT align tmux session")
menuitems+=( "" "" )







stringcenter "Pyramid Camera Align ( 5925 steps / pix )"
menuitems+=( "3 ->" "\Zb\Zr$string\Zn" )

menuitems+=( "pz" "Zero Pcam align  ( $pywfsreimagexposref $pywfsreimageyposref )" )
menuitems+=( "psr" "Update reference for pickoff $pypickoff  [$pywfsreimagexposref0 $pywfsreimageyposref0]" )
menuitems+=( "pst0" "alignment step = 50000" )
menuitems+=( "pst1" "alignment step = 10000" )
menuitems+=( "pst2" "alignment step = 3000" )
menuitems+=( "pst3" "alignment step = 1000" )
menuitems+=( "pxm" "Pcam x -$pcamstep (right)" )
menuitems+=( "pxp" "Pcam x +$pcamstep (left)" )
menuitems+=( "pym" "Pcam y -$pcamstep (top)" )
menuitems+=( "pyp" "Pcam y +$pcamstep (bottom)" )

if [ "$Pcamloopstat" = "OFF" ]; then
menuitems+=( "ps" "\Zb ====== Start Pcam align ====== \Zn" )
menuitems+=( "" "" )
fi

if [ "$Pcamloopstat" = " ON" ]; then
menuitems+=( "pp" "\Zr\Z2 Pcam align running \Zn  \Z1\Zr Press to PAUSE \Zn" )
menuitems+=( "pk" "\Zr\Z2 Pcam align running \Zn  \Z1\Zr Press to STOP \Zn" )
fi

if [ "$Pcamloopstat" = "PAU" ]; then
menuitems+=( "pr" "\Zb ====== RESUME Pcam align (after pause) ====== \Zn" )
menuitems+=( "" "" )
fi

menuitems+=( "pg" "Pcam loop gain = ${Pcamloopgain}" )
menuitems+=( "pm" "Monitor Pcam align tmux session")
menuitems+=( "" "" )

stringcenter "DM flatten"
menuitems+=( "4 ->" "\Zb\Zr$string\Zn" )

menuitems+=( "fl" "Flatten DM for pyWFS" )
menuitems+=( "flk" "End flatten DM process" )
menuitems+=( "flz" "Remove flatten DM solution" )
menuitems+=( "fla" "Apply flatten DM solution" )
menuitems+=( "flm" "Monitor DM flatten tmux session" )
menuitems+=( "" "" )




dialog --colors --title "Alignment" \
--ok-label "Select" \
--cancel-label "Top" \
--help-button --help-label "Exit" \
--default-item "${menualign_default}" \
 --menu "$menuname" \
  $nbwlines $nbwcols 100 "${menuitems[@]}"  2> $tempfile


retval=$?
choiceval=$( head -1 $tempfile)


menualign_default="$choiceval"
state="menualign"

case $retval in
   0) # button
menualign_default="$choiceval"
	case $choiceval in


	pyfr05)
pyfreq="0500"
setpywfsfrequ
;;

	pyfr10)
pyfreq="1000"
setpywfsfrequ
;;

	pyfr15)
pyfreq="1500"
setpywfsfrequ
;;

	pyfr20)
pyfreq="2000"
setpywfsfrequ
;;

	pyfr25)
pyfreq="2500"
setpywfsfrequ
;;
	
	pyfr30)
pyfreq="3000"
setpywfsfrequ
;;

	pyfr35)
pyfreq="3500"
setpywfsfrequ
;;




	pymoda005)
pymodampl="0.05"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas05}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda010)
pymodampl="0.10"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas10}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda015)
pymodampl="0.15"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas15}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda020)
pymodampl="0.20"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas20}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda025)
pymodampl="0.25"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas25}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda030)
pymodampl="0.30"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas30}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda035)
pymodampl="0.35"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas35}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda040)
pymodampl="0.40"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas40}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda045)
pymodampl="0.45"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas45}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda050)
pymodampl="0.50"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas50}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda055)
pymodampl="0.55"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas55}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda060)
pymodampl="0.60"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas60}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda065)
pymodampl="0.65"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas65}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda070)
pymodampl="0.70"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas70}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda075)
pymodampl="0.75"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas75}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda080)
pymodampl="0.80"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas80}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda085)
pymodampl="0.85"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas85}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda090)
pymodampl="0.90"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas90}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda095)
pymodampl="0.95"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas95}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;

	pymoda100)
pymodampl="1.00"
echo "$pymodampl" > ./conf/instconf_pywfs_modampl.txt
if [ "$PyWFSmodOK" = "1" ]; then
pywfs_mod_setup ${pyfreq} ${pymodampl} &> $mesgfile
aoconfLogStatusUpdate "${SLOOPNAME}_rad ${pmodradmas00}"
aoconflogext "Set py modulation amplitude = $pymodampl" &> $mesgfile &
fi
;;




	pyfilt1)
pyfilter="1"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;

	pyfilt2)
pyfilter="2"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;

	pyfilt3)
pyfilter="3"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;

	pyfilt4)
pyfilter="4"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;

	pyfilt5)
pyfilter="5"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;

	pyfilt6)
pyfilter="6"
echo "$pyfilter" > ./conf/instconf_pywfs_filter.txt
pywfs_filter ${pyfilter} &> $mesgfile
aoconflogext "Set py filter = $pyfilter" &> $mesgfile &
;;




	pypick01)
pypickoff="01"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick02)
pypickoff="02"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick03)
pypickoff="03"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick04)
pypickoff="04"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick05)
pypickoff="05"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick06)
pypickoff="06"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick07)
pypickoff="07"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick08)
pypickoff="08"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick09)
pypickoff="09"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick10)
pypickoff="10"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick11)
pypickoff="11"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;

	pypick12)
pypickoff="12"
echo "${pypickoff}" > ./conf/instconf_pywfs_pickoff.txt
func_set_pcampospickoff
pywfs_pickoff ${pypickoff}
aoconflogext "Set py pickoff = $pypickoff"
;;




	tz)
aoconflogext "TT align zero"
./aocscripts/SCExAO_analogoutput C -5.0
./aocscripts/SCExAO_analogoutput D -5.0
#analog_output.py voltage C -5.0
#analog_output.py voltage D -5.0
menualign_default="tz"
state="menualign"
;;


	ttr)
aoconflog "TT move to reference  ${TTposXref} ${TTposYref}"
./aocscripts/SCExAO_analogoutput D $TTposXref
./aocscripts/SCExAO_analogoutput C $TTposYref
menualign_default="ttr"
state="menualign"
;;


	tts)
TTposXref="$TTposX"
echo "$TTposXref" > status/stat_AnalogVoltage_Dref.txt
TTposYref="$TTposY"
echo "$TTposYref" > status/stat_AnalogVoltage_Cref.txt
;;


        tst0)
TTstep="0.05"
menualign_default="tst0"
state="menualign"
;;
        tst1)
TTstep="0.1"
menualign_default="tst1"
state="menualign"
;;
        tst2)
TTstep="0.2"
menualign_default="tst2"
state="menualign"
;;
        tst3)
TTstep="0.5"
menualign_default="tst3"
state="menualign"
;;
        txm)
TTposX=$( head -1 status/stat_AnalogVoltage_D.txt )
TTposXn=$( echo "$TTposX-$TTstep" | bc )
./aocscripts/SCExAO_analogoutput D $TTposXn
aoconflog "TT move x ${TTposXn}"
menualign_default="txm"
state="menualign"
;;
        txp)
TTposX=$( head -1 status/stat_AnalogVoltage_D.txt )
TTposXn=$( echo "$TTposX+$TTstep" | bc )
./aocscripts/SCExAO_analogoutput D $TTposXn
aoconflog "TT move x ${TTposXn}"
menualign_default="txp"
state="menualign"
;;
        tym)
TTposY=$( head -1 status/stat_AnalogVoltage_C.txt )
TTposYn=$( echo "$TTposY-$TTstep" | bc )
./aocscripts/SCExAO_analogoutput C $TTposYn
aoconflog "TT move y ${TTposYn}"
menualign_default="tym"
state="menualign"
;;
        typ)
TTposY=$( head -1 status/stat_AnalogVoltage_C.txt )
TTposYn=$( echo "$TTposY+$TTstep" | bc )
./aocscripts/SCExAO_analogoutput C $TTposYn
aoconflog "TT move y ${TTposYn}"
menualign_default="typ"
state="menualign"
;;


	ts) # start TT align loop
aoconflogext "TT align loop start" &> $mesgfile &
rm stop_PyAlignTT.txt &> $mesgfile 
rm status/pause_PyAlignTT.txt &> $mesgfile 
tmux kill-session -t alignPyrTT &> $mesgfile 
tmux new-session -d -s alignPyrTT
tmux send-keys -t alignPyrTT "$execname -n alignPyrTT" C-m
tmux send-keys -t alignPyrTT "mload scexaocontrol" C-m
tmux send-keys -t alignPyrTT "readshmim aol${LOOPNUMBER}_wfsdark" C-m
tmux send-keys -t alignPyrTT "cp aol${LOOPNUMBER}_wfsdark wfsdark" C-m
tmux send-keys -t alignPyrTT "readshmim aol${LOOPNUMBER}_wfsim" C-m
tmux send-keys -t alignPyrTT "scexaopywfsttalign aol${LOOPNUMBER}_wfsim $TTposX $TTposY" C-m
echo " ON" > ./status/status_alignTT.txt
aoconfLogStatusUpdate "${SLOOPNAME}_cenloop CLOSED 1"
menualign_default="tk"
state="menualign"
;; 

  	 tr) # resume TT align loop
aoconflogext "TT align loop resume" 
rm status/pause_PyAlignTT.txt stop_PyAlignTT.txt
if [ "$( head -1 ./status/status_alignTT.txt)" = "OFF" ]
then
dialog --title "Message" --msgbox "Starting TT align\n (CTRL-C now to abort)\n" 8 30
fi
echo " ON" > ./status/status_alignTT.txt
aoconfLogStatusUpdate "${SLOOPNAME}_cenloop CLOSED 1"
menualign_default="tp"
state="menualign"
;; 

	tg)
dialog --title "PyTT loop gain" --inputbox "Enter loop gain" 8 40 ${pyTTloopgain} 2> $tempfile
pyTTloopgain=$( head -1 $tempfile)
echo ${pyTTloopgain} > ./status/gain_PyAlignTT.txt
aoconflogext "TT align set gain ${pyTTloopgain}"
menualign_default="tg"
state="menualign"
;;

   	 tp)
touch status/pause_PyAlignTT.txt
aoconflogext "TT align loop pause"
echo "PAU" > ./status/status_alignTT.txt
aoconfLogStatusUpdate "${SLOOPNAME}_cenloop PAUSED 2" 
menualign_default="tr"
state="menualign"
;;  

   	 tk) 
touch status/stop_PyAlignTT.txt
tmux kill-session -t alignPyrTT
echo "OFF" > ./status/status_alignTT.txt
aoconfLogStatusUpdate "${SLOOPNAME}_cenloop OPEN 2"
aoconflogext "TT align loop off"
menualign_default="ts"
state="menualign"
;;
	tm) tmux a -t alignPyrTT ;; 

	pz)
aoconflogext "Pupil align zero"
pywfs_pup x home
pywfs_pup x goto $pywfsreimagexposref #150000
pywfs_pup y home
pywfs_pup y goto $pywfsreimageyposref #67000
echo "$pywfsreimagexposref $pywfsreimageyposref" > ./status/pcampos.txt
menualign_default="pz"
state="menualign"
;;

	psr)
cp ./status/pcampos.txt ./status/pcampos_ref${pypickoff}.txt
menualign_default="psr"
;;
        pst0)
pcamstep=50000
menualign_default="pst0"
state="menualign"
;;
        pst1)
pcamstep=10000
menualign_default="pst1"
state="menualign"
;;
        pst2)
pcamstep=3000
menualign_default="pst2"
state="menualign"
;;
        pst3)
pcamstep=1000
menualign_default="pst3"
state="menualign"
;;
        pxm)
pywfsreimagexposref=$(($pywfsreimagexposref-$pcamstep))
pywfs_pup x goto $pywfsreimagexposref #150000
aoconflog "Pupil move x ${pywfsreimagexposref}"
echo "$pywfsreimagexposref $pywfsreimageyposref" > ./status/pcampos.txt
menualign_default="pxm"
state="menualign"
;;
        pxp)
pywfsreimagexposref=$(($pywfsreimagexposref+$pcamstep))
pywfs_pup x goto $pywfsreimagexposref #150000
aoconflog "Pupil move x ${pywfsreimagexposref}"
echo "$pywfsreimagexposref $pywfsreimageyposref" > ./status/pcampos.txt
menualign_default="pxp"
state="menualign"
;;
        pym)
pywfsreimageyposref=$(($pywfsreimageyposref-$pcamstep))
pywfs_pup y goto $pywfsreimageyposref #150000
aoconflog "Pupil move y ${pywfsreimagexposref}"
echo "$pywfsreimagexposref $pywfsreimageyposref" > ./status/pcampos.txt
menualign_default="pym"
state="menualign"
;;
        pyp)
pywfsreimageyposref=$(($pywfsreimageyposref+$pcamstep))
pywfs_pup y goto $pywfsreimageyposref #150000
aoconflog "Pupil move y ${pywfsreimagexposref}"
echo "$pywfsreimagexposref $pywfsreimageyposref" > ./status/pcampos.txt
menualign_default="pyp"
state="menualign"
;;
	ps)
aoconflogext "Pupil align loop start" &> $mesgfile &
rm status/stop_PyAlignCam.txt &> $mesgfile 
rm status/pause_PyAlignCam.txt &> $mesgfile 
tmux kill-session -t alignPcam &> $mesgfile 
tmux new-session -d -s alignPcam
tmux send-keys -t alignPcam "./aocscripts/alignPcam_${LOOPNAME}" C-m
echo " ON" > ./status/status_alignPcam.txt
aoconfLogStatusUpdate "${SLOOPNAME}_puploop CLOSED 1"
menualign_default="pk"
state="menualign"
;; 
   	 pr)
aoconflogext "Pupil align loop resume"
rm status/pause_PyAlignCam.txt status/stop_PyAlignCam.txt
if [ "$( head -1 ./status/status_alignPcam.txt)" == "off" ]
then
dialog --title "Message" --msgbox "Starting Pcam align\n (CTRL-C now to abort)\n" 8 30
fi
echo " ON" > ./status/status_alignPcam.txt
aoconfLogStatusUpdate "${SLOOPNAME}_puploop CLOSED 1"
menualign_default="pp"
state="menualign"
;;  
 	pg)
dialog --title "Pcam loop gain" --inputbox "Enter loop gain" 8 40 ${Pcamloopgain} 2> $tempfile
Pcamloopgain=$( head -1 $tempfile)
echo ${Pcamloopgain} > ./status/gain_PyAlignCam.txt
aoconflog "Pupil align loop set gain ${Pcamloopgain}"
menualign_default="pg"
state="menualign"
;;  	 
	pp) 
touch status/pause_PyAlignCam.txt
echo "PAU" > ./status/status_alignPcam.txt
aoconfLogStatusUpdate "${SLOOPNAME}_puploop PAUSED 2"
aoconflogext "Pupil align loop pause"
menualign_default="pr"
state="menualign"
;;   
   	 pk) 
touch status/stop_PyAlignCam.txt
echo "OFF" > ./status/status_alignPcam.txt
aoconfLogStatusUpdate "${SLOOPNAME}_puploop OPEN 2"
tmux kill-session -t alignPcam
aoconflogext "Pupil align loop kill"
menualign_default="ps"
state="menualign"
;;   
	pm) tmux a -t alignPcam ;;
	
	d)
aoconflogext "Measure DM illumination"
tmux send-keys -t aolconf$LOOPNUMBER "./MeasureActMap" C-m 
menualign_default="d"
state="menualign"
;;
	fl)
aoconflogext "Flatten DM for pyWFS"
tmux kill-session -t pyrflatten
tmux new-session -d -s pyrflatten
menualign_default="fl"
state="menualign"
;;
	flk)
aoconflogext " STOP flatten DM process"
tmux kill-session -t pyrflatten
dmdispzero 5
shmim2fits dmdisp6 dmpyoffset.fits
dmdispzero 6
dm_update_channel 5 dmpyoffset.fits
menualign_default="fl"
state="menualign"
;;
	flz) dmdispzero 5 ;;
	fla) dm_update_channel 5 dmpyoffset.fits ;;
	flm) tmux a -t pyrflatten ;;


	esac;;
   1) state="menutop";;   
   2) state="menuexit";;
   255) state="menuexit";;
esac
fi













if [ $state = "menuexit" ]; then
stateok=1
echo "menuexit -> exit"
exit
fi



if [ $stateok = 0 ]; then
echo "state \"$state\" not recognized ... exit"
exit
fi




done
