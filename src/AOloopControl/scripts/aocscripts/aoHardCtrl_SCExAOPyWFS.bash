#!/bin/bash

execname="Cfits"


tempfile=`tempfile 2>/dev/null` || tempfile=/tmp/test$$
trap "rm -f $tempfile" 0 1 2 5 15


LINES=$( tput lines )
COLUMNS=$( tput cols )
let " nbwlines = $LINES - 10 "
let " nbwcols = $COLUMNS - 10 "
echo "$COLUMNS -> $nbwcols"





LOOPNUMBER_file="LOOPNUMBER"


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

# internal log - logs EVERYTHING
function aoconflog {
./aolconfscripts/aollog "$LOOPNAME" "$@"
}

# external log, less verbose
function aoconflogext {
./aolconfscripts/aollog "$LOOPNAME" "$@"
dologext "$LOOPNAME $@"
}





function stringcenter {
line=$1
    let " col1 = $nbwcols-35"
    columns="$col1"
    string=$(printf "%*s%*s\n" $(( (${#line} + columns) / 2)) "$line" $(( (columns - ${#line}) / 2)) " ")
}






# DM DC level [um] for each Vmax setting

dmDCum025="0.0219"
dmDCum050="0.0875"
dmDCum075="0.1969"
dmDCum100="0.3500"
dmDCum125="0.5469"
dmDCum150="0.7857"




# Set DM Vmax 
function Set_dmVmax {
file="./status/status_dmVmax.txt"
currentfw=$(echo "$( head -1 $file)")
if [ ! "${current_dmVmax}" == "$1" ]
then
echo "CHANGING dmVmax to $1"# &> ${outmesg}

else
echo "WHEEL ALREADY SET TO"# > ${outmesg}
fi
sleep 0.1
currentfw=$1
echo "${currentfw}" > $file
sleep 0.1
}









state="menuhardwarecontrol"


while true; do

stateok=0

mkdir -p status


if [ $state = "menuhardwarecontrol" ]; then
stateok=1
menuname="HARDWARE CONTROL - LOOP ${LOOPNAME} ($LOOPNUMBER})\n"



file="./conf/instconf_dmVmax.txt"
if [ -f $file ]; then
dmVmax=$( head -1 $file)
else
dmVmax="125"
echo "$dmVmax" > $file
fi

file="./conf/instconf_dmDCum.txt"
if [ -f $file ]; then
dmDCum=$( head -1 $file)
else
dmDCum="125"
echo "$dmDCum" > $file
fi



stringcenter "DM control  [ current: Vmax = ${dmVmax} V  DC = ${dmDCum} um ]"
menuitems=( "1 ->" "\Zb\Zr$string\Zn" )
menuitems+=( "" "" )

menuitems+=( "" "" )
menuitems+=( "dmrs" "Re-start all DM processes" )
menuitems+=( "dmk" "Kill all DM processes - do not restart" )
menuitems+=( "dmcommrs" "Re-start scexao2 -> scexao DM communication processes" )
menuitems+=( "dmcommk" "Kill scexao2 -> scexao DM communication processes" )


menuitems+=( "" "" )
if [ "$dmVmax" = " 25" ]; then
menuitems+=( "dmVmax025" "\Zr\Z2 dmVmax =  25 V  (DC level = ${dmDCum025} um)\Zn" )
else
menuitems+=( "dmVmax025" " dmVmax =  25 V  (DC level = ${dmDCum025} um)" )
fi

if [ "$dmVmax" = " 50" ]; then
menuitems+=( "dmVmax050" "\Zr\Z2 dmVmax =  50 V  (DC level = ${dmDCum050} um)\Zn" )
else
menuitems+=( "dmVmax050" " dmVmax =  50 V  (DC level = ${dmDCum050} um)" )
fi

if [ "$dmVmax" = " 75" ]; then
menuitems+=( "dmVmax075" "\Zr\Z2 dmVmax =  75 V  (DC level = ${dmDCum075} um)\Zn" )
else
menuitems+=( "dmVmax075" " dmVmax =  75 V  (DC level = ${dmDCum075} um)" )
fi

if [ "$dmVmax" = "100" ]; then
menuitems+=( "dmVmax100" "\Zr\Z2 dmVmax = 100 V  (DC level = ${dmDCum100} um)\Zn" )
else
menuitems+=( "dmVmax100" " dmVmax = 100 V  (DC level = ${dmDCum100} um)" )
fi

if [ "$dmVmax" = "125" ]; then
menuitems+=( "dmVmax125" "\Zr\Z2 dmVmax = 125 V  (DC level = ${dmDCum125} um)\Zn" )
else
menuitems+=( "dmVmax125" " dmVmax = 125 V  (DC level = ${dmDCum125} um)" )
fi

if [ "$dmVmax" = "150" ]; then
menuitems+=( "dmVmax150" "\Zr\Z2 dmVmax = 150 V  (DC level = ${dmDCum150} um)\Zn" )
else
menuitems+=( "dmVmax150" " dmVmax = 150 V  (DC level = ${dmDCum150} um)" )
fi







menuitems+=( "" "" )
stringcenter "scexao2 streams"
menuitems+=( "2 ->" "\Zb\Zr$string\Zn" )
menuitems+=( " " " " )

menuitems+=( "ir1cs" "\Zr ircam1 \Zn : (re-)start scexao2->scexao TCP transfer [port 30102]" )
menuitems+=( "ir1ck" "\Zr ircam1 \Zn : kill scexao2->scexao TCP transfer       [port 30102]" )
menuitems+=( " " " " )
menuitems+=( "ir1dcs" "\Zr ircam1_dark \Zn: (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "ir1dck" "\Zr ircam1_dark \Zn: kill scexao2->scexao TCP transfer" )
menuitems+=( " " " " )
menuitems+=( "ir2cs" "\Zr ircam2crop \Zn : (re-)start scexao2->scexao TCP transfer [port 30101]" )
menuitems+=( "ir2ck" "\Zr ircam2crop \Zn : kill scexao2->scexao TCP transfer       [port 30101]" )
menuitems+=( " " " " )
menuitems+=( "ir2dcs" "\Zr ircam2crop_dark \Zn : (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "ir2dck" "\Zr ircam2crop_dark \Zn : kill scexao2->scexao TCP transfer" )
menuitems+=( " " " " )
menuitems+=( "saphcs" "\Zr pbimagediff \Zn : (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "saphck" "\Zr pbimagediff \Zn : kill scexao2->scexao TCP transfer" )
menuitems+=( " " " " )
menuitems+=( "lj1cs" "\Zr labjack1 \Zn: (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "lj1ck" "\Zr labjack1 \Zn: kill scexao2->scexao TCP transfer" )
menuitems+=( " " " " )
menuitems+=( "lj2cs" "\Zr labjack2 \Zn: (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "lj2ck" "\Zr labjack2 \Zn: kill scexao2->scexao TCP transfer" )
menuitems+=( " " " " )
menuitems+=( "ljcs" "\Zr labjack \Zn: (re-)start scexao2->scexao TCP transfer" )
menuitems+=( "ljck" "\Zr labjack \Zn: kill scexao2->scexao TCP transfer" )





menuitems+=( "" "" )
stringcenter "scexao2 streams"
menuitems+=( "2 ->" "\Zb\Zr$string\Zn" )
menuitems+=( " " " " )



dialog --colors --title "Hardware Control" \
--ok-label "Select" \
--cancel-label "Top" \
--help-button --help-label "Exit" \
--default-item "${menuhardwarecontrol_default}" \
 --menu "$menuname" \
  $nbwlines $nbwcols 100 "${menuitems[@]}"  2> $tempfile


retval=$?
choiceval=$( head -1 $tempfile)


menualign_default="$choiceval"
state="menuhardwarecontrol"




case $retval in
   0) # button
menuhardwarecontrol_default="$choiceval"
	case $choiceval in


	dmrs)
aoconflogext "DM processes restart"
/home/scexao/bin/dmrestart
;;
	dmk)
aoconflogext "DM processes kill"
/home/scexao/bin/dmrestart -k
;;

	dmcommrs)
aoconflogext "DM scexao2 comm restart"
/home/scexao/bin/dmrestart -C
;;
	dmcommk)
aoconflogext "DM scexao2 comm kill"
/home/scexao/bin/dmrestart -c
;;


	dmVmax025)
dmVmax=" 25"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum025}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum025}
exit
EOF
;;

	dmVmax050)
dmVmax=" 50"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum050}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum050}
exit
EOF
;;

	dmVmax075)
dmVmax=" 75"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum075}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum075}
exit
EOF
;;

	dmVmax100)
dmVmax="100"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum100}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum100}
exit
EOF
;;

	dmVmax125)
dmVmax="125"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum125}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum125}
exit
EOF
;;

	dmVmax150)
dmVmax="150"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum150}" > ./conf/instconf_dmDCum.txt
Cfits << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum150}
exit
EOF
;;


# ircam1        scexao2->scexao, port 30102
	ir1cs)
aoconflogext "(re-)start ircam1 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c ircam1 30102
;;
	ir1ck)
aoconflogext "kill ircam1 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam1 30102
;;


# ircam1_dark   scexao2->scexao, port 30103
	ir1dcs)
aoconflogext "(re-)start ircam1_dark scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c ircam1_dark 30103
;;
	ir1dck)
aoconflogext "kill ircam1_dark scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam1_dark 30103
;;


# ircam2crop    scexao2->scexao, port 30101
	ir2cs)
aoconflogext "(re-)start ircam2crop scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c ircam2crop 30101
;;
	ir2ck)
aoconflogext "kill ircam1 scexao2crop -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam2crop 30101
;;

# ircam2crop    scexao2->scexao, port 30104
	ir2dcs)
aoconflogext "(re-)start ircam2crop_dark scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c ircam2crop_dark 30104
;;
	ir2dck)
aoconflogext "kill ircam1 scexao2crop_dark -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam2crop_dark 30104
;;


# pbimagediff   scexao2->scexao, port 30108
	saphcs)
aoconflogext "(re-)start pbimagediff scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c pbimagediff 30108
;;
	saphck)
aoconflogext "kill pbimagediff scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k pbimagediff 30108
;;


# labjack1      scexao2->scexao, port 30105
	lj1cs)
aoconflogext "(re-)start labjack1 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c labjack1 30105
;;
	lj1ck)
aoconflogext "kill labjack1 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k labjack1 30105
;;


# labjack2      scexao2->scexao, port 30106
	lj2cs)
aoconflogext "(re-)start labjack2 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c labjack2 30106
;;
	lj2ck)
aoconflogext "kill labjack2 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k labjack2 30106
;;


# labjack       scexao2->scexao, port 30107
	ljcs)
aoconflogext "(re-)start labjack scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -c labjack 30109
;;
	ljck)
aoconflogext "kill labjack scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k labjack 30109
;;



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
