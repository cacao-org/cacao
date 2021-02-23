#!/bin/bash

execname="./AOloopControl"


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



file="./conf/instconf_EMgain.txt"
if [ -f $file ]; then
EMgain=$( head -1 $file)
else
EMgain="1"
echo "$EMgain" > $file
fi


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
if [ "$EMgain" = "  0" ]; then
menuitems+=( "em0" "\Zr\Z2 EMgain =   0\Zn" )
else
menuitems+=( "em0" " EMgain =   0" )
fi

menuitems+=( "" "" )
if [ "$EMgain" = "  1" ]; then
menuitems+=( "em1" "\Zr\Z2 EMgain =   1\Zn" )
else
menuitems+=( "em1" " EMgain =   1" )
fi

if [ "$EMgain" = "  2" ]; then
menuitems+=( "em2" "\Zr\Z2 EMgain =   2\Zn" )
else
menuitems+=( "em2" " EMgain =   2" )
fi

if [ "$EMgain" = "  4" ]; then
menuitems+=( "em4" "\Zr\Z2 EMgain =   4\Zn" )
else
menuitems+=( "em4" " EMgain =   4" )
fi

if [ "$EMgain" = "  8" ]; then
menuitems+=( "em8" "\Zr\Z2 EMgain =   8\Zn" )
else
menuitems+=( "em8" " EMgain =   8" )
fi

if [ "$EMgain" = " 16" ]; then
menuitems+=( "em16" "\Zr\Z2 EMgain =  16\Zn" )
else
menuitems+=( "em16" " EMgain =  16" )
fi

if [ "$EMgain" = " 32" ]; then
menuitems+=( "em32" "\Zr\Z2 EMgain =  32\Zn" )
else
menuitems+=( "em32" " EMgain =  32" )
fi

if [ "$EMgain" = " 64" ]; then
menuitems+=( "em64" "\Zr\Z2 EMgain =  64\Zn" )
else
menuitems+=( "em64" " EMgain =  64" )
fi

if [ "$EMgain" = "128" ]; then
menuitems+=( "em128" "\Zr\Z2 EMgain = 128\Zn" )
else
menuitems+=( "em128" " EMgain = 128" )
fi

if [ "$EMgain" = "256" ]; then
menuitems+=( "em256" "\Zr\Z2 EMgain = 256\Zn" )
else
menuitems+=( "em256" " EMgain = 256" )
fi

if [ "$EMgain" = "512" ]; then
menuitems+=( "em512" "\Zr\Z2 EMgain = 512\Zn" )
else
menuitems+=( "em512" " EMgain = 512" )
fi



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
stringcenter "scexaoRTC streams"
menuitems+=( "2 ->" "\Zb\Zr$string\Zn" )
menuitems+=( " " " " )

menuitems+=( "ir0cs" "\Zr ircam0 \Zn : (re-)start scexaoCTRL->scexaoRTC TCP transfer [port 30101]" )
menuitems+=( "ir0ck" "\Zr ircam0 \Zn : kill scexaoCTRL->scexaoRTC TCP transfer       [port 30101]" )
menuitems+=( " " " " )
menuitems+=( "ir0ds" "\Zr ircam0_dark \Zn : (re-)start scexaoCTRL->scexaoRTC TCP transfer [port 30102]" )
menuitems+=( "ir0dk" "\Zr ircam0_dark \Zn : kill scexaoCTRL->scexaoRTC TCP transfer       [port 30102]" )
menuitems+=( " " " " )
menuitems+=( "ir1cs" "\Zr ircam1 \Zn : (re-)start scexaoCTRL->scexaoRTC TCP transfer [port 30103]" )
menuitems+=( "ir1ck" "\Zr ircam1 \Zn : kill scexaoCTRL->scexaoRTC TCP transfer       [port 30103]" )
menuitems+=( " " " " )
#menuitems+=( "ir2cs" "\Zr ircam2 \Zn : (re-)start scexaoCTRL->scexaoRTC TCP transfer [port 30104]" )
#menuitems+=( "ir2ck" "\Zr ircam2 \Zn : kill scexaoCTRL->scexaoRTC TCP transfer       [port 30104]" )
#menuitems+=( " " " " )
menuitems+=( "vcam0s" "\Zr vcamim0 \Zn : (re-)start scexao4->scexaoRTC TCP transfer [port 30104]" )
menuitems+=( "vcam0k" "\Zr vcamim0 \Zn : kill scexao4->scexaoRTC TCP transfer       [port 30104]" )
menuitems+=( " " " " )
menuitems+=( "vcam1s" "\Zr vcamim1 \Zn : (re-)start scexao4->scexaoRTC TCP transfer [port 30105]" )
menuitems+=( "vcam1k" "\Zr vcamim1 \Zn : kill scexao4->scexaoRTC TCP transfer       [port 30105]" )
menuitems+=( " " " " )

#menuitems+=( "ir2cs" "\Zr ircam2crop \Zn : (re-)start scexao2->scexao TCP transfer [port 30101]" )
#menuitems+=( "ir2ck" "\Zr ircam2crop \Zn : kill scexao2->scexao TCP transfer       [port 30101]" )
#menuitems+=( " " " " )
#menuitems+=( "ir2dcs" "\Zr ircam2crop_dark \Zn : (re-)start scexao2->scexao TCP transfer" )
#menuitems+=( "ir2dck" "\Zr ircam2crop_dark \Zn : kill scexao2->scexao TCP transfer" )
#menuitems+=( " " " " )
#menuitems+=( "saphcs" "\Zr pbimagediff \Zn : (re-)start scexao2->scexao TCP transfer" )
#menuitems+=( "saphck" "\Zr pbimagediff \Zn : kill scexao2->scexao TCP transfer" )
#menuitems+=( " " " " )
#menuitems+=( "lj1cs" "\Zr labjack1 \Zn: (re-)start scexao2->scexao TCP transfer" )
#menuitems+=( "lj1ck" "\Zr labjack1 \Zn: kill scexao2->scexao TCP transfer" )
#menuitems+=( " " " " )
#menuitems+=( "lj2cs" "\Zr labjack2 \Zn: (re-)start scexao2->scexao TCP transfer" )
#menuitems+=( "lj2ck" "\Zr labjack2 \Zn: kill scexao2->scexao TCP transfer" )
#menuitems+=( " " " " )
#menuitems+=( "ljcs" "\Zr labjack \Zn: (re-)start scexao2->scexao TCP transfer" )
#menuitems+=( "ljck" "\Zr labjack \Zn: kill scexao2->scexao TCP transfer" )







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


	em0)
EMgain="  0"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em1)
EMgain="  1"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em2)
EMgain="  2"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em4)
EMgain="  4"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em8)
EMgain="  8"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em16)
EMgain=" 16"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em32)
EMgain=" 32"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em64)
EMgain=" 64"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em128)
EMgain="128"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em256)
EMgain="256"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;

	em512)
EMgain="512"
aoconflogext "Set EMgain = ${EMgain}"
echo "${EMgain}" > ./conf/instconf_EMgain.txt
ssh scexao@scexao4 "/home/scexao/bin/ocam2k_gain ${EMgain}"
;;




	dmVmax025)
dmVmax=" 25"
aoconflogext "Set DM max = $dmVmax V"
echo "${dmVmax}" > ./conf/instconf_dmVmax.txt
echo "${dmDCum025}" > ./conf/instconf_dmDCum.txt
$execname << EOF
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
$execname << EOF
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
$execname << EOF
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
$execname << EOF
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
$execname << EOF
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
$execname << EOF
aolsetdmvoltmax 00 ${dmVmax}
aolsetdmDC 00 ${dmDCum150}
exit
EOF
;;


# ircam0        scexaoCTRL->scexao, port 30101
	ir0cs)
aoconflogext "(re-)start ircam0 scexaoCTRL -> scexao TCP transfer"
ssh scexao@scexaoCTRL "/home/scexao/src/hardw-cred2/src/imgtakeCPUconf"
#/home/scexao/bin/getTCPscexao2im -s ircam -r ircam0com ircam0 30101
/home/scexao/bin/getTCPscexao2im -s ircam  ircam0 30101
;;
	ir0ck)
aoconflogext "kill ircam0 scexaoCTRL -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam0 30101
;;



# ircam0_dark        scexaoCTRL->scexao, port 30102
	ir0ds)
aoconflogext "(re-)start ircam0_dark scexaoCTRL -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -s ircam  ircam0_dark 30102
;;
	ir0dk)
aoconflogext "kill ircam0_dark scexaoCTRL -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam0_dark 30102
;;



# ircam1      scexaoCTRL->scexao, port 30103
	ir1cs)
aoconflogext "(re-)start ircam0 scexaoCTRL -> scexao TCP transfer"
ssh scexao@scexaoCTRL "/home/scexao/src/hardw-cred2/src/imgtakeCPUconf"
/home/scexao/bin/getTCPscexao2im -s ircam -r ircam1com ircam1 30103
;;
	ir1ck)
aoconflogext "kill ircam0 scexao2 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam1 30103
;;




# ircam2        scexaoCTRL->scexao, port 30103
	ir2cs)
aoconflogext "(re-)start ircam0 scexaoCTRL -> scexao TCP transfer"
ssh scexao@scexaoCTRL "/home/scexao/src/hardw-cred2/src/imgtakeCPUconf"
#/home/scexao/bin/getTCPscexao2im -s ircam -r ircam2com ircam2 30103
/home/scexao/bin/getTCPscexao2im -s ircam ircam2 30103
;;
	ir2ck)
aoconflogext "kill ircam0 scexaoCTRL -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao2im -k ircam2 30103
;;



# vcamim0        scexao4->scexao, port 30104
	vcam0s)
aoconflogext "(re-)start vcamim0 scexao4 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao4im -s vcamim0 -r vcamim0com vcamim0 30104
;;
	vcam0k)
aoconflogext "kill vcamim0 scexao4 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao4im -k vcamim0 30104
;;

# vcamim1        scexao4->scexao, port 30105
	vcam1s)
aoconflogext "(re-)start vcamim1 scexao4 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao4im -s vcamim1 -r vcamim1com vcamim1 30105
;;
	vcam1k)
aoconflogext "kill vcamim1 scexao4 -> scexao TCP transfer"
/home/scexao/bin/getTCPscexao4im -k vcamim1 30105
;;




# pbimagediff   scexao2->scexao, port 30108
#	saphcs)
#aoconflogext "(re-)start pbimagediff scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -c pbimagediff 30108
#;;
#	saphck)
#aoconflogext "kill pbimagediff scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -k pbimagediff 30108
#;;


# labjack1      scexao2->scexao, port 30105
#	lj1cs)
#aoconflogext "(re-)start labjack1 scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -c labjack1 30105
#;;
#	lj1ck)
#aoconflogext "kill labjack1 scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -k labjack1 30105
#;;


# labjack2      scexao2->scexao, port 30106
#	lj2cs)
#aoconflogext "(re-)start labjack2 scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -c labjack2 30106
#;;
#	lj2ck)
#aoconflogext "kill labjack2 scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -k labjack2 30106
#;;


# labjack       scexao2->scexao, port 30107
#	ljcs)
#aoconflogext "(re-)start labjack scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -c labjack 30109
#;;
#	ljck)
#aoconflogext "kill labjack scexao2 -> scexao TCP transfer"
#/home/scexao/bin/getTCPscexao2im -k labjack 30109
#;;



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
