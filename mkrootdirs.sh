#!/bin/sh

#Check if the user is ROOT
if [ $(id -u) -ne 0 ]
then
    echo ""
    echo "  Usage: $0 <MILK_ROOT path>"
    echo ""
    echo "This script creates the directories's architecture needed by MILK under: (ordered by priority)"
    echo " - MILK_ROOT is this variable is defined"
    echo " - <MILK_ROOT path> put as argument"
    echo " - /milk otherwhise"
    echo ""
    echo "You are not ROOT! Please login as ROOT (or sudo the command)."
    echo ""
    exit
fi

if [ -z $MILK_ROOT ]
then
    if [ -z $1 ]
    then
        MILK_ROOT=/milk
    else
        MILK_ROOT=$1
    fi
fi

echo "Create MILK directories under $MILK_ROOT"
echo ""
mkdir -p $MILK_ROOT

MILK_SHM_DIR=$MILK_ROOT/shm

echo "###########################################################"
echo "# Please define MILK_SHM_DIR as $MILK_SHM_DIR in your .bashrc #"
echo "###########################################################"
echo ""

echo "Create MILK architecture"
echo ""

for dir in "bin" "etc" "proc" "share" "shm" "tmp"; 
do 
    echo "creating $MILK_ROOT/$dir"
    mkdir -p $MILK_ROOT/$dir
done;
echo ""

echo "initializing $MILK_SHM_DIR as tmpfs"
echo "tmpfs $MILK_SHM_DIR tmpfs rw,nosuid,nodev" | tee -a /etc/fstab
mount $MILK_SHM_DIR
chmod a+rwx $MILK_ROOT
