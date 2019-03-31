#!/bin/sh

# Check if the user is ROOT
# if [ $(id -u) -ne 0 ]
# then
#     echo "You are not ROOT! Please login as ROOT."
#     exit
# fi

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
sudo mkdir -p $MILK_ROOT

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
    sudo mkdir -p $MILK_ROOT/$dir
done;
echo ""

echo "initializing $MILK_SHM_DIR as tmpfs"
sudo tee -a /etc/fstab <<< "tmpfs $MILK_SHM_DIR tmpfs rw,nosuid,nodev"
sudo mount $MILK_SHM_DIR

