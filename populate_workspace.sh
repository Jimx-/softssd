#!/bin/bash

DIR=`pwd`

FTL_PROJ_NAME=nvme_ftl
FIL_PROJ_NAME=nvme_fil
ECC_PROJ_NAME=nvme_ecc

WORKSPACE=$1

rm -rf $WORKSPACE/$FTL_PROJ_NAME/src
rm -rf $WORKSPACE/$FTL_PROJ_NAME/include

ln -s $DIR/src $WORKSPACE/$FTL_PROJ_NAME/src
ln -s $DIR/include $WORKSPACE/$FTL_PROJ_NAME/include

rm -rf $WORKSPACE/$FIL_PROJ_NAME/src
ln -s $DIR/r5poll $WORKSPACE/$FIL_PROJ_NAME/src

rm -rf $WORKSPACE/$ECC_PROJ_NAME/src
ln -s $DIR/eccengine $WORKSPACE/$ECC_PROJ_NAME/src
