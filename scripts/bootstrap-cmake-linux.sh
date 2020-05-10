#!/bin/bash

curdir=`pwd`
builddir=`pwd`/build

rm -rf ${builddir}
mkdir ${builddir}

export CXX=clang++
export CC=clang

cd ${builddir} && cmake ..
