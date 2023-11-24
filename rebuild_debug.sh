#!/usr/bin/env bash

cd bootloader
make clean
DEBUG=1 make
cd ..
cd app
make clean
DEBUG=1 make
cd ..

