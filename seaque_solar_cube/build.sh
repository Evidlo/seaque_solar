#!/bin/bash

case "$1" in 
	clean|--clean)
		rm -rf build/
		;;
esac

mkdir -p build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake .. && make -j$(nproc)
