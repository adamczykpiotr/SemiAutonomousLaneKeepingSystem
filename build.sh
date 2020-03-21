#!/bin/bash

if [ "$1" = "install" ]
then
	sudo apt-get -y update
	sudo apt-get -y upgrade
	sudo apt-get -y install libopencv-dev
fi

flags="-O3 -std=c++17"

g++ VisualStudio/LaneKeeping/*.cpp -o app $flags `pkg-config --cflags --libs opencv`
