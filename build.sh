#!/bin/bash

if [ "$1" = "install" ]
then
	sudo apt-get -y update
	sudo apt-get -y upgrade
	sudo apt-get -y install libopencv-dev
fi

flags="-O3"

g++ VisualStudio/LaneKeeping/*.cpp -o app $flags `pkg-config --cflags --libs opencv`
