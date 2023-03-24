#! /bin/bash
# setup_build_deps_archlinux.sh
#
#  Simple script to setup build deps under archlinux
#
#  Created on: Friday, March 24, 2023
#      Author: amyznikov
#

topdir=../..

 
pacman -S \
	cmake \
	extra-cmake-modules \
	tbb \
	libtiff \
	opencv \
	glu \
	freeglut \
	glew \
	qt5-declarative \
	qt5-imageformats \
	qt5-multimedia \
	qt5-graphicaleffects \
	qt5-websockets \
	cfitsio \
	libraw \
	libopenraw \
	ffmpeg 
