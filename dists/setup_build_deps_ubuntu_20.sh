#! /bin/bash
# setup_build_deps_ubuntu.sh
#
#  Simple script to setup build deps under ubuntu.
#  Not actually tested yet
#
#  Created on: Friday, March 24, 2023
#      Author: amyznikov
#

topdir=../..

apt-get install -y \
	git \
	cmake \
	autoconf \
	automake \
	dpkg \
	libtool \
	qt5-default \
	qtmultimedia5-dev \
	libbtbb-dev \
	libtiff-dev \
	libopencv-dev \
	freeglut3-dev \
	libglew-dev \
	libcfitsio-dev \
	libraw-dev \
	libopenraw-dev \
	libavcodec-dev \
	libavformat-dev \
	libavutil-dev \
	libswscale-dev \
	libavdevice-dev \
	v4l-utils \
	libv4l-dev \
	libusb-dev \
	|| exit 1

# libconfig
(cd ${topdir} && \
	git clone https://github.com/hyperrealm/libconfig.git && \
	mkdir libconfig/build && cd libconfig/build && \
	cmake -D BUILD_SHARED_LIBS=OFF -D CMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
	make -j4 install && \
	make clean) || \
	exit 1

