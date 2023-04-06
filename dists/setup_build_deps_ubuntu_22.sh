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
	qtbase5-dev qt5-qmake \
	qtmultimedia5-dev qt5-image-formats-plugins \
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

if [ ! -f /usr/lib/x86_64-linux-gnu/libusb-1.0.so ]
then 
   ln -s -f /usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so
fi
   

# libconfig
if [ ! -d ${topdir}/libconfig ]
then 
  (cd ${topdir} && \
	git clone https://github.com/hyperrealm/libconfig.git && \
	mkdir libconfig/build && cd libconfig/build && \
	cmake -D BUILD_SHARED_LIBS=OFF -D CMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
	make -j4 install && \
	make clean) || \
	exit 1
fi

