#!/bin/bash

#
# Create debian package.
# https://www.internalpointers.com/post/build-binary-deb-package-practical-guide
# 
# Created: Friday, March 24, 2023
# Author:  amyznikov
# 

pkg_name=SerStacker
#pkg_version="$(fgrep 'mvlm VERSION ' ../CMakeLists.txt | sed 's/^.*[^0-9]\([0-9]*\.[0-9]*\.[0-9]*\).*$/\1/')"
pkg_version="0.0.0"
pkg_revision="1"
pkg_architecture=$(dpkg-architecture -q DEB_BUILD_ARCH)
pkg_fullname="${pkg_name}_${pkg_version}-${pkg_revision}_${pkg_architecture}"

# Get current directory
workdir=$(pwd)

# set umask, may be actual for gl-specific laptops
umask 0022

# Create the package directory
pkg_directory="${workdir}/${pkg_fullname}"
mkdir -p "${pkg_directory}" \
	|| exit 1


# Install files into package directory
make -C ../build install DESTDIR="${pkg_directory}" \
	|| exit 1

# Install additional files into package directory
#make -C ../build install DESTDIR="${pkg_directory}" &&\
#	cp /usr/local/lib/libconfig* "${pkg_directory}/usr/local/lib/" \
#	|| exit 1
 
#  the control file
mkdir -p "${pkg_directory}/DEBIAN" && \
	touch "${pkg_directory}/DEBIAN/control" \
	|| exit 1

echo "
Package: ${pkg_name}
Version: ${pkg_version}
Architecture: ${pkg_architecture}
Depends: qtbase5-dev,qtmultimedia5-dev,qt5-image-formats-plugins,libbtbb-dev,libtiff-dev,libopencv-dev,freeglut3,libglew,libcfitsio-dev,libraw-dev,libopenraw-dev,libavcodec-dev,libavformat-dev,libavutil-dev,libswscale-dev,libavdevice-dev,v4l-utils,libv4l,libusb-dev
Maintainer: Andrey Myznikov <andrey.myznikov@gmail.com>
Description: SerStacker app.
" > "${pkg_directory}/DEBIAN/control" \
	|| exit 1

	
# Build the deb package	
dpkg-deb --build --root-owner-group "${pkg_directory}" \
	|| exit 1
