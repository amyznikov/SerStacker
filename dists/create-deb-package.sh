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


# 1. Get current directory
workdir=$(pwd)

# 2. Create the package directory
pkg_directory="${workdir}/${pkg_fullname}"
mkdir -p "${pkg_directory}" \
	|| exit 1


# 3. install files into package directory
make -C ../build install DESTDIR="${pkg_directory}" \
	|| exit 1

# 3. install files into package directory
#make -C ../build install DESTDIR="${pkg_directory}" &&\
#	cp /usr/local/lib/libconfig* "${pkg_directory}/usr/local/lib/" \
#	|| exit 1
 
 # 4. Create the control file
mkdir -p "${pkg_directory}/DEBIAN" && \
	touch "${pkg_directory}/DEBIAN/control" \
	|| exit 1


echo "
Package: ${pkg_name}
Version: ${pkg_version}
Architecture: ${pkg_architecture}
Depends: qt5-default,qtmultimedia5,libbtbb,libtiff,libopencv,freeglut3,libglew,libcfitsio,libraw,libopenraw,libavcodec,libavformat,libavutil,libswscale,libavdevice,v4l-utils,libv4l,libusb
Maintainer: Andrey Myznikov <andrey.myznikov@gmail.com>
Description: SerStacker app.
" > "${pkg_directory}/DEBIAN/control" \
	|| exit 1

	
# 5. Build the deb package	
dpkg-deb --build --root-owner-group "${pkg_directory}" \
	|| exit 1
