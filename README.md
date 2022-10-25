# SerStacker
Highly experimental code for planetary images stacking with OpenCV.

## Example pictures

Compare jovian images captured with two different tubes, processed and derotated with SerStacker.

![compare-scopes](./debug/jovian-derotation/compare-scopes.jpg)


Jovian rotation captured at 2022-10-21 1930 with Arsenal-GSO N305/1200 f/4, ocular projection using x20 objective lens from microscope.


https://user-images.githubusercontent.com/1285263/197896096-c0db0918-69b4-436a-8746-0a7f94ff5705.mov





Montes Alpes and Mare Frigoris.
![MontesAlpes](./debug/MontesAlpes.2021-09-26-2327_8-CapObj-32F.jpg)



This code is intendend mainly for my experimentation.
It is totally bugged and constantly changed. 

Examples of stacked images on my flickr account : 
  https://www.flickr.com/photos/116211323@N02/.

Examples of raw stacks (saved in TIFF format as 32-bit floats) on my google drive : 
  https://drive.google.com/drive/folders/16Ko6ebfuFToQyXbqewETryaVsAN_rmVS?usp=sharing

Jovian derotation+blend quick demo video:
  https://www.youtube.com/watch?v=ARGoWo2w8YY

## Build deps

Qt5 >= 5.10  (qt5-base qt5-declarative qt5-imageformats qt5-multimedia qt5-graphicaleffects)

OpenCV >= 4.2

TBB

libconfig >= 1.7 

libtiff

libraw >= 0.20 (https://www.libraw.org)

libopenraw-0.3 (https://libopenraw.freedesktop.org)

cfitsio

libavcodec

libavformat

libavutil

libavdevice

libswscale

For glddm team:

	qt5-opengl 
	libQGLViewer  (http://libqglviewer.com)
	freeglut
	glew 2.2


Dependency packages for Ubuntu:

libtiff5-dev libraw-dev libopenraw-dev libcfitsio-dev 
libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev libswscale-dev
qt5-default qtmultimedia5-dev 





