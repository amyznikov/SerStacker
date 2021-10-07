# SerStacker
Highly experimental code for planetary images stacking with OpenCV.

This code is intendend mainly for my experimentation only.
It is highly bugged and constantly changed. 

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

## Example pictures

Compare jovian images captured with two different tubes, processed and derotated with this SerStacker.

![compare-scopes](./debug/jovian-derotation/compare-scopes.jpg)






