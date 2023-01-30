#!/bin/bash

LeftInput=20230126_233134_GMT.in.t.avi
RightInput=20230126_233134_GMT.acc.t.avi


#ffmpeg -i ${LeftInput} -vf \
#       "[in] scale=iw:ih, pad=2*iw:ih [left]; movie=${RightInput}, scale=iw:ih, fade=out:413:30:alpha=1 [right]; [left][right] overlay=main_w/2:0 [out]" \
#       -c:v ffv1 compose1.mov



#ffmpeg -i ${LeftInput} -vf \
#       "[in] scale=iw:ih, pad=2*iw:ih [left]; movie=${RightInput}, scale=iw:ih, fade=out:300:30:alpha=1 [right]; [left][right] overlay=main_w/2:0 [out]" \
#       -c:v ffv1 compose1.mov

#LeftInput=FFSRC.avi
#RightInput=FFA.avi

#
#ffmpeg -i ${LeftInput} -vf \
#       "[in] scale=iw/2:ih/2, pad=2*iw:ih [left]; movie=${RightInput}, scale=iw/2:ih/2, fade=out:300:30:alpha=1 [right]; [left][right] overlay=m#ain_w/2:0 [out]" \
#       -c:v ffv1 compose1.mov
#


#ffmpeg -i ${LeftInput} -vf \
#       "[in] scale=iw:ih, pad=2*iw:ih [left]; movie=${RightInput}, scale=iw:ih, fade=out:300:30:alpha=1 [right]; [left][right] overlay=main_w/2:0 [out]" \
#       -c:v ffv1 compose1.mov

#ffmpeg -i ${LeftInput} -vf \
#       "[in] scale=iw:ih, pad=iw:2*ih [top]; movie=${RightInput}, scale=iw:ih, fade=out:300:30:alpha=1 [bottom]; [top][bottom] overlay=0:main_h/2 [out]" \
#       -c:v ffv1 compose1.mov


#ffmpeg -i compose1.mov -r 10 -filter_complex "[0]reverse[r];[0][r]concat,loop=4:16,setpts=N/10/TB" -c:v libx264 -crf 10 -pix_fmt yuv420p FF-TEST.mov


ffmpeg -i compose1.mov -r 30 -filter_complex "setpts=N/60/TB,tpad=stop_mode=clone:stop_duration=5" -c:v libx264 -crf 21 -pix_fmt yuv420p focus_stack.mov
