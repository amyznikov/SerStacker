/*
 * unpack-libcamera-image.h
 *
 *  Created on: Jan 3, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __libcamera_unpack_image_h__
#define __libcamera_unpack_image_h__

#include <opencv2/opencv.hpp>
#include <core/io/debayer.h>

bool unpack_libcamera_image(const std::vector<uint8_t> & data,
    int w, int h, int stride,
    uint32_t fourcc, uint64_t modifier,
    cv::Mat & image,
    COLORID * colorid,
    int * bpp);


#endif /* __libcamera_unpack_image_h__ */
