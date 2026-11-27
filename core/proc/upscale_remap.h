/*
 * upscale_remap.h
 *
 *  Created on: Nov 27, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __upscale_remap_h__
#define __upscale_remap_h__

#include <opencv2/opencv.hpp>

bool upscaleRemap(cv::InputArray _srcmap, cv::OutputArray _dstmap,
    const cv::Size & dstSize);


#endif /* __upscale_remap_h__ */
