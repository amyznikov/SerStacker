/*
 * average_pyramid_inpaint.h
 *
 *  Created on: Jan 28, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __average_pyramid_inpaint_h__
#define __average_pyramid_inpaint_h__

#include <opencv2/opencv.hpp>

/// @brief average_pyramid_inpaint() is used as a fast way for pseudo-interpolation missing image pixels
/// Missing pixels may be critical for image sharpening and alignment algorithms.
/// It is important to fill such pixels with something reasonable despite it may have no any physical sense.
void average_pyramid_inpaint(cv::InputArray _src,
    cv::InputArray _mask,
    cv::OutputArray dst);


#endif /* __average_pyramid_inpaint_h__ */
