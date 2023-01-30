/*
 * linear_interpolation_inpaint.h
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __linear_interpolation_inpaint_h__
#define __linear_interpolation_inpaint_h__

#include <opencv2/opencv.hpp>

/// @brief linear_interpolation_inpaint() is used as a fast way for pseudo-interpolation missing image pixels
/// Missing pixels may be critical for image sharpening and alignment algorithms.
/// It is important to fill such pixels with something reasonable despite it may have no any physical sense.
void linear_interpolation_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst);



#endif /* __linear_interpolation_inpaint_h__ */
