/*
 * inpaint.h
 *
 *  Created on: Aug 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __inpaint_h__
#define __inpaint_h__

#include <opencv2/opencv.hpp>


/// @brief average_pyramid_inpaint() is used as a fast way for pseudo-interpolation missing image pixels
/// Missimng pixels may be critical for image sharpening and alignment algorithms.
/// It is importain to fill such pixels with something reasonable desprite it may have no any physical sense.
void average_pyramid_inpaint(cv::InputArray _src,
    cv::InputArray _mask,
    cv::OutputArray dst);


#endif /* __inpaint_h__ */
