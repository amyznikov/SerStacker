/*
 * gegl_mean_curvature_blur.h
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 *
 *  Derived from gegl/operations/common/mean-curvature-blur.c
 *
 */

#pragma once
#ifndef __gegl_mean_curvature_blur_h__
#define __gegl_mean_curvature_blur_h__

#include <opencv2/opencv.hpp>


void mean_curvature_blur(cv::InputArray src, cv::OutputArray dst, int iterations);



#endif /* __gegl_mean_curvature_blur_h__ */
