/*
 * lpg.h
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 *
 *
 *  This routine creates map from weighted sum of
 *   squared laplacian and squared gradient magnitude.
 *
 *
 *  The intended purpose is for estimation of textured regions for some scenarios like stereo matching and focus measures.
 *  Pure gradient is poor near extremums, pure laplacian is poor on flat gradients.
 *  The sum looks quite good for most of scenarios.
 *
 *
 *
 */

#pragma once
#ifndef __lpg_h__
#define __lpg_h__

#include <opencv2/opencv.hpp>


/**
 * lpg:
 *
 *  Create the map of
 *      alpha * laplacian ^ 2 +  beta * |gradient| ^ 2
 *
 * with
 *    alpha = k / ( k + 1)
 *    beta = 1 / (k + 1)
 *
 *
 *
 * For noise filtering purposes the input image can be optionally downscaled using cv::pyrDown(),
 * and the output map can be upscaled using cv::pyrUp().
 *
 * When average_color_channels is true the color channels of input image are averaged into
 * single channel before processing.
 *
 */

bool lpg(cv::InputArray image, cv::InputArray mask, cv::OutputArray optional_output_map,
    double k, int dscale, int uscale, bool squared, bool average_color_channels,
    cv::Scalar * optional_output_sharpness_metric = nullptr);




#endif /* __lpg_h__ */
