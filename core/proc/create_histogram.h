/*
 * create_histogram.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __create_histogram_h___
#define __create_histogram_h___

#include <opencv2/opencv.hpp>

// @brief build histogram for given multi-channel image.
// Output is single-channel CV_32FC1 matrix of size 'nbins rows' x 'image channels columns'.
// if input mask is not empty then it must be sigle-channel CV_8U matrix of the same size as input image.
bool create_histogram(cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    /*[in, out]*/ double * minval,
    /*[in, out]*/ double * maxval,
    int nbins = -1,
    bool cumulative = false,
    bool scaled = false);

/// @brief  convert conventional image histogram H into cumulative
///         by accumulating the bin values along rows
bool accumulate_histogram(cv::InputArray H,
    cv::OutputArray CH);

#endif /* __create_histogram_h___ */
