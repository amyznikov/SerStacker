/*
 * histogram.h
 *
 *  Created on: Mar 8, 2026
 *      Author: amyznikov
 */

#ifndef ___histogram_h___
#define ___histogram_h___

#include <opencv2/opencv.hpp>

/**
 * @param src - one or more input arrays of CV kind STD_ARRAY_MAT, STD_VECTOR_VECTOR, STD_VECTOR_MAT
 * @param masks array of a single- or multi- channel input masks of the same size as src. some of mask may be empty
 * @param minv - range minimum
 * @param maxv - range maximum
 * @param number of bins in output histogram
 * @param H - output cv::Mat1f or cv::Mat1d matrix of the size num bins x num channels,
 *           the number of columns is set as max number of channels found in src
 * @param cumulative
 * @param scaled
 * @param ddepth Depth of output histogram matrix CV_32F or CV_64F
 *
 * Accepted src items may be single or multi channel data of different data type independently one from other,
 *         but all of them must have equal the number color channels
 *
 */
bool createHistogram(cv::InputArrayOfArrays src,
    cv::InputArrayOfArrays masks,
    double * minv,
    double * maxv,
    uint32_t nbins,
    cv::OutputArray H,
    bool cumulative = false,
    bool scaled = false,
    int ddepth = -1);


#endif /* ___histogram_h___ */
