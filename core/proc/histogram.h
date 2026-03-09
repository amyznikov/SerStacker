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


/**
 * Converts a regular histogram to a cumulative one.
 * If Hdst has a fixed type (e.g., cv::Mat1f was passed),
 * the result will be converted to it. Otherwise, the type is inherited from Hsrc.
 *  */
void makeCumulativeHistogram(cv::InputArray Hsrc,
    cv::OutputArray Hdst);


/**
* Normalizes each histogram column independently to the range [0, 1].
* If isCumulative == true, normalization is performed by the last element of the column.
* Otherwise, normalization is performed by the maximum value in the column.
*  */
void normalizeHistogram(cv::InputArray Hsrc, cv::OutputArray Hdst,
    bool isCumulative = false);

/**
* Compute the actual clipping levels for each channel.
*
* @param cumulativeNormalizedHistogram - cumulative normalized histogram (CV_64F, nbins x cn)
* @param vMin - minimum histogram scale (from createHistogram)
* @param vMax - maximum histogram scale (from createHistogram)
* @param qLow - lower quantile (e.g., 0.01 for 1%)
* @param qHigh - upper quantile (e.g., 0.99 for 99%)
* @param realLow - [out] low clipping levels by channel
* @param realHigh - [out] high clipping levels by channel
*  */
bool computeHistogramClipLevels(cv::InputArray cumulativeNormalizedHistogram,
    double vMin, double vMax,
    double qLow, double qHigh,
    cv::Scalar & realLow,
    cv::Scalar & realHigh);


/**
 * Estimate Median from cumulative normalized histogram
 *
 * @param cumulativeNormalizedHistogram - input cumulative normalized histogram (CV_32F or CV_64F, nbins x cn)
 * @param minv - minimum histogram scale (from createHistogram)
 * @param maxv - maximum histogram scale (from createHistogram)
 */
cv::Scalar computeHistogramMedian(cv::InputArray cumulativeNormalizedHistogram,
    double minv, double maxv);


/**
 * Estimate Mode from image histogram
 *
 * @param H - input image histogram (CV_32F or CV_64F, nbins x cn)
 * @param minv - minimum histogram scale (from createHistogram)
 * @param maxv - maximum histogram scale (from createHistogram)
 */
cv::Scalar computeHistogramMode(cv::InputArray H, double minv, double maxv);


/**
* @param qLow - lower quantile (e.g., 0.01 for 1%)
* @param qHigh - upper quantile (e.g., 0.99 for 99%)
*  */
bool histogramClipWhiteBalance(cv::InputArray src, cv::InputArray srcMask,
    cv::OutputArray dst,
    double qlow, double qhigh);

#endif /* ___histogram_h___ */
