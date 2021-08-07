/*
 * normalize.h
 *
 *  Created on: Jun 21, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __normalize_h__
#define __normalize_h__

#include <opencv2/opencv.hpp>

bool normalize_image(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat & mask,
    bool fillUnmaskedPixels = false,
    const cv::Scalar & unmaskedPixelsValue = cv::Scalar::all(0));

bool normalize_image(cv::InputArray src,
    cv::OutputArray dst,
    double imin,
    double imax,
    double omin,
    double omax,
    cv::InputArray mask = cv::noArray(),
    bool fillUnmaskedPixels = false,
    const cv::Scalar & unmaskedPixelsValue  = cv::Scalar::all(0));

bool normalize_minmax(cv::InputArray src,
    cv::OutputArray dst,
    double omin,
    double omax,
    cv::InputArray mask = cv::noArray(),
    bool fillUnmaskedPixels = false,
    const cv::Scalar & unmaskedPixelsValue  = cv::Scalar::all(0));


#endif /* __normalize_h__ */
