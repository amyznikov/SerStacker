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

// For maskBorderMode:
//  cv::BORDER_CONSTANT - fill unmasked pixels with unmaskedPixelsValue
//  cv::BORDER_TRANSPARENT - process unmasked pixels in the same way as regular masked pixels
//  cv::BORDER_ISOLATED - do not touch unmasked pixels


bool normalize_image(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat & mask,
    enum cv::BorderTypes maskBorderMode = cv::BORDER_TRANSPARENT,
    const cv::Scalar & unmaskedPixelsValue = cv::Scalar::all(0));

bool normalize_image(cv::InputArray src,
    cv::OutputArray dst,
    double imin,
    double imax,
    double omin,
    double omax,
    cv::InputArray mask = cv::noArray(),
    enum cv::BorderTypes maskBorderMode = cv::BORDER_TRANSPARENT,
    const cv::Scalar & unmaskedPixelsValue  = cv::Scalar::all(0));

bool normalize_minmax(cv::InputArray src,
    cv::OutputArray dst,
    double omin,
    double omax,
    cv::InputArray mask = cv::noArray(),
    enum cv::BorderTypes maskBorderMode = cv::BORDER_TRANSPARENT,
    const cv::Scalar & unmaskedPixelsValue  = cv::Scalar::all(0));


#endif /* __normalize_h__ */
