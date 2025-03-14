/*
 * autoclip.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __autoclip_h__
#define __autoclip_h__

#include <opencv2/opencv.hpp>



bool clip_range(cv::Mat & image, double min, double max,
    const cv::Mat1b & mask = cv::Mat1b());

bool clip_range(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat1b & mask = cv::Mat1b());

bool compute_clip_levels(const cv::Mat1f & cumulative_normalized_histogram,
    double minv, double maxv,
    double plo, double phi,
    /*out */ double * minval,
    /*out */ double * maxval);

bool compute_clip_levels(cv::InputArray image,
    cv::InputArray mask,
    double plo,
    double phi,
    /*out */ double * minval,
    /*out */ double * maxval);

bool autoclip(cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    double plo, double phi,
    double omin, double omax,
    double * minval = nullptr,
    double * maxval = nullptr);

bool autoclip(cv::Mat & image,
    cv::InputArray mask,
    double plo, double phi,
    double omin, double omax,
    double * minval = nullptr,
    double * maxval = nullptr);



#endif /* __autoclip_h__ */
