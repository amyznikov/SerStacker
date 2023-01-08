/*
 * weighted_mean.h
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef _weighte_mean_h__
#define _weighte_mean_h__

#include <opencv2/opencv.hpp>

cv::Scalar weighted_mean(cv::InputArray src,
    cv::InputArray weights);


#endif /* _weighte_mean_h__ */
