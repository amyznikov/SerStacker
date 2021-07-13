/*
 * downstrike.h
 *
 *  Created on: May 26, 2018
 *      Author: amyznikov
 */

#ifndef __downstrike_h__
#define __downstrike_h__

#include <opencv2/opencv.hpp>

/*
 * 2x downsampling step by rejecting each EVEN row and column,
 *    keep only uneven
 * */
void downstrike_even(cv::InputArray src, cv::Mat & dst);

/*
 * 2x upsampling step by injecting EVEN ZERO rows and columns ...
 * */
void upject_even(cv::InputArray _src, cv::Mat & dst, cv::Size dstSize,
    cv::Mat * _zmask = NULL, int zmdepth = -1);


/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 *    keep only even
 * */
void downstrike_uneven(cv::InputArray _src, cv::Mat & dst);


/*
 * 2x upsampling step by injecting UNEVEN ZERO rows and columns ...
 * */
void upject_uneven(cv::InputArray _src, cv::Mat & dst, cv::Size dstSize,
    cv::Mat * _zmask = NULL, int zmdepth = -1);


#endif /* __downstrike_h__ */
