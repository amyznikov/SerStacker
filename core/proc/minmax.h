/*
 * minmax.h
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#ifndef __minmax_h__
#define __minmax_h__

#include <opencv2/opencv.hpp>


bool getminmax(cv::InputArray src,
    double * minval,
    double * maxal,
    cv::InputArray mask = cv::noArray());


//bool getminmax(const std::vector<cv::Mat> &src,
//    double * minval,
//    double * maxal,
//    const std::vector<cv::Mat> * masks = nullptr);


#endif /* __minmax_h__ */
