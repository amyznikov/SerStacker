/*
 * minmax.h
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#ifndef __minmax_h__
#define __minmax_h__

#include <opencv2/opencv.hpp>


bool minmax(cv::InputArray src,
    double * minval,
    double * maxal,
    cv::InputArray mask = cv::noArray());




#endif /* __minmax_h__ */
