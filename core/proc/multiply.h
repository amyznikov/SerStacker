/*
 * multiply.h
 *
 *  Created on: Sep 15, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __cv_multiply_h__
#define __cv_multiply_h__

#include <opencv2/opencv.hpp>

bool multiplyImages(cv::InputArray src1, cv::InputArray src2,
    cv::OutputArray dst, int ddepth = -1 );


#endif /* __multiply_h__ */
