/*
 * histogram_white_balance.h
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __histogram_white_balance_h__
#define __histogram_white_balance_h__

#include <opencv2/opencv.hpp>

bool histogram_white_balance(cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    double cl = 1,
    double ch = 99);


#endif /* __histogram_white_balance_h__ */
