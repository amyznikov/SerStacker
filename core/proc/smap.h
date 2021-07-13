/*
 * smap.h
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __smap_h__
#define __smap_h__

#include <opencv2/opencv.hpp>

bool compute_smap(cv::InputArray src, cv::Mat & dst,
    double minv = -1, double scale = 1./16);


#endif /* __smap_h__ */
