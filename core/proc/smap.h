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
    int lksize, int scale_size, double minv);

bool compute_dogsmap(cv::InputArray src, cv::Mat & dst,
    double s1, double s2, int scale, double minv);

bool compute_dogsmapc(cv::InputArray src, cv::Mat & dst,
    double s1, double s2, int scale, double minv);

#endif /* __smap_h__ */
