/*
 * ssdesc.h
 *
 *  Created on: Apr 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __ssdesc_h__
#define __ssdesc_h__

#include <opencv2/opencv.hpp>

enum sscmpflags
{
  sscmp_g  = 0x1,
  sscmp_gx = 0x2,
  sscmp_gy = 0x4,
  sscmp_gxx  = 0x8,
  sscmp_gyy  = 0x10,
  sscmp_gxy  = 0x20,
  sscmp_a  = 0x40,
  sscmp_b  = 0x80,

  sscmp_all = \
    sscmp_g | \
    sscmp_gx | sscmp_gy | \
    sscmp_gxx | sscmp_gyy | sscmp_gxy | \
    sscmp_a | sscmp_b
};


void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc, int flags);
void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc);
void ssdesc_compare(cv::InputArray d1, cv::InputArray d2, cv::OutputArray dists);
void ssdesc_cvtfp32(const cv::Mat & desc, cv::OutputArray output);
void ssdesc_match(cv::InputArray current_descs, cv::InputArray reference_descs, int max_disparity,
    cv::OutputArray disp, cv::OutputArray errs,
    const cv::Mat1b & mask);

#endif /* __ssdesc_h__ */
