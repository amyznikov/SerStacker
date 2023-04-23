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
#include <core/proc/array2d.h>

enum sscmpflags
{
  sscmp_g  = 0x01,
  sscmp_a  = 0x02,
  sscmp_b  = 0x04,
  sscmp_gx = 0x08,
  sscmp_gy = 0x10,
  sscmp_gxx  = 0x20,
  sscmp_gyy  = 0x40,
  sscmp_gxy  = 0x80,

  sscmp_all = \
    sscmp_g | \
    sscmp_a | sscmp_b | \
    sscmp_gx | sscmp_gy | \
    sscmp_gxx | sscmp_gyy | sscmp_gxy
};


#pragma pack(push, 1)
struct ssdesc
{
  union {
    uint64_t u64;
    struct {
      uint8_t g;
      uint8_t a;
      uint8_t b;
      uint8_t gx;
      uint8_t gy;
      uint8_t gxx;
      uint8_t gyy;
      uint8_t gxy;
    };
  };
};
#pragma pack(pop)

class c_ssarray :
    public c_array2d<ssdesc>
{
public :
  typedef c_ssarray this_class;
  typedef c_array2d<ssdesc> base;

  c_ssarray()
  {
  }

  c_ssarray(int rows, int cols)
  {
    create(rows, cols);
  }

  c_ssarray(const cv::Size & s)
  {
    create(s);
  }

  c_ssarray(const c_ssarray & rhs)
  {
    this_class :: operator = (rhs);
  }
};


void ssa_compute(const cv::Mat3b & image, c_ssarray & ssa, int flags = sscmp_all, double ss_sigma = 2, int ss_radius = 0);
void ssa_cvtfp32(const c_ssarray & ssa, cv::OutputArray output, int flags);

void ssa_compare(const c_ssarray & ssa1, const cv::Rect & rc1,
    const c_ssarray & ssa2, const cv::Rect & rc2,
    cv::OutputArray dists);

void ssa_match(const c_ssarray & current_descs, const c_ssarray & reference_descs, int max_disparity,
    cv::OutputArray disp, cv::OutputArray costs,
    const cv::Mat1b & mask);

//void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc, int flags = sscmp_all);
//void ssdesc_cvtfp32(const cv::Mat & desc, cv::OutputArray output, int flags);
//void ssdesc_compare(cv::InputArray d1, cv::InputArray d2, cv::OutputArray dists);
//void ssdesc_match(cv::InputArray current_descs, cv::InputArray reference_descs, int max_disparity,
//    cv::OutputArray disp, cv::OutputArray costs,
//    const cv::Mat1b & mask);

#endif /* __ssdesc_h__ */
