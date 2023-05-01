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
  sscmp_g00  = 0x01,
  sscmp_g01  = 0x02,
  sscmp_g02  = 0x04,
  sscmp_g03  = 0x08,
  sscmp_g04  = 0x10,
  sscmp_g05  = 0x20,
  sscmp_g06  = 0x40,
  sscmp_g07  = 0x80,

  sscmp_all = 0xFF
};


#pragma pack(push, 1)
struct ssdesc
{
  union {
    uint64_t u64;
    uint8_t  g[8];
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

void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags = sscmp_all);


void ssa_cvtfp32(const c_ssarray & ssa, cv::OutputArray output, int flags);

void ssa_compare(const c_ssarray & ssa1, const cv::Rect & rc1,
    const c_ssarray & ssa2, const cv::Rect & rc2,
    cv::OutputArray dists);

void ssa_compare(const std::vector<c_ssarray> & ssa1, const cv::Rect & rc1,
    const std::vector<c_ssarray> & ssa2, const cv::Rect & rc2,
    cv::OutputArray dists);

void ssa_match(const c_ssarray & current_descs, const c_ssarray & reference_descs, int max_disparity,
    cv::OutputArray disp, cv::OutputArray costs,
    const cv::Mat1b & mask);

void ssa_match(const std::vector<c_ssarray> & current_descs,
    const std::vector<c_ssarray> & reference_descs, int max_disparity,
    cv::OutputArray disp, cv::OutputArray costs,
    const cv::Mat1b & mask);


#endif /* __ssdesc_h__ */
