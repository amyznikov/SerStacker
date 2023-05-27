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

#pragma pack(push, 8)
struct ssdesc
{
  // 512 bit (64 bytes) in total
  union {
    uint8_t  g[64];
    uint64_t u64[8];
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

bool ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel);


void ssa_cvtfp32(const c_ssarray & ssa, cv::OutputArray output);

void ssa_texture(const c_ssarray & ssa, cv::Mat1b & output_image);
void ssa_mask(const c_ssarray & ssa, cv::Mat1b & output_mask);

void ssa_compare(const std::vector<c_ssarray> & ssa1, const cv::Rect & rc1,
    const std::vector<c_ssarray> & ssa2, const cv::Rect & rc2,
    cv::OutputArray dists);

void ssa_match(const std::vector<c_ssarray> & current_descs,
    const std::vector<c_ssarray> & reference_descs, int max_disparity,
    cv::OutputArray disp, cv::OutputArray costs,
    const cv::Mat1b & mask,
    bool enable_checks);


#endif /* __ssdesc_h__ */
