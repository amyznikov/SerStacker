/*
 * c_epipolar_matcher.h
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_epipolar_matcher_h__
#define __c_epipolar_matcher_h__

#include <opencv2/opencv.hpp>
#include <core/proc/array2d.h>
#include <core/settings/opencv_settings.h>


class c_epipolar_matcher
{
public:
#pragma pack(push, 8)
  struct c_pixelblock
  {
    // 21 BGR pixels (63) + 1 = 64 bytes = 512 bits
    union
    {
      uint8_t g[64];
      uint64_t u64[8];
    };
  };
#pragma pack(pop)

  class c_block_array:
      public c_array2d<c_pixelblock>
  {
  public:
    typedef c_block_array this_class;
    typedef c_array2d<c_pixelblock> base;

    c_block_array()
    {
    }

    c_block_array(int rows, int cols)
    {
      create(rows, cols);
    }

    c_block_array(const cv::Size & s)
    {
      create(s);
    }

    c_block_array(const c_block_array & rhs)
    {
      this_class::operator =(rhs);
    }
  };


  c_epipolar_matcher();

  bool compute_block_array(cv::InputArray image, cv::InputArray mask,
      c_block_array * output_block_array) const;


  bool serialize(c_config_setting settings, bool save);

};

#endif /* __c_epipolar_matcher_h__ */
