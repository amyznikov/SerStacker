/*
 * c_melp_stereo_matcher.h
 *
 *  Created on: Jun 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_melp_stereo_matcher_h__
#define __c_melp_stereo_matcher_h__

#include <core/proc/laplacian_pyramid.h>
#include <core/proc/array2d.h>

class c_melp_stereo_matcher
{
public:

#pragma pack(push, 8)
  struct c_blockdesc
  {
    // 3x3 BGR pixels, 32 bytes = 256 bits
    union {
      uint8_t g[32];
      uint64_t u64[4];
    };
  };
#pragma pack(pop)

  class c_blockarray:
      public c_array2d<c_blockdesc>
  {
  public:
    typedef c_blockarray this_class;
    typedef c_array2d<c_blockdesc> base;

    c_blockarray()
    {
    }

    c_blockarray(int rows, int cols)
    {
      create(rows, cols);
    }

    c_blockarray(const cv::Size & s)
    {
      create(s);
    }

    c_blockarray(const c_blockarray & rhs)
    {
      this_class::operator =(rhs);
    }
  };

  struct c_block_pyramid
  {
    typedef c_block_pyramid this_class;
    typedef std::shared_ptr<this_class> sptr;
    cv::Mat1w MM[2], M;
    cv::Mat3f image;
    c_blockarray a;
    c_block_pyramid::sptr g, m;
  };

  c_melp_stereo_matcher()
  {
  }

  void set_minimum_image_size(int v)
  {
    minimum_image_size_ = std::max(2, v);
  }

  int minimum_image_size() const
  {
    return minimum_image_size_;
  }

  const c_melp_pyramid::sptr & lmelp() const
  {
    return lmelp_;
  }

  const c_melp_pyramid::sptr & rmelp() const
  {
    return rmelp_;
  }

  const c_block_pyramid::sptr & lp() const
  {
    return lp_;
  }

  const c_block_pyramid::sptr & rp() const
  {
    return rp_;
  }

  bool compute(cv::InputArray left, cv::InputArray right,
      cv::OutputArray disparity);

  static void sad(int disp,
      const c_block_pyramid::sptr & lp,
      const c_block_pyramid::sptr & rp,
      cv::Mat1f & dists);

protected:
  c_block_pyramid::sptr lp_, rp_;
  c_melp_pyramid::sptr lmelp_, rmelp_;
  int minimum_image_size_ = 4;
};

#endif /* __c_melp_stereo_matcher_h__ */
