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



struct c_epipolar_matcher_options
{
  int median_hat_radius = 15;
  int median_hat_threshold = 10;
  int median_hat_close_radius = 2;
  int max_disparity = 160;
  bool enabled = false;
};

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
  c_epipolar_matcher(const c_epipolar_matcher_options & opts);

  void set_options(const c_epipolar_matcher_options & opts);
  const c_epipolar_matcher_options & options() const;
  c_epipolar_matcher_options & options();

  bool enabled() const;
  void set_enabled(bool v);

  bool compute_block_array(cv::InputArray image, cv::InputArray mask,
      c_block_array * output_block_array,
      cv::Mat * output_median_hat = nullptr,
      cv::Mat1b * output_median_hat_mask = nullptr ) const;

  bool compute_matches(const c_block_array & current_image, const c_block_array & previous_image,
      const cv::Mat1b & mask,
      const cv::Point2d & epipole_location);

  const cv::Mat & current_median_hat() const ;
  const cv::Mat & previous_median_hat() const ;
  const cv::Mat1b & current_median_hat_mask() const ;
  const cv::Mat1b & previous_median_hat_mask() const ;
  const cv::Mat2f & matches() const;
  const cv::Mat2i & back_matches() const;
  const cv::Mat1w & costs() const;


  bool serialize(c_config_setting settings, bool save);

protected:
  static void compute_search_range(const cv::Size & image_size, const cv::Point2d & E, int rx, int ry,
      int max_disparity, int * incx, int * incy, int * cxmax, int * cymax,  double * k);

protected:
  c_epipolar_matcher_options options_;
  cv::Mat current_median_hat_;
  cv::Mat previous_median_hat_;
  cv::Mat1b current_median_hat_mask_;
  cv::Mat1b previous_median_hat_mask_;
  cv::Mat2f matches_;
  cv::Mat2i back_matches_;
  cv::Mat1w costs_;
};


cv::Mat1f matchesToEpipolarDisparity(const cv::Mat2f & matches,
    const cv::Point2d & epipole_position);



#endif /* __c_epipolar_matcher_h__ */
