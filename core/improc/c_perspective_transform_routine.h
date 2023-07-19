/*
 * c_perspective_transform_routine.h
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_perspective_transform_routine_h__
#define __c_perspective_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_perspective_transform_routine :
    public c_image_processor_routine
{
public:

  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_perspective_transform_routine,
      "perspective_transform_", "Apply  to image");


  void set_src_point0(const cv::Point2f  & v)
  {
    src_pts[0] = v;
    H.release();
  }

  const cv::Point2f & src_point0() const
  {
    return src_pts[0];
  }

  void set_src_point1(const cv::Point2f  & v)
  {
    src_pts[1] = v;
    H.release();
  }

  const cv::Point2f & src_point1() const
  {
    return src_pts[1];
  }

  void set_src_point2(const cv::Point2f  & v)
  {
    src_pts[2] = v;
    H.release();
  }

  const cv::Point2f & src_point2() const
  {
    return src_pts[2];
  }

  void set_src_point3(const cv::Point2f  & v)
  {
    src_pts[3] = v;
    H.release();
  }

  const cv::Point2f & src_point3() const
  {
    return src_pts[3];
  }

  //

  void set_dst_point0(const cv::Point2f  & v)
  {
    dst_pts[0] = v;
    H.release();
  }

  const cv::Point2f & dst_point0() const
  {
    return dst_pts[0];
  }

  void set_dst_point1(const cv::Point2f  & v)
  {
    dst_pts[1] = v;
    H.release();
  }

  const cv::Point2f & dst_point1() const
  {
    return dst_pts[1];
  }

  void set_dst_point2(const cv::Point2f  & v)
  {
    dst_pts[2] = v;
    H.release();
  }

  const cv::Point2f & dst_point2() const
  {
    return dst_pts[2];
  }

  void set_dst_point3(const cv::Point2f  & v)
  {
    dst_pts[3] = v;
    H.release();
  }

  const cv::Point2f & dst_point3() const
  {
    return dst_pts[3];
  }

  //
  void set_output_image_size(const cv::Size & v)
  {
    output_image_size_ = v;
    H.release();
  }

  const cv::Size & output_image_size() const
  {
    return output_image_size_;
  }

  //

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Mat H; // 3x3 Homography Matrix
  cv::Point2f  src_pts[4];
  cv::Point2f  dst_pts[4];
  cv::Size output_image_size_;
};

#endif /* __c_perspective_transform_routine_h__ */
