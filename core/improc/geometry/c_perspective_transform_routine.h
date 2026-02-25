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
      "perspective_transform", "Apply  to image");


  void set_src_point0(const cv::Point2f  & v)
  {
    _srcp0 = v;
    H.release();
  }

  const cv::Point2f & src_point0() const
  {
    return _srcp0;
  }

  void set_src_point1(const cv::Point2f  & v)
  {
    _srcp1 = v;
    H.release();
  }

  const cv::Point2f & src_point1() const
  {
    return _srcp1;
  }

  void set_src_point2(const cv::Point2f  & v)
  {
    _srcp2 = v;
    H.release();
  }

  const cv::Point2f & src_point2() const
  {
    return _srcp2;
  }

  void set_src_point3(const cv::Point2f  & v)
  {
    _srcp3 = v;
    H.release();
  }

  const cv::Point2f & src_point3() const
  {
    return _srcp3;
  }

  //

  void set_dst_point0(const cv::Point2f  & v)
  {
    _dstp0 = v;
    H.release();
  }

  const cv::Point2f & dst_point0() const
  {
    return _dstp0;
  }

  void set_dst_point1(const cv::Point2f  & v)
  {
    _dstp1 = v;
    H.release();
  }

  const cv::Point2f & dst_point1() const
  {
    return _dstp1;
  }

  void set_dst_point2(const cv::Point2f  & v)
  {
    _dstp2 = v;
    H.release();
  }

  const cv::Point2f & dst_point2() const
  {
    return _dstp2;
  }

  void set_dst_point3(const cv::Point2f  & v)
  {
    _dstp3 = v;
    H.release();
  }

  const cv::Point2f & dst_point3() const
  {
    return _dstp3;
  }

  //
  void set_output_image_size(const cv::Size & v)
  {
    _output_image_size = v;
    H.release();
  }

  const cv::Size & output_image_size() const
  {
    return _output_image_size;
  }

  //

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  // 3x3 Homography Matrix
  cv::Mat H;

  cv::Point2f  _srcp0;
  cv::Point2f  _srcp1;
  cv::Point2f  _srcp2;
  cv::Point2f  _srcp3;
  cv::Point2f  _dstp0;
  cv::Point2f  _dstp1;
  cv::Point2f  _dstp2;
  cv::Point2f  _dstp3;
  cv::Size _output_image_size;
};

#endif /* __c_perspective_transform_routine_h__ */
