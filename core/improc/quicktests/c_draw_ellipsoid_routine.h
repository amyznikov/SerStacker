/*
 * c_draw_ellipsoid_routine.h
 *
 *  Created on: Jul 5, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_draw_ellipsoid_routine_h__
#define __c_draw_ellipsoid_routine_h__

#include <core/improc/c_image_processor.h>

class c_draw_ellipsoid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_draw_ellipsoid_routine,
      "draw_ellipsoid", "test for drawing planetary disk ellipse");


  void set_equatorial_radius1(double v)
  {
    equatorial_radius1_ = v;
  }

  double equatorial_radius1() const
  {
    return equatorial_radius1_;
  }

  void set_equatorial_radius2(double v)
  {
    equatorial_radius2_ = v;
  }

  double equatorial_radius2() const
  {
    return equatorial_radius2_;
  }

  void set_polar_radius(double v)
  {
    polar_radius_ = v;
  }

  double polar_radius() const
  {
    return polar_radius_;
  }

  void set_orientation(const cv::Vec3d & v)
  {
    orientation_ = v;
  }

  const cv::Vec3d & orientation() const
  {
    return orientation_;
  }

  void set_center(const cv::Point2f & v)
  {
    center_ = v;
  }

  const cv::Point2f & center() const
  {
    return center_;
  }

  void set_latidute_step(double v)
  {
    latidute_step_ = v;
  }

  double latidute_step() const
  {
    return latidute_step_;
  }

  void set_longitude_step(double v)
  {
    longitude_step_ = v;
  }

  double longitude_step() const
  {
    return longitude_step_;
  }

  void set_outline_color(const cv::Scalar & v)
  {
    outline_color_ = v;
  }

  const cv::Scalar outline_color() const
  {
    return outline_color_;
  }

  void set_lines_color(const cv::Scalar & v)
  {
    lines_color_ = v;
  }

  const cv::Scalar lines_color() const
  {
    return lines_color_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double equatorial_radius1_ = 100;
  double equatorial_radius2_ = 120;
  double polar_radius_ = 70;
  cv::Vec3d orientation_;

  double latidute_step_ = 30;
  double longitude_step_ = 30;

  cv::Point2f center_ =
      cv::Point2f (-1, -1);

  cv::Scalar outline_color_ =
      cv::Scalar::all(255);

  cv::Scalar lines_color_ =
      cv::Scalar::all(255);

};

#endif /* __c_draw_ellipsoid_routine_h__ */
