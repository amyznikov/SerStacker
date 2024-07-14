/*
 * c_draw_saturn_ellipse_routine.h
 *
 *  Created on: Jul 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_draw_saturn_ellipse_routine_h__
#define __c_draw_saturn_ellipse_routine_h__

#include <core/improc/c_image_processor.h>

class c_draw_saturn_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_draw_saturn_ellipse_routine,
      "draw_saturn_ellipse", "test for drawing saturnian planetary disk ellipse");

  void set_equatorial_radius(double v)
  {
    equatorial_radius_ = v;
  }

  double equatorial_radius() const
  {
    return equatorial_radius_;
  }

  void set_ring_radius(double v)
  {
    ring_radius_ = v;
  }

  double ring_radius() const
  {
    return ring_radius_;
  }

  void set_center(const cv::Point2f & v)
  {
    center_ = v;
  }

  const cv::Point2f & center() const
  {
    return center_;
  }

  void set_orientation(const cv::Vec3d & v)
  {
    orientation_ = v;
  }

  const cv::Vec3d & orientation() const
  {
    return orientation_;
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

  const cv::Scalar & lines_color() const
  {
    return lines_color_;
  }

  void set_auto_location(bool v)
  {
    auto_location_ = v;
  }

  bool auto_location() const
  {
    return auto_location_;
  }

  void set_se_close_radius(int v)
  {
    se_close_radius_ = v;
  }

  int se_close_radius() const
  {
    return se_close_radius_;
  }

  void set_show_smask(bool v)
  {
    show_smask_ = v;
  }

  bool show_smask() const
  {
    return show_smask_;
  }

  void set_show_sbox(bool v)
  {
    show_sbox_ = v;
  }

  bool show_sbox() const
  {
    return show_sbox_;
  }


  void set_zrotation_remap(double v)
  {
    zrotation_remap_ = v;
  }

  double zrotation_remap() const
  {
    return zrotation_remap_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  double equatorial_radius_ = 130;
  double ring_radius_ = 250;

  cv::Point2f center_ =
      cv::Point2f (-1, -1);

  cv::Vec3d orientation_ =
      cv::Vec3d(90, 0, 0);

  double latidute_step_ = 30;
  double longitude_step_ = 30;

  double zrotation_remap_ = 0;

  cv::Scalar outline_color_ =
      cv::Scalar::all(255);

  cv::Scalar lines_color_ =
      cv::Scalar::all(255);

  bool auto_location_ = false;
  bool show_smask_ = false;
  bool show_sbox_ = false;

  int se_close_radius_ = 3;


};

#endif /* __c_draw_saturn_ellipse_routine_h__ */
