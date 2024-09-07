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
#include <core/proc/image_registration/c_saturn_ellipse_detector.h>


class c_draw_saturn_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_draw_saturn_ellipse_routine,
      "draw_saturn_ellipse", "test for drawing saturn planetary disk ellipse");

  void set_equatorial_radius(double v)
  {
    _detector.options().equatorial_radius = v;
  }

  double equatorial_radius() const
  {
    return _detector.options().equatorial_radius;
  }

  void set_ring_radius(double v)
  {
    _detector.options().ring_radius = v;
  }

  double ring_radius() const
  {
    return _detector.options().ring_radius;
  }

  void set_center(const cv::Point2f & v)
  {
    _detector.options().center = v;
  }

  const cv::Point2f & center() const
  {
    return _detector.options().center;
  }

  void set_pose(const cv::Vec3d & v)
  {
    _detector.options().pose = v;
  }

  const cv::Vec3d & pose() const
  {
    return _detector.options().pose;
  }

  void set_latidute_step(double v)
  {
    _detector.options().draw.latidute_step = v;
  }

  double latidute_step() const
  {
    return _detector.options().draw.latidute_step;
  }

  void set_longitude_step(double v)
  {
    _detector.options().draw.longitude_step = v;
  }

  double longitude_step() const
  {
    return _detector.options().draw.longitude_step;
  }

  void set_outline_color(const cv::Scalar & v)
  {
    _detector.options().draw.outline_color = v;
  }

  const cv::Scalar outline_color() const
  {
    return _detector.options().draw.outline_color;
  }

  void set_lines_color(const cv::Scalar & v)
  {
    _detector.options().draw.line_color = v;
  }

  const cv::Scalar & lines_color() const
  {
    return _detector.options().draw.line_color;
  }

  void set_auto_location(bool v)
  {
    _detector.options().auto_location = v;
  }

  bool auto_location() const
  {
    return _detector.options().auto_location;
  }

  void set_se_close_radius(int v)
  {
    _detector.options().se_close_radius = v;
  }

  int se_close_radius() const
  {
    return _detector.options().se_close_radius;
  }

  void set_show_smask(bool v)
  {
    _detector.options().draw.show_smask = v;
  }

  bool show_smask() const
  {
    return _detector.options().draw.show_smask;
  }

  void set_show_sbox(bool v)
  {
    _detector.options().draw.show_sbox = v;
  }

  bool show_sbox() const
  {
    return _detector.options().draw.show_sbox;
  }

  void set_show_ring(bool v)
  {
    _detector.options().draw.show_ring = v;
  }

  bool show_ring() const
  {
    return _detector.options().draw.show_ring;
  }

  void set_print_debug_info(bool v)
  {
    _detector.options().draw.print_debug_info = v;
  }

  bool print_debug_info() const
  {
    return _detector.options().draw.print_debug_info;
  }

  void set_zrotation_remap(double v)
  {
    _detector.options().draw.deltat = v;
  }

  double zrotation_remap() const
  {
    return _detector.options().draw.deltat;
  }

  void set_gbsigma(double v)
  {
    _detector.options().gbsigma = v;
  }

  double gbsigma() const
  {
    return _detector.options().gbsigma;
  }

  void set_stdev_factor(double v)
  {
    _detector.options().stdev_factor = v;
  }

  double stdev_factor() const
  {
    return _detector.options().stdev_factor;
  }


  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  c_saturn_ellipse_detector _detector;
};

#endif /* __c_draw_saturn_ellipse_routine_h__ */
