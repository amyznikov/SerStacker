/*
 * c_draw_jovian_ellipse_routine.h
 *
 *  Created on: Aug 27, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_draw_jovian_ellipse_routine_h__
#define __c_draw_jovian_ellipse_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/image_registration/c_jovian_derotation2.h>

class c_draw_jovian_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_draw_jovian_ellipse_routine,
      "draw_jovian_ellipse", "test for drawing jovian planetary disk ellipse");


  void set_equatorial_radius(double v)
  {
    _derotation.detector_options().equatorial_radius = v;
  }

  double equatorial_radius() const
  {
    return _derotation.detector_options().equatorial_radius;
  }

  void set_pca_blur(double v)
  {
    _derotation.detector_options().pca_blur = v;
  }

  double pca_blur() const
  {
    return _derotation.detector_options().pca_blur;
  }

  void set_center(const cv::Point2f & v)
  {
    _derotation.detector_options().center = v;
  }

  const cv::Point2f & center() const
  {
    return _derotation.detector_options().center;
  }

  void set_pose(const cv::Vec3d & v)
  {
    _derotation.detector_options().pose = v;
  }

  const cv::Vec3d & pose() const
  {
    return _derotation.detector_options().pose;
  }

  void set_offset(const cv::Point2f & v)
  {
    _derotation.detector_options().offset = v;
  }

  const cv::Point2f & offset() const
  {
    return _derotation.detector_options().offset;
  }

  void set_latidute_step(double v)
  {
    _derotation.detector_options().draw.latidute_step = v;
  }

  double latidute_step() const
  {
    return _derotation.detector_options().draw.latidute_step;
  }

  void set_longitude_step(double v)
  {
    _derotation.detector_options().draw.longitude_step = v;
  }

  double longitude_step() const
  {
    return _derotation.detector_options().draw.longitude_step;
  }

  void set_outline_color(const cv::Scalar & v)
  {
    _derotation.detector_options().draw.outline_color = v;
  }

  const cv::Scalar & outline_color() const
  {
    return _derotation.detector_options().draw.outline_color;
  }

  void set_lines_color(const cv::Scalar & v)
  {
    _derotation.detector_options().draw.line_color = v;
  }

  const cv::Scalar & lines_color() const
  {
    return _derotation.detector_options().draw.line_color;
  }

  void set_auto_location(bool v)
  {
    _derotation.detector_options().auto_location = v;
  }

  bool auto_location() const
  {
    return _derotation.detector_options().auto_location;
  }

  void set_se_close_radius(int v)
  {
    _derotation.detector_options().se_close_radius = v;
  }

  int se_close_radius() const
  {
    return _derotation.detector_options().se_close_radius;
  }

  void set_show_sbox(bool v)
  {
    _derotation.detector_options().draw.show_sbox = v;
  }

  bool show_sbox() const
  {
    return _derotation.detector_options().draw.show_sbox;
  }

  void set_show_smask(bool v)
  {
    _derotation.detector_options().draw.show_smask = v;
  }

  bool show_smask() const
  {
    return _derotation.detector_options().draw.show_smask;
  }

  void set_show_bmask(bool v)
  {
    _derotation.detector_options().draw.show_bmask = v;
  }

  bool show_bmask() const
  {
    return _derotation.detector_options().draw.show_bmask;
  }

  void set_show_pcax(bool v)
  {
    _derotation.detector_options().draw.show_pcax = v;
  }

  bool show_pcax() const
  {
    return _derotation.detector_options().draw.show_pcax;
  }

  void set_show_pcay(bool v)
  {
    _derotation.detector_options().draw.show_pcay = v;
  }

  bool show_pcay() const
  {
    return _derotation.detector_options().draw.show_pcay;
  }

  void set_show_wmask(bool v)
  {
    _derotation.detector_options().draw.show_wmask = v;
  }

  bool show_wmask() const
  {
    return _derotation.detector_options().draw.show_wmask;
  }

  void set_print_debug_info(bool v)
  {
    _derotation.detector_options().draw.print_debug_info = v;
  }

  bool print_debug_info() const
  {
    return _derotation.detector_options().draw.print_debug_info;
  }

  void set_zrotation_remap(double v)
  {
    _derotation.detector_options().draw.deltat = v;
  }

  double zrotation_remap() const
  {
    return _derotation.detector_options().draw.deltat;
  }

  void set_gbsigma(double v)
  {
    _derotation.detector_options().gbsigma = v;
  }

  double gbsigma() const
  {
    return _derotation.detector_options().gbsigma;
  }

  void set_stdev_factor(double v)
  {
    _derotation.detector_options().stdev_factor = v;
  }

  double stdev_factor() const
  {
    return _derotation.detector_options().stdev_factor;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_jovian_derotation2 _derotation;
};

#endif /* __c_draw_jovian_ellipse_routine_h__ */
