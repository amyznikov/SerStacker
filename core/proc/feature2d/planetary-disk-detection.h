/*
 * planetary-disk-detection.h
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#ifndef __planetary_disk_detection_h__
#define __planetary_disk_detection_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>

struct c_simple_planetary_disk_detector_options
{
  double gbsigma = 1;
  double stdev_factor = 0.25;
  int se_close_radius = 2;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_simple_planetary_disk_detector_options> & ctx)
{
  using S = c_simple_planetary_disk_detector_options;
  ctlbind(ctls, "gbsigma", ctx(&S::gbsigma), "");
  ctlbind(ctls, "stdev factor", ctx(&S::stdev_factor), "");
  ctlbind(ctls, "se_close_size", ctx(&S::se_close_radius), "");
}

bool serialize_base_planetary_disk_detector_options(c_config_setting section, bool save,
    c_simple_planetary_disk_detector_options & opts);

/**
 * Threshold given grayscale input image and extract maximal area
 * connected component mask assuming it is planetary disk on a frame
 */
bool simple_planetary_disk_detector(cv::InputArray image, cv::InputArray mask,
    double gbsigma = 0,
    double stdev_factor = 0.5,
    int se_close_radius = 2,
    cv::Point2f * optional_output_centroid = nullptr,
    cv::Rect * optional_output_component_rect = nullptr,
    cv::Mat * optional_output_cmponent_mask = nullptr,
    cv::Point2f * optional_output_geometrical_center = nullptr,
    cv::Mat * optional_output_debug_image = nullptr);

inline bool simple_planetary_disk_detector(cv::InputArray image, cv::InputArray mask,
    const c_simple_planetary_disk_detector_options & opts,
    cv::Point2f * optional_output_centroid = nullptr,
    cv::Rect * optional_output_component_rect = nullptr,
    cv::Mat * optional_output_cmponent_mask = nullptr,
    cv::Point2f * optional_output_geometrical_center = nullptr,
    cv::Mat * optional_output_debug_image = nullptr)
{
  return simple_planetary_disk_detector(image, mask,
      opts.gbsigma, opts.stdev_factor, opts.se_close_radius,
      optional_output_centroid,
      optional_output_component_rect,
      optional_output_cmponent_mask,
      optional_output_geometrical_center,
      optional_output_debug_image);
}


bool detect_saturn(cv::InputArray _image, int se_close_radius, cv::RotatedRect & output_bbox,
    cv::OutputArray output_mask = cv::noArray(),
    double gbsigma = 1, double stdev_factor = 0.5);

#endif /* __planetary_disk_detection_h__ */
