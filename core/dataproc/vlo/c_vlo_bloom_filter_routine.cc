/*
 * c_vlo_bloom_filter_routine.cc
 *
 *  Created on: Jan 30, 2024
 *      Author: amyznikov
 */

#include "c_vlo_bloom_filter_routine.h"
#include <core/io/vlo/c_vlo_scan.h>

void c_vlo_bloom_filter_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, intensity_measure, "intensity_measure", "intensity measure");

  BIND_CTRL(ctls, intensity_saturation_level_020m, "SAT 020m", "intensity saturation level at 20 m distance");
  BIND_CTRL(ctls, intensity_saturation_level_100m, "SAT 100m", "intensity saturation level at 100 m distance");
  BIND_CTRL(ctls, bloom_min_intensity, "bloom_min_inten", "bloom_min_intensity");

  BIND_CTRL(ctls, max_reflector_hole_size, "max_refl_hole", "max size of hole inside of single in pixels");
  BIND_CTRL(ctls, bloom_log_intensity_tolerance, "refine_tolerance", "bloom_log_intensity_tolerance");
  BIND_CTRL(ctls, intensity_tolerance, "intensity_tolerance", "intensity tolerance for selected intensity measure");

  BIND_CTRL(ctls, min_bloom_slope, "min_bloom_slope", "min profile slope");
  BIND_CTRL(ctls, max_bloom_slope, "max_bloom_slope", "max profile slope");

  BIND_CTRL(ctls, rreset, "rreset", "rreset");





  BIND_CTRL(ctls, min_distance, "min_distance", "min distance [cm]");
  BIND_CTRL(ctls, max_distance, "max_distance", "max distance [cm]");
  BIND_CTRL(ctls, distance_tolerance, "distance_tolerance", "distance tolerance [cm]");

  BIND_CTRL(ctls, min_segment_height, "min_segment_height", "[px], minimally acceptable vertical dispersion of a wall in pixels");
  BIND_CTRL(ctls, min_segment_size, "min_segment_size", "[points]");
  BIND_CTRL(ctls, max_segment_slope, "max_segment_slope", "max segment slope [deg]"); //
  BIND_CTRL(ctls, counts_threshold, "counts_threshold", "normalized counter value"); //

  BIND_CTRL(ctls, invert_selection, "invert_selection", "invert_selection");
  BIND_CTRL(ctls, mask_mode, "mask_mode", "combine selection mode");
  //BIND_CTRL(ctls, create_bloom_picture, "create_bloom_picture", "create_bloom_picture");
  BIND_CTRL(ctls, display_reflectors, "display_reflectors", "display_reflectors");
  BIND_CTRL(ctls, display_bloom, "display_bloom", "display_bloom");
}

bool c_vlo_bloom_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, intensity_measure);
    SERIALIZE_PROPERTY(settings, save, *this, intensity_saturation_level_020m);
    SERIALIZE_PROPERTY(settings, save, *this, intensity_saturation_level_100m);
    SERIALIZE_PROPERTY(settings, save, *this, bloom_min_intensity);

    SERIALIZE_PROPERTY(settings, save, *this, max_reflector_hole_size);

    SERIALIZE_PROPERTY(settings, save, *this, intensity_tolerance);
    SERIALIZE_PROPERTY(settings, save, *this, bloom_log_intensity_tolerance);
    SERIALIZE_PROPERTY(settings, save, *this, rreset);

    SERIALIZE_PROPERTY(settings, save, *this, min_bloom_slope);
    SERIALIZE_PROPERTY(settings, save, *this, max_bloom_slope);


    SERIALIZE_PROPERTY(settings, save, *this, min_distance);
    SERIALIZE_PROPERTY(settings, save, *this, max_distance);
    SERIALIZE_PROPERTY(settings, save, *this, distance_tolerance);

    SERIALIZE_PROPERTY(settings, save, *this, min_segment_height);
    SERIALIZE_PROPERTY(settings, save, *this, max_segment_slope);
    SERIALIZE_PROPERTY(settings, save, *this, min_segment_size);
    SERIALIZE_PROPERTY(settings, save, *this, counts_threshold);

    SERIALIZE_PROPERTY(settings, save, *this, invert_selection);
    SERIALIZE_PROPERTY(settings, save, *this, mask_mode);

    //SERIALIZE_PROPERTY(settings, save, *this, create_bloom_picture);
    SERIALIZE_PROPERTY(settings, save, *this, display_reflectors);
    SERIALIZE_PROPERTY(settings, save, *this, display_bloom);

    return true;
  }
  return false;
}

bool c_vlo_bloom_filter_routine::process(c_vlo_data_frame * vlo)
{
  cv::Mat bloom;
  cv::Mat reflectors;
  cv::Mat selection;
  cv::Mat debug_image;

  if( !vlo_bloom_detection(vlo->current_scan_, opts_, bloom, reflectors, debug_image) ) {
    CF_ERROR("vlo_bloom_detection() fails");
    return false;
  }

  if ( display_reflectors_ && display_bloom_ ) {
    cv::bitwise_or(bloom, reflectors, selection);
  }
  else if (display_reflectors_) {
    selection = reflectors;
  }
  else if ( display_bloom_ ) {
    selection = bloom;
  }

  if( !selection.empty() ) {

    if( invert_selection_ ) {
      cv::bitwise_not(selection, selection);
    }
  }

  vlo->update_selection(selection,
      mask_mode_);

  vlo->set_data(DataViewType_Image, "BDEBUG",
      debug_image,
      cv::noArray(),
      cv::noArray());

  return true;
}

/**
 *
  if ( create_bloom_picture_ ) {

    const cv::Size scanSize =
        vlo_scan_size(vlo->current_scan_);

    const cv::Rect roi[] = {
        cv::Rect(0,0, scanSize.width, scanSize.height), // TL
        cv::Rect(scanSize.width, 0, scanSize.width, scanSize.height), // TR
        cv::Rect(0, scanSize.height, scanSize.width, scanSize.height), // BL
        cv::Rect(scanSize.width, scanSize.height, scanSize.width, scanSize.height), // BR
    };

    cv::Mat display(cv::Size(2 * scanSize.width, 2 * scanSize.height),
        CV_8UC1,
        cv::Scalar::all(0));

    cv::Mat peak_image;

    get_vlo_image(vlo->current_scan_, VLO_DATA_CHANNEL_PEAK).convertTo(peak_image, display.depth());

    const cv::Mat ambient_image =
        get_vlo_image(vlo->current_scan_,
            VLO_DATA_CHANNEL_AMBIENT);

    cv::extractChannel(peak_image, display(roi[0]), 0); // echo 0 -> TL
    cv::extractChannel(peak_image, display(roi[2]), 1); // echo 1 -> BL
    cv::extractChannel(peak_image, display(roi[3]), 2); // echo 1 -> BR
    ambient_image.convertTo(display(roi[1]), display.type(), 255. / 16383.); // amb -> TR

    cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);


    if ( display_reflectors_ && !reflectors.empty() )  {

      cv::Mat mask_channels[3];
      cv::split(reflectors, mask_channels);

      display(roi[0]).setTo(CV_RGB(0, 255, 0), mask_channels[0]);
      display(roi[2]).setTo(CV_RGB(0, 255, 0), mask_channels[1]);
      display(roi[3]).setTo(CV_RGB(0, 255, 0), mask_channels[2]);
    }

    if ( display_bloom_ && !bloom.empty() )  {

      cv::Mat mask_channels[3];
      cv::split(bloom, mask_channels);

      display(roi[0]).setTo(CV_RGB(255, 255, 0), mask_channels[0]);
      display(roi[2]).setTo(CV_RGB(255, 255, 0), mask_channels[1]);
      display(roi[3]).setTo(CV_RGB(255, 255, 0), mask_channels[2]);
    }


    vlo->set_data(DataViewType_Image, "BLOOMPP", display, cv::noArray(), cv::noArray());

  }
 */
