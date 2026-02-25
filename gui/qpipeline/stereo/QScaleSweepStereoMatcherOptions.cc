/*
 * QScaleSweepStereoMatcherOptions.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "QScaleSweepStereoMatcherOptions.h"

QScaleSweepStereoMatcherOptions::QScaleSweepStereoMatcherOptions(QWidget * parent) :
  Base(parent)
{
  enable_stereo_matching_ctl =
      add_checkbox("enable stereo matching",
          "",
          [this](bool checked) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().enable_stereo_matchning = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _pipeline ) {
              * checked = _pipeline->stereo_matching_options().enable_stereo_matchning;
              return true;
            }
            return false;
          });

  max_disparity_ctl =
      add_numeric_box<int>("max_disparity:",
          "",
          [this](int value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().max_disparity = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().max_disparity;
              return true;
            }
            return false;
          });

  max_scale_ctl =
      add_numeric_box<int>("max_scale:",
          "",
          [this](int value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().max_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().max_scale;
              return true;
            }
            return false;
          });

  texture_threshold_ctl =
      add_numeric_box<int>("text. thresh:",
          "",
          [this](int value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().texture_threshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().texture_threshold;
              return true;
            }
            return false;
          });

  disp12maxDiff_ctl =
      add_numeric_box<int>("disp12maxDiff::",
          "",
          [this](int value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().disp12maxDiff = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().disp12maxDiff;
              return true;
            }
            return false;
          });

  save_debug_images_ctl =
      add_checkbox("save_debug_images",
          "",
          [this](bool checked) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().save_debug_images = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _pipeline ) {
              * checked = _pipeline->stereo_matching_options().save_debug_images;
              return true;
            }
            return false;
          });

  process_only_debug_frames_ctl =
      add_checkbox("process_only_debug_frames",
          "",
          [this](bool checked) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().process_only_debug_frames = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _pipeline ) {
              * checked = _pipeline->stereo_matching_options().process_only_debug_frames;
              return true;
            }
            return false;
          });

  debug_frames_ctl =
      add_numeric_box<std::vector<int>>("debug frames:",
          "",
          [this](const std::vector<int> & value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().debug_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](std::vector<int> * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().debug_frames;
              return true;
            }
            return false;
          });

  debug_points_ctl =
      add_numeric_box<std::vector<cv::Point>>("debug points:",
          "",
          [this](const std::vector<cv::Point> & value) {
            if ( _pipeline ) {
              _pipeline->stereo_matching_options().debug_points = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](std::vector<cv::Point> * value) {
            if ( _pipeline ) {
              * value = _pipeline->stereo_matching_options().debug_points;
              return true;
            }
            return false;
          });


  updateControls();
}


void QScaleSweepStereoMatcherOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  _pipeline = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr& QScaleSweepStereoMatcherOptions::current_pipeline() const
{
  return _pipeline;
}
