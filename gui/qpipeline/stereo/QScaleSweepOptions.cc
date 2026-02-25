/*
 * QScaleSweepOptions.cc
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#include "QScaleSweepOptions.h"

QScaleSweepOptions::QScaleSweepOptions(QWidget * parent) :
    Base(parent)
{
  max_disparity_ctl =
      add_numeric_box<int>("max_disparity",
          "",
          [this](int value) {
            if ( _opts && _opts->max_disparity != value) {
              _opts->max_disparity = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_disparity;
              return true;
            }
            return false;
          });

  max_scale_ctl =
      add_numeric_box<int>("max_scale",
          "",
          [this](int value) {
            if ( _opts && _opts->max_scale != value) {
              _opts->max_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_scale;
              return true;
            }
            return false;
          });

  texture_threshold_ctl =
      add_numeric_box<int>("text. thresh",
          "",
          [this](int value) {
            if ( _opts && _opts->texture_threshold != value) {
              _opts->texture_threshold = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->texture_threshold;
              return true;
            }
            return false;
          });

  disp12maxDiff_ctl =
      add_numeric_box<int>("disp12maxDiff",
          "",
          [this](int value) {
            if ( _opts && _opts->disp12maxDiff != value) {
              _opts->disp12maxDiff = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->disp12maxDiff;
              return true;
            }
            return false;
          });

  debug_directory_ctl =
      add_browse_for_path("",
          "debug_directory:",
          QFileDialog::AcceptSave,
          QFileDialog::Directory,
          [this](const QString & path) {
            if ( _opts ) {
              _opts->debug_directory = path.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( _opts ) {
              * path = _opts->debug_directory.c_str();
              return true;
            }
            return false;
          });

  debug_points_ctl =
      add_numeric_box<std::vector<cv::Point>>("debug_points",
          "",
          [this](const std::vector<cv::Point> & value) {
            if ( _opts ) {
              _opts->debug_points = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](std::vector<cv::Point> * value) {
            if ( _opts ) {
              * value = _opts->debug_points;
              return true;
            }
            return false;
          });

}

