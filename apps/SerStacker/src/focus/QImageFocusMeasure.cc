/*
 * QImageFocusMeasure.cc
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#include "QImageFocusMeasure.h"
#include <core/debug.h>

namespace qserstacker {

QImageFocusMeasure::QImageFocusMeasure(QObject * parent) :
    Base(parent)
{
  sprefix_ = "ImageFocusMeasure";
  load_parameters();
}

void QImageFocusMeasure::measure(const cv::Mat & frame, COLORID colorid, int bpp, const QRect & roi)
{
  if( enabled_ ) {

    cv::Mat image;

    if( getImageROI(&image, frame, roi, false) ) {

      if( is_bayer_pattern(colorid) ) {
        if( !extract_bayer_planes(image, image, colorid) ) {
          CF_ERROR("extract_bayer_planes() fails");
        }
      }

      cv::Scalar v =
          measure_.compute(image);

      const int cn =
          measure_.avgchannel() ? 1 :
              image.channels();

      if( cn != cn_ ) {

        cn_ = cn;

        for( int i = 0; i < MAX_CHANNELS; ++i ) {
          measurements_[i].clear();
        }

      }
      else {

        int datasize = 0;
        for( int i = 0; i < MAX_CHANNELS; ++i ) {
          if( measurements_[i].size() > datasize ) {
            datasize = measurements_[i].size();
          }
        }

        while (datasize >= max_measurements_) {

          for( int i = 0; i < MAX_CHANNELS; ++i ) {
            if( !measurements_[i].isEmpty() ) {
              measurements_[i].pop_front();
            }
          }

          --datasize;
        }
      }

      for( int i = 0; i < cn; ++i ) {
        measurements_[i].push_back(v[i]);
      }

      colorid_ = colorid;
      bpp_ = bpp;

      Q_EMIT dataChanged();
    }
  }
}


} /* namespace qserstacker */
