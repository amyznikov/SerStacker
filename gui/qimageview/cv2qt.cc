/*
 * cv2qt.cc
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#include "cv2qt.h"
#include <core/proc/flow2HSV.h>
#include <core/debug.h>


bool cv2qt(cv::InputArray __src, QImage * dst, bool rgbswap)
{
  cv::Mat src;

  if ( __src.channels() == 2 ) {  // treat as optical flow image
    flow2HSV(__src, src, 0, true);
    rgbswap = false;
  }
  else if ( __src.depth() == CV_8U ) {
    src = __src.getMat();
  }
  else {
    cv::normalize(__src, src, 0, 255, cv::NORM_MINMAX);
    src.convertTo(src, CV_8U);
  }

  switch ( src.type() ) {
  case CV_8UC1 :
    if ( dst->format() != QImage::Format_Grayscale8 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_Grayscale8);
    }
    for ( int i = 0; i < src.rows; ++i ) {
      memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols);
    }
    return true;

  case CV_8UC3 :
    if ( dst->format() != QImage::Format_RGB888 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_RGB888);
    }

    if ( !rgbswap ) {
      for ( int i = 0; i < src.rows; ++i ) {
        memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols * 3);
      }
    }
    else {
      for ( int y = 0; y < src.rows; ++y ) {
        const cv::Vec3b * srcp = src.ptr<const cv::Vec3b>(y);
        cv::Vec3b * dstp = (cv::Vec3b *) dst->scanLine(y);
        for ( int x = 0; x < src.cols; ++x ) {
          dstp[x].val[0] = srcp[x].val[2];
          dstp[x].val[1] = srcp[x].val[1];
          dstp[x].val[2] = srcp[x].val[0];
        }
      }
    }
    return true;

  case CV_8UC4 :
    if ( dst->format() != QImage::Format_ARGB32 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_ARGB32);
    }
    for ( int i = 0; i < src.rows; ++i ) {
      memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols * 4);
    }
    return true;

  default :
    CF_FATAL("Unhandled cv2 type: depth=%d channels=%d", src.depth(), src.channels());
    break;
  }

  return false;
}

QPixmap createPixmap(cv::InputArray src, bool rgbswap, Qt::ImageConversionFlags flags)
{
  QPixmap pixmap;

  const cv::Mat image =
      src.getMat();

  if( image.type() == CV_8UC3 ) {

#if QT_VERSION >= QT_VERSION_CHECK(5, 13, 0)
    QImage qimage(image.data,
        image.cols, image.rows,
        (int) (size_t) (image.step),
        rgbswap ? QImage::Format_BGR888 :
            QImage::Format_RGB888);
#else

    QImage qimage;
    cv2qt(image, &qimage, rgbswap);

#endif

    pixmap =
        QPixmap::fromImage(qimage, flags);

  }
  else if( image.type() == CV_8UC1 ) {

    QImage qimage ( image.data,
        image.cols, image.rows,
        (int) (size_t) (image.step),
        QImage::Format_Grayscale8 );

    pixmap =
        QPixmap::fromImage(qimage, flags);

  }
#if QT_VERSION >= QT_VERSION_CHECK(5, 13, 0)
  else if( image.type() == CV_16UC1 ) {

    QImage qimage ( image.data,
        image.cols, image.rows,
        (int) (size_t) (image.step),
        QImage::Format_Grayscale16 );

    pixmap =
        QPixmap::fromImage(qimage, flags);

  }
#endif

  else {

    QImage qimage;

    cv2qt(image, &qimage, rgbswap);

    pixmap =
        QPixmap::fromImage(qimage, flags);
  }

  return pixmap;
}

