/*
 * QThumbnails.cc
 *
 *  Created on: Mar 6, 2020
 *      Author: amyznikov
 */

#include "QThumbnails.h"
#include <opencv2/opencv.hpp>
#include <core/proc/autoclip.h>
#include <core/proc/flow2HSV.h>
#include <core/io/load_image.h>

#include <core/io/image/c_ser_input_source.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_raw_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>
#include <core/io/image/c_fits_input_source.h>
#include <core/io/hdl/c_hdl_input_source.h>
#include <core/io/sply/c_sply_input_source.h>
#include <core/io/text/c_textfile_input_source.h>
#include <core/io/ply/c_ply_input_source.h>


#if HAVE_LIBOPENRAW // come from cmake
#include <libopenraw/libopenraw.h>
#endif


#include <core/debug.h>

#define HDL_image   ":/qthumbnailsview/icons/lidar1.png"
#define SPLY_image   ":/qthumbnailsview/icons/sply.png"

QSize compute_thumbnail_size(QSize srcSize, int max_thumb_size)
{
  if ( !srcSize.isEmpty() ) {

    if ( max_thumb_size > 0 && (srcSize.width() > max_thumb_size || srcSize.height() > max_thumb_size) ) {
      if ( srcSize.width() > srcSize.height() ) {
        srcSize.setHeight(srcSize.height() * max_thumb_size / srcSize.width());
        srcSize.setWidth(max_thumb_size);
      }
      else {
        srcSize.setWidth(srcSize.width() * max_thumb_size / srcSize.height());
        srcSize.setHeight(max_thumb_size);
      }
    }
  }

  return srcSize;
}


QStringList getSupportedThumbnailsExtensions()
{
  QStringList suffixes;

  //  qt5-imageformats
  QList<QByteArray> supportedFormats = QImageReader::supportedImageFormats();
  for ( const QByteArray & s : supportedFormats ) {
    suffixes.append(QString("*.%1").arg(s.constData()));
  }

#if have_regular_image_input_source
  for ( const std::string & s : c_regular_image_input_source::suffixes()  ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_ffmpeg_input_source
  for ( const std::string & s : c_ffmpeg_reader::supported_input_formats() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_raw_image_input_source
  for ( const std::string & s : c_raw_image_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if HAVE_LIBOPENRAW
  if ( true ) {
    const char ** raw_file_suffixes = or_get_file_extensions();
    if ( raw_file_suffixes ) {
      for ( int i = 0; raw_file_suffixes[i]; ++i ) {
        suffixes.append(raw_file_suffixes[i]);
      }
    }
  }
#endif

#if have_fits_input_source
  for ( const std::string & s : c_fits_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_vlo_input_source
  for ( const std::string & s : c_vlo_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_hdl_input_source
  for ( const std::string & s : c_hdl_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_sply_input_source
  for ( const std::string & s : c_sply_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_textfile_input_source
  for ( const std::string & s : c_textfile_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

#if have_ply_input_source
  for ( const std::string & s : c_ply_input_source::suffixes() ) {
    suffixes.append(s.c_str());
  }
#endif

  std::sort(suffixes.begin(), suffixes.end());
  QStringList::iterator ii = std::unique(suffixes.begin(), suffixes.end());
  if ( ii != suffixes.end() ) {
    suffixes.erase(ii, suffixes.end());
  }

  return suffixes;
}
//
///*
// * Create flow BGR image using HSV color space
// *  flow: 2-channel input flow matrix
// *  dst : output BRG image
// *
// *  The code is extracted from OpenCV example dis_opticalflow.cpp
// * */
//static bool flow2HSV(cv::InputArray flow, cv::Mat & dst, double maxmotion, bool invert_y)
//{
//  if ( flow.channels() != 2 ) {
//    CF_FATAL("Unsupported number of channels: %d. 2-channel image expected", flow.channels());
//    return false;
//  }
//
//  cv::Mat uv[2], mag, ang;
//
//  cv::split(flow.getMat(), uv);
//
//  if ( flow.depth() == CV_32F || flow.depth() == CV_64F ) {
//    if ( invert_y ) {
//      cv::multiply(uv[1], -1, uv[1]);
//    }
//  }
//  else {
//    uv[0].convertTo(uv[0], CV_32F);
//    uv[1].convertTo(uv[1], CV_32F, invert_y ? -1 : 1);
//  }
//
//  cv::cartToPolar(uv[0], uv[1], mag, ang, true);
//
//  if ( maxmotion > 0 ) {
//    cv::threshold(mag, mag, maxmotion, maxmotion, cv::THRESH_TRUNC);
//  }
//
//  cv::normalize(mag, mag, 0, 1, cv::NORM_MINMAX);
//
//  const cv::Mat hsv[3] = {
//      ang,
//      mag,
//      mag/*cv::Mat::ones(ang.size(), ang.type())*/
//  };
//
//  cv::merge(hsv, 3, dst);
//  cv::cvtColor(dst, dst, cv::COLOR_HSV2RGB);
//  dst.convertTo(dst, CV_8U, 255);
//
//  return true;
//}
//

static bool cv2qimage(const cv::Mat & _src, QImage * dst, bool rgbswap)
{
  cv::Mat src;

  if (_src.channels() == 2 ) { // treat as optical flow marrix
    cv::Scalar m, s;
    cv::meanStdDev(_src, m, s);
    flow2HSV(_src, src, std::max(m[0], m[1]) + std::max(0.1, 5 * sqrt(s[0] * s[0] + s[1] * s[1])), true);
    rgbswap = false;
  }
  else if ( _src.depth() == CV_8U ) {
    src = _src;
  }
  else {
    cv::normalize(_src, src, 0, 255, cv::NORM_MINMAX);
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
    CF_FATAL("Unhandled OpenCV type %d", src.type());
    break;
  }

  return false;
}

static QImage loadThumbnailImageFromRaw(const QString & pathFileName, int thumb_size)
{
  QImage qimage;

#if HAVE_LIBOPENRAW
  if( qimage.isNull() ) {

    ORThumbnailRef thumbnail = nullptr;
    or_data_type thumbnailFormat = OR_DATA_TYPE_UNKNOWN;
    size_t dataSize = 0;
    uint32_t cx = 0, cy = 0;

    int status;

    const std::string cfilename = pathFileName.toStdString();

    status = or_get_extract_thumbnail(cfilename.c_str(),
        thumb_size, &thumbnail);

    if( status != OR_ERROR_NONE ) {
      // CF_ERROR("or_get_extract_thumbnail() fails: status=%d", status);
    }
    else {

      thumbnailFormat = or_thumbnail_format(thumbnail);
      dataSize = or_thumbnail_data_size(thumbnail);
      or_thumbnail_dimensions(thumbnail, &cx, &cy);

      switch (thumbnailFormat) {
        case OR_DATA_TYPE_JPEG:
          qimage = QImage::fromData((uint8_t*) or_thumbnail_data(thumbnail), dataSize, "JPG");
          break;

        case OR_DATA_TYPE_TIFF:
          qimage = QImage::fromData((uint8_t*) or_thumbnail_data(thumbnail), dataSize, "TIFF");
          break;

        case OR_DATA_TYPE_PNG:
          qimage = QImage::fromData((uint8_t*) or_thumbnail_data(thumbnail), dataSize, "PNG");
          break;

        case OR_DATA_TYPE_PIXMAP_8RGB:
          qimage = QImage((uint8_t*) or_thumbnail_data(thumbnail),
              cx, cy, QImage::Format::Format_RGB888,
              (QImageCleanupFunction) (or_thumbnail_release), thumbnail);
          break;

        case OR_DATA_TYPE_PIXMAP_16RGB: {
          const uint16_t * thumbnail_data = (const uint16_t*) or_thumbnail_data(thumbnail);
          if( thumbnail_data ) {
            uint8_t * scalled_data = (uint8_t*) malloc(dataSize /= 2);
            for( uint i = 0; i < dataSize; ++i ) {
              scalled_data[i] = thumbnail_data[i] / 256;
            }
            qimage = QImage(scalled_data, cx, cy, QImage::Format::Format_RGB888,
                (QImageCleanupFunction) (free), scalled_data);
          }
        }
          break;

        default:
          // CF_DEBUG("%s: Thumbnail in unsupported format=%d, thumb size is %u, %u",
          //    cfilename.c_str(), thumbnailFormat, cx, cy);
          //or_thumbnail_release(thumbnail);
          break;
      }

      if( thumbnail ) {
        or_thumbnail_release(thumbnail);
      }
    }
  }
#endif

#ifdef __c_video_input_source_h__
#if HAVE_LIBRAW
  if( qimage.isNull() ) {

    static thread_local LibRaw RawProcessor;
    if( RawProcessor.open_file(pathFileName.toStdString().c_str()) == LIBRAW_SUCCESS ) {
      if( RawProcessor.unpack_thumb() == LIBRAW_SUCCESS ) {
        const libraw_thumbnail_t & T = RawProcessor.imgdata.thumbnail;
        if( T.thumb ) {
          switch (T.tformat) {
            case LIBRAW_IMAGE_JPEG:
              qimage = QImage::fromData((uint8_t*) T.thumb, T.tlength, "JPG");
              break;
            case LIBRAW_IMAGE_BITMAP: {
              uint8_t * pixbuf = (uint8_t*) malloc(T.tlength);
              if( pixbuf ) {
                memcpy(pixbuf, T.thumb, T.tlength);
                qimage = QImage(pixbuf, T.twidth, T.theight, QImage::Format_RGB888, free, pixbuf);
              }
              break;
            }
            default:
              break;
          }
        }
      }
      RawProcessor.recycle();
    }
  }
#endif
#endif

  return qimage;
}

static QImage loadThumbnailImageOpenCV(const QString & pathFileName, int thumb_size)
{
  cv::Mat cvimage;
  QImage qimage;

  if ( load_image(pathFileName.toStdString(), cvimage) ) {

    if ( cvimage.channels() ==  2 ) {
      // interpret second channel as mask and ignore for preview,
      // because 2-channel image is interpreted by cv2qimage() as optical flow
      cv::extractChannel(cvimage, cvimage, 0);
    }

    const QSize thumbSize =
        compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
            thumb_size);

    if ( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

      cv::resize(cvimage, cvimage,
          cv::Size(thumbSize.width(), thumbSize.height()),
          0, 0,
          cv::INTER_AREA);

    }

    cv2qimage(cvimage, &qimage, true);
  }

  return qimage;
}

QImage loadThumbnailImage(const QString & pathFileName, int thumb_size)
{
  QImage qimage;
  cv::Mat cvimage;

  static const auto match_suffix =
      [](const std::string & suffix, const std::vector<std::string> & suffixes) -> bool {

        const char * csuffix = suffix.c_str();
        for ( const std::string & s : suffixes ) {
          if ( strcasecmp(csuffix, s.c_str()) == 0 ) {
            return true;
          }
        }
        return false;
      };


  const std::string suffix =
      QString(".%1").arg(QFileInfo(pathFileName).suffix()).toStdString();


  ///////////////////////////////////////////////////////////////


#if have_ser_input_source
  if( match_suffix(suffix, c_ser_input_source::suffixes()) ) {

    c_ser_reader ser(pathFileName.toStdString());

    if ( ser.read(cvimage) ) {

      const QSize thumbSize =
          compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
              thumb_size);

      if ( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

        cv::resize(cvimage, cvimage,
            cv::Size(thumbSize.width(), thumbSize.height()),
            0, 0,
            cv::INTER_AREA);
      }

      cv2qimage(cvimage, &qimage, true);
      return qimage;
    }
  }
#endif

  ///////////////////////////////////////////////////////////////


#if have_vlo_input_source
  if( match_suffix(suffix, c_vlo_input_source::suffixes()) ) {

    c_vlo_reader vlo;

    if( vlo.open(pathFileName.toStdString()) ) {

      std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());

      if( vlo.read(scan.get()) ) {
        if( !(cvimage = c_vlo_file::get_thumbnail_image(*scan)).empty() ) {

          const QSize thumbSize =
              compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
                  thumb_size);

          if( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

            cv::resize(cvimage, cvimage,
                cv::Size(thumbSize.width(), thumbSize.height()),
                0, 0,
                cv::INTER_AREA);
          }

          autoclip(cvimage, cv::noArray(),
              0.5, 99.5,
              0, 255);
          cvimage.convertTo(cvimage,
              CV_8U);

          cv2qimage(cvimage, &qimage, true);
          return qimage;
        }
      }
    }
  }
#endif

  ///////////////////////////////////////////////////////////////
#if have_hdl_input_source
  if( match_suffix(suffix, c_hdl_input_source::suffixes()) ) {
    return QImage(HDL_image);
  }
#endif

#if have_sply_input_source
  if( match_suffix(suffix, c_sply_input_source::suffixes()) ) {
    return QImage(SPLY_image);
  }
#endif

  ///////////////////////////////////////////////////////////////

#if have_fits_input_source
  if( match_suffix(suffix, c_fits_input_source::suffixes()) ) {

    c_fits_reader fits(pathFileName.toStdString());

    if ( fits.read(cvimage) ) {

      const QSize thumbSize =
          compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
              thumb_size);

      if ( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

        cv::resize(cvimage, cvimage,
            cv::Size(thumbSize.width(), thumbSize.height()),
            0, 0,
            cv::INTER_AREA);
      }

      cv2qimage(cvimage, &qimage, true);
      return qimage;
    }
  }
#endif


  ///////////////////////////////////////////////////////////////
#if have_raw_image_input_source
  if( match_suffix(suffix, c_raw_image_input_source::suffixes()) ) {
    if ( !(qimage = loadThumbnailImageFromRaw(pathFileName, thumb_size)).isNull() ) {
      return qimage;
    }
  }
#endif

  ///////////////////////////////////////////////////////////////

  if( strcasecmp(suffix.c_str(), ".flo") == 0 ) {
    if( !(cvimage = cv::readOpticalFlow(pathFileName.toStdString())).empty() ) {

      const QSize thumbSize =
          compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
              thumb_size);

      if( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

        cv::resize(cvimage, cvimage,
            cv::Size(thumbSize.width(), thumbSize.height()),
            0, 0,
            cv::INTER_AREA);

      }

      cv2qimage(cvimage, &qimage, true);
      return qimage;
    }
  }

  ///////////////////////////////////////////////////////////////

  static QMimeDatabase g_mimedb;
  const QMimeType mime = g_mimedb.mimeTypeForFile(pathFileName);

  if ( mime.name().contains("image/", Qt::CaseInsensitive) ) {

    if ( !mime.inherits("image/tiff") ) {

      QImageReader reader(pathFileName);
      if ( reader.canRead() ) {

        const QSize thumbSize =
            compute_thumbnail_size(reader.size(),
                thumb_size);

        reader.setAutoDetectImageFormat(true);
        reader.setQuality(75);
        reader.setScaledSize(thumbSize);

        qimage = reader.read();
        if ( !qimage.isNull() ) {
          if ( qimage.size().width() > thumbSize.width() || qimage.size().height() > thumbSize.height() ) {

            qimage = qimage.scaled(thumbSize,
                Qt::IgnoreAspectRatio,
                Qt::FastTransformation);

          }
          return qimage;
        }
      }
    }

    if( !(qimage = loadThumbnailImageOpenCV(pathFileName, thumb_size)).isNull() ) {
      return qimage;
    }
  }


  ///////////////////////////////////////////////////////////////

#if have_ffmpeg_input_source
  const bool is_video =
      mime.name().contains("video/", Qt::CaseInsensitive) ||
          match_suffix(suffix, c_ffmpeg_input_source::suffixes());
#else
  const bool is_video =
      mime.name().contains("video/", Qt::CaseInsensitive);
#endif

  if ( is_video ) {

#if have_ffmpeg_input_source
    c_ffmpeg_reader ffmpeg;

    if ( ffmpeg.open(pathFileName.toStdString()) ) {

      const QSize thumbSize =
          compute_thumbnail_size(QSize(ffmpeg.coded_size().width, ffmpeg.coded_size().height),
              thumb_size);

      if ( thumbSize.width() < ffmpeg.coded_size().width || thumbSize.height() < ffmpeg.coded_size().height ) {
        ffmpeg.set_frame_size(cv::Size(thumbSize.width(), thumbSize.height()));
      }

      //CF_DEBUG("pathFileName: %s ffmpeg.num_frames()=%d", pathFileName.toStdString().c_str(), ffmpeg.num_frames());
      if ( ffmpeg.num_frames() > 0 ) {
        ffmpeg.seek_frame(std::max(0, std::min(ffmpeg.num_frames() - 1, 5)));
      }

      if ( ffmpeg.read(cvimage, nullptr) ) {
        cv2qimage(cvimage, &qimage, true);
      }
    }

#else

    cv::VideoCapture cap;

    if ( cap.open(pathFileName.toStdString()) ) {

      // int nb_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
      // CF_DEBUG("pathFileName: %s CAP_PROP_FRAME_COUNT: %d", pathFileName.toStdString().c_str(), nb_frames);
      if ( cap.read(cvimage) ) {

        const QSize thumbSize =
        compute_thumbnail_size(QSize(cvimage.cols, cvimage.rows),
            thumb_size);

        if ( !thumbSize.isEmpty() && (thumbSize.width() != cvimage.cols || thumbSize.height() != cvimage.rows) ) {

          cv::resize(cvimage, cvimage,
              cv::Size(thumbSize.width(), thumbSize.height()),
              0, 0,
              cv::INTER_AREA);

        }

        cv2qimage(cvimage, &qimage, true);
      }
    }
#endif
  }

  return qimage;
}

QPixmap loadThumbnailPixmap(const QString & pathFileName, int maxSize)
{
  return QPixmap::fromImage(loadThumbnailImage(pathFileName, maxSize));
}

QIcon loadThumbnailIcon(const QString & pathFileName, int maxSize)
{
  return QIcon(loadThumbnailPixmap(pathFileName, maxSize));
}

bool isTextFileSuffix(const QString & suffix)
{
#if have_textfile_input_source

  const std::string csuffix =
      suffix.toStdString();

  const char * cs =
      csuffix.c_str();

  for( const std::string & s : c_textfile_input_source::suffixes() ) {
    if( strcasecmp(cs, s.c_str()) == 0 ) {
      return true;
    }
  }

#endif

  return false;
}

bool isPlyFileSuffix(const QString & suffix)
{
#if have_ply_input_source

  const std::string csuffix =
      suffix.toStdString();

  const char * cs =
      csuffix.c_str();

  for( const std::string & s : c_ply_input_source::suffixes() ) {
    if( strcasecmp(cs, s.c_str()) == 0 ) {
      return true;
    }
  }

#endif

  return false;
}

bool isTextFile(const QString & abspath)
{
  return isTextFileSuffix(QFileInfo(abspath).suffix());
}

bool isPlyFile(const QString & abspath)
{
  return isPlyFileSuffix(QFileInfo(abspath).suffix());
}
