/*
 * unpack-libcamera-image.cc
 *
 *  Created on: Jan 3, 2026
 *      Author: amyznikov
 */

#include "unpack-libcamera-image.h"
//#include <QtGui/QImage>
#include <libcamera/version.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>
#include <libcamera/logging.h>
#include <core/io/debayer.h>
#include <core/debug.h>


static void unpackScanlineRaw10_CSI2P(uint16_t *dst, const uint8_t * src, int width)
{
  for (int dx = 0, sx = 0; dx < width; dx += 4, sx += 5 ) {
    const uint16_t s0 = src[sx + 0];
    const uint16_t s1 = src[sx + 1];
    const uint16_t s2 = src[sx + 2];
    const uint16_t s3 = src[sx + 3];
    const uint16_t s4 = src[sx + 4];
    dst[dx+0] = (s0 << 2) | ((s4>>0) & 0x03);
    dst[dx+1] = (s1 << 2) | ((s4>>2) & 0x03);
    dst[dx+2] = (s2 << 2) | ((s4>>4) & 0x03);
    dst[dx+3] = (s3 << 2) | ((s4>>6) & 0x03);
  }
}

static void unpackScanlineRaw12_CSI2P(uint16_t *dst, const uint8_t *src, int width)
{
  for (int dx = 0, sx = 0; dx < width; dx += 2, sx += 3 ) {
    const uint16_t s0 = src[sx + 0];
    const uint16_t s1 = src[sx + 1];
    const uint16_t s2 = src[sx + 2];
    dst[dx+0] = (s0 << 4) | (s2 & 0x0f);
    dst[dx+1] = (s1 << 4) | (s2 >> 4);
  }
}

bool unpack_libcamera_image(const std::vector<uint8_t> & data, int w, int h, int stride, uint32_t fourcc, uint64_t modifier,
    cv::Mat & image, COLORID * colorid, int * bpp)
{
  using namespace libcamera;

  const PixelFormat format(fourcc, modifier);

  if( format == formats::MJPEG ) {
    CF_DEBUG("formats::MJPEG");
    *colorid = COLORID_UNKNOWN;
    *bpp = 8;
    return !(image = cv::imdecode(data, cv::IMREAD_ANYCOLOR)).empty();
  }

  if( format == formats::R8 ) {
    *colorid = COLORID_MONO;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::R10 ) {
    *colorid = COLORID_MONO;
    *bpp = 10;
    cv::Mat(h, w, CV_16UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::R12 ) {
    *colorid = COLORID_MONO;
    *bpp = 12;
    cv::Mat(h, w, CV_16UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::R10_CSI2P ) {
    *colorid = COLORID_MONO;
    *bpp = 10;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw10_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::R12_CSI2P ) {
    *colorid = COLORID_MONO;
    *bpp = 12;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw12_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::R16 ) {
    *colorid = COLORID_MONO;
    *bpp = 16;
    cv::Mat(h, w, CV_16UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::BGR888 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC3, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::RGB888 ) {
    *colorid = COLORID_RGB;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC3, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::BGRA8888 || format == formats::BGRX8888 || format == formats::ABGR8888
      || format == formats::XBGR8888 ) {
    *colorid = COLORID_BGRA;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC4, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::RGBA8888 || format == formats::RGBX8888 || format == formats::ARGB8888
      || format == formats::XRGB8888 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h, w, CV_8UC4, (void*) data.data(), stride), image, cv::COLOR_RGBA2BGR);
    return true;
  }
  if( format == formats::YUYV ) {
      * colorid = COLORID_BGR;
      * bpp = 8;
      cv::cvtColor(cv::Mat(h, w, CV_8UC2, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_YUYV);
      return true;
  }
  if( format == formats::VYUY ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_UYVY);
    return true;
  }
  if( format == formats::YVYU ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_YVYU);
    return true;
  }
  if( format == formats::UYVY ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_UYVY);
    return true;
  }
  if( format == formats::YUV420 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h + h / 2, w, CV_8UC1, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_I420);
    return true;
  }
  if( format == formats::YVU420 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h + h / 2, w, CV_8UC1, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_YV12);
    return true;
  }
  if( format == formats::YUV422 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_YUYV);
    return true;
  }
  if( format == formats::NV12 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h * 3 / 2, w, CV_8UC1, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_NV12);
    return true;
  }
  if( format == formats::NV21 ) {
    *colorid = COLORID_BGR;
    *bpp = 8;
    cv::cvtColor(cv::Mat(h + h / 2, w, CV_8UC1, (void*) data.data(), stride), image, cv::COLOR_YUV2BGR_NV21);
    return true;
  }
  if( format == formats::SRGGB8 ) {
    *colorid = COLORID_BAYER_RGGB;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::SBGGR8 ) {
    *colorid = COLORID_BAYER_BGGR;
    *bpp = 8;
    cv::Mat(h, w, CV_8UC1, (void*) data.data(), stride).copyTo(image);
    return true;
  }
  if( format == formats::SRGGB10_CSI2P ) {
    *colorid = COLORID_BAYER_RGGB;
    *bpp = 10;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw10_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SGRBG10_CSI2P ) {
    *colorid = COLORID_BAYER_GRBG;
    *bpp = 10;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw10_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SGBRG10_CSI2P ) {
    *colorid = COLORID_BAYER_GBRG;
    *bpp = 10;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw10_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SBGGR10_CSI2P ) {
    *colorid = COLORID_BAYER_BGGR;
    *bpp = 10;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw10_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SRGGB12_CSI2P ) {
    *colorid = COLORID_BAYER_RGGB;
    *bpp = 12;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw12_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SGRBG12_CSI2P ) {
    *colorid = COLORID_BAYER_GRBG;
    *bpp = 12;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw12_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SGBRG12_CSI2P ) {
    *colorid = COLORID_BAYER_GBRG;
    *bpp = 12;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw12_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }
  if( format == formats::SBGGR12_CSI2P ) {
    *colorid = COLORID_BAYER_BGGR;
    *bpp = 12;
    image.create(h, w, CV_16UC1);
    for( int y = 0; y < h; ++y ) {
      unpackScanlineRaw12_CSI2P(image.ptr<uint16_t>(y), data.data() + y * stride, w);
    }
    return true;
  }

//    case formats::NV16:
//      cv::cvtColor(cv::Mat(, CV_8UC1, (void*) data.data(), stride), image, cv::);
//      return true;
//    case formats::NV61:
//      cv::cvtColor(cv::Mat(, (void*) data.data(), stride), image, cv::);
//      return true;
//    case formats::NV24:
//      cv::cvtColor(cv::Mat(, (void*) data.data(), stride), image, cv::);
//      return true;
//    case formats::NV42:
//      cv::cvtColor(cv::Mat(, (void*) data.data(), stride), image, cv::);
//      return true;
//    case formats::R8:
//      formatFamily_ = RGB;
//      r_pos_ = 0;
//      g_pos_ = 0;
//      b_pos_ = 0;
//      bpp_ = 1;
//      break;
//    case formats::RGB888:
//      formatFamily_ = RGB;
//      r_pos_ = 2;
//      g_pos_ = 1;
//      b_pos_ = 0;
//      bpp_ = 3;
//      break;
//    case formats::ARGB8888:
//      case formats::XRGB8888:
//      formatFamily_ = RGB;
//      r_pos_ = 2;
//      g_pos_ = 1;
//      b_pos_ = 0;
//      bpp_ = 4;
//      break;
//    case formats::RGBA8888:
//      case formats::RGBX8888:
//      formatFamily_ = RGB;
//      r_pos_ = 3;
//      g_pos_ = 2;
//      b_pos_ = 1;
//      bpp_ = 4;
//      break;
//    case formats::ABGR8888:
//      case formats::XBGR8888:
//      formatFamily_ = RGB;
//      r_pos_ = 0;
//      g_pos_ = 1;
//      b_pos_ = 2;
//      bpp_ = 4;
//      break;
//    case formats::BGRA8888:
//      case formats::BGRX8888:
//      formatFamily_ = RGB;
//      r_pos_ = 1;
//      g_pos_ = 2;
//      b_pos_ = 3;
//      bpp_ = 4;
//      break;
//
//    default:
//      errno = EINVAL;
//      break;
//  };

//  format_ = format;
//  width_ = size.width();
//  height_ = size.height();
//  stride_ = stride;

  CF_DEBUG("Unknown format: fourcc=%u modifier=%llu '%s'", fourcc, modifier, format.toString().c_str());
  return false;
}


