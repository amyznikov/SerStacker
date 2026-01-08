/*
 * unpack-libcamera-image.cc
 *
 *  Created on: Jan 3, 2026
 *      Author: amyznikov
 */

#include "unpack-libcamera-image.h"
#include <core/io/debayer.h>
#include <core/debug.h>

#if 0
#include <libcamera/formats.h>
#else
namespace {
namespace libcamera {
class PixelFormat
{
public:
  constexpr PixelFormat() : _fourcc(0), _modifier(0) {}
  explicit constexpr PixelFormat(uint32_t fourcc, uint64_t modifier = 0) : _fourcc(fourcc), _modifier(modifier) {}
  bool operator==(const PixelFormat &other) const { return _fourcc == other._fourcc && _modifier == other._modifier; }
private:
  uint32_t _fourcc;
  uint64_t _modifier;
};

namespace formats {

namespace {
constexpr uint32_t __fourcc(char a, char b, char c, char d)
{
  return (static_cast<uint32_t>(a) <<  0) |
         (static_cast<uint32_t>(b) <<  8) |
         (static_cast<uint32_t>(c) << 16) |
         (static_cast<uint32_t>(d) << 24);
}

constexpr uint64_t __mod(unsigned int vendor, unsigned int mod)
{
  return (static_cast<uint64_t>(vendor) << 56) |
         (static_cast<uint64_t>(mod) << 0);
}

constexpr uint32_t kDrmFormatBigEndian = uint32_t(1) << 31; /* DRM_FORMAT_BIG_ENDIAN */
} /* namespace */

constexpr PixelFormat R8(__fourcc('R', '8', ' ', ' '), __mod(0, 0));
constexpr PixelFormat R10(__fourcc('R', '1', '0', ' '), __mod(0, 0));
constexpr PixelFormat R12(__fourcc('R', '1', '2', ' '), __mod(0, 0));
constexpr PixelFormat R16(__fourcc('R', '1', '6', ' '), __mod(0, 0));
constexpr PixelFormat RGB565(__fourcc('R', 'G', '1', '6'), __mod(0, 0));
constexpr PixelFormat RGB565_BE(__fourcc('R', 'G', '1', '6') | kDrmFormatBigEndian, __mod(0, 0));
constexpr PixelFormat RGB888(__fourcc('R', 'G', '2', '4'), __mod(0, 0));
constexpr PixelFormat BGR888(__fourcc('B', 'G', '2', '4'), __mod(0, 0));
constexpr PixelFormat XRGB8888(__fourcc('X', 'R', '2', '4'), __mod(0, 0));
constexpr PixelFormat XBGR8888(__fourcc('X', 'B', '2', '4'), __mod(0, 0));
constexpr PixelFormat RGBX8888(__fourcc('R', 'X', '2', '4'), __mod(0, 0));
constexpr PixelFormat BGRX8888(__fourcc('B', 'X', '2', '4'), __mod(0, 0));
constexpr PixelFormat ARGB8888(__fourcc('A', 'R', '2', '4'), __mod(0, 0));
constexpr PixelFormat ABGR8888(__fourcc('A', 'B', '2', '4'), __mod(0, 0));
constexpr PixelFormat RGBA8888(__fourcc('R', 'A', '2', '4'), __mod(0, 0));
constexpr PixelFormat BGRA8888(__fourcc('B', 'A', '2', '4'), __mod(0, 0));
constexpr PixelFormat RGB161616(__fourcc('R', 'G', '4', '8'), __mod(0, 0));
constexpr PixelFormat BGR161616(__fourcc('B', 'G', '4', '8'), __mod(0, 0));
constexpr PixelFormat YUYV(__fourcc('Y', 'U', 'Y', 'V'), __mod(0, 0));
constexpr PixelFormat YVYU(__fourcc('Y', 'V', 'Y', 'U'), __mod(0, 0));
constexpr PixelFormat UYVY(__fourcc('U', 'Y', 'V', 'Y'), __mod(0, 0));
constexpr PixelFormat VYUY(__fourcc('V', 'Y', 'U', 'Y'), __mod(0, 0));
constexpr PixelFormat AVUY8888(__fourcc('A', 'V', 'U', 'Y'), __mod(0, 0));
constexpr PixelFormat XVUY8888(__fourcc('X', 'V', 'U', 'Y'), __mod(0, 0));
constexpr PixelFormat NV12(__fourcc('N', 'V', '1', '2'), __mod(0, 0));
constexpr PixelFormat NV21(__fourcc('N', 'V', '2', '1'), __mod(0, 0));
constexpr PixelFormat NV16(__fourcc('N', 'V', '1', '6'), __mod(0, 0));
constexpr PixelFormat NV61(__fourcc('N', 'V', '6', '1'), __mod(0, 0));
constexpr PixelFormat NV24(__fourcc('N', 'V', '2', '4'), __mod(0, 0));
constexpr PixelFormat NV42(__fourcc('N', 'V', '4', '2'), __mod(0, 0));
constexpr PixelFormat YUV420(__fourcc('Y', 'U', '1', '2'), __mod(0, 0));
constexpr PixelFormat YVU420(__fourcc('Y', 'V', '1', '2'), __mod(0, 0));
constexpr PixelFormat YUV422(__fourcc('Y', 'U', '1', '6'), __mod(0, 0));
constexpr PixelFormat YVU422(__fourcc('Y', 'V', '1', '6'), __mod(0, 0));
constexpr PixelFormat YUV444(__fourcc('Y', 'U', '2', '4'), __mod(0, 0));
constexpr PixelFormat YVU444(__fourcc('Y', 'V', '2', '4'), __mod(0, 0));
constexpr PixelFormat MJPEG(__fourcc('M', 'J', 'P', 'G'), __mod(0, 0));
constexpr PixelFormat SRGGB8(__fourcc('R', 'G', 'G', 'B'), __mod(0, 0));
constexpr PixelFormat SGRBG8(__fourcc('G', 'R', 'B', 'G'), __mod(0, 0));
constexpr PixelFormat SGBRG8(__fourcc('G', 'B', 'R', 'G'), __mod(0, 0));
constexpr PixelFormat SBGGR8(__fourcc('B', 'A', '8', '1'), __mod(0, 0));
constexpr PixelFormat SRGGB10(__fourcc('R', 'G', '1', '0'), __mod(0, 0));
constexpr PixelFormat SGRBG10(__fourcc('B', 'A', '1', '0'), __mod(0, 0));
constexpr PixelFormat SGBRG10(__fourcc('G', 'B', '1', '0'), __mod(0, 0));
constexpr PixelFormat SBGGR10(__fourcc('B', 'G', '1', '0'), __mod(0, 0));
constexpr PixelFormat SRGGB12(__fourcc('R', 'G', '1', '2'), __mod(0, 0));
constexpr PixelFormat SGRBG12(__fourcc('B', 'A', '1', '2'), __mod(0, 0));
constexpr PixelFormat SGBRG12(__fourcc('G', 'B', '1', '2'), __mod(0, 0));
constexpr PixelFormat SBGGR12(__fourcc('B', 'G', '1', '2'), __mod(0, 0));
constexpr PixelFormat SRGGB14(__fourcc('R', 'G', '1', '4'), __mod(0, 0));
constexpr PixelFormat SGRBG14(__fourcc('B', 'A', '1', '4'), __mod(0, 0));
constexpr PixelFormat SGBRG14(__fourcc('G', 'B', '1', '4'), __mod(0, 0));
constexpr PixelFormat SBGGR14(__fourcc('B', 'G', '1', '4'), __mod(0, 0));
constexpr PixelFormat SRGGB16(__fourcc('R', 'G', 'B', '6'), __mod(0, 0));
constexpr PixelFormat SGRBG16(__fourcc('G', 'R', '1', '6'), __mod(0, 0));
constexpr PixelFormat SGBRG16(__fourcc('G', 'B', '1', '6'), __mod(0, 0));
constexpr PixelFormat SBGGR16(__fourcc('B', 'Y', 'R', '2'), __mod(0, 0));
constexpr PixelFormat R10_CSI2P(__fourcc('R', '1', '0', ' '), __mod(11, 1));
constexpr PixelFormat R12_CSI2P(__fourcc('R', '1', '2', ' '), __mod(11, 1));
constexpr PixelFormat SRGGB10_CSI2P(__fourcc('R', 'G', '1', '0'), __mod(11, 1));
constexpr PixelFormat SGRBG10_CSI2P(__fourcc('B', 'A', '1', '0'), __mod(11, 1));
constexpr PixelFormat SGBRG10_CSI2P(__fourcc('G', 'B', '1', '0'), __mod(11, 1));
constexpr PixelFormat SBGGR10_CSI2P(__fourcc('B', 'G', '1', '0'), __mod(11, 1));
constexpr PixelFormat SRGGB12_CSI2P(__fourcc('R', 'G', '1', '2'), __mod(11, 1));
constexpr PixelFormat SGRBG12_CSI2P(__fourcc('B', 'A', '1', '2'), __mod(11, 1));
constexpr PixelFormat SGBRG12_CSI2P(__fourcc('G', 'B', '1', '2'), __mod(11, 1));
constexpr PixelFormat SBGGR12_CSI2P(__fourcc('B', 'G', '1', '2'), __mod(11, 1));
constexpr PixelFormat SRGGB14_CSI2P(__fourcc('R', 'G', '1', '4'), __mod(11, 1));
constexpr PixelFormat SGRBG14_CSI2P(__fourcc('B', 'A', '1', '4'), __mod(11, 1));
constexpr PixelFormat SGBRG14_CSI2P(__fourcc('G', 'B', '1', '4'), __mod(11, 1));
constexpr PixelFormat SBGGR14_CSI2P(__fourcc('B', 'G', '1', '4'), __mod(11, 1));
constexpr PixelFormat SRGGB10_IPU3(__fourcc('R', 'G', '1', '0'), __mod(1, 13));
constexpr PixelFormat SGRBG10_IPU3(__fourcc('B', 'A', '1', '0'), __mod(1, 13));
constexpr PixelFormat SGBRG10_IPU3(__fourcc('G', 'B', '1', '0'), __mod(1, 13));
constexpr PixelFormat SBGGR10_IPU3(__fourcc('B', 'G', '1', '0'), __mod(1, 13));
constexpr PixelFormat RGGB_PISP_COMP1(__fourcc('R', 'G', 'B', '6'), __mod(12, 1));
constexpr PixelFormat GRBG_PISP_COMP1(__fourcc('G', 'R', '1', '6'), __mod(12, 1));
constexpr PixelFormat GBRG_PISP_COMP1(__fourcc('G', 'B', '1', '6'), __mod(12, 1));
constexpr PixelFormat BGGR_PISP_COMP1(__fourcc('B', 'Y', 'R', '2'), __mod(12, 1));
constexpr PixelFormat MONO_PISP_COMP1(__fourcc('R', '1', '6', ' '), __mod(12, 1));


} /* namespace formats */
} /* namespace libcamera */
} /* namespace */
#endif

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
  if( format == formats::SBGGR12 ) {
    *colorid = COLORID_BAYER_BGGR;
    *bpp = 12;
    cv::Mat(h, w, CV_16UC1, (void*) data.data(), stride).copyTo(image);
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

  //CF_DEBUG("Unknown format: fourcc=%u modifier=%llu '%s'", fourcc, modifier, format.toString().c_str());
  CF_DEBUG("Unknown format: fourcc=%u modifier=%llu'", fourcc, modifier);
  return false;
}


