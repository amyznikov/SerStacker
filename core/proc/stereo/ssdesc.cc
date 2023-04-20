/*
 * ssdesc.cc
 *
 *  Created on: Apr 20, 2023
 *      Author: amyznikov
 */

#include "ssdesc.h"
#include <tbb/tbb.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<sscmpflags>()
{
  static constexpr c_enum_member members[] = {
      {sscmp_g, "g", "g"},
      {sscmp_gx, "gx", "gx"},
      {sscmp_gy, "gy", "gy"},
      {sscmp_gxx, "gxx", "gxx"},
      {sscmp_gyy, "gyy", "gyy"},
      {sscmp_gxy, "gxy", "gxy"},
      {sscmp_a, "a", "a"},
      {sscmp_b, "b", "b"},
      {sscmp_all},
  };

  return members;
}


namespace {

typedef tbb::blocked_range<int> tbb_range;
const int tbb_grain_size = 256;

constexpr double ss_sigma = 3;
constexpr int ss_size = 15;

#pragma pack(push, 1)
// sizeof(ssdesc) = 8
struct ssdesc
{
  uint8_t g;
  uint8_t gx;
  uint8_t gy;
  uint8_t gxx;
  uint8_t gyy;
  uint8_t gxy;
  uint8_t a;
  uint8_t b;
};

#pragma pack(pop)

void compute_gradients(const cv::Mat & src, cv::Mat1f & gx, cv::Mat1f & gy,
    cv::Mat1f & gxx, cv::Mat1f & gyy,
    cv::Mat1f & gxy)
{
  static const thread_local cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  static const thread_local cv::Matx<float, 5, 1> Kt =
      K.t();

  constexpr cv::BorderTypes border_type =
      cv::BORDER_REPLICATE;

  cv::filter2D(src, gx, CV_32F, K, cv::Point(-1, -1), 0, border_type);
  cv::filter2D(src, gy, CV_32F, Kt, cv::Point(-1, -1), 0, border_type);
  cv::filter2D(gx, gxx, CV_32F, K, cv::Point(-1, -1), 0, border_type);
  cv::filter2D(gy, gyy, CV_32F, Kt, cv::Point(-1, -1), 0, border_type);
  cv::filter2D(gx, gxy, CV_32F, Kt, cv::Point(-1, -1), 0, border_type);
}

static inline uint8_t absdiff(uint8_t a, uint8_t b)
{
  return a > b ? a - b : b - a;
}

static inline uint16_t absdiff(const ssdesc & a, const ssdesc & b)
{
  return absdiff(a.g, b.g) +
      absdiff(a.gx, b.gx) +
      absdiff(a.gy, b.gy) +
      absdiff(a.gxx, b.gxx) +
      absdiff(a.gyy, b.gyy) +
      absdiff(a.gxy, b.gxy) +
      absdiff(a.a, b.a) +
      absdiff(a.b, b.b);
}

}

void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc, int flags)
{
  cv::Mat s;
  cv::Mat1f gx, gy, gxx, gyy, gxy;
  std::vector<cv::Mat> lab;

  cv::GaussianBlur(image, s, cv::Size(ss_size, ss_size), ss_sigma, ss_sigma, cv::BORDER_REPLICATE);
  cv::cvtColor(s, s, cv::COLOR_BGR2Lab);
  cv::split(s, lab);

  const cv::Mat1b g = lab[0];
  const cv::Mat1b a = lab[1];
  const cv::Mat1b b = lab[2];

  compute_gradients(g, gx, gy, gxx, gyy, gxy);

  cv::Mat4w desc(g.size(), cv::Vec4w(0, 0, 0, 0));

  tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
      [&](const tbb_range & r) {

        const int width = g.cols;

        for( int y = r.begin(); y < r.end(); ++y ) {

          ssdesc *ssp = reinterpret_cast<ssdesc*>(desc[y]);

          for( int x = 0; x < width; ++x ) {

            ssdesc & ss = ssp[x];

            if ( flags & sscmp_g ) {
              ss.g = (uint8_t) (g[y][x]);
            }
            if ( flags & sscmp_gx ) {
              ss.gx = (uint8_t) (gx[y][x] * 2 + 128);
            }
            if ( flags & sscmp_gy ) {
              ss.gy = (uint8_t) (gy[y][x] * 2 + 128);
            }
            if ( flags & sscmp_gxx ) {
              ss.gxx = (uint8_t) (gxx[y][x] * 4 + 128);
            }
            if ( flags & sscmp_gyy ) {
              ss.gyy = (uint8_t) (gyy[y][x] * 4 + 128);
            }
            if ( flags & sscmp_gxy ) {
              ss.gxy = (uint8_t) (gxy[y][x] * 8 + 128);
            }
            if ( flags & sscmp_a ) {
              ss.a = (uint8_t) (a[y][x]);
            }
            if ( flags & sscmp_b ) {
              ss.b = (uint8_t) (b[y][x]);
            }
          }
        }

      });

  _desc.move(desc);
}

void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc)
{
  cv::Mat s;
  cv::Mat1f gx, gy, gxx, gyy, gxy;
  std::vector<cv::Mat> lab;


  cv::GaussianBlur(image, s, cv::Size(ss_size, ss_size), ss_sigma, ss_sigma, cv::BORDER_REPLICATE);
  cv::cvtColor(s, s, cv::COLOR_BGR2Lab);
  cv::split(s, lab);

  const cv::Mat1b g = lab[0];
  const cv::Mat1b a = lab[1];
  const cv::Mat1b b = lab[2];

  compute_gradients(g, gx, gy, gxx, gyy, gxy);

  cv::Mat4w desc(g.size(), cv::Vec4w(0, 0, 0, 0));

  tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
      [&](const tbb_range & r) {

        const int width = g.cols;

        for( int y = r.begin(); y < r.end(); ++y ) {

          ssdesc *ssp = reinterpret_cast<ssdesc*>(desc[y]);

          for( int x = 0; x < width; ++x ) {

            ssdesc & ss = ssp[x];
            ss.g = (uint8_t) (g[y][x]);
            ss.gx = (uint8_t) (gx[y][x] * 2 + 128);
            ss.gy = (uint8_t) (gy[y][x] * 2 + 128);
            ss.gxx = (uint8_t) (gxx[y][x] * 4 + 128);
            ss.gyy = (uint8_t) (gyy[y][x] * 4 + 128);
            ss.gxy = (uint8_t) (gxy[y][x] * 8 + 128);
            ss.a = (uint8_t) (a[y][x]);
            ss.b = (uint8_t) (b[y][x]);
          }
        }

      });

  _desc.move(desc);
}


void ssdesc_cvtfp32(const cv::Mat & _desc, cv::OutputArray output)
{
  const cv::Mat4w desc = _desc;

  cv::Mat1f dst =
      output.getMatRef();

  for( int y = 0; y < desc.rows; ++y ) {

    const ssdesc *ssp = reinterpret_cast<const ssdesc*>(desc[y]);

    for( int x = 0; x < desc.cols; ++x ) {

      const ssdesc &ss = ssp[x];

      const uint64_t v =
          ((uint64_t) ss.g << (64 - 8)) |
          ((uint64_t) ss.gx << (64 - 16)) |
          ((uint64_t) ss.gy << (64 - 24)) |
          ((uint64_t) ss.gxx << (64 - 32)) |
          ((uint64_t) ss.gyy << (64 - 40)) |
          ((uint64_t) ss.gxy << (64 - 48)) |
          ((uint64_t) ss.a << (64 - 56)) |
          ((uint64_t) ss.b << (64 - 64));

      dst[y][x] = v;
    }
  }
}


void ssdesc_compare(cv::InputArray d1, cv::InputArray d2, cv::OutputArray dists)
{
  const cv::Mat4w desc1 = d1.getMat();
  const cv::Mat4w desc2 = d2.getMat();

  cv::Mat1f distances = dists.getMatRef();

  //distances.create(desc1.size());

  for( int y = 0; y < desc1.rows; ++y ) {

    const ssdesc *ssp1 = reinterpret_cast<const ssdesc*>(desc1[y]);
    const ssdesc *ssp2 = reinterpret_cast<const ssdesc*>(desc2[y]);

    for( int x = 0; x < desc1.cols; ++x ) {
      distances[y][x] =
          absdiff(ssp1[x], ssp2[x]);
    }
  }
}


void ssdesc_match(cv::InputArray current_descs, cv::InputArray reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray errs,
    const cv::Mat1b & mask)
{
  const cv::Mat4w current_desc = current_descs.getMat();
  const cv::Mat4w reference_desc = reference_descs.getMat();

  disps.create(reference_desc.size(), CV_16UC1);
  disps.setTo(0);

  errs.create(reference_desc.size(), CV_16UC1);
  errs.setTo(0);

  //err.create(reference_desc.size(), CV_16U);

  cv::Mat1w disp = disps.getMatRef();
  cv::Mat1w err = errs.getMatRef();

  for( int y = 0; y < current_desc.rows; ++y ) {

    const uint8_t * mskp = mask[y];

    const ssdesc *cssp = reinterpret_cast<const ssdesc*>(current_desc[y]);
    const ssdesc *rssp = reinterpret_cast<const ssdesc*>(reference_desc[y]);

    uint16_t s, sbest;

    for( int x = 0; x < reference_desc.cols; ++x ) {
      if( mskp[x] ) {

        const ssdesc & rss =
            rssp[x];

        sbest =
            absdiff(rss, cssp[x]);

        int xxbest = x;

        for( int xx = x + 1, xxmax = std::min(x + max_disparity, current_desc.cols); xx < xxmax; ++xx ) {

  //        if( (s = absdiff(rss, cssp[xx])) < sbest ) {
  //          xxbest = xx;
  //          sbest = s;
  //        }

          const ssdesc &css = cssp[xx];

          if( (s = absdiff(css.g, rss.g)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.gx, rss.gx)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.gy, rss.gy)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.gxx, rss.gxx)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.gyy, rss.gyy)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.gxy, rss.gxy)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.a, rss.a)) >= sbest ) {
            continue;
          }
          if( (s += absdiff(css.b, rss.b)) >= sbest ) {
            continue;
          }
          sbest = s;
          xxbest = xx;
        }

        disp[y][x] = xxbest - x;
        err[y][x] = sbest;
      }
    }
  }
}
