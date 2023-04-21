/*
 * ssdesc.cc
 *
 *  Created on: Apr 20, 2023
 *      Author: amyznikov
 */

#include "ssdesc.h"
#include <tbb/tbb.h>
#include <xmmintrin.h>
#include <core/ssprintf.h>

#include <core/debug.h>

template<>
const c_enum_member* members_of<sscmpflags>()
{
  static constexpr c_enum_member members[] = {
      {sscmp_g, "g", "g"},
      {sscmp_a, "a", "a"},
      {sscmp_b, "b", "b"},
      {sscmp_gx, "gx", "gx"},
      {sscmp_gy, "gy", "gy"},
      {sscmp_gxx, "gxx", "gxx"},
      {sscmp_gyy, "gyy", "gyy"},
      {sscmp_gxy, "gxy", "gxy"},
      {sscmp_all},
  };

  return members;
}


namespace {

typedef tbb::blocked_range<int> tbb_range;
const int tbb_grain_size = 256;

constexpr double ss_sigma = 3;
constexpr int ss_size = 15;


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

//static inline uint8_t absdiff(uint8_t a, uint8_t b)
//{
//  return a > b ? a - b : b - a;
//}

static inline uint8_t absdiff(const ssdesc & a, const ssdesc & b)
{
  //  uint8_t s = absdiff(a.arr[0], b.arr[0]);
  //  for( int i = 1; i < 8; ++i ) {
  //    s = std::max(s, absdiff(a.arr[i], b.arr[i]));
  //  }
  //  return s;

  //  ssdesc c;
  //  c.u64 = _m_to_int64(_mm_sub_pi8(_mm_max_pu8(ma, mb), _mm_min_pu8(ma, mb)));
  //  uint8_t s = c.arr[0];
  //  for( int i = 1; i < 8; ++i ) {
  //    s = std::max(s, c.arr[i]);
  //  }

//  const __m64 ma = _m_from_int64(a.u64);
//  const __m64 mb = _m_from_int64(b.u64);

  const __m64 &ma = a.m64;
  const __m64 &mb = b.m64;

  const union {
    __m64 m;
    uint8_t arr[8];
  } u = {
      .m = _mm_sub_pi8(_mm_max_pu8(ma, mb), _mm_min_pu8(ma, mb))
  };

  uint8_t s = u.arr[0];
  for( int i = 1; i < 8; ++i ) {
    s = std::max(s, u.arr[i]);
  }

  return s;
}


} // namespace

void ssa_compute(const cv::Mat3b & image, c_ssa_array & ssa, int flags)
{
  cv::Mat1f g, gx, gy, gxx, gyy, gxy;
  cv::Mat s;

  static const thread_local cv::Mat1f G =
      cv::getGaussianKernel(ss_size, ss_sigma, CV_32F);

  cv::sepFilter2D(image, s, CV_32F, G, G);
  cv::cvtColor(s, s, cv::COLOR_BGR2YCrCb);

  cv::extractChannel(s, g, 0);
  compute_gradients(g, gx, gy, gxx, gyy, gxy);

  ssa.create(s.size());

  const cv::Mat3f gab = s;

  if( (flags & sscmp_all) == sscmp_all ) {

    tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
        [&](const tbb_range & r) {

          const int width = g.cols;

          for( int y = r.begin(); y < r.end(); ++y ) {

            ssdesc *ssp = ssa[y];

            for( int x = 0; x < width; ++x ) {

              ssdesc & ss = ssp[x];
              ss.g =  (gab[y][x][0]);
              ss.a = (gab[y][x][1] + 128);
              ss.b = (gab[y][x][2] + 128);
              ss.gx = (gx[y][x] * 4 + 128);
              ss.gy = (gy[y][x] * 4 + 128);
              ss.gxx = (gxx[y][x] * 10 + 128);
              ss.gyy = (gyy[y][x] * 10 + 128);
              ss.gxy = (gxy[y][x] * 24 + 128);
            }
          }

        });
  }
  else {

    tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
        [&](const tbb_range & r) {

          const int width = g.cols;

          for( int y = r.begin(); y < r.end(); ++y ) {

            ssdesc *ssp = ssa[y];

            for( int x = 0; x < width; ++x ) {

              ssdesc & ss = ssp[x];

              if ( flags & sscmp_g ) {
                ss.g = (gab[y][x][0]);
              }
              if ( flags & sscmp_a ) {
                ss.a = (gab[y][x][1] + 128);
              }
              if ( flags & sscmp_b ) {
                ss.b = (gab[y][x][2] + 128);
              }
              if ( flags & sscmp_gx ) {
                ss.gx = (gx[y][x] * 4 + 128);
              }
              if ( flags & sscmp_gy ) {
                ss.gy = (gy[y][x] * 4 + 128);
              }
              if ( flags & sscmp_gxx ) {
                ss.gxx = (gxx[y][x] * 10 + 128);
              }
              if ( flags & sscmp_gyy ) {
                ss.gyy = (gyy[y][x] * 10 + 128);
              }
              if ( flags & sscmp_gxy ) {
                ss.gxy = (gxy[y][x] * 24 + 128);
              }
            }
          }

        });
  }
}

void ssa_cvtfp32(const c_ssa_array & ssa, cv::OutputArray output, int flags)
{
  if( output.empty() || output.size() != ssa.size() || output.type() != CV_32FC1 ) {
    output.create(ssa.size(), CV_32FC1);
  }

  cv::Mat1f dst =
      output.getMatRef();

  for( int y = 0, ny = ssa.rows(); y < ny; ++y ) {

    const ssdesc *ssp =
        ssa[y];

    for( int x = 0, nx = ssa.cols(); x < nx; ++x ) {

      const ssdesc &ss = ssp[x];

      uint64_t v = 0;

      if( flags & sscmp_g ) {
        v = ss.g;
      }
      else if( flags & sscmp_a ) {
        v = ss.a;
      }
      else if( flags & sscmp_b ) {
        v = ss.b;
      }
      else if( flags & sscmp_gx ) {
        v = ss.gx;
      }
      else if( flags & sscmp_gy ) {
        v = ss.gy;
      }
      else if( flags & sscmp_gxx ) {
        v = ss.gxx;
      }
      else if( flags & sscmp_gyy ) {
        v = ss.gyy;
      }
      else if( flags & sscmp_gxy ) {
        v = ss.gxy;
      }

      dst[y][x] = v;
    }
  }
}


void ssa_compare(const c_ssa_array & ssa1, const cv::Rect & rc1,
    const c_ssa_array & ssa2, const cv::Rect & rc2,
    cv::OutputArray dists)
{
  if ( rc1.width != rc2.width ) {
    CF_ERROR("ERROR: ROI WIDTH NOT MATCH");
    return;
  }

  if ( rc1.height != rc2.height ) {
    CF_ERROR("ERROR: ROI HEIGHT NOT MATCH");
    return;
  }


  if( dists.empty() || dists.size() != rc1.size() || dists.type() != CV_32FC1 ) {
    dists.create(rc1.size(), CV_32FC1);
  }

  cv::Mat1f distances =
      dists.getMatRef();

  for( int y = 0; y < rc1.height; ++y ) {

    const ssdesc *ssp1 = ssa1[y + rc1.y];
    const ssdesc *ssp2 = ssa2[y + rc2.y];

    for( int x = 0; x < rc1.width; ++x ) {
      distances[y][x] =
          absdiff(ssp1[x + rc1.x], ssp2[x + rc2.x]);
    }
  }
}

void ssa_match(const c_ssa_array & current_descs, const c_ssa_array & reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray costs,
    const cv::Mat1b & mask)
{
  disps.create(reference_descs.size(), CV_16UC1);
  disps.setTo(0);

  costs.create(reference_descs.size(), CV_16UC1);
  costs.setTo(0);

  cv::Mat1w disp = disps.getMatRef();
  cv::Mat1w cost = costs.getMatRef();

  tbb::parallel_for(tbb_range(0, current_descs.rows(), 16),
      [&](const tbb_range & r) {

        const int xmax = reference_descs.cols();

        for( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

          const uint8_t * mskp = mask[y];

          const ssdesc *cssp = current_descs[y];
          const ssdesc *rssp = reference_descs[y];

          for( int x = 0; x < xmax; ++x ) {
            if( mskp[x] ) {

              const ssdesc & rss = rssp[x];

              uint8_t cbest = absdiff(rss, cssp[x]);
              int xxbest = x;

              for( int xx = x + 1, xxmax = std::min(x + max_disparity, current_descs.cols()); xx < xxmax; ++xx ) {

                const uint8_t c = absdiff(rss, cssp[xx]);
                if( c < cbest ) {
                  xxbest = xx;
                  cbest = c;
                }

              }

              disp[y][x] = xxbest - x;
              cost[y][x] = cbest;
            }
          }
        }
      });
}
//
//void ssdesc_compute(const cv::Mat3b & image, cv::OutputArray & _desc, int flags)
//{
//  cv::Mat s;
//  cv::Mat1f gx, gy, gxx, gyy, gxy;
//  std::vector<cv::Mat> lab;
//
//  static const thread_local cv::Mat1f G = cv::getGaussianKernel(ss_size, ss_sigma, CV_32F);
//  cv::sepFilter2D(image, s, CV_32F, G, G);
//
//  //cv::GaussianBlur(image, s, cv::Size(ss_size, ss_size), ss_sigma, ss_sigma, cv::BORDER_REPLICATE);
//  //cv::cvtColor(s, s, cv::COLOR_BGR2Lab);
//  cv::cvtColor(s, s, cv::COLOR_BGR2YCrCb);
//
//  cv::split(s, lab);
//
//  const cv::Mat1f g = lab[0];
//  const cv::Mat1f a = lab[1];
//  const cv::Mat1f b = lab[2];
//
//  compute_gradients(g, gx, gy, gxx, gyy, gxy);
//
//  cv::Mat4w desc(g.size(), cv::Vec4w(0, 0, 0, 0));
//
//  if ( (flags & sscmp_all) == sscmp_all ) {
//
//    tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
//        [&](const tbb_range & r) {
//
//          const int width = g.cols;
//
//          for( int y = r.begin(); y < r.end(); ++y ) {
//
//            ssdesc *ssp = reinterpret_cast<ssdesc*>(desc[y]);
//
//            for( int x = 0; x < width; ++x ) {
//
//              ssdesc & ss = ssp[x];
//              ss.g = (uint8_t) (g[y][x]);
//              ss.gx = (uint8_t) (gx[y][x] * 4 + 128);
//              ss.gy = (uint8_t) (gy[y][x] * 4 + 128);
//              ss.gxx = (uint8_t) (gxx[y][x] * 10 + 128);
//              ss.gyy = (uint8_t) (gyy[y][x] * 10 + 128);
//              ss.gxy = (uint8_t) (gxy[y][x] * 24 + 128);
//              ss.a = (uint8_t) (a[y][x] + 128);
//              ss.b = (uint8_t) (b[y][x] + 128);
//            }
//          }
//
//        });
//  }
//  else {
//
//    tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
//        [&](const tbb_range & r) {
//
//          const int width = g.cols;
//
//          for( int y = r.begin(); y < r.end(); ++y ) {
//
//            ssdesc *ssp = reinterpret_cast<ssdesc*>(desc[y]);
//
//            for( int x = 0; x < width; ++x ) {
//
//              ssdesc & ss = ssp[x];
//
//              if ( flags & sscmp_g ) {
//                ss.g = (uint8_t) (g[y][x]);
//              }
//              if ( flags & sscmp_gx ) {
//                ss.gx = (uint8_t) (gx[y][x] * 4 + 128);
//              }
//              if ( flags & sscmp_gy ) {
//                ss.gy = (uint8_t) (gy[y][x] * 4 + 128);
//              }
//              if ( flags & sscmp_gxx ) {
//                ss.gxx = (uint8_t) (gxx[y][x] * 10 + 128);
//              }
//              if ( flags & sscmp_gyy ) {
//                ss.gyy = (uint8_t) (gyy[y][x] * 10 + 128);
//              }
//              if ( flags & sscmp_gxy ) {
//                ss.gxy = (uint8_t) (gxy[y][x] * 24 + 128);
//              }
//              if ( flags & sscmp_a ) {
//                ss.a = (uint8_t) (a[y][x] + 128);
//              }
//              if ( flags & sscmp_b ) {
//                ss.b = (uint8_t) (b[y][x] + 128);
//              }
//            }
//          }
//
//        });
//  }
//
//  _desc.move(desc);
//}
//
//
//void ssdesc_cvtfp32(const cv::Mat & _desc, cv::OutputArray output, int flags)
//{
//  const cv::Mat4w desc = _desc;
//
//  cv::Mat1f dst =
//      output.getMatRef();
//
//  for( int y = 0; y < desc.rows; ++y ) {
//
//    const ssdesc *ssp =
//        reinterpret_cast<const ssdesc*>(desc[y]);
//
//    for( int x = 0; x < desc.cols; ++x ) {
//
//      const ssdesc &ss = ssp[x];
//
//      uint64_t v = 0;
//
//      if( flags & sscmp_g ) {
//        v = ss.g;
//      }
//      else if( flags & sscmp_gx ) {
//        v = ss.gx;
//      }
//      else if( flags & sscmp_gy ) {
//        v = ss.gy;
//      }
//      else if( flags & sscmp_gxx ) {
//        v = ss.gxx;
//      }
//      else if( flags & sscmp_gyy ) {
//        v = ss.gyy;
//      }
//      else if( flags & sscmp_gxy ) {
//        v = ss.gxy;
//      }
//      else if( flags & sscmp_a ) {
//        v = ss.a;
//      }
//      else if( flags & sscmp_b ) {
//        v = ss.b;
//      }
//
//      dst[y][x] = v;
//    }
//  }
//}
//
//
//void ssdesc_compare(cv::InputArray d1, cv::InputArray d2, cv::OutputArray dists)
//{
//  const cv::Mat4w desc1 = d1.getMat();
//  const cv::Mat4w desc2 = d2.getMat();
//
//  cv::Mat1f distances = dists.getMatRef();
//
//  //distances.create(desc1.size());
//
//  for( int y = 0; y < desc1.rows; ++y ) {
//
//    const ssdesc *ssp1 = reinterpret_cast<const ssdesc*>(desc1[y]);
//    const ssdesc *ssp2 = reinterpret_cast<const ssdesc*>(desc2[y]);
//
//    for( int x = 0; x < desc1.cols; ++x ) {
//      distances[y][x] =
//          absdiff(ssp1[x], ssp2[x]);
//    }
//  }
//}
//
//
//void ssdesc_match(cv::InputArray current_descs, cv::InputArray reference_descs, int max_disparity,
//    cv::OutputArray disps, cv::OutputArray costs,
//    const cv::Mat1b & mask)
//{
//  const cv::Mat4w current_desc = current_descs.getMat();
//  const cv::Mat4w reference_desc = reference_descs.getMat();
//
//  disps.create(reference_desc.size(), CV_16UC1);
//  disps.setTo(0);
//
//  costs.create(reference_desc.size(), CV_16UC1);
//  costs.setTo(0);
//
//  //err.create(reference_desc.size(), CV_16U);
//
//  cv::Mat1w disp = disps.getMatRef();
//  cv::Mat1w cost = costs.getMatRef();
//
//  tbb::parallel_for(tbb_range(0, current_desc.rows, 16),
//      [&](const tbb_range & r) {
//
//        for( int y = r.begin(); y < r.end(); ++y ) {
//
//          const uint8_t * mskp = mask[y];
//
//          const ssdesc *cssp = reinterpret_cast<const ssdesc*>(current_desc[y]);
//          const ssdesc *rssp = reinterpret_cast<const ssdesc*>(reference_desc[y]);
//
//          for( int x = 0; x < reference_desc.cols; ++x ) {
//            if( mskp[x] ) {
//
//              const ssdesc & rss = rssp[x];
//
//              uint8_t cbest = absdiff(rss, cssp[x]);
//              int xxbest = x;
//
//              for( int xx = x + 1, xxmax = std::min(x + max_disparity, current_desc.cols); xx < xxmax; ++xx ) {
//
//                const uint8_t c = absdiff(rss, cssp[xx]);
//                if( c < cbest ) {
//                  xxbest = xx;
//                  cbest = c;
//                }
//
//              }
//
//              disp[y][x] = xxbest - x;
//              cost[y][x] = cbest;
//            }
//          }
//        }
//
//      });
//}
