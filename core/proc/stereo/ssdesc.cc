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
const int tbb_grain_size = 128;

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
//    uint8_t s = absdiff(a.arr[0], b.arr[0]);
//    for( int i = 1; i < 8; ++i ) {
//      s = std::max(s, absdiff(a.arr[i], b.arr[i]));
//    }
//    return s;

  const __m64 ma = _m_from_int64(a.u64);
  const __m64 mb = _m_from_int64(b.u64);

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

static inline uint8_t absdiff(const ssdesc *const* a[/*scales*/],
    const ssdesc *const* b[/*scales*/], int xa, int xb, int y, int scales, uint8_t worst)
{
  uint8_t d = absdiff(a[0][y][xa], b[0][y][xb]);

  for( int s = 1; s < scales; ++s ) {

    y >>= 1;
    xa >>= 1;
    xb >>= 1;

    if( (d = std::max(d, absdiff(a[s][y][xa], b[s][y][xb]))) > worst ) {
      break;
    }
  }

  return d;
}


} // namespace

void ssa_compute(const cv::Mat3b & image, c_ssarray & ssa, int flags,
    double ss_sigma, int ss_radius)
{
  cv::Mat1f g, gx, gy, gxx, gyy, gxy;
  cv::Mat s;

  if( ss_sigma <= 0 ) {
    ss_sigma = 2;
  }
  if( ss_radius < 1 ) {
    ss_radius = (int) (3 * ss_sigma);
  }

  const int ss_size = 2 * ss_radius + 1;

  static thread_local double old_ss_sigma = 0;
  static thread_local cv::Mat1f G;

  if( ss_sigma != old_ss_sigma || ss_size != G.rows ) {
    old_ss_sigma = ss_sigma;
    G = cv::getGaussianKernel(ss_size, ss_sigma, CV_32F);
  }

  cv::sepFilter2D(image, s, CV_32F, G, G);
  cv::cvtColor(s, s, cv::COLOR_BGR2YCrCb);

  cv::extractChannel(s, g, 0);
  compute_gradients(g, gx, gy, gxx, gyy, gxy);

  ssa.create(s.size());

  const cv::Mat3f gab = s;

  tbb::parallel_for(tbb_range(0, g.rows, tbb_grain_size),
      [&](const tbb_range & r) {

        const int width = g.cols;

        for( int y = r.begin(); y < r.end(); ++y ) {

          ssdesc *ssp = ssa[y];

          if( (flags & sscmp_all) == sscmp_all ) {

            for( int x = 0; x < width; ++x ) {

              ssdesc & ss = ssp[x];

              ss.g = (uint8_t) (gab[y][x][0]);
              ss.a = (uint8_t) (gab[y][x][1] + 128);
              ss.b = (uint8_t) (gab[y][x][2] + 128);
              ss.gx = (uint8_t) (gx[y][x] + 128);
              ss.gy = (uint8_t) (gy[y][x] + 128);
              ss.gxx = (uint8_t) (gxx[y][x] + 128);
              ss.gyy = (uint8_t) (gyy[y][x] + 128);
              ss.gxy = (uint8_t) (gxy[y][x] + 128);
            }

          }
          else {

            memset(ssp, 0, sizeof(*ssp) * width);

            for( int x = 0; x < width; ++x ) {

              ssdesc & ss = ssp[x];

              if ( flags & sscmp_g ) {
                ss.g = (uint8_t) (gab[y][x][0]);
              }
              if ( flags & sscmp_a ) {
                ss.a = (uint8_t) (gab[y][x][1] + 128);
              }
              if ( flags & sscmp_b ) {
                ss.b = (uint8_t) (gab[y][x][2] + 128);
              }
              if ( flags & sscmp_gx ) {
                ss.gx = (uint8_t) (gx[y][x] + 128);
              }
              if ( flags & sscmp_gy ) {
                ss.gy = (uint8_t) (gy[y][x] + 128);
              }
              if ( flags & sscmp_gxx ) {
                ss.gxx = (uint8_t) (gxx[y][x] + 128);
              }
              if ( flags & sscmp_gyy ) {
                ss.gyy = (uint8_t) (gyy[y][x] + 128);
              }
              if ( flags & sscmp_gxy ) {
                ss.gxy = (uint8_t) (gxy[y][x] + 128);
              }
            }
          }
        }
      });

}

void ssa_pyramid(const cv::Mat3b & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags,
    double ss_sigma,
    int ss_radius)
{
  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  ssa_compute(image, pyramid[0], flags, ss_sigma, ss_radius);

  cv::Mat3b cimg;

  for( int i = 1; i <= maxlevel; ++i ) {
    cv::pyrDown(i == 1 ? image : cimg, cimg);
    ssa_compute(cimg, pyramid[i], flags, ss_sigma, ss_radius);
  }
}



void ssa_cvtfp32(const c_ssarray & ssa, cv::OutputArray output, int flags)
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


void ssa_compare(const c_ssarray & ssa1, const cv::Rect & rc1,
    const c_ssarray & ssa2, const cv::Rect & rc2,
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


void ssa_compare(const std::vector<c_ssarray> & ssa1, const cv::Rect & rc1,
    const std::vector<c_ssarray> & ssa2, const cv::Rect & rc2,
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

  const int lmax =
      ssa1.size();

  for( int y = 0; y < rc1.height; ++y ) {

    for( int x = 0; x < rc1.width; ++x ) {

      uint8_t d =
          absdiff(ssa1[0][y + rc1.y][x + rc1.x],
              ssa2[0][y + rc2.y][x + rc2.x]);

      for( int l = 1, s = 2; l < lmax; ++l, s *= 2 ) {

        d = std::max(d,
            absdiff(ssa1[l][(y + rc1.y) / s][(x + rc1.x) / s],
                ssa2[l][(y + rc2.y) / s][(x + rc2.x) / s]));
      }

      distances[y][x] = d;
    }
  }

}



void ssa_match(const c_ssarray & current_descs, const c_ssarray & reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray costs,
    const cv::Mat1b & mask)
{
  disps.create(reference_descs.size(), CV_16UC1);
  disps.setTo(0);

  costs.create(reference_descs.size(), CV_16UC1);
  costs.setTo(0);

  cv::Mat1w disp = disps.getMatRef();
  cv::Mat1w cost = costs.getMatRef();

  tbb::parallel_for(tbb_range(0, reference_descs.rows(), 16),
      [&](const tbb_range & r) {

        const int xmax = reference_descs.cols();


        for( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

          const uint8_t * mskp = mask[y];

          const ssdesc *rssp = reference_descs[y];
          const ssdesc *cssp = current_descs[y];

          for( int x = 0; x < xmax; ++x ) {
            if( mskp[x] ) {

              const ssdesc & rss = rssp[x];

              uint8_t cbest =
                  absdiff(rss, cssp[x]);

              int xbest = x;

              for( int xx = x + 1, xxmax = std::min(x + max_disparity, current_descs.cols()); xx < xxmax; ++xx ) {

                const uint8_t c =
                    absdiff(rss, cssp[xx]);

                if( c < cbest ) {
                  xbest = xx;
                  if ( !(cbest = c) ) {
                    break;
                  }
                }
              }

              disp[y][x] = xbest - x;
              cost[y][x] = cbest;
            }
          }
        }
      });
}


void ssa_match(const std::vector<c_ssarray> & current_descs,
    const std::vector<c_ssarray> & reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray costs,
    const cv::Mat1b & mask)
{
  disps.create(reference_descs[0].size(), CV_16UC1);
  disps.setTo(0);

  costs.create(reference_descs[0].size(), CV_16UC1);
  costs.setTo(0);

  cv::Mat1w disp = disps.getMatRef();
  cv::Mat1w cost = costs.getMatRef();

  const int scales =
      reference_descs.size();

  const ssdesc *const*rptrs[scales];
  const ssdesc *const*cptrs[scales];

  for( int s = 0; s < scales; ++s ) {
    rptrs[s] = reference_descs[s].ptr();
    cptrs[s] = current_descs[s].ptr();
  }

  tbb::parallel_for(tbb_range(0, reference_descs[0].rows(), 16),
      [&](const tbb_range & r) {

        const int rwidth = reference_descs[0].cols();
        const int cwidth = current_descs[0].cols();

        for( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

          const uint8_t * mskp = mask[y];

          for( int x = 0; x < rwidth; ++x ) {
            if( mskp[x] ) {

              uint8_t cbest = absdiff(rptrs, cptrs, x, x, y, scales, UINT8_MAX);
              int xbest = x;

              for( int xx = x + 1, xxmax = std::min(x + max_disparity, cwidth); xx < xxmax; ++xx ) {

                const uint8_t c = absdiff(rptrs, cptrs, x, xx, y, scales, cbest);
                if( c < cbest ) {
                  xbest = xx;
                  cbest = c;
                  if ( !(cbest = c) ) {
                    break;
                  }
                }
              }

              disp[y][x] = xbest - x;
              cost[y][x] = cbest;
            }
          }
        }
      });
}

