/*
 * ssdesc.cc
 *
 *  Created on: Apr 20, 2023
 *      Author: amyznikov
 *
 *  https://en.wikipedia.org/wiki/Finite_difference_coefficient
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
      {sscmp_g00, "g00", "g00"},
      {sscmp_g01, "g01", "g01"},
      {sscmp_g02, "g02", "g02"},
      {sscmp_g03, "g03", "g03"},
      {sscmp_g10, "g10", "g10"},
      {sscmp_g11, "g11", "g11"},
      {sscmp_g12, "g12", "g12"},
      {sscmp_g13, "g13", "g13"},
      {sscmp_all},
  };

  return members;
}


namespace {

typedef tbb::blocked_range<int> tbb_range;
const int tbb_grain_size = 128;

/*
 * https://en.wikipedia.org/wiki/Finite_difference_coefficient
 * */
void compute_gradients(const cv::Mat & src, cv::Mat4b & g1, cv::Mat4b & g2)
{

//  static float k1[4][5 * 5] = {
//      {
//          0, 0, 0, 0, 0,
//          0, 0, 0, 0, 0,
//          +1./12, -2./3, 0, +2./3, -1./12,
//          0, 0, 0, 0, 0,
//          0, 0, 0, 0, 0,
//      },
//      {
//          0, 0, 0, 0, -1./12,
//          0, 0, 0, +2./3, 0,
//          0, 0, 0, 0, 0,
//          0, -2./3, 0, 0, 0,
//          +1./12, 0, 0, 0, 0,
//      },
//      {
//          0, 0, -1./12, 0, 0,
//          0, 0, +2./3, 0, 0,
//          0, 0, 0, 0, 0,
//          0, 0, -2./3, 0, 0,
//          0, 0, +1./12, 0, 0,
//      },
//      {
//          -1./12, 0, 0, 0, 0,
//          0, +2./3, 0, 0, 0,
//          0, 0, 0, 0, 0,
//          0, 0, 0, -2./3, 0,
//          0, 0, 0, 0, +1./12,
//      },
//  };
//  static float k2[4][5 * 5] = {
//      {
//          0, 0, 0, 0, 0,
//          0, 0, 0, 0, 0,
//          -1./12, 4./3, -5./ 2, 4./3, -1./ 12,
//          0, 0, 0, 0, 0,
//          0, 0, 0, 0, 0,
//      },
//      {
//          0, 0, 0, 0, -1./12,
//          0, 0, 0, 4./3, 0,
//          0, 0, -5./2, 0, 0,
//          0, 4./3, 0, 0, 0,
//          -1./12, 0, 0, 0, 0,
//      },
//      {
//          0, 0, -1./12, 0, 0,
//          0, 0, 4./3, 0, 0,
//          0, 0, -5./2, 0, 0,
//          0, 0, 4./3, 0, 0,
//          0, 0, -1./12, 0, 0,
//      },
//      {
//          -1./12, 0, 0, 0, 0,
//          0, 4./3, 0, 0, 0,
//          0, 0, -5./2, 0, 0,
//          0, 0, 0, 4./3, 0,
//          0, 0, 0, 0, -1./12,
//      },
//  };

  static float k1[4][5 * 5] = {
      {
         -.25, -.25,  0,  +.25,  +.25,
          -.5,  -.5,  0,  +.5,   +.5,
           -1,   -1,  0,   +1,   +1,
          -.5,  -.5,  0,  +.5,   +.5,
        -.25,  -.25,  0,  +.25,  +.25,
      },
      {
          0, +.25, +.25, +.5, +1,
          -.25, 0, +.5, +1, +.5,
          -.25, -.5, 0, +.5, +.25,
          -.5, -1, -.5, 0, +.25,
          -1, -.5, -.25, -.25, 0,

      },
      {
          +.25, +.5, +1, +.5, +.25,
          +.25, +.5, +1, +.5, +.25,
                0, 0, 0, 0, 0,
          -.25, -.5, -1, -.5, -.25,
          -.25, -.5, -1, -.5, -.25,
      },
      {
            +1,  +.5,  +.25, +.25,   0,
           +.5,   +1,  +.5,    0,  -.25,
          +.25,  +.5,    0,  -.5,  -.25,
          +.25,   0,   -.5,   -1,  -.5,
           0,   -.25, -.25,  -.5,   -1,
      },
  };

  static float k2[4][5 * 5] = {
      {
          .25,  .25,    -1,   .25,   .25,
           .5,   .5,    -2,    .5,    .5,
            1,    1,    -4,     1,     1,
           .5,   .5,    -2,    .5,    .5,
          .25,  .25,    -1,   .25,   .25,
      },
      {
           -1, .25,  .25,  .5,   1,
          .25,  -2,   .5,   1,  .5,
          .25,  .5,   -4,  .5, .25,
           .5,   1,   .5,  -2, .25,
            1,  .5,  .25, .25,  -1,
      },
      {
         .25,  .5,   1,  .5, .25,
         .25,  .5,   1,  .5, .25,
          -1,  -2,  -4,  -2,  -1,
         .25,  .5,   1,  .5, .25,
         .25,  .5,   1,  .5, .25,
      },
      {
           1,  .5, .25, .25,  -1,
          .5,   1,  .5,  -2, .25,
         .25,  .5,  -4,  .5, .25,
         .25,  -2,  .5,   1,  .5,
          -1, .25, .25,  .5,   1,
      },
  };

  static const thread_local cv::Matx<float, 5, 5> K1[4] = {
      cv::Matx<float, 5, 5>(k1[0]) / 12.f,
      cv::Matx<float, 5, 5>(k1[1]) / 12.f,
      cv::Matx<float, 5, 5>(k1[2]) / 12.f,
      cv::Matx<float, 5, 5>(k1[3]) / 12.f,
  };

  static const thread_local cv::Matx<float, 5, 5> K2[4] = {
      cv::Matx<float, 5, 5>(k2[0]) / 14.f,
      cv::Matx<float, 5, 5>(k2[1]) / 14.f,
      cv::Matx<float, 5, 5>(k2[2]) / 14.f,
      cv::Matx<float, 5, 5>(k2[3]) / 14.f,
  };

  std::vector<cv::Mat> gg[2] = {
      std::vector<cv::Mat>(4),
      std::vector<cv::Mat>(4)
  };

  for( int i = 0; i < 4; ++i ) {
    cv::filter2D(src, gg[0][i], CV_8U, K1[i], cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);
  }
  for( int i = 0; i < 4; ++i ) {
    cv::filter2D(src, gg[1][i], CV_8U, K2[i], cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);
  }

  cv::merge(gg[0], g1);
  cv::merge(gg[1], g2);
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
    if( (d = std::max(d, absdiff(a[s][y][xa], b[s][y][xb]))) > worst ) {
      break;
    }
  }

  return d;
}


} // namespace

void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags)
{
  cv::Mat s;
  cv::Mat4b g[2];
  std::vector<cv::Size> sizes;

  if( image.channels() == 1 ) {
    s = image;
  }
  else {
    cv::cvtColor(image, s, cv::COLOR_BGR2GRAY);
  }

  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  for( int scale = 0; scale <= maxlevel; ++scale ) {

    if( scale > 0 ) {
      cv::pyrDown(s, s);
    }

    sizes.emplace_back(s.size());

    compute_gradients(s, g[0], g[1]);

    for( int upscale = scale - 1; upscale >= 0; --upscale ) {
      cv::pyrUp(g[0], g[0], sizes[upscale]);
      cv::pyrUp(g[1], g[1], sizes[upscale]);
    }

    c_ssarray &ssa = pyramid[scale];
    ssa.create(image.size());

    tbb::parallel_for(tbb_range(0, image.rows, tbb_grain_size),
        [&](const tbb_range & r) {

          const int width = image.cols;

          for( int y = r.begin(); y < r.end(); ++y ) {

            ssdesc *ssp = ssa[y];

            if( (flags & sscmp_all) == sscmp_all ) {

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                for ( int i = 0; i < 4; ++i ) {
                  ss.g[i] = (uint8_t) g[0][y][x][i];
                }
                for ( int i = 4; i < 8; ++i ) {
                  ss.g[i] = (uint8_t) g[1][y][x][i-4];
                }
              }

            }
            else {

              memset(ssp, 0, sizeof(*ssp) * width);

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                for ( int i = 0; i < 4; ++i ) {
                  if ( flags & (1<< i) ) {
                    ss.g[i] = (uint8_t) g[0][y][x][i];
                  }
                }
                for ( int i = 4; i < 8; ++i ) {
                  if ( flags & (1<< i) ) {
                    ss.g[i] = (uint8_t) g[1][y][x][i-4];
                  }
                }
              }
            }
          }
        });

  }

//  CF_DEBUG("leave");
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

      for( int i = 0; i < 8; ++i ) {
        if( flags & (1 << i) ) {
          v = ss.g[i];
          break;
        }
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

      for( int l = 1; l < lmax; ++l ) {

        d = std::max(d, absdiff(ssa1[l][(y + rc1.y)][(x + rc1.x)],
            ssa2[l][(y + rc2.y)][(x + rc2.x)]));
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

