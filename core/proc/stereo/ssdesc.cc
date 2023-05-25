/*
 * ssdesc.cc
 *
 *  Created on: Apr 20, 2023
 *      Author: amyznikov
 *
 *  https://en.wikipedia.org/wiki/Finite_difference_coefficient
 *
 *  Header files for x86 SIMD intrinsics
 *  https://stackoverflow.com/questions/11228855/header-files-for-x86-simd-intrinsics/31185861
 *
 *  <mmintrin.h>  MMX
 *  <xmmintrin.h> SSE
 *  <emmintrin.h> SSE2
 *  <pmmintrin.h> SSE3
 *  <tmmintrin.h> SSSE3
 *  <smmintrin.h> SSE4.1
 *  <nmmintrin.h> SSE4.2
 *  <ammintrin.h> SSE4A
 *  <wmmintrin.h> AES
 *  <immintrin.h> AVX, AVX2, FMA
 *
 *  Intel® Intrinsics Guide
 *  https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html
 *
 */

#ifndef __OPTIMIZE__
# define __OPTIMIZE__
#endif

#include "ssdesc.h"
#include <tbb/tbb.h>
#include <xmmintrin.h>
#include <immintrin.h>
#include <core/proc/gradient.h>
#include <core/ssprintf.h>

#include <core/debug.h>

namespace {

typedef tbb::blocked_range<int> tbb_range;
const int tbb_grain_size = 128;
//
///*
// * https://en.wikipedia.org/wiki/Finite_difference_coefficient
// * */
//void filter_image(const cv::Mat & src, cv::Mat3b & gl, cv::Mat3b & gr, cv::Mat1b & gx, cv::Mat1b & gy)
//{
//
//  static float kl[5] = {
//     0.25, 0.5, 1, 1, 0
//  };
//  static const cv::Matx<float, 1, 5> KL = cv::Matx<float, 1, 5>(kl) / 2.75;
//
//  static float kr[5] = {s
//     0, 1, 1, 0.5, 0.25
//  };
//  static const cv::Matx<float, 1, 5> KR = cv::Matx<float, 1, 5>(kr) / 2.75;
//
//  cv::filter2D(src, gl, CV_8U, KL, cv::Point(4, 0), 0, cv::BORDER_REPLICATE);
//  cv::filter2D(src, gr, CV_8U, KR, cv::Point(0, 0), 0, cv::BORDER_REPLICATE);
//
//  cv::Mat g;
//  if ( src.channels() == 1 ) {
//    g = src;
//  }
//  else {
//    cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
//  }
//
//  static float kg[] = { -1, -1, 0, +1, +1 };
//  static const cv::Matx<float, 1, 5> KGx = cv::Matx<float, 1, 5>(kg) / 4.0;
//  static const cv::Matx<float, 5, 1> KGy = cv::Matx<float, 5, 1>(kg) / 4.0;
//
//  cv::filter2D(g, gx, CV_8U, KGx, cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);
//  cv::filter2D(g, gy, CV_8U, KGy, cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);
//
//  //cv::absdiff(g1, cv::Scalar::all(128), g1);
//}

//static inline uint8_t absdiff(uint8_t a, uint8_t b)
//{
//  return a > b ? a - b : b - a;
//}

static inline uint16_t absdiff(const ssdesc & a, const ssdesc & b)
{
  uint16_t s = 0;

#if __AVX2__

  for( int i = 0; i < 2; ++i ) {

    union {
      __m256i m;
      uint16_t g[4];
    } u = {
        .m =
            _mm256_sad_epu8( _mm256_lddqu_si256((__m256i const*) &a.g[i * 32]),
                _mm256_lddqu_si256((__m256i const*) &b.g[i * 32]))
    };

    s += u.g[0];
  }

#elif __SSE__

  for( int i = 0; i < 8; ++i ) {

    union {
      __m64 m;
      uint16_t g[4];
    } u = {
        .m = _mm_sad_pu8(_m_from_int64(a.u64[0]), _m_from_int64(b.u64[0]))
    };

    s += u.g[0];
  }

#else

  for( int i = 0; i < 64; ++i ) {
    s += (uint16_t) (std::max(a.g[i], b.g[i]) - std::min(a.g[i], b.g[i]));
  }

#endif

  return s;
}

static inline uint16_t absdiff(const ssdesc *const* a[/*scales*/],
    const ssdesc *const* b[/*scales*/], int xa, int xb, int y, int scales, uint16_t worst)
{
  uint16_t d = absdiff(a[0][y][xa], b[0][y][xb]);
  for( int s = 1; d < worst && s < scales; ++s ) {
    d += absdiff(a[s][y>>s][xa>>s], b[s][y>>s][xb>>s]);
  }

  return d;
}


} // namespace

void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel)
{
  INSTRUMENT_REGION("");

  cv::Mat3b src;
  cv::Mat1b g;
  cv::Mat1b gx;

  if( image.channels() != 3 ) {
    CF_ERROR("ERROR: 3 channel color image required");
    return;
  }

  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  //CF_DEBUG("pyramid.size=%d", (int)(pyramid.size()));

  for( int scale = 0; scale <= maxlevel; ++scale ) {

    if( scale == 0 ) {
      src = image;
    }
    else {
      cv::pyrDown(src, src);
    }

    cv::cvtColor(src, gx, cv::COLOR_BGR2GRAY);

    static const cv::Matx<float, 1, 5> KGx =
        cv::Matx<float, 1, 5>(-0.25, -0.25, 0, +0.25, +0.25);

    cv::filter2D(gx, gx, CV_8U, KGx, cv::Point(-1, -1), 128,
        cv::BORDER_REPLICATE);

    //cv::absdiff(gx, cv::Scalar::all(128), gx);


    c_ssarray &ssa = pyramid[scale];
    ssa.create(src.size());

    {
      INSTRUMENT_REGION("parallel_for");

      tbb::parallel_for(tbb_range(2, src.rows - 2, tbb_grain_size),
          [&](const tbb_range & r) {

            const int width = src.cols;
            constexpr uint8_t u128 = (uint8_t)128;

            for( int y = r.begin(); y < r.end(); ++y ) {

              ssdesc *ssp = ssa[y];

              memset(ssp, 0, 2 * sizeof(*ssp));
              memset(ssp + width - 2, 0, 2 * sizeof(*ssp));

              for( int x = 2; x < width - 2; ++x ) {

                ssdesc & ss = ssp[x];

                static constexpr uint8_t mask[5][5] = {
                    0, 1, 1, 1, 0,
                    1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1,
                    0, 1, 1, 1, 0,
                };

                int k = 0;

                for ( int yy = 0; yy < 5; ++yy ) {
                  for ( int xx = 0; xx < 5; ++xx ) {
                    if ( mask[yy][xx] ) {
                      ss.g[k++] = src[y + yy - 2][x + xx - 2][0];
                      ss.g[k++] = src[y + yy - 2][x + xx - 2][1];
                      ss.g[k++] = src[y + yy - 2][x + xx - 2][2];
                    }
                  }
                }

                ss.g[k++] = gx[y][x];
//                if ( k != sizeof(ss.g) ) {
//                  CF_ERROR("FATAL APP BUG: k=%d", k);
//                  exit (1);
//                }
              }
            }
          });
    }
  }

//  CF_DEBUG("leave");
}



void ssa_cvtfp32(const c_ssarray & ssa, cv::OutputArray output)
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
      dst[y][x] = ss.g[63];
    }
  }
}

static inline uint8_t absv(uint8_t v)
{
  constexpr uint8_t u128 = (uint8_t) 128;
  return v >= u128 ? v - u128 : 128 - v;
}

void ssa_texture(const c_ssarray & ssa, cv::Mat1b & output_image)
{
  output_image.create(ssa.size());

  for( int y = 0; y < output_image.rows; ++y ) {

    const ssdesc *ssp = ssa[y];

    for( int x = 0; x < output_image.cols; ++x ) {

      const ssdesc &ss = ssp[x];
      output_image[y][x] = absv(ss.g[63]);
    }
  }

}

void ssa_mask(const c_ssarray & ssa, cv::Mat1b & output_mask)
{
  output_mask.create(ssa.size());

  for( int y = 0; y < output_mask.rows; ++y ) {

    const ssdesc *ssp = ssa[y];

    for( int x = 0; x < output_mask.cols; ++x ) {

      const ssdesc &ss = ssp[x];
      output_mask[y][x] = absv(ss.g[63]) > 1 ? 255 : 0;
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

  const int scales =
      ssa1.size();

  const int s = scales - 1;

  //CF_DEBUG("rc1:(x=%d y=%d) rc2:(x=%d y=%d)", rc1.x, rc1.y, rc2.x, rc2.y);

  for( int y = 0; y < rc1.height; ++y ) {

    for( int x = 0; x < rc1.width; ++x ) {

//      distances[y][x] =
//          absdiff(ssa1[s][(y + rc1.y) >> s][(x + rc1.x) >> s],
//              ssa2[s][(y + rc2.y) >> s][(x + rc2.x) >> s]);

      uint16_t d =
          absdiff(ssa1[0][y + rc1.y][x + rc1.x],
              ssa2[0][y + rc2.y][x + rc2.x]);

      for( int s = 1; s < scales; ++s ) {

        d += absdiff(ssa1[s][(y + rc1.y) >> s][(x + rc1.x) >> s],
            ssa2[s][(y + rc2.y) >> s][(x + rc2.x) >> s]);
      }

      distances[y][x] = d;
    }
  }

}

void ssa_match(const std::vector<c_ssarray> & current_descs,
    const std::vector<c_ssarray> & reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray costs,
    const cv::Mat1b & mask,
    bool enable_checks)
{
  INSTRUMENT_REGION("");

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

          struct rmatch {
            int16_t rx = -1;
            uint16_t score = UINT16_MAX;
          } imatch[cwidth];


          for( int xr = 0; xr < rwidth; ++xr ) {
            if( mskp[xr] ) {

              uint16_t best_score =
                  absdiff(rptrs, cptrs,
                      xr, xr, y, scales,
                      UINT16_MAX);

              int best_xc = xr;

              for( int xc = xr + 1, xcm = std::min(xr + max_disparity, cwidth); xc < xcm; ++xc ) {

                const uint16_t score =
                    absdiff(rptrs, cptrs,
                        xr, xc, y, scales,
                        best_score);

                if( score < best_score ) {
                  best_xc = xc;
                  if ( !(best_score = score) ) {
                    break;
                  }
                }
              }

              if ( !enable_checks ) {
                disp[y][xr] = best_xc - xr;
                cost[y][xr] = best_score;
              }
              else  if ( (imatch[best_xc].rx <= 0) ) {
                disp[y][xr] = best_xc - xr;
                cost[y][xr] = best_score;
                imatch[best_xc].score = best_score;
                imatch[best_xc].rx = xr;
              }
              else if ((best_score < imatch[best_xc].score)) {
                disp[y][imatch[best_xc].rx] = 0;
                cost[y][imatch[best_xc].rx] = 0;
                disp[y][xr] = best_xc - xr;
                cost[y][xr] = best_score;
                imatch[best_xc].score = best_score;
                imatch[best_xc].rx = xr;
              }
            }
          }
        }
      });
}

