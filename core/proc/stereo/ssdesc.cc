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
#include <core/proc/gradient.h>
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
      {sscmp_g04, "g04", "g04"},
      {sscmp_g05, "g05", "g05"},
      {sscmp_g06, "g06", "g06"},
      {sscmp_g07, "g07", "g07"},
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
void filter_image(const cv::Mat & src, cv::Mat3b & gl, cv::Mat3b & gr, cv::Mat1b & gx, cv::Mat1b & gy)
{

  static float kl[5] = {
     0.25, 0.5, 1, 1, 0
  };
  static const cv::Matx<float, 1, 5> KL = cv::Matx<float, 1, 5>(kl) / 2.75;

  static float kr[5] = {
     0, 1, 1, 0.5, 0.25
  };
  static const cv::Matx<float, 1, 5> KR = cv::Matx<float, 1, 5>(kr) / 2.75;

  cv::filter2D(src, gl, CV_8U, KL, cv::Point(4, 0), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gr, CV_8U, KR, cv::Point(0, 0), 0, cv::BORDER_REPLICATE);

  cv::Mat g;
  if ( src.channels() == 1 ) {
    g = src;
  }
  else {
    cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
  }

  static float kg[] = { -1, -1, 0, +1, +1 };
  static const cv::Matx<float, 1, 5> KGx = cv::Matx<float, 1, 5>(kg) / 4.0;
  static const cv::Matx<float, 5, 1> KGy = cv::Matx<float, 5, 1>(kg) / 4.0;

  cv::filter2D(g, gx, CV_8U, KGx, cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);
  cv::filter2D(g, gy, CV_8U, KGy, cv::Point(-1, -1), 128, cv::BORDER_REPLICATE);

  //cv::absdiff(g1, cv::Scalar::all(128), g1);
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
    uint8_t g[8];
  } u = {
      .m = _mm_sub_pi8(_mm_max_pu8(ma, mb), _mm_min_pu8(ma, mb))
  };

  const uint16_t sl = (uint16_t) (u.g[0]) + (uint16_t) (u.g[1]) + (uint16_t) (u.g[2]);
  const uint16_t sr = (uint16_t) (u.g[3]) + (uint16_t) (u.g[4]) + (uint16_t) (u.g[5]);

//  uint8_t s = u.a[0];
//  for( int i = 1; i < 4; ++i ) {
//    s = std::max(s, u.a[i]);
//  }
//
//  if ( scmp ) {
//    s = std::max(s, std::min(u.a[4], u.a[5]));
//    s = std::max(s, std::min(u.a[6], u.a[7]));
//  }

  return (uint8_t) (std::min((uint16_t) UINT8_MAX, std::min(sl, sr)));
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

#if 1
void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags)
{
  INSTRUMENT_REGION("");

  cv::Mat3b src;
  cv::Mat3b gl, gr;
  cv::Mat1b gx, gy;

  std::vector<cv::Size> sizes;

  if( image.channels() != 3 ) {
    CF_ERROR("ERROR: 3 channel color image required");
    return;
  }
  else {
    //cv::cvtColor(image, s, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(s, s, cv::COLOR_GRAY2BGR);
    //cv::cvtColor(image, s, cv::COLOR_BGR2YCrCb);
    //cv::cvtColor(image, s, cv::COLOR_BGR2Lab);
  }

  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  for( int scale = 0; scale <= maxlevel; ++scale ) {

    if( scale == 0 ) {
      src = image;
    }
    else {
      cv::pyrDown(src, src);
    }

    sizes.emplace_back(src.size());

    filter_image(src, gl, gr, gx, gy);

    {
      INSTRUMENT_REGION("pyrUp");
      for( int upscale = scale - 1; upscale >= 0; --upscale ) {
        cv::pyrUp(gl, gl, sizes[upscale]);
        cv::pyrUp(gr, gr, sizes[upscale]);
        cv::pyrUp(gx, gx, sizes[upscale]);
        cv::pyrUp(gy, gy, sizes[upscale]);
      }
    }

    c_ssarray &ssa = pyramid[scale];
    ssa.create(image.size());

    {
      INSTRUMENT_REGION("parallel_for");

      tbb::parallel_for(tbb_range(0, image.rows, tbb_grain_size),
          [&](const tbb_range & r) {

            const int width = src.cols;
            constexpr uint8_t u128 = (uint8_t)128;

            for( int y = r.begin(); y < r.end(); ++y ) {

              ssdesc *ssp = ssa[y];

              if( (flags & sscmp_all) == sscmp_all ) {

                for( int x = 0; x < width; ++x ) {

                  ssdesc & ss = ssp[x];

                  ss.g[0] = gl[y][x][0];
                  ss.g[1] = gl[y][x][1];
                  ss.g[2] = gl[y][x][2];
                  ss.g[3] = gr[y][x][0];
                  ss.g[4] = gr[y][x][1];
                  ss.g[5] = gr[y][x][2];
                  ss.g[6] = gx[y][x];
                  ss.g[7] = gy[y][x];
                }

              }
              else {

                memset(ssp, u128, sizeof(*ssp) * width);

                for( int x = 0; x < width; ++x ) {

                  ssdesc & ss = ssp[x];

                  if ( (flags & (1 << 0)) ) {
                    ss.g[0] = gl[y][x][0];
                  }
                  if ( (flags & (1 << 1)) ) {
                    ss.g[1] = gl[y][x][1];
                  }
                  if ( (flags & (1 << 2)) ) {
                    ss.g[2] = gl[y][x][2];
                  }
                  if ( (flags & (1 << 3)) ) {
                    ss.g[3] = gr[y][x][0];
                  }
                  if ( (flags & (1 << 4)) ) {
                    ss.g[4] = gr[y][x][1];
                  }
                  if ( (flags & (1 << 5)) ) {
                    ss.g[5] = gr[y][x][2];
                  }
                  if ( (flags & (1 << 6)) ) {
                    ss.g[6] = gx[y][x];
                  }
                  if ( (flags & (1 << 7)) ) {
                    ss.g[7] = gy[y][x];
                  }
                }

              }
            }
          });
    }
  }

//  CF_DEBUG("leave");
}
#elif ( 1 )
void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags)
{
  // TODO:
  // cv::stackBlur();

  cv::Mat s;
  std::vector<cv::Mat4b> gpyr[2];
  std::vector<cv::Size> sizes;

  if( image.channels() == 1 ) {
    s = image;
  }
  else {
    cv::cvtColor(image, s, cv::COLOR_BGR2GRAY);
  }

  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  gpyr[0].resize(maxlevel + 1);
  gpyr[1].resize(maxlevel + 1);

  for( int scale = 0; scale <= maxlevel; ++scale) {

    if( scale > 0 ) {
      cv::pyrDown(s, s);
    }

    sizes.emplace_back(s.size());
    filter_image(s, gpyr[0][scale], gpyr[1][scale]);
  }

  for( int scale = 0; scale <= maxlevel; ++scale) {

    const cv::Mat4b g[2] = {
        gpyr[0][scale],
        gpyr[1][scale]
    };

    c_ssarray &ssa = pyramid[scale];
    ssa.create(g[0].size());

    tbb::parallel_for(tbb_range(0, g[0].rows, tbb_grain_size),
        [&](const tbb_range & r) {

          const int width = g[0].cols;
          constexpr uint8_t u128 = (uint8_t)128;

          for( int y = r.begin(); y < r.end(); ++y ) {

            ssdesc *ssp = ssa[y];

            if( (flags & sscmp_all) == sscmp_all ) {

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                // TODO: implement this with SSE
                for ( int i = 0; i < 4; ++i ) {
                  ss.g[i] = g[0][y][x][i] >= u128 ? g[0][y][x][i] - u128 : u128 - g[0][y][x][i];
                }
                for ( int i = 4; i < 8; ++i ) {
                  ss.g[i] = g[1][y][x][i-4] >= u128 ? g[1][y][x][i-4] - u128 : u128 - g[1][y][x][i-4];
                }
              }

            }
            else {

              memset(ssp, 0, sizeof(*ssp) * width);

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                for ( int i = 0; i < 4; ++i ) {
                  if ( flags & (1<< i) ) {
                    ss.g[i] = g[0][y][x][i] > u128 ? g[0][y][x][i] - u128 : u128 - g[0][y][x][i];
                  }
                }
                for ( int i = 4; i < 8; ++i ) {
                  if ( flags & (1 << i) ) {
                    ss.g[i] = g[1][y][x][i-4] > u128 ? g[1][y][x][i-4] - u128 : u128 - g[1][y][x][i-4];
                  }
                }
              }
            }
          }
        });

  }

//  CF_DEBUG("leave");
}


#else

void ssa_pyramid(const cv::Mat & image,
    std::vector<c_ssarray> & pyramid,
    int maxlevel,
    int flags)
{
  // TODO:
  // cv::stackBlur();

  cv::Mat s, sb;
  cv::Mat4b g[2];
  //std::vector<cv::Size> sizes;

  if( image.channels() == 1 ) {
    s = image;
  }
  else {
    cv::cvtColor(image, s, cv::COLOR_BGR2GRAY);
  }

  pyramid.clear();
  pyramid.resize(maxlevel + 1);

  int r = 1;
  for( int scale = 0; scale <= maxlevel; ++scale, r <<= 1 ) {

    cv::stackBlur(s, sb, cv::Size(2 * r + 1, 2 * r + 1));
    filter_image(sb, g[0], g[1], scale + 1);

    c_ssarray &ssa = pyramid[scale];
    ssa.create(image.size());

    tbb::parallel_for(tbb_range(0, image.rows, tbb_grain_size),
        [&](const tbb_range & r) {

          const int width = image.cols;
          constexpr uint8_t u128 = (uint8_t)128;

          for( int y = r.begin(); y < r.end(); ++y ) {

            ssdesc *ssp = ssa[y];



            if( (flags & sscmp_all) == sscmp_all ) {

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                // TODO: implement this with SSE
                for ( int i = 0; i < 4; ++i ) {
                  ss.g[i] = g[0][y][x][i] >= u128 ? g[0][y][x][i] - u128 : u128 - g[0][y][x][i];
                }
                for ( int i = 4; i < 8; ++i ) {
                  ss.g[i] = g[1][y][x][i-4] >= u128 ? g[1][y][x][i-4] - u128 : u128 - g[1][y][x][i-4];
                }
              }

            }
            else {

              memset(ssp, 0, sizeof(*ssp) * width);

              for( int x = 0; x < width; ++x ) {

                ssdesc & ss = ssp[x];

                for ( int i = 0; i < 4; ++i ) {
                  if ( flags & (1<< i) ) {
                    ss.g[i] = g[0][y][x][i] > u128 ? g[0][y][x][i] - u128 : u128 - g[0][y][x][i];
                  }
                }
                for ( int i = 4; i < 8; ++i ) {
                  if ( flags & (1<< i) ) {
                    ss.g[i] = g[1][y][x][i-4] > u128 ? g[1][y][x][i-4] - u128 : u128 - g[1][y][x][i-4];
                  }
                }
              }
            }
          }
        });

  }

//  CF_DEBUG("leave");
}
#endif


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

static inline uint8_t absv(uint8_t v)
{
  constexpr uint8_t u128 = (uint8_t) 128;
  return v >= u128 ? v - u128 : 128 - v;
}

void ssa_mask(const c_ssarray & ssa, cv::Mat1b & output_mask)
{
  output_mask.create(ssa.size());

  for( int y = 0; y < output_mask.rows; ++y ) {

    const ssdesc *ssp = ssa[y];

    for( int x = 0; x < output_mask.cols; ++x ) {

      const ssdesc &ss = ssp[x];
      output_mask[y][x] = absv(ss.g[6]) > 3 ? 255 : 0;
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

        d = std::max(d, absdiff(ssa1[l][(y + rc1.y) ][(x + rc1.x)],
            ssa2[l][(y + rc2.y) ][(x + rc2.x)]));
      }

      distances[y][x] = d;
    }
  }

}



//void ssa_match(const c_ssarray & current_descs, const c_ssarray & reference_descs, int max_disparity,
//    cv::OutputArray disps, cv::OutputArray costs,
//    const cv::Mat1b & mask)
//{
//  disps.create(reference_descs.size(), CV_16UC1);
//  disps.setTo(0);
//
//  costs.create(reference_descs.size(), CV_16UC1);
//  costs.setTo(UINT16_MAX);
//
//  cv::Mat1w disp = disps.getMatRef();
//  cv::Mat1w cost = costs.getMatRef();
//
//  tbb::parallel_for(tbb_range(0, reference_descs.rows(), 16),
//      [&](const tbb_range & r) {
//
//        const int ccols = current_descs.cols();
//        const int rcols = reference_descs.cols();
//
//        for( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
//
//          const uint8_t * mskp = mask[y];
//
//          const ssdesc *rssp = reference_descs[y];
//          const ssdesc *cssp = current_descs[y];
//
//          for( int x = 0; x < rcols; ++x ) {
//            if( mskp[x] ) {
//
//              const ssdesc & rss = rssp[x];
//
//              uint8_t cbest =
//                  absdiff(rss, cssp[x],
//                      true);
//
//              int xbest = x;
//
//              for( int xx = x + 1, xxmax = std::min(x + max_disparity, ccols); xx < xxmax; ++xx ) {
//
//                const uint8_t c =
//                    absdiff(rss, cssp[xx],
//                        true);
//
//                if( c < cbest ) {
//                  xbest = xx;
//                  if ( !(cbest = c) ) {
//                    break;
//                  }
//                }
//              }
//
//              disp[y][x] = xbest - x;
//              cost[y][x] = cbest;
//            }
//          }
//        }
//      });
//}


void ssa_match(const std::vector<c_ssarray> & current_descs,
    const std::vector<c_ssarray> & reference_descs, int max_disparity,
    cv::OutputArray disps, cv::OutputArray costs,
    const cv::Mat1b & mask,
    bool enable_checks)
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

          struct rmatch {
            int16_t rx = -1;
            uint8_t score = UINT8_MAX;
          } imatch[cwidth];


          for( int xr = 0; xr < rwidth; ++xr ) {
            if( mskp[xr] ) {

              uint8_t best_score =
                  absdiff(rptrs, cptrs,
                      xr, xr, y, scales,
                      UINT8_MAX);

              int best_xc = xr;

              for( int xc = xr + 1, xcm = std::min(xr + max_disparity, cwidth); xc < xcm; ++xc ) {

                const uint8_t score =
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

