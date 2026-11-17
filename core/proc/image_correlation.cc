/*
 * image_correlation.cc
 *
 *  Created on: Nov 14, 2026
 *      Author: amyznikov
 */
#include "image_correlation.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/io/c_stdio_file.h>
#include <core/debug.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>

template<class _Tp>
static double _compute_image_correlation_sw(cv::InputArray _src1, cv::InputArray _src2,
    cv::InputArray _mask, int max_grid_size = 16)
{
  const cv::Mat src1 = _src1.getMat();
  const cv::Mat src2 = _src2.getMat();
  const cv::Mat mask = _mask.getMat();

  const cv::Size size = src1.size();
  if ( size.width < 2 || size.height < 2 ) {
    return -1.0;
  }

  constexpr int MinBlockSize = 128;

  const int grid_n = std::max(1, max_grid_size);
  const int nx = std::max(1, std::min(grid_n, size.width / MinBlockSize));
  const int ny = std::max(1, std::min(grid_n, size.height / MinBlockSize));

  const int base_bw = size.width / nx;
  const int base_bh = size.height / ny;

  struct BlockStat {
    int bx = -1;
    int by = -1;
    cv::Rect roi;
    double meanX = 0.0;
    double meanY = 0.0;
    double varX  = 0.0;
    double varY  = 0.0;
    double covXY = 0.0;
    double AADX  = 0.0;
    double AADY  = 0.0;
    double rho   = 0.0;
    double sgnlX = 0.0;
    double sgnlY = 0.0;
    double sgnl = 0.0;
    size_t npxls = 0;
    bool valid   = false;
  };

  std::vector<BlockStat> stats(nx * ny);

  for ( int by = 0; by < ny; ++by ) {
    const int y0 = by * base_bh;
    const int y1 = (by == ny - 1) ? size.height : (y0 + base_bh);

    for ( int bx = 0; bx < nx; ++bx ) {
      const int x0 = bx * base_bw;
      const int x1 = (bx == nx - 1) ? size.width : (x0 + base_bw);

      BlockStat & st = stats[by * nx + bx];
      st.bx = bx;
      st.by = by;
      st.roi = cv::Rect(x0, y0, x1 - x0, y1 - y0);
    }
  }

  const uint8_t * src1_base = src1.ptr();
  const size_t src1_stride  = src1.step;
  const uint8_t * src2_base = src2.ptr();
  const size_t src2_stride  = src2.step;
  const uint8_t * mask_base = mask.empty() ? nullptr : mask.ptr();
  const size_t mask_stride  = mask.empty() ? 0 : mask.step;

  parallel_for(0, nx * ny, [&](const auto & range) {
    using _Tpc = std::conditional_t<std::is_floating_point_v<_Tp>, _Tp, float>;

    constexpr _Tpc safe_min = std::numeric_limits<_Tpc>::min();

    for ( int idx = rbegin(range), end_idx = rend(range); idx < end_idx; ++idx ) {
      BlockStat & st = stats[idx];
      const int x0 = st.roi.x;
      const int y0 = st.roi.y;
      const int x1 = st.roi.x + st.roi.width;
      const int y1 = st.roi.y + st.roi.height;
      const int area = st.roi.area();

      _Tpc SX = 0, SY = 0;
      int npxls = mask_base ? 0 : area;

      for ( int y = y0; y < y1; ++y ) {
        const _Tp * srcp1 = (const _Tp *)(src1_base + y * src1_stride);
        const _Tp * srcp2 = (const _Tp *)(src2_base + y * src2_stride);

        if ( !mask_base ) {
          for ( int x = x0; x < x1; ++x ) {
            SX += srcp1[x];
            SY += srcp2[x];
          }
        }
        else {
          const uint8_t * mskp = (const uint8_t *)(mask_base + y * mask_stride);
          for ( int x = x0; x < x1; ++x ) {
            if ( mskp[x] ) {
              SX += srcp1[x];
              SY += srcp2[x];
              ++npxls;
            }
          }
        }
      }

      st.npxls = npxls;
      if ( npxls < area * 3 / 4 || npxls < 2 ) {
        continue;
      }

      const _Tpc inv_n = _Tpc(1) / _Tpc(npxls);
      const _Tpc meanX = SX * inv_n;
      const _Tpc meanY = SY * inv_n;

      _Tpc SXX = 0, SYY = 0, SXY = 0, AADX = 0, AADY = 0;

      for ( int y = y0; y < y1; ++y ) {
        const _Tp * srcp1 = (const _Tp *)(src1_base + y * src1_stride);
        const _Tp * srcp2 = (const _Tp *)(src2_base + y * src2_stride);

        if ( !mask_base ) {
          for ( int x = x0; x < x1; ++x ) {
            const _Tpc dx = srcp1[x] - meanX, dy = srcp2[x] - meanY;
            SXX += dx * dx, SYY += dy * dy, SXY += dx * dy;
            AADX += std::abs(dx), AADY += std::abs(dy);
          }
        }
        else {
          const uint8_t * mskp = (const uint8_t *)(mask_base + y * mask_stride);
          for ( int x = x0; x < x1; ++x ) {
            if ( mskp[x] ) {
              const _Tpc dx = srcp1[x] - meanX, dy = srcp2[x] - meanY;
              SXX += dx * dx, SYY += dy * dy, SXY += dx * dy;
              AADX += std::abs(dx), AADY += std::abs(dy);
            }
          }
        }
      }
      AADX *= inv_n, AADY *= inv_n;

      const _Tpc varX  = SXX * inv_n;
      const _Tpc varY  = SYY * inv_n;
      const _Tpc covar = SXY * inv_n;
      const _Tpc rho = (varX > safe_min && varY > safe_min) ? covar / std::sqrt(varX * varY) : _Tpc(0);

      st.meanX = meanX;
      st.meanY = meanY;
      st.varX  = varX;
      st.varY  = varY;
      st.covXY = covar;
      st.AADX  = AADX;
      st.AADY  = AADY;
      st.rho   = rho;
      st.sgnlX = std::sqrt(varX) / AADX;
      st.sgnlY = std::sqrt(varY) / AADY;
      st.sgnl = std::max(st.sgnlX, st.sgnlY);
      st.valid = true;
    }
  });

  c_stdio_file fp;

  double avg_sgnl = 0;
  double rho_min = DBL_MAX;
  int valid_blocks = 0;

  for( size_t i = 0, n = stats.size(); i < n; ++i ) {
    auto & s = stats[i];
    if( s.valid ) {
      avg_sgnl += s.sgnl;
      ++valid_blocks;
    }
  }

  avg_sgnl /= valid_blocks;

  if( false ) {
    if( !fp.open("/home/projects/temp/ECCDBG/blockstats.txt", "a") ) {
      CF_ERROR("Can not create block stats dump file: %s", strerror(errno));
    }
    else {
      fprintf(fp, "BLK\tBX\tBY\tnpix\tl\tt\tr\tb"
          "\tmeanX\tmeanY"
          "\tvarX\tvarY\tcovXY"
          "\tAADX\tAADY"
          "\tSgnlX\tSgnlY\tSgnl"
          "\trho_expected\trho\trho_recomp"
          "\tvalid"
          "\n");
    }
  }

  for( size_t i = 0, n = stats.size(); i < n; ++i ) {
    const auto & s = stats[i];

    const double rho_expected = 1. - 0.5 * CV_PI / (s.sgnl * s.sgnl);
    const double rho_recomputed = 1. - (rho_expected - s.rho) * s.sgnl / avg_sgnl;

    if( s.valid && rho_recomputed < rho_min ) {
      rho_min = rho_recomputed;
    }

    if ( fp.is_open() ) {

      fprintf(fp, "%6zu\t%4d\t%4d\t%zu"
          "\t%4d\t%4d\t%4d\t%4d"
          "\t%g\t%g"
          "\t%g\t%g\t%g"
          "\t%g\t%g"
          "\t%g\t%g\t%g"
          "\t%g\t%g\t%g"
          "\t%d"
          "\n",
          i, s.bx, s.by, s.npxls,
          s.roi.x, s.roi.y, s.roi.x + s.roi.width, s.roi.y + s.roi.height,
          s.meanX, s.meanY,
          s.varX, s.varY, s.covXY,
          s.AADX, s.AADY,
          s.sgnlX, s.sgnlY, s.sgnl,
          rho_expected, s.rho, rho_recomputed,
          (int) s.valid);
    }
  }

  if ( fp.is_open() ) {
    fprintf(fp, "\n\n\n");
    fp.close();
  }


  return rho_min < DBL_MAX ? rho_min : -1;
}

// wrapper for external calls
double compute_image_correlation_sw(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask)
{
  INSTRUMENT_REGION("");
  if ( src1.type() != src2.type() || src1.size() != src2.size() ) {
    CF_ERROR("src1 and src2 must be single-chanel of the sae size and type");
    return -1.0;
  }

  if ( src1.channels() != 1 ) {
    CF_ERROR("Bad input images: must be single-channel of the same size");
    return -1.0;
  }

  if ( !mask.empty() ) {
    if ( mask.size() != src1.size() || mask.type() != CV_8UC1 ) {
      CF_ERROR("Bad input mask: must be CV_8UC1 of the same size as innut images");
      return -1.0;
    }
  }

  CV_DISPATCH(src1.depth(), _compute_image_correlation_sw, src1, src2, mask);
  CF_ERROR("Not supported image depth=%d", src1.depth());
  return -1.0;
}


template<class _Tp>
static double _compute_image_correlation(cv::InputArray _src1, cv::InputArray _src2, cv::InputArray _mask)
{
  const cv::Size size = _src1.size();
  const cv::Mat src1 = _src1.getMat();
  const cv::Mat src2 = _src2.getMat();
  const cv::Mat mask = _mask.getMat();

  const uint8_t * src1_base = src1.ptr();
  const size_t src1_stride  = src1.step;

  const uint8_t * src2_base = src2.ptr();
  const size_t src2_stride  = src2.step;

  const uint8_t * mask_base = mask.empty() ? nullptr : mask.ptr();
  const size_t mask_stride  = mask.empty() ? 0 : mask.step;

  struct Accumulator {
    double SX = 0;
    double SY = 0;
    double SXX = 0;
    double SYY = 0;
    double SXY = 0;
    double npxls = 0;
  } acc;

  std::mutex mtx;

  parallel_for(0, size.height, [=, &acc, &mtx](const auto & range) {

    using _Tcomp = std::conditional_t<std::is_floating_point_v<_Tp>, _Tp, float>;

    _Tcomp SX = 0;
    _Tcomp SY = 0;
    _Tcomp SXX = 0;
    _Tcomp SYY = 0;
    _Tcomp SXY = 0;
    size_t npxls = 0;

    for ( int y = rbegin(range); y < rend(range); ++y ) {
      const _Tp * srcp1 = (const _Tp * )(src1_base + y * src1_stride);
      const _Tp * srcp2 = (const _Tp * )(src2_base + y * src2_stride);

      if ( !mask_base ) {
        for ( int x = 0; x < size.width; ++x ) {
          const _Tcomp xv = _Tcomp(srcp1[x]);
          const _Tcomp yy = _Tcomp(srcp2[x]);
          SX += xv;
          SY += yy;
          SXX += xv * xv;
          SYY += yy * yy;
          SXY += xv * yy;
        }
      }
      else {
        const uint8_t * mskp = mask_base ? (const uint8_t * )(mask_base + y * mask_stride) : nullptr;
        for ( int x = 0; x < size.width; ++x ) {
          if ( mskp[x] ) {
            const _Tcomp xv = _Tcomp(srcp1[x]);
            const _Tcomp yv = _Tcomp(srcp2[x]);
            SX += xv;
            SY += yv;
            SXX += xv * xv;
            SYY += yv * yv;
            SXY += xv * yv;
            npxls += 1;
          }
        }
      }
    }

    std::lock_guard<std::mutex> lock(mtx);
    acc.SX  += SX;
    acc.SY  += SY;
    acc.SXX += SXX;
    acc.SYY += SYY;
    acc.SXY += SXY;
    acc.npxls += npxls;
  });

  const double npxls = mask.empty() ?  size.area() : acc.npxls;
  if ( npxls < 2 ) {
    return -1.0;
  }

  const double invpxls = 1.0 / npxls;
  const double meanX = acc.SX * invpxls;
  const double meanY = acc.SY  * invpxls;
  const double varX = acc.SXX  * invpxls - meanX * meanX;
  const double varY = acc.SYY  * invpxls - meanY * meanY;
  const double covar = acc.SXY  * invpxls - meanX * meanY;
  constexpr double safe_min = std::numeric_limits<double>::min();
  const double total_correlation = (varX > safe_min && varY > safe_min) ?
      covar / (std::sqrt(varX) * std::sqrt(varY)) : 0.0;

  return total_correlation;
}

double compute_image_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask)
{
  INSTRUMENT_REGION("");
  if ( src1.type() != src2.type() || src1.size() != src2.size() ) {
    CF_ERROR("src1 and src2 must be single-chanel of the sae size and type");
    return -1.0;
  }

  if ( src1.channels() != 1 ) {
    CF_ERROR("Bad input images: must be single-channel of the same size");
    return -1.0;
  }

  if ( !mask.empty() ) {
    if ( mask.size() != src1.size() || mask.type() != CV_8UC1 ) {
      CF_ERROR("Bad input mask: must be CV_8UC1 of the same size as innut images");
      return -1.0;
    }
  }

  CV_DISPATCH(src1.depth(), _compute_image_correlation, src1, src2, mask);
  CF_ERROR("Not supported image depth=%d", src1.depth());
  return -1.0;
}


double compute_image_correlation(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const cv::Mat2f & rmap)
{
  INSTRUMENT_REGION("");
  if( current_image.type() != reference_image.type() ) {
    CF_ERROR("src1 and src2 types must be equal");
    return -1.0;
  }

  if( current_image.channels() != 1 ) {
    CF_ERROR("mutlt-channel input images are not supported");
    return -1.0;
  }

  cv::Mat remapped_image, remapped_mask;

  cv::remap(current_image, remapped_image,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  if ( !current_mask.empty() ) {
    cv::remap(current_mask, remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));
  }
  else {
    cv::remap(cv::Mat1b(current_image.size(), (uint8_t) 255), remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(0));
  }

  if ( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  return compute_image_correlation(reference_image, remapped_image, remapped_mask);
}
