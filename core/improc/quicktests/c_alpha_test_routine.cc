/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>
#include <core/proc/divide.h>
#include <core/proc/multiply.h>


//template<>
//const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
//{
//  static const c_enum_member members[] = {
//      { c_alpha_test_routine::DISPLAY_SRC1, "SRC1", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2, "SRC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC1, "SPEC1", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC2, "SPEC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC3, "SPEC3", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13_DIFF, "SPEC13_DIFF", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13, "CROSS_SPEC13", "" },
//      { c_alpha_test_routine::DISPLAY_CORR_MAP13, "CORR_MAP13", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2}
//  };
//  return members;
//}

/////////////////////////////////////
namespace {

struct HoughMaps
{
  cv::Mat grid; // CV_32FC1 (512x512): signal with background removed
  cv::Mat accumulator; // CV_32FC1 (90 x numRho): voting sinogram
  float noiseFloor; // Noise cutoff threshold (mean + 3*stddev)
  float starClamp; // Brightness ceiling for suppressing stars (mean + 8*stddev)
};

template<typename T>
static bool _generateHoughMaps(cv::InputArray _src, HoughMaps & res)
{
  constexpr int BLOCK = 8;
  const int gridW = _src.cols() / BLOCK;
  const int gridH = _src.rows() / BLOCK;

  const cv::Mat raw4K = _src.getMat();

  res.grid = cv::Mat::zeros(gridH, gridW, CV_32FC1);

  std::vector<T> rowMax(gridW);
  std::vector<double> rowSum(gridW);

  double totalSum = 0.0;
  double totalSqSum = 0.0;

  for( int gy = 0; gy < gridH; ++gy ) {
    std::fill(rowMax.begin(), rowMax.end(), T(0));
    std::fill(rowSum.begin(), rowSum.end(), 0.0);

    for( int dy = 0; dy < BLOCK; ++dy ) {
      const T * ptr = raw4K.ptr<T>(gy * BLOCK + dy);
      for( int gx = 0; gx < gridW; ++gx ) {
        const T * blk = ptr + gx * BLOCK;
        T m = rowMax[gx];
        double s = 0.0;

        for( int dx = 0; dx < BLOCK; ++dx ) {
          T v = blk[dx];
          if( v > m )
            m = v;
          s += static_cast<double>(v);
        }

        rowMax[gx] = m;
        rowSum[gx] += s;
      }
    }

    float * gridRow = res.grid.ptr<float>(gy);
    for( int gx = 0; gx < gridW; ++gx ) {
      double maxD = static_cast<double>(rowMax[gx]);
      double bg = (rowSum[gx] - maxD) / 63.0;
      float sig = static_cast<float>(std::max(0.0, maxD - bg));

      gridRow[gx] = sig;
      totalSum += sig;
      totalSqSum += static_cast<double>(sig) * sig;
    }
  }


  const int gridCells = gridW * gridH;
  double mean = totalSum / gridCells;
  double variance = (totalSqSum / gridCells) - (mean * mean);
  double stddev = std::sqrt(std::max(0.0, variance));

  res.noiseFloor = static_cast<float>(mean + 3.0 * stddev);
  res.starClamp = static_cast<float>(mean + 8.0 * stddev);

  constexpr int NUM_THETA = 90;
  constexpr int RHO_SHIFT = 1;
  const int maxRho = (static_cast<int>(std::hypot(gridW, gridH)) >> RHO_SHIFT) + 2;
  const int numRho = maxRho * 2;

  res.accumulator = cv::Mat::zeros(NUM_THETA, numRho, CV_32FC1);

  if( res.starClamp <= res.noiseFloor ) {
    return false;
  }

  constexpr int FIX_SHIFT = 16;
  int32_t cosStep[NUM_THETA];
  int32_t sinStep[NUM_THETA];
  for( int t = 0; t < NUM_THETA; ++t ) {
    double rad = (t * 2.0) * CV_PI / 180.0;
    cosStep[t] = static_cast<int32_t>((std::cos(rad) / (1 << RHO_SHIFT)) * (1 << FIX_SHIFT));
    sinStep[t] = static_cast<int32_t>((std::sin(rad) / (1 << RHO_SHIFT)) * (1 << FIX_SHIFT));
  }

  for( int gy = 0; gy < gridH; ++gy ) {
    const float * gridRow = res.grid.ptr<float>(gy);
    for( int gx = 0; gx < gridW; ++gx ) {
      float w = gridRow[gx];
      if( w <= res.noiseFloor )
        continue;
      if( w > res.starClamp )
        w = res.starClamp;

      for( int t = 0; t < NUM_THETA; ++t ) {
        int rIdx = ((gx * cosStep[t] + gy * sinStep[t]) >> FIX_SHIFT) + maxRho;
        res.accumulator.ptr<float>(t)[rIdx] += w;
      }
    }
  }

  return true;
}

static bool generateHoughMaps(cv::InputArray _src, HoughMaps & res)
{
  CV_DISPATCH(_src.depth(), _generateHoughMaps, _src, res);
  return false;
}

}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
//    SERIALIZE_OPTION(settings, save, *this, sigma);
//    SERIALIZE_OPTION(settings, save, *this, eps);
//    SERIALIZE_OPTION(settings, save, *this, kradius);
//    SERIALIZE_OPTION(settings, save, *this, dscale);
//    SERIALIZE_OPTION(settings, save, *this, uscale);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
//  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, sigma), "");
//  ctlbind(ctls, "eps", CTL_CONTEXT(ctx, eps), "");
//  ctlbind(ctls, "kradius", CTL_CONTEXT(ctx, kradius), "");
//  ctlbind(ctls, "dscale", CTL_CONTEXT(ctx, dscale), "");
//  ctlbind(ctls, "uscale", CTL_CONTEXT(ctx, uscale), "");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  HoughMaps hmps;
  generateHoughMaps(image, hmps);

  image.assign(hmps.accumulator);

  return true;
}
