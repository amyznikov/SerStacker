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

template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC, "SRC", "" },
//      { c_alpha_test_routine::DISPLAY_BGMAP, "BGMAP", "" },
//      { c_alpha_test_routine::DISPLAY_STDMAP, "STDMAP", "" },
      { c_alpha_test_routine::DISPLAY_DOG, "DOG", "" },
      { c_alpha_test_routine::DISPLAY_MASK, "MASK", "" },
      { c_alpha_test_routine::DISPLAY_DOG}
  };
  return members;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "lvls", CTL_CONTEXT(ctx, lvls), "");
  ctlbind(ctls, "se_radius", CTL_CONTEXT(ctx, se_radius), "");
  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, sigma), "");
  ctlbind(ctls, "weight_decay", CTL_CONTEXT(ctx, weight_decay), "");
  ctlbind(ctls, "kmad", CTL_CONTEXT(ctx, kmad), "");

}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, lvls);
    SERIALIZE_OPTION(settings, save, *this, se_radius);
    SERIALIZE_OPTION(settings, save, *this, sigma);
    SERIALIZE_OPTION(settings, save, *this, weight_decay);
    SERIALIZE_OPTION(settings, save, *this, kmad);
    return true;
  }
  return false;
}

static void build_dog(cv::InputArray _src, cv::OutputArray _dst, int maxlevels, double sigma, double weight_decay = 0.5)
{
  std::vector<cv::Mat> dogs;
  cv::Mat current_image, lpass;

  const int ksize = std::max(5, 2 * int(sigma * 3) + 1);
  const cv::Mat G = cv::getGaussianKernel(ksize, sigma, CV_32F);

  dogs.reserve(maxlevels);

  cv::sepFilter2D(_src, current_image, CV_32F, G, G, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

  for( int level = 0; level < maxlevels; ++level ) {

    dogs.emplace_back();
    cv::sepFilter2D(current_image, lpass, CV_32F, G, G, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
    cv::subtract(current_image, lpass, dogs.back());

    if( level < maxlevels - 1 ) {
      downstrike_uneven(lpass, current_image);
    }
  }

  current_image = dogs.back();
  for( int i = maxlevels - 2; i >= 0; --i ) {
    const cv::Mat & current_layer = dogs[i];
    cv::pyrUp(current_image, lpass, current_layer.size());
    if ( weight_decay == 1 ) {
      cv::add(lpass, current_layer, current_image);
    }
    else {
      cv::scaleAdd(lpass, weight_decay, current_layer, current_image);
    }
  }

  _dst.move(current_image);
}

static void computeMAD(cv::InputArray _src, cv::Scalar &outputMean, cv::Scalar &outputMAD,
    cv::InputArray _mask = cv::noArray())
{
  cv::Mat tmp;
  cv::absdiff(_src, outputMean = cv::mean(_src, _mask), tmp);
  outputMAD = cv::mean(tmp, _mask);
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _display == DISPLAY_SRC ) {
    return true;
  }

  cv::Scalar dogMean, dogMAD;
  cv::Mat1f dog;
  cv::Mat1b dogMask;
  cv::Mat1i labels;

  cv::Mat src;
  if (image.channels() == 1 ) {
    src = image.getMat();
  }
  else {
    cv::cvtColor(image, src, cv::COLOR_BGR2GRAY);
  }

  CF_DEBUG("Call build_dog");
  build_dog(src, dog, lvls, sigma, weight_decay);
  CF_DEBUG("Leave build_dog");

  dog.copyTo(image);
  if( _display == DISPLAY_DOG ) {
    return true;
  }


  CF_DEBUG("Call computeMAD");
  computeMAD(dog, dogMean, dogMAD, mask);
  CF_DEBUG("computeMAD: mean={%g %g %g} MAD={%g %g %g}",
      dogMean[0], dogMean[1], dogMean[2],
      dogMAD[0], dogMAD[1], dogMAD[2]);

  cv::compare(dog, dogMean + kmad * dogMAD, dogMask, cv::CMP_GT );

  if ( se_radius > 0 ) {
    CF_DEBUG("Call dilate");
    const cv::Mat SE = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * se_radius + 1, 2 * se_radius + 1));
    cv::dilate(dogMask, dogMask, SE);
    CF_DEBUG("Leave dilate");
 }

  dogMask.copyTo(mask);

  CF_DEBUG("Call connectedComponents");
  const int N = cv::connectedComponents(dogMask, labels, 8, labels.type());
  CF_DEBUG("connectedComponents: N=%d", N);
  if ( N > 1 ) {

    struct Blob
    {
      double x = 0;
      double y = 0;
      double x2 = 0;
      double y2 = 0;
      double xy = 0;
      double I = 0;
      double n = 0;
      double a;
      double b;
      double theta;
    };

    std::vector<Blob> blobs(N);

    int ngood = 0;

    CF_DEBUG("Beg moments");

    for( int y = 0; y < labels.rows; ++y ) {

      const int32_t * lbp = labels[y];
      const float * dogp = dog[y];

      for( int x = 0; x < labels.cols; ++x ) {
        const int32_t lb = lbp[x];
        if( lb ) {

          Blob & b = blobs[lb];
          const float I = dogp[x];

          b.x += I * x;
          b.y += I * y;
          b.x2 += I * x * x;
          b.y2 += I * y * y;
          b.xy += I * x * y;
          b.I += I;
          b.n += 1;
        }
      }
    }

    for( int lb = 1; lb < N; ++lb ) {
      Blob & b = blobs[lb];
      if( b.n > 3 ) {
        b.x /= b.I;
        b.y /= b.I;
        b.x2 = b.x2 / b.I - b.x * b.x;
        b.y2 = b.y2 / b.I - b.y * b.y;
        b.xy = b.xy / b.I - b.x * b.y;
        b.theta = (b.x2 == b.y2) ? 0.0 : 0.5 * std::atan2(2 * b.xy, b.x2 - b.y2);

        const double D = b.theta == 0 ? 0.0 : b.xy / std::sin(2 * b.theta);
        b.a = std::sqrt(0.5 * (b.x2 + b.y2 + D));
        b.b = std::sqrt(0.5 * (b.x2 + b.y2 - D));

        if ( b.b > 0 ) {
          ++ngood;
        }
      }
    }

    CF_DEBUG("moments: ngood=%d", ngood);
  }


  return true;
}


//bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  if( _display == DISPLAY_SRC ) {
//    return true;
//  }
//
//  const cv::Mat SE = cv::getStructuringElement(cv::MORPH_ELLIPSE,
//      cv::Size(2 * se_radius + 1, 2 * se_radius + 1));
//
//
//  cv::Mat bgmap, stdmap, tophat, snr;
//
//
//  CF_DEBUG("Call create_bgmap");
//  create_bgmap(image, SE, bgmap, stdmap, lvls);
//  CF_DEBUG("Leave create_bgmap");
//
//  if( _display == DISPLAY_BGMAP ) {
//    image.move(bgmap);
//    return true;
//  }
//
//  if( _display == DISPLAY_STDMAP ) {
//    image.move(stdmap);
//    return true;
//  }
//
//  cv::subtract(image, bgmap, tophat);
//  if( _display == DISPLAY_TOPHAT ) {
//    image.move(tophat);
//    return true;
//  }
//
//  // DISPLAY_SNR
//  cv::divide(tophat, stdmap, snr);
//
//  cv::Mat1d H;
//  double minv= -1, maxv = -1;
//
//  cv::Scalar snrmean, snrstdev, snrmedian, snrmode, snrmad;
//  cv::meanStdDev(snr, snrmean, snrstdev);
//
//  snrmad = cv::mean(cv::abs(snr - snrmean));
//
//  cv::Mat tmp;
//  cv::absdiff(snr, snrmean, tmp);
//  snrmad = cv::mean(tmp);
//
//  createHistogram(snr, mask, &minv, &maxv, 0, H, false, true);
//  snrmode = computeHistogramMode(H, minv, maxv);
//
//  makeCumulativeHistogram(H, H, true);
//  snrmedian = computeHistogramMedian(H, minv, maxv);
//
//  CF_DEBUG("\nSNR: minv=%g maxv=%g\n"
//      "mean={%g %g %g} stdev={%g %g %g} MAD={%g %g %g} median={%g %g %g} mode={%g %g %g}",
//      minv, maxv,
//      snrmean[0], snrmean[1], snrmean[2],
//      snrstdev[0], snrstdev[1], snrstdev[2],
//      snrmad[0], snrmad[1], snrmad[2],
//      snrmedian[0], snrmedian[1], snrmedian[2],
//      snrmode[0], snrmode[1], snrmode[2]
//    );
//
//  image.move(snr);
//
//  return true;
//}
