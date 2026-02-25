/*
 * c_star_extractor_routine.cc
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#include "c_star_extractor_routine.h"
#include <core/proc/estimate_noise.h>
#include <core/proc/fast_gaussian_blur.h>

template<>
const c_enum_member * members_of<c_star_extractor_routine::DisplayType>()
{
  static const c_enum_member members[] = {
    {c_star_extractor_routine::DisplayRichKeypoints, "RichKeypoints", "Display Rich Keypoints"},
    {c_star_extractor_routine::DisplaySourceImage, "SourceImage", "Display Source Image"},
    {c_star_extractor_routine::DisplayFilteredImage, "FilteredImage", "Display Filtered Image"},
    {c_star_extractor_routine::DisplayRichKeypoints}
  };

  return members;
}

static void compute_dog(cv::InputArray src, cv::OutputArray dst, double sigma1, double sigma2, int ddepth)
{
  cv::Mat gb1, gb2;

  if( sigma1 <= 0 ) {
    gb1 = src.getMat();
  }
  else {
    fast_gaussian_blur(src.getMat(), cv::noArray(), gb1, sigma1 ); // , cv::BORDER_DEFAULT, ddepth
  }

  if( sigma2 <= 0 ) {
    gb2 = src.getMat();
  }
  else {
    fast_gaussian_blur(src.getMat(), cv::noArray(), gb2, sigma2);
  }

  cv::subtract(gb1, gb2, dst, cv::noArray(), ddepth);
}

void c_star_extractor_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display_type), "");
  ctlbind(ctls, "median_filter_size", ctx(&this_class::_median_filter_size), "");
  ctlbind(ctls, "sigma1", ctx(&this_class::_sigma1), "");
  ctlbind(ctls, "sigma2", ctx(&this_class::_sigma2), "");
  ctlbind(ctls, "noise_sigma", ctx(&this_class::_noise_sigma), "");
  ctlbind(ctls, "noise_scale", ctx(&this_class::_noise_scale), "");
}

bool c_star_extractor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _median_filter_size);
    SERIALIZE_OPTION(settings, save, *this, _sigma1);
    SERIALIZE_OPTION(settings, save, *this, _sigma2);
    SERIALIZE_OPTION(settings, save, *this, _noise_sigma);
    SERIALIZE_OPTION(settings, save, *this, _noise_scale);
    return true;
  }
  return false;
}

bool c_star_extractor_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1f noisemap;
  cv::Mat1f dog;
  cv::Mat1b cc;
  cv::Mat1i labels;

  create_noise_map(image, noisemap);
  fast_gaussian_blur(cv::abs(noisemap), cv::noArray(), noisemap, _noise_sigma, cv::BORDER_DEFAULT, CV_32F);

  if( _median_filter_size > 1 ) {
    cv::medianBlur(image.getMat(), image, _median_filter_size);
  }

  compute_dog(image.getMat(), dog, _sigma1, _sigma2, CV_32F);
  cv::scaleAdd(noisemap, -_noise_scale, dog, dog);
  cv::compare(dog, cv::Scalar::all(0), cc, cv::CMP_GT);
  dog.copyTo(image);
  cc.copyTo(mask);

  const int N = cv::connectedComponents(mask, labels, 8, labels.type());
  if ( N > 1 ) {

    struct Blob {
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

    double mamb = 0;
    double mabr = 0;
    double mtheta = 0;
    double stheta = 0;
    double w = 0;

    for( int lb = 1; lb < N; ++lb ) {

      Blob & b = blobs[lb];
      if( b.n > 5 ) {
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

          const double ww = b.a;
          mamb += (b.a - b.b) * ww;
          mabr += b.b / b.a * ww;
          mtheta += b.theta * ww;
          stheta += b.theta * b.theta * ww;
          w += ww;

          const double R = 5 * b.a;

          cv::line(mask, cv::Point(cvRound(b.x - R), cvRound(b.y)),
              cv::Point(cvRound(b.x + R), cvRound(b.y)),
              255, 1,
              cv::LINE_4);

          cv::line(mask, cv::Point(cvRound(b.x), cvRound(b.y - R)),
              cv::Point(cvRound(b.x), cvRound(b.y + R)),
              255, 1,
              cv::LINE_4);

          const cv::RotatedRect box(cv::Point2f(b.x, b.y),
              cv::Size2f(5 * b.a, 10 * b.b),
              b.theta * 180 / CV_PI);

          cv::ellipse(mask, box, cv::Scalar::all(255), 1, cv::LINE_8);
        }

      }
    }

    if (w > 0 ) {
      mamb = mamb / w;
      mabr = mabr / w;
      mtheta = mtheta / w;
      stheta = sqrt(stheta/w - mtheta * mtheta);
    }

    CF_DEBUG("mamb = %g  mabr = %g  mtheta = %g  stheta = %g score=%g",
        mamb, mabr, mtheta, stheta, stheta * mabr );
  }


  return true;
}

