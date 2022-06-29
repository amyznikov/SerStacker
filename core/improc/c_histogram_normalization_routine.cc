/*
 * c_histogram_normalization_routine.cc
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#include "c_histogram_normalization_routine.h"
//#include <core/proc/median.h>
#include <core/proc/histogram.h>
#include <core/ssprintf.h>


//static cv::Scalar compute_median(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
//{
//  if ( image.channels() == 1 ) {
//    return median(image, _mask);
//  }
//
//  std::vector<cv::Mat> channels;
//  cv::split(image, channels);
//
//  cv::Scalar m = cv::Scalar::all(0);
//
//  for ( uint i = 0; i < channels.size(); ++i ) {
//    m[i] = median(channels[i], _mask);
//  }
//
//  return m;
//}

static cv::Scalar compute_median(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
{
  cv::Mat1f H;
  double minval = -1, maxval = -1;
  int nbins = -1;

  bool fOK =
      create_histogram(image,
          _mask,
          H,
          &minval,
          &maxval,
          nbins,
          true,
          true);

  if( !fOK ) {
    CF_ERROR("create_histogram() fails");
    return cv::Scalar();
  }

  cv::Scalar s;

  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {

    if ( H.rows < 2 ) {
      s[i] = minval;
    }
    else {
      for( int j = 0, m = H.rows; j < m; ++j ) {
        if( H[j][i] >= 0.5 ) {
          s[i] = minval + j * (maxval - minval) / (m - 1);
          break;
        }
      }
    }
  }

  return s;
}


static cv::Scalar compute_mode(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
{
  cv::Mat1f H;
  double minval = -1, maxval = -1;
  int nbins = -1;

  bool fOK =
      create_histogram(image,
          _mask,
          H,
          &minval,
          &maxval,
          nbins,
          false,
          false);

  if( !fOK ) {
    CF_ERROR("create_histogram() fails");
    return cv::Scalar();
  }

  cv::Scalar s;

  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {

    if ( H.rows < 2 ) {
      s[i] = minval;
    }
    else {

      int jmax = 0;

      float hmax = H[jmax][i];

      for( int j = 1, m = H.rows; j < m; ++j ) {
        if( H[j][i] > hmax ) {
          jmax = j;
          hmax = H[j][i];
        }
      }

      s[i] = minval + jmax * (maxval - minval) / (H.rows - 1);
    }
  }

  return s;
}



template<>
const c_enum_member * members_of<c_histogram_normalization_routine::histogram_normalization_type>()
{
  static constexpr c_enum_member members[] = {
      { c_histogram_normalization_routine::normalize_mean, "mean" },
      { c_histogram_normalization_routine::normalize_median, "median" },
      { c_histogram_normalization_routine::normalize_mode, "mode" },
      { c_histogram_normalization_routine::normalize_mean, nullptr, }  // must  be last
  };

  return members;
}


c_histogram_normalization_routine::c_class_factory c_histogram_normalization_routine::class_factory;



c_histogram_normalization_routine::c_histogram_normalization_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_histogram_normalization_routine::ptr c_histogram_normalization_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_histogram_normalization_routine::set_normalization_type(histogram_normalization_type v)
{
  normalization_type_ = v;
}

c_histogram_normalization_routine::histogram_normalization_type c_histogram_normalization_routine::normalization_type() const
{
  return normalization_type_;
}

void c_histogram_normalization_routine::set_offset(const cv::Scalar & v)
{
  offset_ = v;
}

const cv::Scalar & c_histogram_normalization_routine::offset() const
{
  return offset_;
}

void c_histogram_normalization_routine::set_stretch(const cv::Scalar & v)
{
  stretch_ = v;
}

const cv::Scalar & c_histogram_normalization_routine::stretch() const
{
  return stretch_;
}

bool c_histogram_normalization_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, normalization_type);
  LOAD_PROPERTY(settings, this, offset);
  LOAD_PROPERTY(settings, this, stretch);

  return true;
}

bool c_histogram_normalization_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, normalization_type);
  SAVE_PROPERTY(settings, *this, offset);
  SAVE_PROPERTY(settings, *this, stretch);

  return true;
}

bool c_histogram_normalization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    // v' = (v - mv ) * stretch + offset
    // v' = v* stretch + offset - mv * stretch


    cv::Scalar mv;

    switch (normalization_type_) {
    case normalize_mean:
      mv = cv::mean(image, mask);
      break;

    case normalize_mode:
      mv = compute_mode(image.getMat(), mask);
      break;

    default:
      mv = compute_median(image.getMat(), mask);
      break;
    }

    switch (image.channels()) {
    case 1: {

      const cv::Matx12f m(
          stretch_(0), offset_(0) - mv(0) * stretch_(0));

      cv::transform(image.getMat(), image, m);
      break;
    }
    case 2: {

      const cv::Matx23f m(
          stretch_(0), 0, offset_(0) - mv(0) * stretch_(0),
          0, stretch_(1), offset_(1) - mv(1) * stretch_(1));

      cv::transform(image.getMat(), image, m);
      break;
    }
    case 3: {

      const cv::Matx34f m(
          stretch_(0), 0, 0, offset_(0) - mv(0) * stretch_(0),
          0, stretch_(1), 0, offset_(1) - mv(1) * stretch_(1),
          0, 0, stretch_(2), offset_(2) - mv(2) * stretch_(2));

      cv::transform(image.getMat(), image, m);
      break;
    }
    case 4: {
      typedef cv::Matx<float, 4, 5> Matx45f;

      Matx45f m = Matx45f::zeros();

      m(0, 0) = stretch_(0), m(0, 4) = offset_(0) - mv(0) * stretch_(0);
      m(1, 1) = stretch_(1), m(1, 4) = offset_(1) - mv(1) * stretch_(1);
      m(2, 2) = stretch_(2), m(2, 4) = offset_(2) - mv(2) * stretch_(2);
      m(3, 3) = stretch_(3), m(3, 4) = offset_(3) - mv(3) * stretch_(3);

      cv::transform(image.getMat(), image, m);
      break;
    }
    }
  }

  return true;
}
