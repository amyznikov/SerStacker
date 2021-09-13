/*
 * c_star_field_registration.cc
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#include "c_star_field_registration.h"
#include <core/debug.h>

static inline float hyp(float x, float y)
{
  return sqrt(x * x + y * y);
}

static bool detect_stars(cv::InputArray _src, cv::InputArray aperture_mask, std::vector<cv::KeyPoint> & keypoints)
{
  cv::Mat1f src, filtered, mean, sharpen;
  cv::Mat1b mask;
  cv::Mat1i labels;
  int N;

  src = _src.getMat();

  cv::medianBlur(src, filtered, 3);
  cv::GaussianBlur(filtered, filtered, cv::Size(0, 0), 1);

  cv::GaussianBlur(filtered, mean, cv::Size(0,0), 25);
  cv::scaleAdd(mean, -2, filtered, sharpen);
  cv::max(sharpen, 0, sharpen);
  cv::compare(sharpen, 0, mask, cv::CMP_GT);

  if ( aperture_mask.size() == mask.size() ) {
    cv::bitwise_and(mask, aperture_mask, mask);
  }


  keypoints.clear();

  if ( (N = cv::connectedComponents(mask, labels, 8, labels.type())) < 2 ) {
    CF_FATAL("connectedComponents() can not find connected components, N=%d", N);
    return false;
  }


  keypoints.resize(N - 1);

  for ( int y = 0; y < src.rows; ++y ) {
    for ( int x = 0; x < src.cols; ++x ) {
      const int lb = labels[y][x];
      if ( lb > 0 ) {
        const float I = src[y][x] - mean[y][x];
        if ( I > 0 ) {
          cv::KeyPoint & kp = keypoints[lb - 1];
          kp.pt.x += x * I;
          kp.pt.y += y * I;
          kp.response += I;
        }
      }
    }
  }

  for ( cv::KeyPoint & kp : keypoints) {
    kp.pt.x /= kp.response;
    kp.pt.y /= kp.response;
    kp.size = 25;
  }

  for ( int i = 0, n = keypoints.size(); i < n - 1; ++i ) {
    cv::KeyPoint & cp = keypoints[i];
    if ( cp.response > 0 ) {
      for ( int j = i + 1; j < n; ++j ) {
        cv::KeyPoint & sp = keypoints[j];
        if ( sp.response > 0 ) {
          const float dist = hyp(sp.pt.x - cp.pt.x, sp.pt.y - cp.pt.y);
          if ( dist < 7 ) {
            const float w = 1.f / (cp.response + sp.response);
            cp.pt.x = (cp.pt.x * cp.response + sp.pt.x * sp.response) * w;
            cp.pt.y = (cp.pt.y * cp.response + sp.pt.y * sp.response) * w;
            //cp.size = (cp.size * cp.response + sp.size * sp.response) * w;
            cp.response += sp.response;
            sp.response  = 0;
          }
        }
      }
    }
  }

  std::sort(keypoints.begin(), keypoints.end(),
      [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
        return next.response < prev.response;
      });

  for ( auto ii = keypoints.begin(); ii != keypoints.end(); ++ii ) {
    if ( ii->response <= 0 ) {
      keypoints.erase(ii, keypoints.end());
      break;
    }
  }

  CF_DEBUG("keypoints.size()=%zu", keypoints.size());
  if ( keypoints.size() > 150 ) {
    keypoints.erase(keypoints.begin()+150, keypoints.end());
  }

  return true;
}

c_star_field_registration::c_star_field_registration()
{

}

c_star_field_registration::c_star_field_registration(const c_star_field_registration_options & opts)
  : options_(opts)
{

}

c_star_field_registration::c_star_field_registration(const c_frame_registration_base_options & base_opts, const c_star_field_registration_options & opts)
  : base(base_opts), options_(opts)
{

}

c_star_field_registration::ptr c_star_field_registration::create()
{
  return c_star_field_registration::ptr(new c_star_field_registration());
}

c_star_field_registration::ptr c_star_field_registration::create(const c_star_field_registration_options & opts)
{
  return c_star_field_registration::ptr(new c_star_field_registration(opts));
}

c_star_field_registration::ptr c_star_field_registration::create(const c_frame_registration_base_options & base_opts, const c_star_field_registration_options & opts)
{
  return c_star_field_registration::ptr(new c_star_field_registration(base_opts, opts));
}

c_star_field_registration_options & c_star_field_registration::options()
{
  return options_;
}

const c_star_field_registration_options & c_star_field_registration::options() const
{
  return options_;
}


bool c_star_field_registration::setup_referece_frame(cv::InputArray image, cv::InputArray mask)
{
  return base::setup_referece_frame(image, mask);
}

bool c_star_field_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk) const
{
  bool fOk = extract_channel(src, dst, srcmsk, dstmsk,
      registration_channel(),
      feature_scale(),
      CV_32F);

  if ( !fOk ) {
    CF_ERROR("extract_channel() fails");
  }

  return fOk;
}

bool c_star_field_registration::extract_reference_features(cv::InputArray feature_image, cv::InputArray feature_mask)
{
  if ( !detect_stars(feature_image, feature_mask, reference_keypoints_) ) {
    CF_FATAL("detect_stars(master_frame) fails");
    return 1;
  }

  CF_DEBUG("reference_keypoints_.size()=%zu", reference_keypoints_.size());

  switch (motion_type()) {
    case ECC_MOTION_TRANSLATION:
      if ( reference_keypoints_.size() < 1 ) {
        CF_ERROR("Not enough reference stars detected (%d)",
            reference_keypoints_.size());
        return false;
      }
      break;

    case ECC_MOTION_EUCLIDEAN:
      if ( reference_keypoints_.size() < 3 ) {
        CF_ERROR("Not enough reference stars detected (%d)",
            reference_keypoints_.size());
        return false;
      }
      break;
    case ECC_MOTION_EUCLIDEAN_SCALED : // FIXME: update this code to estimate ECC_MOTION_EUCLIDEAN_SCALED CORRECTLY !!!!
    case ECC_MOTION_AFFINE:
      if ( reference_keypoints_.size() < 3 ) {
        CF_ERROR("Not enough reference stars detected (%d)",
            reference_keypoints_.size());
        return false;
      }
      break;
    case ECC_MOTION_HOMOGRAPHY:
      if ( reference_keypoints_.size() < 4 ) {
        CF_ERROR("Not enough reference stars detected (%d)",
            reference_keypoints_.size());
        return false;
      }
      break;
    case ECC_MOTION_QUADRATIC:
      if ( reference_keypoints_.size() < 6 ) {
        CF_ERROR("Not enough reference stars detected (%d)",
            reference_keypoints_.size());
        return false;
      }
      break;
    default:
      break;
  }

  if ( reference_keypoints_.size() >= 3 ) {
    build_triangles(reference_keypoints_, reference_triangles_, reference_descriptors_);
    reference_index_ = index_triangles(reference_descriptors_);
  }

  CF_DEBUG("reference_keypoints.size=%zu reference_asterisms.size=%zu",
      reference_keypoints_.size(),
      reference_triangles_.size());


  return true;
}


bool c_star_field_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return base::create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}


bool c_star_field_registration::estimate_feature_transform(cv::InputArray current_feature_image,
    cv::InputArray current_feature_mask,
    cv::Mat1f * current_transform)
{
  if ( !detect_stars(current_feature_image, current_feature_mask, current_keypoints_) ) {
    CF_FATAL("detect_stars() fails");
    return false;
  }

  CF_DEBUG("current_keypoints.size()=%zu", current_keypoints_.size());

  if ( current_keypoints_.size() < 3 ) {
    if ( motion_type() != ECC_MOTION_TRANSLATION ) {
      CF_FATAL("too few  keypoints detected: %zu", current_keypoints_.size());
      return false;
    }
    // spcecial case
  }
  else {
    build_triangles(current_keypoints_, current_triangles_, current_descriptors_);

    match_triangles(current_keypoints_, reference_keypoints_, current_triangles_, reference_triangles_,
        current_descriptors_, reference_descriptors_,
        *reference_index_, current_matches_, 1e-4);

    CF_DEBUG("current_keypoints.size=%zu current_asterisms.size=%zu matches.size=%zu",
        current_keypoints_.size(), current_triangles_.size(), current_matches_.size());
    if ( current_matches_.size() < 3 ) {
      return false;
    }

    extract_matches(current_keypoints_, reference_keypoints_,  current_matches_,
        &current_positions_, &reference_positions_);


    if ( motion_type() < ECC_MOTION_AFFINE ) {
      *current_transform = cv::estimateAffinePartial2D(reference_positions_, current_positions_, cv::noArray(),
          cv::LMEDS, 3, 2000, 0.99, 10);
    }
    else {
      *current_transform = estimateAffine2D(reference_positions_, current_positions_, cv::noArray(),
          cv::LMEDS, 3, 2000, 0.99, 10);
    }

    if ( current_transform->empty() ) {
      CF_FATAL("estimateAffinePartial2D() fails");
      return false;
    }

    if ( motion_type() > ECC_MOTION_AFFINE ) {
      *current_transform =
          expandAffineTransform(*current_transform,
              motion_type());
    }
  }

  CF_DEBUG("feature_scale_=%g", feature_scale());
  if ( feature_scale() != 1. ) {
    scaleTransform(motion_type(),
        *current_transform,
        1. / feature_scale());
  }


  return true;
}
