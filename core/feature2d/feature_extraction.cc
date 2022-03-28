/*
 * feature_extraction.cc
 *
 *  Created on: Oct 23, 2021
 *      Author: amyznikov
 *
 *
 * Uniform wrappers for standard OpenCV sparse feature detectors
 * and descriptor extractors.
 */

#include "feature_extraction.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member *members_of<FEATURE2D_TYPE>()
{
  static const c_enum_member members[] = {

      { FEATURE2D_ORB, "ORB", "" },
      { FEATURE2D_BRISK, "BRISK", "" },
      { FEATURE2D_MSER, "MSER", "" },
      { FEATURE2D_FAST, "FAST", "" },
      { FEATURE2D_AGAST, "AGAST", "" },
      { FEATURE2D_GFTT, "GFTT", "" },
      { FEATURE2D_BLOB, "BLOB", "" },
      { FEATURE2D_KAZE, "KAZE", "" },
      { FEATURE2D_AKAZE, "AKAZE", "" },
#if  HAVE_FEATURE2D_BRIEF
      { FEATURE2D_BRIEF, "BRIEF", "" },
#endif
#if HAVE_FEATURE2D_SIFT
      { FEATURE2D_SIFT, "SIFT", "" },
#endif
#if HAVE_FEATURE2D_SURF
      { FEATURE2D_SURF, "SURF", "" },
#endif
#if HAVE_FEATURE2D_FREAK
      { FEATURE2D_FREAK, "FREAK", "" },
#endif
#if HAVE_FEATURE2D_STAR
      { FEATURE2D_STAR, "STAR", "" },
#endif
#if HAVE_FEATURE2D_LUCID
      { FEATURE2D_LUCID, "LUCID", "" },
#endif
#if HAVE_FEATURE2D_LATCH
      { FEATURE2D_LATCH, "LATCH", "" },
#endif
#if HAVE_FEATURE2D_DAISY
      { FEATURE2D_DAISY, "DAISY", "" },
#endif
#if HAVE_FEATURE2D_MSD
      { FEATURE2D_MSD, "MSD", "" },
#endif
#if HAVE_FEATURE2D_VGG
      { FEATURE2D_VGG, "VGG", "" },
#endif
#if HAVE_FEATURE2D_BOOST
      { FEATURE2D_BOOST, "BOOST", "" },
#endif
#if HAVE_FEATURE2D_HL
      { FEATURE2D_HL, "HL", "" },
#endif
      { FEATURE2D_UNKNOWN, nullptr, "" },
  };

  return members;
}

template<>
const c_enum_member * members_of<cv::ORB::ScoreType>()
{
  static const c_enum_member members[] = {
      { cv::ORB::HARRIS_SCORE, "HARRIS_SCORE", "" },
      { cv::ORB::FAST_SCORE, "FAST_SCORE", "" },
      { cv::ORB::HARRIS_SCORE, nullptr, "" },
  };

  return members;
}

template<>
const c_enum_member * members_of<cv::KAZE::DiffusivityType>()
{
  static const c_enum_member members[] = {
      { cv::KAZE::DIFF_PM_G1, "DIFF_PM_G1", "" },
      { cv::KAZE::DIFF_PM_G2, "DIFF_PM_G2", "" },
      { cv::KAZE::DIFF_WEICKERT, "DIFF_WEICKERT", "" },
      { cv::KAZE::DIFF_CHARBONNIER, "DIFF_CHARBONNIER", "" },
      { cv::KAZE::DIFF_PM_G2, nullptr, "" },
  };

  return members;
}

template<>
const c_enum_member * members_of<cv::AKAZE::DescriptorType>()
{
  static const c_enum_member members[] = {
      { cv::AKAZE::DESCRIPTOR_KAZE_UPRIGHT, "DESCRIPTOR_KAZE_UPRIGHT", "Upright descriptors, not invariant to rotation" },
      { cv::AKAZE::DESCRIPTOR_KAZE, "DESCRIPTOR_KAZE", "" },
      { cv::AKAZE::DESCRIPTOR_MLDB_UPRIGHT, "DESCRIPTOR_MLDB_UPRIGHT", "Upright descriptors, not invariant to rotation" },
      { cv::AKAZE::DESCRIPTOR_MLDB, "DESCRIPTOR_MLDB",  "" },
      { cv::AKAZE::DESCRIPTOR_MLDB, nullptr, "" },
  };

  return members;
}

template<>
const c_enum_member * members_of<cv::FastFeatureDetector::DetectorType>()
{
  static const c_enum_member members[] = {
      { cv::FastFeatureDetector::TYPE_5_8, "TYPE_5_8" "" },
      { cv::FastFeatureDetector::TYPE_7_12, "TYPE_7_12", "" },
      { cv::FastFeatureDetector::TYPE_9_16, "TYPE_9_16", "" },
      { cv::FastFeatureDetector::TYPE_9_16, nullptr, "" },
  };

  return members;
}

template<>
const c_enum_member * members_of<cv::AgastFeatureDetector::DetectorType>()
{
  static const c_enum_member members[] = {
      { cv::AgastFeatureDetector::AGAST_5_8, "AGAST_5_8", "" },
      { cv::AgastFeatureDetector::AGAST_7_12d, "AGAST_7_12d", "" },
      { cv::AgastFeatureDetector::AGAST_7_12s, "AGAST_7_12s", "" },
      { cv::AgastFeatureDetector::OAST_9_16, "OAST_9_16", "" },
      { cv::AgastFeatureDetector::OAST_9_16, nullptr, "" },
  };

  return members;
}

#if HAVE_FEATURE2D_DAISY
template<>
const c_enum_member * members_of<cv::xfeatures2d::DAISY::NormalizationType>()
{
  static const c_enum_member members[] = {
      { cv::xfeatures2d::DAISY::NRM_NONE, "NRM_NONE", "" },
      { cv::xfeatures2d::DAISY::NRM_PARTIAL, "NRM_PARTIAL", "" },
      { cv::xfeatures2d::DAISY::NRM_FULL, "NRM_FULL", "" },
      { cv::xfeatures2d::DAISY::NRM_SIFT, "NRM_SIFT", "" },
      { cv::xfeatures2d::DAISY::NRM_NONE, nullptr, "" },
  };

  return members;
}
#endif

#if HAVE_FEATURE2D_BOOST
template<>
const c_enum_member * members_of<BoostDesc_Type>()
{
  static const c_enum_member members[] = {
      { cv::xfeatures2d::BoostDesc::BGM, "BGM", "" },
      { cv::xfeatures2d::BoostDesc::BGM_HARD, "BGM_HARD", "" },
      { cv::xfeatures2d::BoostDesc::BGM_BILINEAR, "BGM_BILINEAR", "" },
      { cv::xfeatures2d::BoostDesc::LBGM, "LBGM", "" },
      { cv::xfeatures2d::BoostDesc::BINBOOST_64, "BINBOOST_64", "" },
      { cv::xfeatures2d::BoostDesc::BINBOOST_128, "BINBOOST_128", "" },
      { cv::xfeatures2d::BoostDesc::BINBOOST_256, "BINBOOST_256", "" },
      { cv::xfeatures2d::BoostDesc::BINBOOST_256, nullptr, "" },
  };

  return members;
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_sparse_feature_extractor::c_sparse_feature_extractor(const c_feature2d::ptr & detector,
    const c_feature2d::ptr & descriptor) :
    detector_(detector),
        descriptor_(descriptor)
{
}


bool c_sparse_feature_extractor::detect(cv::InputArray image,
    CV_OUT std::vector<cv::KeyPoint> & keypoints,
    cv::InputArray mask) const
{
  INSTRUMENT_REGION("");
  detector_->detect(image, keypoints, mask);
  return true;
}

bool c_sparse_feature_extractor::compute(cv::InputArray image,
    CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
    cv::OutputArray descriptors)
{
  INSTRUMENT_REGION("");

  if ( descriptor_ ) {
    descriptor_->compute(image, keypoints, descriptors);
    return true;
  }

  if ( can_compute_decriptors(detector_->type()) ) {
    detector_->compute(image, keypoints, descriptors);
    return true;
  }

  CF_ERROR("sparse featute descriptor extractor was not specified,"
      "but provided featute detector %s can not compute descriptors",
      typeid(*detector_.get()).name());

  return false;
}

bool c_sparse_feature_extractor::detectAndCompute(cv::InputArray image, cv::InputArray mask,
    CV_OUT std::vector<cv::KeyPoint> & keypoints,
    cv::OutputArray descriptors,
    bool useProvidedKeypoints)
{
  INSTRUMENT_REGION("");

  if ( !descriptor_ && !can_compute_decriptors(detector_->type()) ) {
    CF_ERROR("specified keypoints detector %s can not compute feature descriptors",
        toString(detector_->type()));
    return false;
  }

  //
  // Detect sparse 2D features (keypoints) and compute descriptots
  //

  keypoints.clear();
  keypoints.reserve(std::max(max_keypoints_to_extract_, 1000));

  // Prefer detectAndCompute() if possible because it can be faster for some detectors
  if ( !descriptor_ || descriptor_.get() == detector_.get() ) {

    detector_->detectAndCompute(
            image,
            mask,
            keypoints,
            descriptors,
            useProvidedKeypoints);

    if ( keypoints.size() < 1 ) {
      CF_ERROR("No keypoints detected on image");
      return false;
    }

    // TODO: keep no more than max_keypoints_to_extract if requested ?
    // if ( max_keypoints_to_extract > 0 && (int)output_keypoints.size() > max_keypoints_to_extract ) {
    // }

  }
  else {

    detector_->detect(image,
        keypoints,
        mask);

    if ( keypoints.size() < 1 ) {
      CF_ERROR("No keypoints detected on image");
      return false;
    }


    if ( max_keypoints_to_extract_ > 0 && (int)keypoints.size() > max_keypoints_to_extract_ ) {

      std::sort(keypoints.begin(), keypoints.end(),
          [](const cv::KeyPoint & prev, const cv::KeyPoint & next )-> bool {
            return prev.response > next.response;
          });

      keypoints.erase(keypoints.begin() + max_keypoints_to_extract_, keypoints.end());
    }

    descriptor_->compute(image,
        keypoints,
        descriptors);
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define PARSE_PARAM(p)  \
  if ( strcasecmp(name, #p) == 0 ) { \
    if ( *value && fromString(value, &opts.p) != 1 ) { \
      CF_ERROR("Syntax error: can not parse value '%s' for parameter %s:: %s", value, this_objtype, name); \
      return false; \
    } \
    continue; \
  }

static bool parse_params(c_feature2d_orb::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p: params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(nfeatures);
    PARSE_PARAM(scaleFactor);
    PARSE_PARAM(nlevels);
    PARSE_PARAM(edgeThreshold);
    PARSE_PARAM(firstLevel);
    PARSE_PARAM(WTA_K);
    PARSE_PARAM(scoreType);
    PARSE_PARAM(patchSize);
    PARSE_PARAM(fastThreshold);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);
    return false;
  }


  return true;
}

static bool parse_params(c_feature2d_brisk::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(thresh);
    PARSE_PARAM(octaves);
    PARSE_PARAM(patternScale);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

static bool parse_params(c_feature2d_kaze::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);


  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(extended);
    PARSE_PARAM(upright);
    PARSE_PARAM(threshold);
    PARSE_PARAM(nOctaves);
    PARSE_PARAM(nOctaveLayers);
    PARSE_PARAM(diffusivity);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

static bool parse_params(c_feature2d_akaze::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);


  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(descriptor_type );
    PARSE_PARAM(descriptor_size);
    PARSE_PARAM(descriptor_channels);
    PARSE_PARAM(threshold);
    PARSE_PARAM(nOctaves);
    PARSE_PARAM(nOctaveLayers);
    PARSE_PARAM(diffusivity);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

#if HAVE_FEATURE2D_SIFT
static bool parse_params(c_feature2d_sift::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(nfeatures);
    PARSE_PARAM(nOctaveLayers);
    PARSE_PARAM(contrastThreshold);
    PARSE_PARAM(edgeThreshold);
    PARSE_PARAM(sigma);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_SURF
static bool parse_params(c_feature2d_surf::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(hessianThreshold);
    PARSE_PARAM(nOctaves);
    PARSE_PARAM(nOctaveLayers);
    PARSE_PARAM(extended);
    PARSE_PARAM(upright);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

static bool parse_params(c_feature2d_mser::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(delta);
    PARSE_PARAM(min_area);
    PARSE_PARAM(max_area);
    PARSE_PARAM(max_variation);
    PARSE_PARAM(min_diversity );
    PARSE_PARAM(max_evolution);
    PARSE_PARAM(area_threshold);
    PARSE_PARAM(min_margin);
    PARSE_PARAM(edge_blur_size);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}


static bool parse_params(c_feature2d_fast::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(threshold);
    PARSE_PARAM(nonmaxSuppression);
    PARSE_PARAM(type);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

static bool parse_params(c_feature2d_agast::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(threshold);
    PARSE_PARAM(nonmaxSuppression);
    PARSE_PARAM(type);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);


    return false;
  }

  return true;
}

static bool parse_params(c_feature2d_gftt::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(maxCorners);
    PARSE_PARAM(qualityLevel);
    PARSE_PARAM(minDistance);
    PARSE_PARAM(blockSize);
    PARSE_PARAM(gradiantSize);
    PARSE_PARAM(useHarrisDetector);
    PARSE_PARAM(k);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

static bool parse_params(c_feature2d_blob::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(thresholdStep);
    PARSE_PARAM(minThreshold);
    PARSE_PARAM(maxThreshold);
    PARSE_PARAM(minRepeatability);
    PARSE_PARAM(minDistBetweenBlobs);

    PARSE_PARAM(filterByColor);
    PARSE_PARAM(blobColor);

    PARSE_PARAM(filterByArea);
    PARSE_PARAM(minArea);
    PARSE_PARAM(maxArea);

    PARSE_PARAM(filterByCircularity);
    PARSE_PARAM(minCircularity);
    PARSE_PARAM(maxCircularity);

    PARSE_PARAM(filterByInertia);
    PARSE_PARAM(minInertiaRatio);
    PARSE_PARAM(maxInertiaRatio);

    PARSE_PARAM(filterByConvexity);
    PARSE_PARAM(minConvexity);
    PARSE_PARAM(maxConvexity);


    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}

#if HAVE_FEATURE2D_STAR
static bool parse_params(c_feature2d_star::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(maxSize);
    PARSE_PARAM(responseThreshold);
    PARSE_PARAM(lineThresholdProjected);
    PARSE_PARAM(lineThresholdBinarized);
    PARSE_PARAM(suppressNonmaxSize);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }
  return true;
}
#endif

#if HAVE_FEATURE2D_MSD
static bool parse_params(c_feature2d_msd::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(m_patch_radius);
    PARSE_PARAM(m_search_area_radius);
    PARSE_PARAM(m_nms_radius);
    PARSE_PARAM(m_nms_scale_radius);
    PARSE_PARAM(m_th_saliency);
    PARSE_PARAM(m_kNN);
    PARSE_PARAM(m_scale_factor);
    PARSE_PARAM(m_n_scales);
    PARSE_PARAM(m_compute_orientation);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_HL
static bool parse_params(c_feature2d_hl::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(numOctaves);
    PARSE_PARAM(corn_thresh);
    PARSE_PARAM(DOG_thresh);
    PARSE_PARAM(maxCorners);
    PARSE_PARAM(num_layers);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_FREAK
static bool parse_params( c_feature2d_freak::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(orientationNormalized);
    PARSE_PARAM(scaleNormalized);
    PARSE_PARAM(patternScale);
    PARSE_PARAM(nOctaves);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_BRIEF
static bool parse_params(c_feature2d_brief::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(bytes);
    PARSE_PARAM(use_orientation);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_LUCID
static bool parse_params(c_feature2d_lucid::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(lucid_kernel);
    PARSE_PARAM(blur_kernel);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_LATCH
static bool parse_params(c_feature2d_latch::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(bytes);
    PARSE_PARAM(rotationInvariance);
    PARSE_PARAM(half_ssd_size);
    PARSE_PARAM(sigma);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;

  }

  return true;
}
#endif

#if HAVE_FEATURE2D_DAISY
static bool parse_params(c_feature2d_daisy::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(radius);
    PARSE_PARAM(q_radius);
    PARSE_PARAM(q_theta);
    PARSE_PARAM(q_hist);
    PARSE_PARAM(norm);
    PARSE_PARAM(interpolation);
    PARSE_PARAM(use_orientation);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_VGG
static bool parse_params(c_feature2d_vgg::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(desc);
    PARSE_PARAM(isigma);
    PARSE_PARAM(img_normalize);
    PARSE_PARAM(use_scale_orientation);
    PARSE_PARAM(scale_factor);
    PARSE_PARAM(dsc_normalize);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }

  return true;
}
#endif

#if HAVE_FEATURE2D_BOOST
static bool parse_params(c_feature2d_boost::options & opts,
    const std::vector<std::pair<std::string, std::string>> & params)
{
  static const char * this_objtype =
      toString(opts.type);

  for ( const std::pair<std::string, std::string> & p : params ) {

    const char * name = p.first.c_str();
    const char * value = p.second.c_str();

    PARSE_PARAM(desc);
    PARSE_PARAM(use_scale_orientation);
    PARSE_PARAM(scale_factor);

    CF_ERROR("Struct %s has no parameter '%s'='%s'",
        this_objtype, name, value);

    return false;
  }
  return true;
}
#endif



c_feature2d::ptr create_sparse_feature_detector(const std::string & detector_spec)
{
  std::string objtype;
  std::vector<std::pair<std::string, std::string>> params;

  if ( !parse_object_type_and_args(detector_spec, objtype, params) ) {
    CF_ERROR("parse_object_type_and_params() fails");
    return nullptr;
  }

  FEATURE2D_TYPE kobjtype =
      FEATURE2D_UNKNOWN;

  if ( !fromString(objtype, &kobjtype) || kobjtype == FEATURE2D_UNKNOWN ) {
    CF_ERROR("Unknown or not supported feature2d type requested: %s", objtype.c_str());
    return nullptr;
  }

  switch ( kobjtype ) {
  case FEATURE2D_ORB : {
    c_feature2d_orb::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_BRISK : {
    c_feature2d_brisk::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_KAZE : {
    c_feature2d_kaze::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_AKAZE : {
    c_feature2d_akaze::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#if HAVE_FEATURE2D_SIFT
  case FEATURE2D_SIFT : {
    c_feature2d_sift::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_SURF
  case FEATURE2D_SURF : {
    c_feature2d_surf::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
  case FEATURE2D_MSER : {
    c_feature2d_mser::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_FAST : {
    c_feature2d_fast::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_AGAST : {
    c_feature2d_agast::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_GFTT : {
    c_feature2d_gftt::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_BLOB : {
    c_feature2d_blob::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#if HAVE_FEATURE2D_STAR
  case FEATURE2D_STAR : {
    c_feature2d_star::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_MSD
  case FEATURE2D_MSD : {
    c_feature2d_msd::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_HL
  case FEATURE2D_HL : {
    c_feature2d_hl::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
  default :
    CF_ERROR("Unknown or not supported "
        "feature detector requested: %s",
        objtype.c_str());
    break;
  }

  return nullptr;
}


c_feature2d::ptr create_sparse_feature_detector(const c_sparse_feature_detector_options & options)
{
  switch ( options.type ) {
  case FEATURE2D_UNKNOWN :
    CF_ERROR("No detector type soecified in sparse_feature_detector_options");
    return nullptr;
  case FEATURE2D_ORB :
    return create_feature2d(options.orb);
  case FEATURE2D_BRISK :
    return create_feature2d(options.brisk);
  case FEATURE2D_KAZE :
    return create_feature2d(options.kaze);
  case FEATURE2D_AKAZE :
    return create_feature2d(options.akaze);
#if HAVE_FEATURE2D_SIFT
  case FEATURE2D_SIFT :
    return create_feature2d(options.sift);
#endif
#if HAVE_FEATURE2D_SURF
  case FEATURE2D_SURF :
    return create_feature2d(options.surf);
#endif
  case FEATURE2D_MSER :
    return create_feature2d(options.mser);
  case FEATURE2D_FAST :
    return create_feature2d(options.fast);
  case FEATURE2D_AGAST :
    return create_feature2d(options.agast);
  case FEATURE2D_GFTT :
    return create_feature2d(options.gftt);
  case FEATURE2D_BLOB :
    return create_feature2d(options.blob);
#if HAVE_FEATURE2D_STAR
  case FEATURE2D_STAR :
    return create_feature2d(options.star);
#endif
#if HAVE_FEATURE2D_MSD
  case FEATURE2D_MSD :
    return create_feature2d(options.msd);
#endif
#if HAVE_FEATURE2D_HL
  case FEATURE2D_HL :
    return create_feature2d(options.hl);
#endif
  default :
    CF_ERROR("Unknown or not supported "
        "feature detector requested: %d",
        options.type);
    break;
  }

  return nullptr;
}

c_feature2d::ptr create_sparse_descriptor_extractor(const c_sparse_feature_descriptor_options & options)
{
  switch ( options.type ) {
  case FEATURE2D_UNKNOWN :
    CF_ERROR("No descriptor type soecified in sparse_feature_descriptor_options");
    return nullptr;
  case FEATURE2D_ORB :
    return create_feature2d(options.orb);
  case FEATURE2D_BRISK :
    return create_feature2d(options.brisk);
  case FEATURE2D_KAZE :
    return create_feature2d(options.kaze);
  case FEATURE2D_AKAZE :
    return create_feature2d(options.akaze);
#if HAVE_FEATURE2D_SIFT
  case FEATURE2D_SIFT :
    return create_feature2d(options.sift);
#endif
#if HAVE_FEATURE2D_SURF
  case FEATURE2D_SURF :
    return create_feature2d(options.surf);
#endif
#if HAVE_FEATURE2D_FREAK
    return create_feature2d(options.freak);
#endif
#if HAVE_FEATURE2D_BRIEF
  case FEATURE2D_BRIEF :
    return create_feature2d(options.brief);
#endif
#if HAVE_FEATURE2D_LUCID
  case FEATURE2D_LUCID :
    return create_feature2d(options.lucid);
#endif
#if HAVE_FEATURE2D_LATCH
  case FEATURE2D_LATCH :
    return create_feature2d(options.latch);
#endif
#if HAVE_FEATURE2D_DAISY
  case FEATURE2D_DAISY :
    return create_feature2d(options.daisy);
#endif
#if HAVE_FEATURE2D_VGG
  case FEATURE2D_VGG :
    return create_feature2d(options.vgg);
#endif
#if HAVE_FEATURE2D_BOOST
  case FEATURE2D_BOOST :
    return create_feature2d(options.boost);
#endif
  default :
    CF_ERROR("Unknown or not supported descriptor "
        "extractor requested: %d",
        options.type);
    break;
  }

  return nullptr;
}

c_feature2d::ptr create_sparse_descriptor_extractor(const std::string & descroptor_spec)
{
  std::string objtype;
  std::vector<std::pair<std::string, std::string>> params;

  if ( !parse_object_type_and_args(descroptor_spec, objtype, params) ) {
    CF_ERROR("parse_object_type_and_params() fails");
    return nullptr;
  }

  FEATURE2D_TYPE kobjtype =
      FEATURE2D_UNKNOWN;

  if ( !fromString(objtype, &kobjtype) || kobjtype == FEATURE2D_UNKNOWN ) {
    CF_ERROR("Unknown or not supported feature2d type requested: %s", objtype.c_str());
    return nullptr;
  }

  switch ( kobjtype ) {

  case FEATURE2D_ORB : {
    c_feature2d_orb::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_BRISK : {
    c_feature2d_brisk::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_KAZE : {
    c_feature2d_kaze::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
  case FEATURE2D_AKAZE : {
    c_feature2d_akaze::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#if HAVE_FEATURE2D_SIFT
  case FEATURE2D_SIFT : {
    c_feature2d_sift::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_SURF
  case FEATURE2D_SURF : {
    c_feature2d_surf::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_FREAK
  case FEATURE2D_FREAK : {
    c_feature2d_freak::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_BRIEF
  case FEATURE2D_BRIEF : {
    c_feature2d_brief::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_LUCID
  case FEATURE2D_LUCID : {
    c_feature2d_lucid::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_LATCH
  case FEATURE2D_LATCH : {
    c_feature2d_latch::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_DAISY
  case FEATURE2D_DAISY : {
    c_feature2d_daisy::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_VGG
  case FEATURE2D_VGG : {
    c_feature2d_vgg::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
#if HAVE_FEATURE2D_BOOST
  case FEATURE2D_BOOST : {
    c_feature2d_boost::options opts;
    if ( parse_params(opts, params) ) {
      return create_feature2d(opts);
    }
    break;
  }
#endif
  default :
    CF_ERROR("Unknown or not supported descriptor "
        "extractor requested: %s",
        objtype.c_str());
    break;
  }

  return nullptr;
}


c_sparse_feature_extractor::ptr create_sparse_feature_extractor(const std::string & detector_spec,
    const std::string & descriptor_spec)
{
  c_feature2d::ptr sparse_feature_detector;
  c_feature2d::ptr sparse_descriptor_extractor;

  if ( !detector_spec.empty() ) {
    if ( !(sparse_feature_detector = create_sparse_feature_detector(detector_spec)) ) {
      CF_ERROR("create_feature_detector() fails");
      return nullptr;
    }
  }

  if ( descriptor_spec.empty() ) {
    if ( !can_detect_features_and_compute_descriptors(sparse_feature_detector->type()) ) {
      CF_ERROR("ERROR: Requested feature detector can not extract feature descriptors");
      return nullptr;
    }
  }
  else if ( !(sparse_descriptor_extractor = create_sparse_descriptor_extractor(descriptor_spec)) ) {
    CF_ERROR("create_descriptor_extractor() fails");
    return nullptr;
  }

  return c_sparse_feature_extractor::create(sparse_feature_detector, sparse_descriptor_extractor);
}



c_sparse_feature_extractor::ptr create_sparse_feature_extractor(const c_sparse_feature_extractor_options & options)
{
  c_feature2d::ptr detector;
  c_feature2d::ptr descriptor;

  if ( !(detector = create_sparse_feature_detector(options.detector)) ) {
    CF_ERROR("create_sparse_feature_detector() fails");
    return nullptr;
  }

  if ( options.descriptor.type != FEATURE2D_UNKNOWN ) {
    if ( !(descriptor = create_sparse_descriptor_extractor(options.descriptor)) ) {
      CF_ERROR("create_sparse_descriptor_extractor() fails");
      return nullptr;
    }
  }

  return c_sparse_feature_extractor::create(detector, descriptor);
}


void dump_supported_feature_detectors_and_descriptor_extractors(FILE * fp)
{
  fprintf(fp, "SUPPORTED SPARSE FEATURE DETECTORS AND DESCRIPTOR EXTRACTORS:\n"
      "\n");

  fprintf(fp, "%s:\n"
      "int nfeatures = 500;\n"
      "float scaleFactor = 1.2f;\n"
      "int nlevels = 8;\n"
      "int edgeThreshold = 31;\n"
      "int firstLevel = 0;\n"
      "int WTA_K = 2;\n"
      "int scoreType = 0 (cv::ORB::HARRIS_SCORE);\n"
      "int patchSize = 31;\n"
      "int fastThreshold = 20;\n"
      "\n",
      toString(FEATURE2D_ORB));

  fprintf(fp, "%s:\n"
      "int thresh = 30;\n"
      "int octaves = 3;\n"
      "float patternScale = 1.0f;\n"
      "\n",
      toString(FEATURE2D_BRISK));

  fprintf(fp, "%s:\n"
      "bool extended = false;\n"
      "bool upright = false;\n"
      "float threshold = 0.001f;\n"
      "int nOctaves = 4;\n"
      "int nOctaveLayers = 4;\n"
      "int diffusivity = 1 (cv::KAZE::DIFF_PM_G2);\n"
      "\n",
      toString(FEATURE2D_KAZE));

  fprintf(fp, "%s:\n"
      "int descriptor_type = 5 (cv::AKAZE::DESCRIPTOR_MLDB);\n"
      "int descriptor_size = 0;\n"
      "int descriptor_channels = 3;\n"
      "float threshold = 0.001f;\n"
      "int nOctaves = 4;\n"
      "int nOctaveLayers = 4;\n"
      "int diffusivity = 1 (cv::KAZE::DIFF_PM_G2);\n"
      "\n",
      toString(FEATURE2D_AKAZE));

#if HAVE_FEATURE2D_SIFT
fprintf(fp, "%s:\n"
    "int nfeatures = 0;\n"
    "int nOctaveLayers = 3;\n"
    "double contrastThreshold = 0.04;\n"
    "double edgeThreshold = 10;\n"
    "double sigma = 1.6;\n"
    "\n",
    toString(FEATURE2D_SIFT));

#endif
#if HAVE_FEATURE2D_SURF
  fprintf(fp, "%s:\n"
      "double hessianThreshold = 100;\n"
      "int nOctaves = 4;\n"
      "int nOctaveLayers = 3;\n"
      "bool extended = false;\n"
      "bool upright = false;\n"
      "\n",
      toString(FEATURE2D_SURF));
#endif
}

void dump_supported_feature_detectors(FILE * fp)
{
  dump_supported_feature_detectors_and_descriptor_extractors(fp);

  fprintf(fp, "SUPPORTED SPARSE FEATURE DETECTORS:\n"
      "\n");

  fprintf(fp, "%s:\n"
      "int delta = 5;\n"
      "int min_area = 60;\n"
      "int max_area = 14400;\n"
      "double max_variation = 0.25;\n"
      "double min_diversity = .2;\n"
      "int max_evolution = 200;\n"
      "double area_threshold = 1.01;\n"
      "double min_margin = 0.003;\n"
      "int edge_blur_size = 5;\n"
      "\n",
      toString(FEATURE2D_MSER));

  fprintf(fp, "%s:\n"
      "int threshold = 10;\n"
      "bool nonmaxSuppression = true;\n"
      "int type = 2 (cv::FastFeatureDetector::TYPE_9_16);\n"
      "\n",
      toString(FEATURE2D_FAST));

  fprintf(fp, "%s:\n"
      "int threshold = 10;\n"
      "bool nonmaxSuppression = true;\n"
      "int type = 3 (cv::AgastFeatureDetector::OAST_9_16);\n"
      "\n",
      toString(FEATURE2D_AGAST));

  fprintf(fp, "%s:\n"
      "int maxCorners = 1000;\n"
      "double qualityLevel = 0.01;\n"
      "double minDistance = 1;\n"
      "int blockSize = 3;\n"
      "int gradiantSize = 3;\n"
      "bool useHarrisDetector = false;\n"
      "double k = 0.04;\n"
      "\n",
      toString(FEATURE2D_GFTT));

  fprintf(fp, "%s:\n"
      "float thresholdStep;\n"
      "float minThreshold;\n"
      "float maxThreshold;\n"
      "size_t minRepeatability;\n"
      "float minDistBetweenBlobs;\n"
      "bool filterByColor;\n"
      "uchar blobColor;\n"
      "bool filterByArea;\n"
      "float minArea;\n"
      "float maxArea;\n"
      "bool filterByCircularity;\n"
      "float minCircularity;\n"
      "float maxCircularity;\n"
      "bool filterByInertia;\n"
      "float minInertiaRatio;\n"
      "float maxInertiaRatio;\n"
      "bool filterByConvexity;\n"
      "float minConvexity;\n"
      "float maxConvexity;\n"
      "\n",
      toString(FEATURE2D_BLOB));

#if HAVE_FEATURE2D_STAR
  fprintf(fp, "%s:\n"
      "int maxSize = 45;\n"
      "int responseThreshold = 30;\n"
      "int lineThresholdProjected = 10;\n"
      "int lineThresholdBinarized = 8;\n"
      "int suppressNonmaxSize = 5;\n"
      "\n",
      toString(FEATURE2D_STAR));
#endif
#if HAVE_FEATURE2D_MSD
  fprintf(fp, "%s:\n"
      "int m_patch_radius = 3;\n"
      "int m_search_area_radius = 5;\n"
      "int m_nms_radius = 5;\n"
      "int m_nms_scale_radius = 0;\n"
      "float m_th_saliency = 250.0f;\n"
      "int m_kNN = 4;\n"
      "float m_scale_factor = 1.25f;\n"
      "int m_n_scales = -1;\n"
      "bool m_compute_orientation = false;\n"
      "\n",
      toString(FEATURE2D_MSD));
#endif
#if HAVE_FEATURE2D_HL
  fprintf(fp, "%s:\n"
      "int numOctaves = 6;\n"
      "float corn_thresh = 0.01f;\n"
      "float DOG_thresh = 0.01f;\n"
      "int maxCorners = 5000;\n"
      "int num_layers = 4;\n"
      "\n",
      toString(FEATURE2D_HL));
#endif // HAVE_FEATURE2D_

}

void dump_supported_feature_descriptor_extractors(FILE * fp)
{
  dump_supported_feature_detectors_and_descriptor_extractors(fp);

  fprintf(fp, "SUPPORTED FEATURE DESCRIPTOR EXTRACTORS:\n"
      "\n");

#if HAVE_FEATURE2D_FREAK
  fprintf(fp, "FREAK:\n"
      "bool orientationNormalized = true;\n"
      "bool scaleNormalized = true;\n"
      "float patternScale = 22.0f;\n"
      "int nOctaves = 4;\n"
      "\n");
#endif
#if HAVE_FEATURE2D_BRIEF
  fprintf(fp, "Brief:\n"
      "int bytes = 32;\n"
      "bool use_orientation = false;\n"
      "\n");
#endif

#if HAVE_FEATURE2D_LUCID
  fprintf(fp, "LUCID:\n"
      "int lucid_kernel = 1;\n"
      "int blur_kernel = 2;\n"
      "\n");
#endif

#if HAVE_FEATURE2D_LATCH
  fprintf(fp, "LATCH:\n"
      "int bytes = 32;\n"
      "bool rotationInvariance = true;\n"
      "int half_ssd_size = 3;\n"
      "double sigma = 2.0;\n"
      "\n");
#endif
#if HAVE_FEATURE2D_DAISY
  fprintf(fp, "DAISY:\n"
      "float radius = 15;\n"
      "int q_radius = 3;\n"
      "int q_theta = 8;\n"
      "int q_hist = 8;\n"
      "int norm = 100 (cv::xfeatures2d::DAISY::NRM_NONE);\n"
      "bool interpolation = true;\n"
      "bool use_orientation = false;\n"
      "\n");
#endif

#if HAVE_FEATURE2D_VGG
  fprintf(fp, "VGG:\n"
      "int desc = cv::xfeatures2d::VGG::VGG_120;\n"
      "float isigma = 1.4f;\n"
      "bool img_normalize = true;\n"
      "bool use_scale_orientation = true;\n"
      "float scale_factor = 6.25f;\n"
      "bool dsc_normalize = false;\n"
      "\n");
#endif

#if HAVE_FEATURE2D_BOOST
  fprintf(fp, "BOOST:\n"
      "int desc = 302 (cv::xfeatures2d::BoostDesc::BINBOOST_256);\n"
      "bool use_scale_orientation = true;\n"
      "float scale_factor = 6.25f;\n"
      "\n");
#endif

}


