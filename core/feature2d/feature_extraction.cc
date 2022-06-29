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
  static constexpr c_enum_member members[] = {
#if HAVE_FEATURE2D_SURF
      { FEATURE2D_SURF, "SURF", "" },
#endif
#if HAVE_SIMPLE_PLANETARY_DISK_DETECTOR
      { FEATURE2D_PLANETARY_DISK, "PLANETARY_DISK" },
#endif
#if HAVE_STAR_EXTRACTOR
      { FEATURE2D_STAR_EXTRACTOR, "STAR_EXTRACTOR", "Detect stars on astro image" },
#endif
#if HAVE_TRIANGLE_EXTRACTOR
      { SPARSE_FEATURE_DESCRIPTOR_TRIANGLE, "TRIANGLE", "Build triangles from the set of sparse keypoint locations" },
#endif
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

template<> const c_enum_member*
members_of<SPARSE_FEATURE_DETECTOR_TYPE>()
{
  static constexpr c_enum_member members[] = {
#if HAVE_FEATURE2D_SURF
      { SPARSE_FEATURE_DETECTOR_SURF, "SURF" },
#endif
#if HAVE_SIMPLE_PLANETARY_DISK_DETECTOR
      { SPARSE_FEATURE_DETECTOR_PLANETARY_DISK, "PLANETARY_DISK" },
#endif
#if HAVE_STAR_EXTRACTOR
      { FEATURE2D_STAR_EXTRACTOR, "STAR_EXTRACTOR", "Detect stars on astro image" },
#endif
      { SPARSE_FEATURE_DETECTOR_AKAZE, "AKAZE" },
#if HAVE_FEATURE2D_SIFT
      { SPARSE_FEATURE_DETECTOR_SIFT, "SIFT" },
#endif
      { SPARSE_FEATURE_DETECTOR_ORB, "ORB" },
      { SPARSE_FEATURE_DETECTOR_BRISK, "BRISK" },
      { SPARSE_FEATURE_DETECTOR_MSER, "MSER" },
      { SPARSE_FEATURE_DETECTOR_FAST, "FAST" },
      { SPARSE_FEATURE_DETECTOR_AGAST, "AGAST" },
      { SPARSE_FEATURE_DETECTOR_GFTT, "GFTT" },
      { SPARSE_FEATURE_DETECTOR_BLOB, "BLOB" },
      { SPARSE_FEATURE_DETECTOR_KAZE, "KAZE" },
#if HAVE_FEATURE2D_STAR
      { SPARSE_FEATURE_DETECTOR_STAR, "STAR" },
#endif
#if HAVE_FEATURE2D_MSD
      { SPARSE_FEATURE_DETECTOR_MSD, "MSD" },
#endif
#if HAVE_FEATURE2D_HL
      { SPARSE_FEATURE_DETECTOR_HL, "HL" },
#endif
      { SPARSE_FEATURE_DETECTOR_UNKNOWN, nullptr },
  };

  return members;
}

template<> const c_enum_member *
members_of<SPARSE_FEATURE_DESCRIPTOR_TYPE>()
{
  static constexpr c_enum_member members[] = {
#if HAVE_FEATURE2D_SURF
      {SPARSE_FEATURE_DESCRIPTOR_SURF, "SURF"},
#endif
#if HAVE_TRIANGLE_EXTRACTOR
      { SPARSE_FEATURE_DESCRIPTOR_TRIANGLE, "TRIANGLE", "Build triangles from the set of sparse keypoint locations" },
#endif
      {SPARSE_FEATURE_DESCRIPTOR_AKAZE, "AKAZE"},
      {SPARSE_FEATURE_DESCRIPTOR_ORB, "ORB"},
      {SPARSE_FEATURE_DESCRIPTOR_BRISK, "BRISK"},
      {SPARSE_FEATURE_DESCRIPTOR_KAZE, "KAZE"},
#if HAVE_FEATURE2D_SIFT
      {SPARSE_FEATURE_DESCRIPTOR_SIFT, "SIFT"},
#endif
#if HAVE_FEATURE2D_FREAK
      {SPARSE_FEATURE_DESCRIPTOR_FREAK, "FREAK"},
#endif
#if HAVE_FEATURE2D_BRIEF
      {SPARSE_FEATURE_DESCRIPTOR_BRIEF, "BRIEF"},
#endif
#if HAVE_FEATURE2D_LUCID
      {SPARSE_FEATURE_DESCRIPTOR_LUCID, "LUCID"},
#endif
#if HAVE_FEATURE2D_LATCH
      {SPARSE_FEATURE_DESCRIPTOR_LATCH, "LATCH"},
#endif
#if HAVE_FEATURE2D_DAISY
      {SPARSE_FEATURE_DESCRIPTOR_DAISY, "DAISY"},
#endif
#if HAVE_FEATURE2D_VGG
      {SPARSE_FEATURE_DESCRIPTOR_VGG, "VGG"},
#endif
#if HAVE_FEATURE2D_BOOST
      {SPARSE_FEATURE_DESCRIPTOR_BOOST, "BOOST"},
#endif
      {SPARSE_FEATURE_DESCRIPTOR_UNKNOWN, nullptr}
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
    const c_feature2d::ptr & descriptor, int max_keypoints) :
    detector_(detector),
    descriptor_(descriptor),
    max_keypoints_(max_keypoints)
{
}


bool c_sparse_feature_extractor::detect(cv::InputArray image,
    CV_OUT std::vector<cv::KeyPoint> & keypoints,
    cv::InputArray mask) const
{
  INSTRUMENT_REGION("");
  detector_->detect(image, keypoints, mask);

  if ( max_keypoints_ > 0 && keypoints.size() > max_keypoints_ ) {
    std::sort(keypoints.begin(), keypoints.end(),
        [](const cv::KeyPoint & prev, const cv::KeyPoint & next )-> bool {
          return prev.response > next.response;
        });

    keypoints.erase(keypoints.begin() + max_keypoints_, keypoints.end());
  }

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
    // simple planetary disk detector not required to compute descriptors
    if ( detector_->type() != FEATURE2D_PLANETARY_DISK ) {
      CF_ERROR("specified keypoints detector %s can not compute feature descriptors",
        toString(detector_->type()));
      return false;
    }
  }

  //
  // Detect sparse 2D features (keypoints) and compute descriptors
  //

  keypoints.clear();
  keypoints.reserve(std::max(max_keypoints_, 1000));

  // Prefer detectAndCompute() if possible because it can be faster for some detectors
  if ( !descriptor_ || descriptor_.get() == detector_.get() ) {

    if( detector_->type() == FEATURE2D_PLANETARY_DISK ) {
      // simple planetary disk detector not required to compute descriptors
      detector_->detect(image, keypoints, mask);
      descriptors.release();
    }
    else {
      detector_->detectAndCompute(
              image,
              mask,
              keypoints,
              descriptors,
              useProvidedKeypoints);
    }

    if ( keypoints.size() < 1 ) {
      CF_ERROR("No keypoints detected on image");
      return false;
    }
  }
  else {

    detector_->detect(image,
        keypoints,
        mask);

    if ( keypoints.size() < 1 ) {
      CF_ERROR("No keypoints detected on image");
      return false;
    }


    if ( max_keypoints_ > 0 && (int)keypoints.size() > max_keypoints_ ) {

      std::sort(keypoints.begin(), keypoints.end(),
          [](const cv::KeyPoint & prev, const cv::KeyPoint & next )-> bool {
            return prev.response > next.response;
          });

      keypoints.erase(keypoints.begin() + max_keypoints_, keypoints.end());
    }

    descriptor_->compute(image,
        keypoints,
        descriptors);
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


c_feature2d::ptr create_sparse_feature_detector(const c_sparse_feature_detector_options & options)
{
  switch ( options.type ) {
  case FEATURE2D_UNKNOWN :
    CF_ERROR("No detector type specified in sparse_feature_detector_options");
    return nullptr;
  case SPARSE_FEATURE_DETECTOR_ORB :
    return create_feature2d(options.orb);
  case SPARSE_FEATURE_DETECTOR_BRISK :
    return create_feature2d(options.brisk);
  case SPARSE_FEATURE_DETECTOR_KAZE :
    return create_feature2d(options.kaze);
  case SPARSE_FEATURE_DETECTOR_AKAZE :
    return create_feature2d(options.akaze);
#if HAVE_FEATURE2D_SIFT
  case SPARSE_FEATURE_DETECTOR_SIFT :
    return create_feature2d(options.sift);
#endif
#if HAVE_FEATURE2D_SURF
  case SPARSE_FEATURE_DETECTOR_SURF :
    return create_feature2d(options.surf);
#endif
  case SPARSE_FEATURE_DETECTOR_MSER :
    return create_feature2d(options.mser);
  case SPARSE_FEATURE_DETECTOR_FAST :
    return create_feature2d(options.fast);
  case SPARSE_FEATURE_DETECTOR_AGAST :
    return create_feature2d(options.agast);
  case SPARSE_FEATURE_DETECTOR_GFTT :
    return create_feature2d(options.gftt);
  case SPARSE_FEATURE_DETECTOR_BLOB :
    return create_feature2d(options.blob);
#if HAVE_FEATURE2D_STAR
  case SPARSE_FEATURE_DETECTOR_STAR :
    return create_feature2d(options.star);
#endif
#if HAVE_FEATURE2D_MSD
  case SPARSE_FEATURE_DETECTOR_MSD :
    return create_feature2d(options.msd);
#endif
#if HAVE_FEATURE2D_HL
  case SPARSE_FEATURE_DETECTOR_HL :
    return create_feature2d(options.hl);
#endif
#if HAVE_STAR_EXTRACTOR
  case SPARSE_FEATURE_DETECTOR_STAR_EXTRACTOR:
    return create_feature2d(options.star_extractor);
#endif
#if HAVE_SIMPLE_PLANETARY_DISK_DETECTOR
  case SPARSE_FEATURE_DETECTOR_PLANETARY_DISK:
    return create_feature2d(options.planetary_disk_detector);
#endif

  default :
    CF_ERROR("Unknown or not supported "
        "feature detector requested: %d",
        options.type);
    break;
  }

  return nullptr;
}

c_sparse_feature_extractor::ptr create_sparse_feature_extractor(const c_sparse_feature_extractor_options & options)
{
  c_feature2d::ptr detector;
  c_feature2d::ptr descriptor;

  if ( !(detector = create_sparse_feature_detector(options.detector)) ) {
    CF_ERROR("create_sparse_feature_detector() fails");
    return nullptr;
  }

  if( options.descriptor.use_detector_options || options.descriptor.type == SPARSE_FEATURE_DESCRIPTOR_UNKNOWN ) {

    bool ignore_errors = false;

    switch (options.detector.type) {
    case SPARSE_FEATURE_DETECTOR_MSER:
    case SPARSE_FEATURE_DETECTOR_FAST:
    case SPARSE_FEATURE_DETECTOR_AGAST:
    case SPARSE_FEATURE_DETECTOR_GFTT:
    case SPARSE_FEATURE_DETECTOR_BLOB:
#if HAVE_FEATURE2D_MSD
    case SPARSE_FEATURE_DETECTOR_MSD:
#endif
#if HAVE_FEATURE2D_STAR
    case SPARSE_FEATURE_DETECTOR_STAR:
#endif
#if HAVE_FEATURE2D_HL
    case SPARSE_FEATURE_DETECTOR_HL:
#endif
#if HAVE_FEATURE2D_SURF
      descriptor = create_feature2d(options.descriptor.surf);
#else
      descriptor = create_feature2d(options.descriptor.orb);
#endif
      break;
#if HAVE_STAR_EXTRACTOR
    case SPARSE_FEATURE_DETECTOR_STAR_EXTRACTOR:
      descriptor = create_feature2d(options.descriptor.triangles);
      break;
#endif
    default:
      ignore_errors = true;
      break;
    }

    if ( !ignore_errors && !descriptor ) {
      CF_ERROR("create_sparse_descriptor_extractor() fails");
      return nullptr;
    }
  }

  if ( !descriptor ) {
    if ( !options.descriptor.use_detector_options && options.descriptor.type != SPARSE_FEATURE_DESCRIPTOR_UNKNOWN) {
      if ( !(descriptor = create_sparse_descriptor_extractor(options.descriptor)) ) {
        CF_ERROR("create_sparse_descriptor_extractor() fails");
        return nullptr;
      }
    }
  }

  return c_sparse_feature_extractor::create(detector, descriptor, options.detector.max_keypoints);
}


c_feature2d::ptr create_sparse_descriptor_extractor(const c_sparse_feature_descriptor_options & options)
{
  switch ( options.type ) {
  case FEATURE2D_UNKNOWN :
    CF_ERROR("No descriptor type soecified in sparse_feature_descriptor_options");
    return nullptr;
  case SPARSE_FEATURE_DESCRIPTOR_ORB :
    return create_feature2d(options.orb);
  case SPARSE_FEATURE_DESCRIPTOR_BRISK :
    return create_feature2d(options.brisk);
  case SPARSE_FEATURE_DESCRIPTOR_KAZE :
    return create_feature2d(options.kaze);
  case SPARSE_FEATURE_DESCRIPTOR_AKAZE :
    return create_feature2d(options.akaze);
#if HAVE_FEATURE2D_SIFT
  case SPARSE_FEATURE_DESCRIPTOR_SIFT :
    return create_feature2d(options.sift);
#endif
#if HAVE_FEATURE2D_SURF
  case SPARSE_FEATURE_DESCRIPTOR_SURF :
    return create_feature2d(options.surf);
#endif
#if HAVE_FEATURE2D_FREAK
  case SPARSE_FEATURE_DESCRIPTOR_FREAK :
    return create_feature2d(options.freak);
#endif
#if HAVE_FEATURE2D_BRIEF
  case SPARSE_FEATURE_DESCRIPTOR_BRIEF :
    return create_feature2d(options.brief);
#endif
#if HAVE_FEATURE2D_LUCID
  case SPARSE_FEATURE_DESCRIPTOR_LUCID :
    return create_feature2d(options.lucid);
#endif
#if HAVE_FEATURE2D_LATCH
  case SPARSE_FEATURE_DESCRIPTOR_LATCH :
    return create_feature2d(options.latch);
#endif
#if HAVE_FEATURE2D_DAISY
  case SPARSE_FEATURE_DESCRIPTOR_DAISY :
    return create_feature2d(options.daisy);
#endif
#if HAVE_FEATURE2D_VGG
  case SPARSE_FEATURE_DESCRIPTOR_VGG :
    return create_feature2d(options.vgg);
#endif
#if HAVE_FEATURE2D_BOOST
  case SPARSE_FEATURE_DESCRIPTOR_BOOST :
    return create_feature2d(options.boost);
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  case FEATURE2D_TRIANGLE_EXTRACTOR :
    return create_feature2d(options.triangles);
#endif

  default :
    CF_ERROR("Unknown or not supported descriptor "
        "extractor requested: %d",
        options.type);
    break;
  }

  return nullptr;
}

