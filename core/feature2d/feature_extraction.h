/*
 * feature_extraction.h
 *
 *  Created on: Oct 23, 2021
 *      Author: amyznikov
 *
 * Uniform wrappers for standard OpenCV sparse feature detectors
 * and descriptor extractors.
 *
 */

#pragma once
#ifndef __feature_detection_h__
#define __feature_detection_h__

#include <opencv2/opencv.hpp>

#if HAVE_xfeatures2d
# include <opencv2/xfeatures2d.hpp>
#endif // HAVE_xfeatures2d

#include "c_star_extractor.h"
#define HAVE_STAR_EXTRACTOR 1

#include "feature_matching/c_triangle_matcher.h"
#define HAVE_TRIANGLE_EXTRACTOR 1


#include <core/ssprintf.h>

#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) \
    ((a)<<16 | (b)<<8 | (c))
#endif

#ifndef CV_VERSION_CURRRENT
#define CV_VERSION_CURRRENT \
    CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

#if ( CV_VERSION_CURRRENT < CV_VERSION_INT(4,4,0) )
#if HAVE_xfeatures2d
using SIFT = xfeatures2d::SIFT;
using SURF = xfeatures2d::SURF;

#define HAVE_FEATURE2D_BRIEF  1
#define HAVE_FEATURE2D_SIFT   1
#define HAVE_FEATURE2D_SURF   1
#define HAVE_FEATURE2D_FREAK  1
#define HAVE_FEATURE2D_STAR   1
#define HAVE_FEATURE2D_LUCID  1
#define HAVE_FEATURE2D_LATCH  1
#define HAVE_FEATURE2D_DAISY  1
#define HAVE_FEATURE2D_MSD    1
#define HAVE_FEATURE2D_VGG    1
#define HAVE_FEATURE2D_BOOST  1
#define HAVE_FEATURE2D_HL     1
#endif
#else
using SIFT = cv::SIFT;
#define HAVE_FEATURE2D_SIFT 1
#if HAVE_xfeatures2d
using SURF = cv::xfeatures2d::SURF;

#define HAVE_FEATURE2D_BRIEF  1
#define HAVE_FEATURE2D_SURF   1
#define HAVE_FEATURE2D_FREAK  1
#define HAVE_FEATURE2D_STAR   1
#define HAVE_FEATURE2D_LUCID  1
#define HAVE_FEATURE2D_LATCH  1
#define HAVE_FEATURE2D_DAISY  1
#define HAVE_FEATURE2D_MSD    1
#define HAVE_FEATURE2D_VGG    1
#define HAVE_FEATURE2D_BOOST  1
#define HAVE_FEATURE2D_HL     1
#endif
#endif

enum FEATURE2D_TYPE {
  FEATURE2D_UNKNOWN = -1,

  FEATURE2D_ORB = 0,
  FEATURE2D_BRISK = 1,
  FEATURE2D_MSER = 2,
  FEATURE2D_FAST = 3,
  FEATURE2D_AGAST = 4,
  FEATURE2D_GFTT = 5,
  FEATURE2D_BLOB = 6,  // SimpleBlobDetector
  FEATURE2D_KAZE = 7,
  FEATURE2D_AKAZE = 8,
#if  HAVE_FEATURE2D_BRIEF
  FEATURE2D_BRIEF = 9,
#endif
#if HAVE_FEATURE2D_SIFT
  FEATURE2D_SIFT = 10,
#endif
#if HAVE_FEATURE2D_SURF
  FEATURE2D_SURF = 11,
#endif
#if HAVE_FEATURE2D_FREAK
  FEATURE2D_FREAK = 12,
#endif
#if HAVE_FEATURE2D_STAR
  FEATURE2D_STAR = 13,  // StarDetector
#endif
#if HAVE_FEATURE2D_LUCID
  FEATURE2D_LUCID = 14,
#endif
#if HAVE_FEATURE2D_LATCH
  FEATURE2D_LATCH = 15,
#endif
#if HAVE_FEATURE2D_DAISY
  FEATURE2D_DAISY = 16,
#endif
#if HAVE_FEATURE2D_MSD
  FEATURE2D_MSD = 17,  // MSDDetector
#endif
#if HAVE_FEATURE2D_VGG
  FEATURE2D_VGG = 18,
#endif
#if HAVE_FEATURE2D_BOOST
  FEATURE2D_BOOST = 19,
#endif
#if HAVE_FEATURE2D_HL
  FEATURE2D_HL = 20,  // HarrisLaplaceFeatureDetector
#endif
#if HAVE_STAR_EXTRACTOR
  FEATURE2D_STAR_EXTRACTOR,
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  FEATURE2D_TRIANGLE_EXTRACTOR,
#endif

};


enum SPARSE_FEATURE_DETECTOR_TYPE
{
  SPARSE_FEATURE_DETECTOR_UNKNOWN = -1,
  SPARSE_FEATURE_DETECTOR_ORB = FEATURE2D_ORB,
  SPARSE_FEATURE_DETECTOR_BRISK = FEATURE2D_BRISK,
  SPARSE_FEATURE_DETECTOR_MSER = FEATURE2D_MSER,
  SPARSE_FEATURE_DETECTOR_FAST = FEATURE2D_FAST,
  SPARSE_FEATURE_DETECTOR_AGAST = FEATURE2D_AGAST,
  SPARSE_FEATURE_DETECTOR_GFTT = FEATURE2D_GFTT,
  SPARSE_FEATURE_DETECTOR_BLOB = FEATURE2D_BLOB,
  SPARSE_FEATURE_DETECTOR_KAZE = FEATURE2D_KAZE,
  SPARSE_FEATURE_DETECTOR_AKAZE = FEATURE2D_AKAZE,
#if HAVE_FEATURE2D_SIFT
  SPARSE_FEATURE_DETECTOR_SIFT = FEATURE2D_SIFT,
#endif
#if HAVE_FEATURE2D_SURF
  SPARSE_FEATURE_DETECTOR_SURF = FEATURE2D_SURF,
#endif
#if HAVE_FEATURE2D_STAR
  SPARSE_FEATURE_DETECTOR_STAR = FEATURE2D_STAR,
#endif
#if HAVE_FEATURE2D_MSD
  SPARSE_FEATURE_DETECTOR_MSD = FEATURE2D_MSD,
#endif
#if HAVE_FEATURE2D_HL
  SPARSE_FEATURE_DETECTOR_HL = FEATURE2D_HL,
#endif
#if HAVE_STAR_EXTRACTOR
  SPARSE_FEATURE_DETECTOR_STAR_EXTRACTOR = FEATURE2D_STAR_EXTRACTOR,
#endif

};

enum SPARSE_FEATURE_DESCRIPTOR_TYPE
{
  SPARSE_FEATURE_DESCRIPTOR_UNKNOWN = -1,
  SPARSE_FEATURE_DESCRIPTOR_ORB = FEATURE2D_ORB,
  SPARSE_FEATURE_DESCRIPTOR_BRISK = FEATURE2D_BRISK,
  SPARSE_FEATURE_DESCRIPTOR_KAZE = FEATURE2D_KAZE,
  SPARSE_FEATURE_DESCRIPTOR_AKAZE = FEATURE2D_AKAZE,
#if HAVE_FEATURE2D_SIFT
  SPARSE_FEATURE_DESCRIPTOR_SIFT = FEATURE2D_SIFT,
#endif
#if HAVE_FEATURE2D_SURF
  SPARSE_FEATURE_DESCRIPTOR_SURF = FEATURE2D_SURF,
#endif
#if HAVE_FEATURE2D_FREAK
  SPARSE_FEATURE_DESCRIPTOR_FREAK = FEATURE2D_FREAK,
#endif
#if HAVE_FEATURE2D_BRIEF
  SPARSE_FEATURE_DESCRIPTOR_BRIEF = FEATURE2D_BRIEF,
#endif
#if HAVE_FEATURE2D_LUCID
  SPARSE_FEATURE_DESCRIPTOR_LUCID = FEATURE2D_LUCID,
#endif
#if HAVE_FEATURE2D_LATCH
  SPARSE_FEATURE_DESCRIPTOR_LATCH = FEATURE2D_LATCH,
#endif
#if HAVE_FEATURE2D_DAISY
  SPARSE_FEATURE_DESCRIPTOR_DAISY = FEATURE2D_DAISY,
#endif
#if HAVE_FEATURE2D_VGG
  SPARSE_FEATURE_DESCRIPTOR_VGG = FEATURE2D_VGG,
#endif
#if HAVE_FEATURE2D_BOOST
  SPARSE_FEATURE_DESCRIPTOR_BOOST = FEATURE2D_BOOST,
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  SPARSE_FEATURE_DESCRIPTOR_TRIANGLE = FEATURE2D_TRIANGLE_EXTRACTOR,
#endif
};

template<> const c_enum_member *
members_of<FEATURE2D_TYPE>();

template<> const c_enum_member *
members_of<SPARSE_FEATURE_DETECTOR_TYPE>();

template<> const c_enum_member *
members_of<SPARSE_FEATURE_DESCRIPTOR_TYPE>();

template<> const c_enum_member *
members_of<cv::ORB::ScoreType>();

template<> const c_enum_member *
members_of<cv::FastFeatureDetector::DetectorType>();

template<> const c_enum_member *
members_of<cv::AgastFeatureDetector::DetectorType>();

template<> const c_enum_member *
members_of<cv::KAZE::DiffusivityType>();

template<> const c_enum_member *
members_of<cv::AKAZE::DescriptorType>();

#if HAVE_FEATURE2D_DAISY
template<> const c_enum_member *
members_of<cv::xfeatures2d::DAISY::NormalizationType>();
#endif

#if HAVE_FEATURE2D_BOOST
typedef decltype(cv::xfeatures2d::BoostDesc::BINBOOST_256)
BoostDesc_Type;
template<> const c_enum_member *
members_of<BoostDesc_Type>();
#endif



template<class cvFeature2D_type>
struct feature2d_traits;

template<> struct feature2d_traits<cv::ORB> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_ORB;
};
template<> struct feature2d_traits<cv::BRISK> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_BRISK;
};
template<> struct feature2d_traits<cv::MSER> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_MSER;
};
template<> struct feature2d_traits<cv::FastFeatureDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_FAST;
};
template<> struct feature2d_traits<cv::AgastFeatureDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_AGAST;
};
template<> struct feature2d_traits<cv::GFTTDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_GFTT;
};
template<> struct feature2d_traits<cv::SimpleBlobDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_BLOB;
};
template<> struct feature2d_traits<cv::KAZE> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_KAZE;
};
template<> struct feature2d_traits<cv::AKAZE> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_AKAZE;
};
#if  HAVE_FEATURE2D_BRIEF
template<> struct feature2d_traits<cv::xfeatures2d::BriefDescriptorExtractor> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_BRIEF;
};
#endif
#if HAVE_FEATURE2D_SIFT
template<> struct feature2d_traits<SIFT> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_SIFT;
};
#endif
#if HAVE_FEATURE2D_SURF
template<> struct feature2d_traits<SURF> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_SURF;
};
#endif
#if HAVE_FEATURE2D_FREAK
template<> struct feature2d_traits<cv::xfeatures2d::FREAK> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_FREAK;
};
#endif
#if HAVE_FEATURE2D_STAR
template<> struct feature2d_traits<cv::xfeatures2d::StarDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_STAR;
};
#endif
#if HAVE_FEATURE2D_LUCID
template<> struct feature2d_traits<cv::xfeatures2d::LUCID> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_LUCID;
};
#endif
#if HAVE_FEATURE2D_LATCH
template<> struct feature2d_traits<cv::xfeatures2d::LATCH> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_LATCH;
};
#endif
#if HAVE_FEATURE2D_DAISY
template<> struct feature2d_traits<cv::xfeatures2d::DAISY> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_DAISY;
};
#endif
#if HAVE_FEATURE2D_MSD
template<> struct feature2d_traits<cv::xfeatures2d::MSDDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_MSD;
};
#endif
#if HAVE_FEATURE2D_VGG
template<> struct feature2d_traits<cv::xfeatures2d::VGG> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_VGG;
};
#endif
#if HAVE_FEATURE2D_BOOST
template<> struct feature2d_traits<cv::xfeatures2d::BoostDesc> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_BOOST;
};
#endif
#if HAVE_FEATURE2D_HL
template<> struct feature2d_traits<cv::xfeatures2d::HarrisLaplaceFeatureDetector> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_HL;
};
#endif
#if HAVE_STAR_EXTRACTOR
template<> struct feature2d_traits<c_star_extractor> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_STAR_EXTRACTOR;
};
#endif
#if HAVE_TRIANGLE_EXTRACTOR
template<> struct feature2d_traits<c_triangle_extractor> {
  static constexpr FEATURE2D_TYPE type = FEATURE2D_TRIANGLE_EXTRACTOR;
};
#endif



class c_feature2d
{
public:
  typedef c_feature2d this_class;
  typedef std::shared_ptr<this_class> ptr;

  struct options
  {
    /*const */FEATURE2D_TYPE type;

    options(FEATURE2D_TYPE _type) :
        type(_type)
    {
    }
  };

  FEATURE2D_TYPE type() const
  {
    return options_->type;
  }

  const struct options * opts() const
  {
    return options_;
  }

  void detect(cv::InputArray image,
      CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::InputArray mask = cv::noArray()) const
  {
    feature2d_->detect(image, keypoints, mask);
  }

  void compute(cv::InputArray image,
      CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors)
  {
    feature2d_->compute(image, keypoints, descriptors);
  }

  void detectAndCompute(cv::InputArray image, cv::InputArray mask,
      CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors,
      bool useProvidedKeypoints = false)
  {
    feature2d_->detectAndCompute(image, mask,
        keypoints, descriptors,
        useProvidedKeypoints);
  }

  int descriptorSize() const
  {
    return feature2d_->descriptorSize();
  }

  int descriptorType() const
  {
    return feature2d_->descriptorType();
  }

  int defaultNorm() const
  {
    return feature2d_->defaultNorm();
  }

protected:
  c_feature2d(const struct options * options) :
      options_(options)
  {
  }

  const struct options * const options_;
  cv::Ptr<cv::Feature2D> feature2d_;
};

template<class cvFeature2D_type>
class c_feature2d_base :
    public c_feature2d
{
public:
  typedef c_feature2d_base this_class;
  typedef c_feature2d base;

  struct options :
      public base::options
  {
    options() :
        base::options(feature2d_traits<cvFeature2D_type>::type)
    {
    }
  };

protected:
  c_feature2d_base(const options * options) :
      base(options)
  {
  }
};

class c_feature2d_orb :
    public c_feature2d_base<cv::ORB>
{
public:
  typedef c_feature2d_orb this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int nfeatures = 500;
    float scaleFactor = 1.2f;
    int nlevels = 8;
    int edgeThreshold = 31;
    int firstLevel = 0;
    int WTA_K = 2;
    decltype (cv::ORB::HARRIS_SCORE) scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize = 31;
    int fastThreshold = 20;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_orb(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::ORB::create(opts_.nfeatures,
            opts_.scaleFactor,
            opts_.nlevels,
            opts_.edgeThreshold,
            opts_.firstLevel,
            opts_.WTA_K,
            opts_.scoreType,
            opts_.patchSize,
            opts_.fastThreshold);
  }

protected:
  const options opts_;
};

class c_feature2d_brisk :
    public c_feature2d_base<cv::BRISK>
{
public:
  typedef c_feature2d_brisk this_class;
  typedef c_feature2d_base base;
  typedef std::shared_ptr<this_class> ptr;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int thresh = 30;
    int octaves = 3;
    float patternScale = 1.0f;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_brisk(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::BRISK::create(opts_.thresh,
            opts_.octaves,
            opts_.patternScale);
  }

protected:
  const options opts_;
};

class c_feature2d_mser :
    public c_feature2d_base<cv::MSER>
{
public:
  typedef c_feature2d_mser this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int delta = 5;
    int min_area = 60;
    int max_area = 14400;
    double max_variation = 0.25;
    double min_diversity = .2;
    int max_evolution = 200;
    double area_threshold = 1.01;
    double min_margin = 0.003;
    int edge_blur_size = 5;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_mser(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::MSER::create(opts_.delta,
            opts_.min_area,
            opts_.max_area,
            opts_.max_variation,
            opts_.min_diversity,
            opts_.max_evolution,
            opts_.area_threshold,
            opts_.min_margin,
            opts_.edge_blur_size);
  }

protected:
  const options opts_;
};

class c_feature2d_fast :
    public c_feature2d_base<cv::FastFeatureDetector>
{
public:
  typedef c_feature2d_fast this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int threshold = 10;
    bool nonmaxSuppression = true;
    decltype (cv::FastFeatureDetector::TYPE_9_16) type =
        cv::FastFeatureDetector::TYPE_9_16;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_fast(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::FastFeatureDetector::create(opts_.threshold,
            opts_.nonmaxSuppression,
            opts_.type);
  }

protected:
  const options opts_;
};

class c_feature2d_agast :
    public c_feature2d_base<cv::AgastFeatureDetector>
{
public:
  typedef c_feature2d_agast this_class;
  typedef c_feature2d_base base;
  typedef std::shared_ptr<this_class> ptr;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int threshold = 10;
    bool nonmaxSuppression = true;
    decltype (cv::AgastFeatureDetector::OAST_9_16) type = cv::AgastFeatureDetector::OAST_9_16;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_agast(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::AgastFeatureDetector::create(opts_.threshold,
            opts_.nonmaxSuppression,
            opts_.type);
  }

protected:
  const options opts_;
};

class c_feature2d_gftt :
    public c_feature2d_base<cv::GFTTDetector>
{
public:
  typedef c_feature2d_gftt this_class;
  typedef c_feature2d_base base;
  typedef std::shared_ptr<this_class> ptr;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int maxCorners = 1000;
    double qualityLevel = 0.01;
    double minDistance = 1;
    int blockSize = 3;
    int gradiantSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_gftt(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::GFTTDetector::create(opts_.maxCorners,
            opts_.qualityLevel,
            opts_.minDistance,
            opts_.blockSize,
            opts_.gradiantSize,
            opts_.useHarrisDetector,
            opts_.k);
  }

protected:
  const options opts_;
};

class c_feature2d_blob :
    public c_feature2d_base<cv::SimpleBlobDetector>
{
public:
  typedef c_feature2d_blob this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options,
      public cv::SimpleBlobDetector::Params
  {
    using feature2d_class = this_class;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_blob(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::SimpleBlobDetector::create(opts_);
  }

protected:
  const options opts_;
};

class c_feature2d_kaze :
    public c_feature2d_base<cv::KAZE>
{
public:
  typedef c_feature2d_kaze this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    bool extended = false;
    bool upright = false;
    float threshold = 0.001f;
    int nOctaves = 4;
    int nOctaveLayers = 4;
    decltype (cv::KAZE::DIFF_PM_G2) diffusivity =
        cv::KAZE::DIFF_PM_G2;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_kaze(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::KAZE::create(opts_.extended,
            opts_.upright,
            opts_.threshold,
            opts_.nOctaves,
            opts_.nOctaveLayers,
            opts_.diffusivity);
  }

protected:
  const options opts_;
};

class c_feature2d_akaze :
    public c_feature2d_base<cv::AKAZE>
{
public:
  typedef c_feature2d_akaze this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    decltype (cv::AKAZE::DESCRIPTOR_MLDB) descriptor_type =
        cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size = 256;
    int descriptor_channels = 3;
    float threshold = 0.001f;
    int nOctaves = 4;
    int nOctaveLayers = 4;
    decltype (cv::KAZE::DIFF_PM_G2) diffusivity =
        cv::KAZE::DIFF_PM_G2;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_akaze(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::AKAZE::create(opts_.descriptor_type,
            opts_.descriptor_size,
            opts_.descriptor_channels,
            opts_.threshold,
            opts_.nOctaves,
            opts_.nOctaveLayers,
            opts_.diffusivity);
  }

protected:
  const options opts_;

};

#if HAVE_FEATURE2D_SIFT
class c_feature2d_sift :
    public c_feature2d_base<SIFT>
{
public:
  typedef c_feature2d_sift this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int nfeatures = 0;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.04;
    double edgeThreshold = 10;
    double sigma = 1.6;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_sift(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        SIFT::create(opts_.nfeatures,
            opts_.nOctaveLayers,
            opts_.contrastThreshold,
            opts_.edgeThreshold,
            opts_.sigma);
  }

protected:
  const options opts_;
};
#endif // HAVE_FEATURE2D_SIFT

#if HAVE_FEATURE2D_SURF
class c_feature2d_surf :
    public c_feature2d_base<SURF>
{
public:
  typedef c_feature2d_surf this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    double hessianThreshold = 100;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    bool extended = false;
    bool upright = false;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_surf(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        SURF::create(opts_.hessianThreshold,
            opts_.nOctaves,
            opts_.nOctaveLayers,
            opts_.extended,
            opts_.upright);
  }

protected:
  const options opts_;
};
#endif // HAVE_SURF

#if HAVE_FEATURE2D_FREAK
class c_feature2d_freak :
    public c_feature2d_base<cv::xfeatures2d::FREAK>
{
public:
  typedef c_feature2d_freak this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    bool orientationNormalized = true;
    bool scaleNormalized = true;
    float patternScale = 22.0f;
    int nOctaves = 4;
    const std::vector<int> * selectedPairs = NULL;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_freak(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::FREAK::create(opts_.orientationNormalized,
            opts_.scaleNormalized,
            opts_.patternScale,
            opts_.nOctaves,
            opts_.selectedPairs ?
                *opts_.selectedPairs :
                std::vector<int>());
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_STAR
class c_feature2d_star :
    public c_feature2d_base<cv::xfeatures2d::StarDetector>
{
public:
  typedef c_feature2d_star this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int maxSize = 45;
    int responseThreshold = 30;
    int lineThresholdProjected = 10;
    int lineThresholdBinarized = 8;
    int suppressNonmaxSize = 5;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_star(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::StarDetector::create(opts_.maxSize,
            opts_.responseThreshold,
            opts_.lineThresholdProjected,
            opts_.lineThresholdBinarized,
            opts_.suppressNonmaxSize);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_BRIEF
class c_feature2d_brief :
    public c_feature2d_base<cv::xfeatures2d::BriefDescriptorExtractor>
{
public:
  typedef c_feature2d_brief this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int bytes = 32;
    bool use_orientation = false;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_brief(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::BriefDescriptorExtractor::create(opts_.bytes,
            opts_.use_orientation);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_LUCID
class c_feature2d_lucid :
    public c_feature2d_base<cv::xfeatures2d::LUCID>
{
public:
  typedef c_feature2d_lucid this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int lucid_kernel = 1;
    int blur_kernel = 2;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_lucid(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::LUCID::create(opts_.lucid_kernel,
            opts_.blur_kernel);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_LATCH
class c_feature2d_latch :
    public c_feature2d_base<cv::xfeatures2d::LATCH>
{
public:
  typedef c_feature2d_latch this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int bytes = 32;
    bool rotationInvariance = true;
    int half_ssd_size = 3;
    double sigma = 2.0;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_latch(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::LATCH::create(opts_.bytes,
            opts_.rotationInvariance,
            opts_.half_ssd_size,
            opts_.sigma);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_DAISY
class c_feature2d_daisy :
    public c_feature2d_base<cv::xfeatures2d::DAISY>
{
public:
  typedef c_feature2d_daisy this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    float radius = 15;
    int q_radius = 3;
    int q_theta = 8;
    int q_hist = 8;
    decltype (cv::xfeatures2d::DAISY::NRM_NONE) norm = cv::xfeatures2d::DAISY::NRM_NONE;
    cv::Mat H;
    bool interpolation = true;
    bool use_orientation = false;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_daisy(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::DAISY::create(opts_.radius,
            opts_.q_radius,
            opts_.q_theta,
            opts_.q_hist,
            opts_.norm,
            opts_.H,
            opts_.interpolation,
            opts_.use_orientation);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_FEATURE2D_MSD
class c_feature2d_msd :
    public c_feature2d_base<cv::xfeatures2d::MSDDetector>
{
public:
  typedef c_feature2d_msd this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int m_patch_radius = 3;
    int m_search_area_radius = 5;
    int m_nms_radius = 5;
    int m_nms_scale_radius = 0;
    float m_th_saliency = 250.0f;
    int m_kNN = 4;
    float m_scale_factor = 1.25f;
    int m_n_scales = -1;
    bool m_compute_orientation = false;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_msd(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::MSDDetector::create(opts_.m_patch_radius,
            opts_.m_search_area_radius,
            opts_.m_nms_radius,
            opts_.m_nms_scale_radius,
            opts_.m_th_saliency,
            opts_.m_kNN,
            opts_.m_scale_factor,
            opts_.m_n_scales,
            opts_.m_compute_orientation);
  }

protected:
  const options opts_;

};
#endif

#if HAVE_FEATURE2D_VGG
class c_feature2d_vgg :
    public c_feature2d_base<cv::xfeatures2d::VGG>
{
public:
  typedef c_feature2d_vgg this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int desc = cv::xfeatures2d::VGG::VGG_120;
    float isigma = 1.4f;
    bool img_normalize = true;
    bool use_scale_orientation = true;
    float scale_factor = 6.25f;
    bool dsc_normalize = false;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_vgg(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::VGG::create(opts_.desc,
            opts_.isigma,
            opts_.img_normalize,
            opts_.use_scale_orientation,
            opts_.scale_factor,
            opts_.dsc_normalize);
  }

protected:
  const options opts_;
};

#endif

#if HAVE_FEATURE2D_BOOST
class c_feature2d_boost :
    public c_feature2d_base<cv::xfeatures2d::BoostDesc>
{
public:
  typedef c_feature2d_boost this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    decltype(cv::xfeatures2d::BoostDesc::BINBOOST_256) desc = cv::xfeatures2d::BoostDesc::BINBOOST_256;
    bool use_scale_orientation = true;
    float scale_factor = 6.25f;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_boost(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::BoostDesc::create(opts_.desc,
            opts_.use_scale_orientation,
            opts_.scale_factor);
  }

protected:
  const options opts_;

};
#endif

#if HAVE_FEATURE2D_HL
class c_feature2d_hl :
    public c_feature2d_base<cv::xfeatures2d::HarrisLaplaceFeatureDetector>
{
public:
  typedef c_feature2d_hl this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int numOctaves = 6;
    float corn_thresh = 0.01f;
    float DOG_thresh = 0.01f;
    int maxCorners = 5000;
    int num_layers = 4;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_hl(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        cv::xfeatures2d::HarrisLaplaceFeatureDetector::create(opts_.numOctaves,
            opts_.corn_thresh,
            opts_.DOG_thresh,
            opts_.maxCorners,
            opts_.num_layers);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_STAR_EXTRACTOR
class c_feature2d_star_extractor :
    public c_feature2d_base<c_star_extractor>
{
public:
  typedef c_feature2d_star_extractor this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int numOctaves = 4;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_star_extractor(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        c_star_extractor::create(
            opts_.numOctaves);
  }

protected:
  const options opts_;
};
#endif

#if HAVE_TRIANGLE_EXTRACTOR
class c_feature2d_triangle_extractor :
    public c_feature2d_base<c_triangle_extractor>
{
public:
  typedef c_feature2d_triangle_extractor this_class;
  typedef c_feature2d_base base;

  struct options :
      public base::options
  {
    using feature2d_class = this_class;
    int min_side_size = 10;
  };

  static ptr create(const options * opts = nullptr)
  {
    return ptr(new this_class(opts));
  }

protected:
  c_feature2d_triangle_extractor(const options * opts) :
      base(&this->opts_),
          opts_(opts ? *opts : options())
  {
    feature2d_ =
        c_triangle_extractor::create(opts_.min_side_size);
  }

protected:
  const options opts_;
};
#endif


template<class feature2d_options>
c_feature2d::ptr create_feature2d(const feature2d_options & options)
{
  return feature2d_options::feature2d_class::create(&options);
}

inline constexpr bool can_detect_features(enum FEATURE2D_TYPE type)
{
  switch ( type ) {
  case FEATURE2D_ORB :
    case FEATURE2D_BRISK :
    case FEATURE2D_KAZE :
    case FEATURE2D_AKAZE :
#if HAVE_FEATURE2D_SIFT
    case FEATURE2D_SIFT :
#endif
#if HAVE_FEATURE2D_SURF
    case FEATURE2D_SURF :
#endif
    case FEATURE2D_MSER :
    case FEATURE2D_FAST :
    case FEATURE2D_AGAST :
    case FEATURE2D_GFTT :
    case FEATURE2D_BLOB :
#if HAVE_FEATURE2D_STAR
    case FEATURE2D_STAR :
#endif
#if HAVE_FEATURE2D_MSD
    case FEATURE2D_MSD :
#endif
#if HAVE_FEATURE2D_HL
    case FEATURE2D_HL :
#endif
#if HAVE_STAR_EXTRACTOR
    case FEATURE2D_STAR_EXTRACTOR:
#endif
    return true;

  default :
    break;
  }

  return false;
}

inline constexpr bool can_compute_decriptors(enum FEATURE2D_TYPE type)
{
  switch ( type ) {
    case FEATURE2D_ORB :
    case FEATURE2D_BRISK :
    case FEATURE2D_KAZE :
    case FEATURE2D_AKAZE :
#if HAVE_FEATURE2D_SIFT
    case FEATURE2D_SIFT :
#endif
#if HAVE_FEATURE2D_SURF
    case FEATURE2D_SURF :
#endif
#if HAVE_FEATURE2D_FREAK
    case FEATURE2D_FREAK :
#endif
#if HAVE_FEATURE2D_BRIEF
    case FEATURE2D_BRIEF :
#endif
#if HAVE_FEATURE2D_LUCID
    case FEATURE2D_LUCID :
#endif
#if HAVE_FEATURE2D_LATCH
    case FEATURE2D_LATCH :
#endif
#if HAVE_FEATURE2D_DAISY
    case FEATURE2D_DAISY :
#endif
#if HAVE_FEATURE2D_VGG
    case FEATURE2D_VGG :
#endif
#if HAVE_FEATURE2D_BOOST
    case FEATURE2D_BOOST :
#endif
    return true;
  default :
    break;
  }
  return false;
}

inline constexpr bool can_detect_features_and_compute_descriptors(enum FEATURE2D_TYPE type)
{
  return can_detect_features(type) && can_compute_decriptors(type);
}



struct c_sparse_feature_detector_options
{
  SPARSE_FEATURE_DETECTOR_TYPE type =
#if HAVE_FEATURE2D_SURF
      SPARSE_FEATURE_DETECTOR_SURF;
#else
      SPARSE_FEATURE_DETECTOR_AKAZE;
#endif

  int max_keypoints = 1000;

  c_feature2d_orb::options orb;
  c_feature2d_brisk::options brisk;
  c_feature2d_kaze::options kaze;
  c_feature2d_akaze::options akaze;
#if HAVE_FEATURE2D_SIFT
  c_feature2d_sift::options sift;
#endif
#if HAVE_FEATURE2D_SURF
  c_feature2d_surf::options surf;
#endif
  c_feature2d_mser::options mser;
  c_feature2d_fast::options fast;
  c_feature2d_agast::options agast;
  c_feature2d_gftt::options gftt;
  c_feature2d_blob::options blob;
#if HAVE_FEATURE2D_STAR
  c_feature2d_star::options star;
#endif
#if HAVE_FEATURE2D_MSD
  c_feature2d_msd::options msd;
#endif
#if HAVE_FEATURE2D_HL
  c_feature2d_hl::options hl;
#endif
#if HAVE_STAR_EXTRACTOR
  c_feature2d_star_extractor::options star_extractor;
#endif
};


struct c_sparse_feature_descriptor_options
{
  SPARSE_FEATURE_DESCRIPTOR_TYPE type =
#if HAVE_FEATURE2D_SURF
      SPARSE_FEATURE_DESCRIPTOR_SURF;
#else
      SPARSE_FEATURE_DESCRIPTOR_AKAZE;
#endif

  bool use_detector_options = true;

  c_feature2d_orb::options orb;
  c_feature2d_brisk::options brisk;
  c_feature2d_kaze::options kaze;
  c_feature2d_akaze::options akaze;
#if HAVE_FEATURE2D_SIFT
  c_feature2d_sift::options sift;
#endif
#if HAVE_FEATURE2D_SURF
  c_feature2d_surf::options surf;
#endif
#if HAVE_FEATURE2D_FREAK
  c_feature2d_freak::options freak;
#endif
#if HAVE_FEATURE2D_BRIEF
  c_feature2d_brief::options brief;
#endif
#if HAVE_FEATURE2D_LUCID
  c_feature2d_lucid::options lucid;
#endif
#if HAVE_FEATURE2D_LATCH
  c_feature2d_latch::options latch;
#endif
#if HAVE_FEATURE2D_DAISY
  c_feature2d_daisy::options daisy;
#endif
#if HAVE_FEATURE2D_VGG
  c_feature2d_vgg::options vgg;
#endif
#if HAVE_FEATURE2D_BOOST
  c_feature2d_boost::options boost;
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  c_feature2d_triangle_extractor::options triangles;
#endif
};

struct c_sparse_feature_extractor_options
{
  // Detector options
  c_sparse_feature_detector_options detector;

  // Decriptor options
  c_sparse_feature_descriptor_options descriptor;
};



class c_sparse_feature_extractor
{
public:
  typedef c_sparse_feature_extractor this_class;
  typedef std::shared_ptr<this_class> ptr;

  static ptr create(const c_feature2d::ptr & feature_detector,
      const c_feature2d::ptr & descriptor_extrator = nullptr,
      int max_keypoints = -1)
  {
    return ptr(new this_class(feature_detector, descriptor_extrator, max_keypoints));
  }


  enum FEATURE2D_TYPE detector_type() const
  {
    return detector_->type();
  }

  enum FEATURE2D_TYPE descriptor_type() const
  {
    return descriptor_ ? descriptor_->type() : FEATURE2D_UNKNOWN;
  }

  void set_max_keypoints(int v)
  {
    max_keypoints_ = v;
  }

  int max_keypoints() const
  {
    return max_keypoints_;
  }

  const c_feature2d::ptr & detector() const
  {
    return detector_;
  }

  const c_feature2d::ptr & descriptor() const
  {
    return descriptor_ ? descriptor_ : detector_;
  }

  bool detect(cv::InputArray image,
      CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::InputArray mask = cv::noArray()) const;

  bool compute(cv::InputArray image,
      CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

  bool detectAndCompute(cv::InputArray image, cv::InputArray mask,
      CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors,
      bool useProvidedKeypoints = false);

protected:
  c_sparse_feature_extractor(const c_feature2d::ptr & detector,
      const c_feature2d::ptr & descriptor = nullptr,
      int max_keypoints = -1);

protected:
  c_feature2d::ptr detector_;
  c_feature2d::ptr descriptor_;
  int max_keypoints_;
};


c_feature2d::ptr create_sparse_feature_detector(
    const c_sparse_feature_detector_options & options);

c_feature2d::ptr create_sparse_descriptor_extractor(
    const c_sparse_feature_descriptor_options & options);

c_sparse_feature_extractor::ptr create_sparse_feature_extractor(
    const c_sparse_feature_extractor_options & options);


#endif /* __feature_detection_h__ */
