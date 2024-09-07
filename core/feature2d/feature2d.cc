/*
 * feature2d.cc
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#include "feature2d.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>
/////////////////////

const c_sparse_feature_extractor_and_matcher_options & c_sparse_feature_extractor_and_matcher::options() const
{
  return options_;
}

const c_feature2d::sptr & c_sparse_feature_extractor_and_matcher::detector() const
{
  return detector_;
}

const c_feature2d::sptr & c_sparse_feature_extractor_and_matcher::descriptor() const
{
  return descriptor_;
}

const c_feature2d_matcher::sptr & c_sparse_feature_extractor_and_matcher::matcher() const
{
  return matcher_;
}

SPARSE_FEATURE_DETECTOR_TYPE c_sparse_feature_extractor_and_matcher::detector_type() const
{
  return options_.detector.type;
}

SPARSE_FEATURE_DESCRIPTOR_TYPE c_sparse_feature_extractor_and_matcher::descriptor_type() const
{
  return options_.descriptor.type;
}

FEATURE2D_MATCHER_TYPE c_sparse_feature_extractor_and_matcher::matcher_type() const
{
  return options_.matcher.type;
}

//const cv::Mat & c_sparse_feature_extractor_and_matcher::referece_image() const
//{
//  return reference_image_;
//}
//
//const cv::Mat & c_sparse_feature_extractor_and_matcher::referece_mask() const
//{
//  return reference_mask_;
//}

const cv::Mat & c_sparse_feature_extractor_and_matcher::referece_descriptors() const
{
  return reference_descriptors_;
}

const std::vector<cv::KeyPoint> & c_sparse_feature_extractor_and_matcher::referece_keypoints() const
{
  return reference_keypoints_;
}

const std::vector<cv::Point2f> & c_sparse_feature_extractor_and_matcher::matched_reference_positions() const
{
  return matched_reference_positions_;
}

//const cv::Mat & c_sparse_feature_extractor_and_matcher::current_image() const
//{
//  return current_image_;
//}
//
//const cv::Mat & c_sparse_feature_extractor_and_matcher::current_mask() const
//{
//  return current_mask_;
//}

const cv::Mat & c_sparse_feature_extractor_and_matcher::current_descriptors() const
{
  return current_descriptors_;
}

const std::vector<cv::KeyPoint> & c_sparse_feature_extractor_and_matcher::current_keypoints() const
{
  return current_keypoints_;
}

const std::vector<cv::Point2f> & c_sparse_feature_extractor_and_matcher::matched_current_positions() const
{
  return matched_current_positions_;
}


c_sparse_feature_extractor_and_matcher::c_sparse_feature_extractor_and_matcher()
{
}

c_sparse_feature_extractor_and_matcher::c_sparse_feature_extractor_and_matcher(const c_sparse_feature_extractor_and_matcher_options & opts) :
    options_(opts)
{
}


c_sparse_feature_extractor_and_matcher::sptr c_sparse_feature_extractor_and_matcher::create(const c_sparse_feature_extractor_and_matcher_options & options)
{
  if( options.detector.type == SPARSE_FEATURE_DETECTOR_UNKNOWN ) {
    CF_ERROR("c_sparse_feature_extractor_and_matcher::create(): no sparse feature dtector type specified");
    return nullptr;
  }

  this_class::sptr obj(new this_class(options));

  //////////////

  if( !(obj->detector_ = create_sparse_feature_detector(obj->options_.detector)) ) {
    CF_ERROR("create_sparse_feature_detector(type=%s) fails", toCString(obj->options_.detector.type));
    return nullptr;
  }

  //////////////
  if( obj->options_.descriptor.type == SPARSE_FEATURE_DESCRIPTOR_AUTO_SELECT ) {

    switch (obj->options_.matcher.type) {
      case FEATURE2D_MATCHER_OptFlowPyrLK:
        break;
      case FEATURE2D_MATCHER_AUTO_SELECT:
        if ( !can_compute_decriptors(obj->detector_->type()) ) {
          obj->options_.matcher.type = FEATURE2D_MATCHER_OptFlowPyrLK;
        }
        else {
          obj->options_.descriptor.type =
              (SPARSE_FEATURE_DESCRIPTOR_TYPE) obj->detector_->type();
        }
        break;
    }
  }

  if( obj->options_.descriptor.type != SPARSE_FEATURE_DESCRIPTOR_UNKNOWN ) {
    if( (int)obj->options_.descriptor.type != (int)obj->detector_->type() ) {

      if( !(obj->descriptor_ = create_sparse_descriptor_extractor(obj->options_.descriptor)) ) {
        CF_ERROR("create_sparse_descriptor_extractor(type=%s) fails", toCString(obj->options_.descriptor.type));
        return nullptr;
      }

      obj->options_.descriptor.type =
          (SPARSE_FEATURE_DESCRIPTOR_TYPE) obj->detector_->type();
    }
  }
  //////////////

  if( obj->options_.matcher.type == FEATURE2D_MATCHER_AUTO_SELECT ) {

    switch (obj->options_.descriptor.type) {

      case SPARSE_FEATURE_DESCRIPTOR_UNKNOWN:
        obj->options_.matcher.type = FEATURE2D_MATCHER_OptFlowPyrLK;
        break;

      case SPARSE_FEATURE_DESCRIPTOR_ORB: {

        const c_feature2d_orb::options &opts =
            obj->descriptor_ ? obj->options_.descriptor.orb :
                obj->options_.detector.orb;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        obj->options_.matcher.flann.index.type = FlannIndex_lsh;
        // FIXME: opts.WTA_K > 2  with flann ?
        obj->options_.matcher.flann.distance_type = cvflann::FLANN_DIST_DNAMMING;

        break;
      }

      case SPARSE_FEATURE_DESCRIPTOR_BRISK: {

        const c_feature2d_brisk::options &opts =
            obj->descriptor_ ? obj->options_.descriptor.brisk :
                obj->options_.detector.brisk;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
       }

      case SPARSE_FEATURE_DESCRIPTOR_KAZE:{

        const c_feature2d_kaze::options &opts =
            obj->descriptor_ ? obj->options_.descriptor.kaze :
                obj->options_.detector.kaze;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
       }

      case SPARSE_FEATURE_DESCRIPTOR_AKAZE:{

        const c_feature2d_akaze::options &opts =
            obj->descriptor_ ? obj->options_.descriptor.akaze :
                obj->options_.detector.akaze;

        switch (opts.descriptor_type) {
          case cv::AKAZE::DESCRIPTOR_MLDB:
          case cv::AKAZE::DESCRIPTOR_MLDB_UPRIGHT:
            obj->options_.matcher.type = FEATURE2D_MATCHER_HAMMING;
            break;
          default:
            obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
            obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
            break;
        }
        break;
       }

      #if HAVE_FEATURE2D_SIFT
      case SPARSE_FEATURE_DESCRIPTOR_SIFT: {

       const c_feature2d_sift::options &opts =
           obj->descriptor_ ? obj->options_.descriptor.sift :
               obj->options_.detector.sift;

       obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
       obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
       break;
      }
      #endif

      #if HAVE_FEATURE2D_SURF
      case SPARSE_FEATURE_DESCRIPTOR_SURF:{

        const c_feature2d_surf::options &opts =
            obj->descriptor_ ? obj->options_.descriptor.surf :
                obj->options_.detector.surf;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }
      #endif

      #if HAVE_FEATURE2D_FREAK
      case SPARSE_FEATURE_DESCRIPTOR_FREAK:{

        const c_feature2d_freak::options &opts =
            obj->options_.descriptor.freak;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }

      #endif

      #if HAVE_FEATURE2D_BRIEF
      case SPARSE_FEATURE_DESCRIPTOR_BRIEF:{

        const c_feature2d_brief::options &opts =
            obj->options_.descriptor.brief;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }

      #endif
      #if HAVE_FEATURE2D_LUCID
      case SPARSE_FEATURE_DESCRIPTOR_LUCID:{

        const c_feature2d_lucid::options &opts =
            obj->options_.descriptor.lucid;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }

      #endif
      #if HAVE_FEATURE2D_LATCH
      case SPARSE_FEATURE_DESCRIPTOR_LATCH:{

        const c_feature2d_latch::options &opts =
            obj->options_.descriptor.latch;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }
      #endif
      #if HAVE_FEATURE2D_DAISY
      case SPARSE_FEATURE_DESCRIPTOR_DAISY:{

        const c_feature2d_daisy::options &opts =
            obj->options_.descriptor.daisy;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }

      #endif
      #if HAVE_FEATURE2D_VGG
      case SPARSE_FEATURE_DESCRIPTOR_VGG:{

        const c_feature2d_vgg::options &opts =
            obj->options_.descriptor.vgg;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }
      #endif
      #if HAVE_FEATURE2D_BOOST
      case SPARSE_FEATURE_DESCRIPTOR_BOOST:{

        const c_feature2d_boost::options &opts =
            obj->options_.descriptor.boost;

        obj->options_.matcher.type = FEATURE2D_MATCHER_FLANN;
        //obj->options_.matcher.flann.index.type = FlannIndex_kdtree;
        break;
      }
      #endif
      #if HAVE_TRIANGLE_EXTRACTOR
      case SPARSE_FEATURE_DESCRIPTOR_TRIANGLE:{

        const c_feature2d_triangle_extractor::options &opts =
            obj->options_.descriptor.triangles;

        obj->options_.matcher.type = FEATURE2D_MATCHER_TRIANGLES;
        break;
      }

      #endif
      default:
        break;
    }
  }

  switch ( obj->options_.matcher.type ) {
    case FEATURE2D_MATCHER_OptFlowPyrLK:
      break;
    default:
      if( !(obj->matcher_ = create_sparse_feature_matcher(obj->options_.matcher)) ) {
        CF_ERROR("create_sparse_feature_matcher(type='%s') fails", toCString(obj->options_.matcher.type));
        return nullptr;
      }
      break;
  }

  CF_DEBUG("c_sparse_feature_extractor_and_matcher: \n"
      "detector='%s' descriptor='%s' matcher='%s'",
      toCString(obj->detector_->type()),
      obj->descriptor_ ? toCString(obj->descriptor_->type()) : "null",
      obj->matcher_ ? toCString(obj->options_.matcher.type) : "null");

  return obj;
}

void c_sparse_feature_extractor_and_matcher::extract_positions(const std::vector<cv::KeyPoint> & keypoints,
    std::vector<cv::Point2f> & positions) const
{
  positions.clear();
  positions.reserve(keypoints.size());

  for( const cv::KeyPoint &p : keypoints ) {
    positions.emplace_back(p.pt);
  }
}


bool c_sparse_feature_extractor_and_matcher::setup_reference_frame(cv::InputArray image, cv::InputArray mask)
{
  if( !matcher_ ) {
    reference_descriptors_.release();
    image.copyTo(reference_image_);
    mask.copyTo(reference_mask_);
    detector_->detect(image, reference_keypoints_, mask);
    extract_positions(reference_keypoints_, reference_positions_);
  }
  else {
    if( !descriptor_ ) {
      detector_->detectAndCompute(image, mask, reference_keypoints_, reference_descriptors_);
    }
    else {
      detector_->detect(image, reference_keypoints_, mask);
      descriptor_->compute(image, reference_keypoints_, reference_descriptors_);
    }
    matcher_->train(&reference_keypoints_, reference_descriptors_);
    reference_positions_.clear();
  }

  return true;
}

bool c_sparse_feature_extractor_and_matcher::match_current_frame(cv::InputArray current_image,
    cv::InputArray current_mask)
{
  bool fOk = false;

  current_matches_.clear();
  matched_reference_positions_.clear();
  matched_current_positions_.clear();

  try {

    if( matcher_ ) {

      if( !descriptor_ ) {
        detector_->detectAndCompute(current_image, current_mask, current_keypoints_, current_descriptors_);
      }
      else {
        detector_->detect(current_image, current_keypoints_, current_mask);
        descriptor_->compute(current_image, current_keypoints_, current_descriptors_);
      }

      if( !matcher_->match(&current_keypoints_, current_descriptors_, current_matches_) ) {
        CF_ERROR("matcher_->match() fails");
        return false;
      }

      for( const cv::DMatch & m : current_matches_ ) {
        matched_reference_positions_.emplace_back(reference_keypoints_[m.trainIdx].pt);
        matched_current_positions_.emplace_back(current_keypoints_[m.queryIdx].pt);
      }

      CF_DEBUG("current_descriptors: rows=%d cols=%d depth=%d channels=%d",
          current_descriptors_.rows,
          current_descriptors_.cols,
          current_descriptors_.depth(),
          current_descriptors_.channels());


    }
    else {

      std::vector<uint8_t> status;
      std::vector<float> err;

      const c_optflowpyrlk_feature2d_matcher_options & opts =
          options_.matcher.optflowpyrlk;

      cv::calcOpticalFlowPyrLK(reference_image_, current_image,
          reference_positions_, current_positions_,
          status, err,
          opts.winSize,
          opts.maxLevel,
          cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
              opts.maxIterations,
              opts.eps),
          opts.flags,
          opts.minEigThreshold);

      for( int i = 0, n = reference_positions_.size(); i < n; ++i ) {
        if( status[i] && (opts.maxErr <= 0 || err[i] < opts.maxErr) ) {

          matched_reference_positions_.emplace_back(reference_positions_[i]);
          matched_current_positions_.emplace_back(current_positions_[i]);
        }
      }
    }

    fOk = true;

  }

  catch( const cv::Exception & e ) {

    CF_ERROR("OpenCV Exception caught in %s():\n"
        "%s\n"
        "%s() : %d\n"
        "file : %s\n",
        __func__,
        e.err.c_str(), ///< error description
        e.func.c_str(),///< function name. Available only when the compiler supports getting it
        e.line,///< line number in the source file where the error has occurred
        e.file.c_str()///< source file name where the error has occurred
        );
  }
  catch( const std::exception & e ) {
    CF_ERROR("std::exception caught in %s(): %s\n", __func__, e.what());
  }
  catch( ... ) {
    CF_ERROR("Unknown exception caught in %s()\n", __func__);
  }

  CF_DEBUG("matched keypoints: %zu", matched_reference_positions_.size());

  return fOk;
}

void c_sparse_feature_extractor_and_matcher::detect(cv::InputArray image,
    CV_OUT std::vector<cv::KeyPoint> & keypoints,
    cv::InputArray mask) const
{
  detector_->detect(image, keypoints, mask);
}

void c_sparse_feature_extractor_and_matcher::detectAndCompute(cv::InputArray image, cv::InputArray mask,
    CV_OUT std::vector<cv::KeyPoint>& keypoints,
    cv::OutputArray descriptors,
    bool useProvidedKeypoints)
{
  if ( !descriptor_ )  {
    detector_->detectAndCompute(image, mask, keypoints, descriptors, useProvidedKeypoints);
  }
  else {
    detector_->detect(image, keypoints, mask);
    descriptor_->compute(image, keypoints, descriptors);
  }
}


/////////////////



/**
 * save_matched_points_in_octave_format()
 *
 * Dump matched keypoints (inliers only) into specified text file in octave format.
 * If image_size is not empty then all keypoimnts will shifted relative to image center as asked iuspeniev.
 */
bool save_matched_points_in_octave_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & matched_reference_keypoints,
    const std::vector<cv::Point2f> & matched_current_keypoints,
    const cv::InputArray inliers_mask,
    const cv::Size & image_size)
{

  // check input arguments
  if ( matched_reference_keypoints.size() != matched_current_keypoints.size() ) {
    CF_ERROR("Invalid args: matched_reference_keypoints.size()=%zu not equal to matched_current_keypoints.size()=%zu",
        matched_reference_keypoints.size(), matched_current_keypoints.size());
    return false;
  }

  if ( !inliers_mask.empty() )  {

    if ( inliers_mask.rows() != matched_reference_keypoints.size() ) {
      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%zu",
          inliers_mask.rows(), matched_reference_keypoints.size());
      return false;
    }

    if ( inliers_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid args: incorrect inliers_mask.type()=%d. Must be CV_8UC1",
          inliers_mask.type());
      return false;
    }
  }

  const cv::Mat1b M =
      inliers_mask.getMat();

  const cv::Point C = image_size.empty() ?
      cv::Point(0, 0) :
      cv::Point(image_size.width / 2, image_size.height / 2);

  const int N = M.empty() ?
      matched_reference_keypoints.size() :
      cv::countNonZero(M);

  CF_DEBUG("Num inliers = %d / %d", N, M.rows);

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen(%s) fails: %s", output_file_name.c_str(), strerror(errno));
    return false;
  }

  fprintf(fp, "reference_keypoints=[\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      fprintf(fp, "%+15.3f", r.x);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      if ( image_size.empty() ) {
        fprintf(fp, "%+15.3f", r.y);
      }
      else {
        fprintf(fp, "%+15.3f", C.y - r.y);
      }
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_reference_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & r = matched_reference_keypoints[i];
      fprintf(fp, "%+15.3f", 1.);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, "\n];\n");

  fprintf(fp, "current_keypoints=[\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      fprintf(fp, "%+15.3f", c.x);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      if ( image_size.empty() ) {
        fprintf(fp, "%+15.3f", c.y);
      }
      else {
        fprintf(fp, "%+15.3f", C.y - c.y);
      }
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, ";\n");

  for ( uint i = 0, n = matched_current_keypoints.size(); i < n; ++i ) {
    if ( M.empty() || M[i][0] ) {
      const cv::Point2f & c = matched_current_keypoints[i];
      fprintf(fp, "%+15.3f", 1.);
      if ( i < n - 1 ) {
        fprintf(fp, ",");
      }
    }
  }
  fprintf(fp, "\n];\n");

  fclose(fp);

  return true;
}


/**
 * save_matches_in_csv_format()
 *
 * Dump matched keypoints into specified text file in csv format.
 */
bool save_matches_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::DMatch> & matches,
    const std::vector<cv::KeyPoint> & query_keypoints,
    const std::vector<cv::KeyPoint> & train_keypoints,
    cv::InputArray mask)
{
  cv::Mat1b M;

  if ( !mask.empty() ) {

    if ( mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type: %d. Must be CV_8UC1",
          mask.type());
      return false;
    }

    if ( mask.rows() != matches.size() ) {
      CF_ERROR("Invalid mask size: %dx1. Must be %zux1",
          mask.rows(), matches.size());
      return false;
    }

    M = mask.getMat();
  }


  const std::string filepath =
      get_parent_directory(output_file_name);

  if ( !filepath.empty() && !create_path(filepath) ) {

    CF_ERROR("create_path('%s') fails: %s",
        filepath.c_str(),
        strerror(errno));

    return false;
  }

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {

    CF_ERROR("fopen(%s) fails: %s",
        output_file_name.c_str(),
        strerror(errno));

    return false;
  }


  fprintf(fp, "I\ttx\tty\ttr\ttl\tqx\tqy\tqr\tql\tdistance\n");


  for ( uint i = 0, n = matches.size(); i < n; ++i ) {

    if ( M.empty() || M[i][0] ) {

      const cv::DMatch & m =
          matches[i];

      const cv::KeyPoint & pt =
          train_keypoints[m.trainIdx];

      const cv::KeyPoint & pq =
          query_keypoints[m.queryIdx];

      fprintf(fp, "%6u"
          "\t%12.3f\t%12.3f\t%g\t%d"
          "\t%12.3f\t%12.3f\t%g\t%d"
          "\t%g"
          "\n",
          i,
          pt.pt.x, pt.pt.y, pt.response, pt.octave,
          pq.pt.x, pq.pt.y, pq.response, pq.octave,
          m.distance
          );
    }
  }


  fclose(fp);

  return true;
}

/**
 * save_matched_positions_in_csv_format()
 *
 * Utility routine to dump matched positions into text file in CSV format.
 *
 */
bool save_matched_positions_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & query_keypoints,
    const std::vector<cv::Point2f> & train_keypoints,
    cv::InputArray mask)
{
  cv::Mat1b M;

  if ( query_keypoints.size() != train_keypoints.size() ) {
    CF_ERROR("query_keypoints.size=%zu not equal to train_keypoints.size=%zu",
        query_keypoints.size(), train_keypoints.size());
    return false;
  }

  if ( !mask.empty() ) {

    if ( mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type: %d. Must be CV_8UC1. Mask ignored",
          mask.type());
    }
    else if ( mask.rows() != query_keypoints.size() ) {
      CF_ERROR("Invalid mask size: %dx%d. Must be %zux1. Mask ignored",
          mask.rows(), mask.cols(),
          query_keypoints.size());
    }
    else {
      M = mask.getMat();
    }
  }

  const std::string filepath =
      get_parent_directory(output_file_name);

  if ( !filepath.empty() && !create_path(filepath) ) {
    CF_ERROR("create_path('%s') fails: %s", filepath.c_str(), strerror(errno));
    return false;
  }

  FILE * fp = fopen(output_file_name.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen(%s) fails: %s", output_file_name.c_str(),
        strerror(errno));
    return false;
  }

  fprintf(fp, "I\ttx\tty\tqx\tqy\n");



  for ( uint i = 0, n = query_keypoints.size(); i < n; ++i ) {

    if ( M.empty() || M[i][0] ) {

      const cv::Point2f & pt =
          train_keypoints[i];

      const cv::Point2f & pq =
          query_keypoints[i];

      fprintf(fp, "%6u"
          "\t%12.3f\t%12.3f"
          "\t%12.3f\t%12.3f"
          "\n",
          i,
          pt.x, pt.y,
          pq.x, pq.y
          );
    }
  }


  fclose(fp);

  return true;
}

bool draw_matched_positions(cv::OutputArray _all_matches_image,
    cv::OutputArray _selected_matches_image,
    cv::InputArray _current_image,
    cv::InputArray _reference_image,
    const std::vector<cv::Point2f> & current_positions,
    const std::vector<cv::Point2f> & reference_positions,
    cv::InputArray _mask)
{
  INSTRUMENT_REGION("");


  if ( !_all_matches_image.needed() && !_selected_matches_image.needed() ) {
    return true;
  }

  if ( current_positions.size() != reference_positions.size() ) {
    CF_ERROR("current_positions.size()=%zu not match to reference_positions.size()=%zu",
        current_positions.size(), reference_positions.size());
    return false;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("Empty current image specified");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("Empty reference image specified");
    return false;
  }

  if ( !_mask.empty() ) {

    if ( _mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type %d: must be CV_8UC1",
          _mask.type());
      return false;
    }

    if ( _mask.rows() != (int) current_positions.size() ) {
      CF_ERROR("Invalid mask size %dx%d: mask must be of CV_8UC1 type and %dx%d of the size",
          _mask.rows(), _mask.cols(), (int )current_positions.size(), 1);
      return false;
    }
  }



  cv::Mat current_image, reference_image;

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, current_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    current_image =
        _current_image.getMat();
  }

  if ( _reference_image.channels() == 1 ) {
    cv::cvtColor(_reference_image, reference_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    reference_image =
        _reference_image.getMat();
  }

  const cv::Mat1b mask =
      _mask.getMat();


  cv::Size current_image_size =
      current_image.size();

  cv::Size reference_image_size =
      reference_image.size();

  cv::Size dst_size;

  enum {
    LAYOUT_HORZ,
    LAYOUT_VERT,
  } layout_type;


  if ( current_image_size.width + reference_image_size.width >= current_image_size.height + reference_image_size.height ) {
    dst_size.width = std::max(current_image_size.width, reference_image_size.width);
    dst_size.height = current_image_size.height + reference_image_size.height;
    layout_type = LAYOUT_VERT;
  }
  else {
    dst_size.width = current_image_size.width + reference_image_size.width;
    dst_size.height = std::max(current_image_size.height, reference_image_size.height);
    layout_type = LAYOUT_HORZ;
  }


  cv::Mat all_matches_image;
  cv::Mat selected_matches_image;

  if ( _all_matches_image.needed() ) {

    _all_matches_image.create(
        dst_size,
        CV_8UC3);

    all_matches_image =
        _all_matches_image.getMatRef();

    current_image.copyTo(all_matches_image(cv::Rect(0, 0,
        current_image_size.width,
        current_image_size.height)));
  }

  if ( _selected_matches_image.needed() ) {

    _selected_matches_image.create(
        dst_size,
        CV_8UC3);

    selected_matches_image =
        _selected_matches_image.getMatRef();

    current_image.copyTo(selected_matches_image(cv::Rect(0, 0,
            current_image_size.width,
            current_image_size.height)));
  }


  switch ( layout_type ) {
  case LAYOUT_VERT :


    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = current_positions.size(); i < n; ++i ) {

      const cv::Point2f & cp =
          current_positions[i];

      const cv::Point2f rp =
          cv::Point2f(reference_positions[i].x,
              reference_positions[i].y + current_image_size.height);

      const cv::Scalar c(
          rand() % 255,
          rand() % 255,
          rand() % 255);

      if ( !all_matches_image.empty() ) {
        cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
      }

      if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
        cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
      }
    }

    break;

  case LAYOUT_HORZ :

    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = current_positions.size(); i < n; ++i ) {

      const cv::Point2f & cp =
          current_positions[i];

      const cv::Point2f rp =
          cv::Point2f(reference_positions[i].x + current_image_size.width,
              reference_positions[i].y);

      const cv::Scalar c(
          rand() % 255,
          rand() % 255,
          rand() % 255);

      if ( !all_matches_image.empty() ) {
        cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
      }

      if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
        cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
        cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
        cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
      }
    }
    break;
  }


  return true;
}


/**
 * draw_matched_keyppints()
 *
 * Utility routine to draw sparse feature matches.
 *
 * Similar to cv::drawMatches()
 *
 */
bool draw_matched_keyppints(cv::OutputArray _all_matches_image,
    cv::OutputArray _selected_matches_image,
    cv::InputArray _current_image,
    cv::InputArray _reference_image,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch> & matches,
    cv::InputArray _mask)
{
  INSTRUMENT_REGION("");


  if ( !_all_matches_image.needed() && !_selected_matches_image.needed() ) {
    return true;
  }

  if ( _current_image.empty() ) {
    CF_ERROR("Empty current image specified");
    return false;
  }

  if ( _reference_image.empty() ) {
    CF_ERROR("Empty reference image specified");
    return false;
  }

  if ( !_mask.empty() ) {

    if ( _mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid mask type %d: must be CV_8UC1",
          _mask.type());
      return false;
    }

    if ( _mask.rows() != (int) matches.size() ) {
      CF_ERROR("Invalid mask size %dx%d: mask must be of CV_8UC1 type and %dx%d of the size",
          _mask.rows(), _mask.cols(), (int )matches.size(), 1);
      return false;
    }
  }



  cv::Mat current_image, reference_image;

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, current_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    current_image =
        _current_image.getMat();
  }

  if ( _reference_image.channels() == 1 ) {
    cv::cvtColor(_reference_image, reference_image,
        cv::COLOR_GRAY2BGR);
  }
  else {
    reference_image =
        _reference_image.getMat();
  }

  const cv::Mat1b mask =
      _mask.getMat();


  cv::Size current_image_size =
      current_image.size();

  cv::Size reference_image_size =
      reference_image.size();

  cv::Size dst_size;

  enum {
    LAYOUT_HORZ,
    LAYOUT_VERT,
  } layout_type;


  if ( current_image_size.width + reference_image_size.width >= current_image_size.height + reference_image_size.height ) {
    dst_size.width = std::max(current_image_size.width, reference_image_size.width);
    dst_size.height = current_image_size.height + reference_image_size.height;
    layout_type = LAYOUT_VERT;
  }
  else {
    dst_size.width = current_image_size.width + reference_image_size.width;
    dst_size.height = std::max(current_image_size.height, reference_image_size.height);
    layout_type = LAYOUT_HORZ;
  }


  cv::Mat all_matches_image;
  cv::Mat selected_matches_image;

  if ( _all_matches_image.needed() ) {

    _all_matches_image.create(
        dst_size,
        CV_8UC3);

    all_matches_image =
        _all_matches_image.getMatRef();

    current_image.copyTo(all_matches_image(cv::Rect(0, 0,
        current_image_size.width,
        current_image_size.height)));
  }

  if ( _selected_matches_image.needed() ) {

    _selected_matches_image.create(
        dst_size,
        CV_8UC3);

    selected_matches_image =
        _selected_matches_image.getMatRef();

    current_image.copyTo(selected_matches_image(cv::Rect(0, 0,
            current_image_size.width,
            current_image_size.height)));
  }


  switch ( layout_type ) {
  case LAYOUT_VERT :


    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(0, current_image_size.height,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = matches.size(); i < n; ++i ) {
      if ( mask.empty() || mask[i][0] ) {

        const cv::DMatch & m =
            matches[i];

        const cv::Point2f & cp =
            current_keypoints[m. queryIdx].pt;

        const cv::Point2f rp =
            cv::Point2f(reference_keypoints[m.trainIdx].pt.x,
                reference_keypoints[m.trainIdx].pt.y + current_image_size.height);

        const cv::Scalar c(
            rand() % 255,
            rand() % 255,
            rand() % 255);

        if ( !all_matches_image.empty() ) {
          cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
        }

        if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
          cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
        }
      }
    }

    break;

  case LAYOUT_HORZ :

    if ( !all_matches_image.empty() ) {
      reference_image.copyTo(all_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    if ( !selected_matches_image.empty() ) {
      reference_image.copyTo(selected_matches_image(cv::Rect(current_image_size.height, 0,
          reference_image_size.width,
          reference_image_size.height)));
    }

    for ( uint i = 0, n = matches.size(); i < n; ++i ) {
      if ( mask.empty() || mask[i][0] ) {

        const cv::DMatch & m =
            matches[i];

        const cv::Point2f & cp =
            current_keypoints[m. queryIdx].pt;

        const cv::Point2f rp =
            cv::Point2f(reference_keypoints[m.trainIdx].pt.x + current_image_size.width,
                reference_keypoints[m.trainIdx].pt.y);

        const cv::Scalar c(
            rand() % 255,
            rand() % 255,
            rand() % 255);

        if ( !all_matches_image.empty() ) {
          cv::circle(all_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(all_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(all_matches_image, cp, rp, c, 1, cv::LINE_8);
        }

        if ( !selected_matches_image.empty() && (mask.empty() || mask[i][0]) ) {
          cv::circle(selected_matches_image, cp, 1, c, 1, cv::LINE_8);
          cv::circle(selected_matches_image, rp, 1, c, 1, cv::LINE_8);
          cv::line(selected_matches_image, cp, rp, c, 1, cv::LINE_8);
        }
      }
    }
    break;
  }


  return true;

}
