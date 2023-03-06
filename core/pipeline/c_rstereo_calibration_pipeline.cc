/*
 * c_rstereo_calibration_pipeline.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#include "c_rstereo_calibration_pipeline.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/debug.h>

c_rstereo_calibration_pipeline::c_rstereo_calibration_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
  feature2d_options_.sparse_feature_extractor.detector.type = SPARSE_FEATURE_DETECTOR_AKAZE;
  feature2d_options_.sparse_feature_extractor.descriptor.type = SPARSE_FEATURE_DESCRIPTOR_AKAZE;
  feature2d_options_.sparse_feature_matcher.type = FEATURE2D_MATCHER_AUTO_SELECT;
}

c_rstereo_calibration_pipeline::~c_rstereo_calibration_pipeline()
{
  cancel();
}

c_rstereo_calibration_input_options & c_rstereo_calibration_pipeline::input_options()
{
  return input_options_;
}

const c_rstereo_calibration_input_options & c_rstereo_calibration_pipeline::input_options() const
{
  return input_options_;
}

c_rstereo_feature2d_options & c_rstereo_calibration_pipeline::feature2d_options()
{
  return feature2d_options_;
}

const c_rstereo_feature2d_options & c_rstereo_calibration_pipeline::feature2d_options() const
{
  return feature2d_options_;
}

c_rstereo_calibrate_options & c_rstereo_calibration_pipeline::stereo_calibrate_options()
{
  return calibration_options_;
}

const c_rstereo_calibrate_options & c_rstereo_calibration_pipeline::stereo_calibrate_options() const
{
  return calibration_options_;
}

c_rstereo_calibration_output_options & c_rstereo_calibration_pipeline::output_options()
{
  return output_options_;
}

const c_rstereo_calibration_output_options & c_rstereo_calibration_pipeline::output_options() const
{
  return output_options_;
}

bool c_rstereo_calibration_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }


  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, input_options_, left_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, right_stereo_source);
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {
    SERIALIZE_OPTION(section, save, feature2d_options_, scale);
    SERIALIZE_OPTION(section, save, feature2d_options_, sparse_feature_extractor);
    SERIALIZE_OPTION(section, save, feature2d_options_, sparse_feature_matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, calibration_options_, min_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, max_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, save_rectified_images);
    SERIALIZE_OPTION(section, save, output_options_, rectified_images_file_name);
  }

  return true;
}


bool c_rstereo_calibration_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);
  display_frame_.copyTo(frame);
  mask.release();
  return true;
}

void c_rstereo_calibration_pipeline::update_output_path()
{
  if( output_directory_.empty() ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/rstereo",
            parent_directory.c_str());

  }
  else if( !is_absolute_path(output_directory_) ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            output_directory_.c_str());
  }
  else {
    output_path_ =
        output_directory_;
  }

}

bool c_rstereo_calibration_pipeline::open_input_streams()
{
  for ( int i = 0; i < 2; ++i ) {
    if ( !input_sources_[i]->open() ) {
      CF_ERROR("ERROR: can not open input source '%s'",
          input_sources_[i]->cfilename());
      return false;
    }
  }

  if( input_sources_[0]->size() != input_sources_[1]->size() ) {
    CF_ERROR("ERROR: input sources sizes not match: left size=%d right size=%d ",
        input_sources_[0]->size(),
        input_sources_[1]->size());
    return false;
  }

  const int start_pos =
      std::max(input_options_.start_frame_index, 0);

  const int end_pos =
      input_options_.max_input_frames < 1 ?
          input_sources_[0]->size() :
          std::min(input_sources_[0]->size(),
              input_options_.start_frame_index + input_options_.max_input_frames);


  total_frames_ = end_pos - start_pos;
  processed_frames_ = 0;
  accumulated_frames_ = 0;

  if( total_frames_ < 1 ) {
    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
        total_frames_);
    return false;
  }


  for( int i = 0; i < 2; ++i ) {
    if( !input_sources_[i]->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sources_[%d]->seek(start_pos=%d) fails", i, start_pos);
      return false;
    }
  }

  return true;
}

bool c_rstereo_calibration_pipeline::read_stereo_frame()
{
  for( int i = 0; i < 2; ++i ) {

    if( canceled() ) {
      CF_ERROR("canceled");
      return false;
    }

    if( !read_input_frame(input_sources_[i], current_frame_.images[i], current_frame_.masks[i]) ) {
      CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
      return false;
    }
  }

  return true;
}

bool c_rstereo_calibration_pipeline::detect_and_match_keypoints()
{
  for( int i = 0; i < 2; ++i ) {
    current_frame_.keypoints[i].clear();
    current_frame_.descriptors[i].release();
    current_frame_.matched_positions[i].clear();
  }

  current_frame_.matches.clear();

  //current_frame_.matched_inliers.clear();


  //
  // Extract keypoints
  //

  for( int i = 0; i < 2; ++i ) {

    if( canceled() ) {
      CF_ERROR("canceled");
      return false;
    }

    bool fOK =
        keypoints_extractor_->detectAndCompute(current_frame_.images[i],
            current_frame_.masks[i],
            current_frame_.keypoints[i],
            current_frame_.descriptors[i]);

    if( !fOK ) {
      CF_ERROR("keypoints_extractor_->detectAndCompute() fails");
      return false;
    }
  }

  if( canceled() ) {
    CF_ERROR("canceled");
    return false;
  }

  //
  // Match keypoints
  //

  // from left image
  const std::vector<cv::KeyPoint> &reference_keypoints =
      current_frame_.keypoints[0];

  const cv::Mat &reference_descriptors =
      current_frame_.descriptors[0];

  std::vector<cv::Point2f> &reference_positions =
      current_frame_.matched_positions[0];

  // from right image
  const std::vector<cv::KeyPoint> &current_keypoints =
      current_frame_.keypoints[1];

  const cv::Mat &current_descriptors =
      current_frame_.descriptors[1];

  std::vector<cv::Point2f> &current_positions =
      current_frame_.matched_positions[1];

  if( current_frame_.keypoints[0].empty() || current_frame_.keypoints[1].empty() ) {
    CF_ERROR("No keypoints to match on current stereo pair");
  }
  else {

    if( canceled() ) {
      CF_ERROR("canceled");
      return false;
    }

    // match sparse stereo keypoints
    const size_t num_matches =
        match_keypoints(keypoints_matcher_,
            current_keypoints,
            current_descriptors,
            reference_keypoints,
            reference_descriptors,
            &current_frame_.matches,
            &current_positions,
            &reference_positions);

    if( !num_matches ) {
      CF_ERROR("match_keypoints() fails, no strereo matches found");
    }
  }

  return true;
}

bool c_rstereo_calibration_pipeline::read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const
{
  lock_guard lock(display_lock_);

  INSTRUMENT_REGION("");

  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;


  if ( !source->read(output_image, &colorid, &bpp) ) {
    CF_FATAL("source->read() fails\n");
    return false;
  }

  if ( is_bayer_pattern(colorid) ) {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, colorid, algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, colorid) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << bpp)));

        if ( !nninterpolation(output_image, output_image, colorid) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toString(algo));
        return false;
    }
  }
  else if ( colorid == COLORID_OPTFLOW || (output_image.channels() != 4 && output_image.channels() != 2) ) {
    output_mask.release();
  }
  else if( !splitbgra(output_image, output_image, &output_mask) ) {
    output_mask.release();
    return false;
  }

  if( input_options_.enable_color_maxtrix && source->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        source->color_matrix());
  }

//  if ( anscombe_.method() != anscombe_none ) {
//    anscombe_.apply(output_image, output_image);
//  }

  if ( !missing_pixel_mask_.empty() ) {

    if ( output_image.size() != missing_pixel_mask_.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          missing_pixel_mask_.cols, missing_pixel_mask_.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      missing_pixel_mask_.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, missing_pixel_mask_,
          output_mask);
    }
  }

  if ( !output_mask.empty() && input_options_.inpaint_missing_pixels ) {
    linear_interpolation_inpaint(output_image, output_mask, output_image);
  }

  return true;
}

void c_rstereo_calibration_pipeline::update_remap()
{
}
//
//void c_rstereo_calibration_pipeline::update_display_image(bool drawpoints)
//{
//  if ( !current_frame_.images[0].empty() ) {
//
//    lock_guard lock(display_lock_);
//
//    const cv::Size sizes[2] =  {
//        current_frame_.images[0].size(),
//        current_frame_.images[1].size(),
//    };
//
//    const cv::Size size(sizes[0].width + sizes[1].width,
//        std::max(sizes[0].height, sizes[1].height));
//
//    CF_DEBUG("size: %dx%d", size.width, size.height);
//
//    const cv::Rect rc0(0,0, sizes[0].width, sizes[0].height);
//    const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height);
//
//    display_frame_.create(size, current_frame_.images[0].type());
//
//    current_frame_.images[0].copyTo(display_frame_(rc0));
//    // current_frame_.images[1].copyTo(display_frame_(rc1));
//
//    //
//    // Apply derotation homography to current frame
//    //
//
//    cv::warpPerspective(current_frame_.images[1], display_frame_(rc1),
//        currentDerotationHomography,
//        rc1.size(),
//        cv::INTER_LINEAR, // cv::INTER_LINEAR introduces blur on kitti images
//        cv::BORDER_CONSTANT);
//
//
//    if( display_frame_.channels() == 1 ) {
//      cv::cvtColor(display_frame_, display_frame_, cv::COLOR_GRAY2BGR);
//    }
//
//    if( drawpoints ) {
//
//      cv::Mat3b display = display_frame_;
//
//      std::vector<cv::Point2f> warped_matched_positions;
//
//      CF_DEBUG("matched_positions: %zu -> %zu", current_frame_.matched_positions[0].size(),
//          current_frame_.matched_positions[1].size());
//
//      const int nvecs = matched_positions[0].size();
//
//      if( nvecs > 0 ) {
//
//        for( int i = 0; i < nvecs; ++i ) {
//
//          cv::perspectiveTransform(matched_positions[1][i],
//              warped_matched_positions,
//              currentDerotationHomography);
//
//          for( int j = 0, m = matched_positions[0][i].size(); j < m; ++j ) {
//
//            cv::Vec3b c(std::max(32, rand() % 255), std::max(32, rand() % 255), std::max(32, rand() % 255));
//
//            const cv::Point2f &p1 =
//                matched_positions[0][i][j];
//
////            const cv::Point2f &p2 =
////                matched_positions[1][i][j] + cv::Point2f(sizes[0].width, 0);
//            const cv::Point2f &p2 =
//                warped_matched_positions[j] + cv::Point2f(sizes[0].width, 0);
//
////            display[(int) (p1.y)][(int) (p1.x)] = c;
////            display[(int) (p2.y)][(int) (p2.x)] = c;
//            cv::circle(display, p1, 1, c, 1);
//            cv::circle(display, p2, 1, c, 1);
//
//
//          }
//        }
//      }
//    }
//  }
//
//  on_accumulator_changed();
//}

void c_rstereo_calibration_pipeline::update_display_image(bool drawpoints)
{
  if ( !current_frame_.images[0].empty() ) {

    lock_guard lock(display_lock_);

    const cv::Size sizes[2] =  {
      current_frame_.images[0].size(),
      current_frame_.images[1].size(),
    };

    const cv::Size size(sizes[0].width + sizes[1].width,
        sizes[0].height + sizes[1].height);


    cv::Mat tmp;

    if( !calibration_options_.enable_calibration ) {
      tmp = current_frame_.images[1];
    }
    else {

      // Apply derotation homography to current frame
      cv::warpPerspective(current_frame_.images[1], tmp,
          currentDerotationHomography,
          current_frame_.images[1].size(),
          cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
          cv::BORDER_CONSTANT);
    }


    const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height);      // tl image[0]
    const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height); // tr image[1]
    const cv::Rect rc2(0, sizes[0].height, sizes[1].width, sizes[1].height); // bl image[1]
    const cv::Rect rc3(sizes[0].width, sizes[0].height, sizes[1].width, sizes[1].height); // br blend


    display_frame_.create(size, current_frame_.images[0].type());
    current_frame_.images[0].copyTo(display_frame_(rc0));
    tmp.copyTo(display_frame_(rc1));
    tmp.copyTo(display_frame_(rc2));
    cv::addWeighted(current_frame_.images[0], 0.5, tmp, 0.5, 0, display_frame_(rc3));


    if( display_frame_.channels() == 1 ) {
      cv::cvtColor(display_frame_, display_frame_, cv::COLOR_GRAY2BGR);
    }

  }

  on_accumulator_changed();
}

double c_rstereo_calibration_pipeline::estimate_grid_subset_quality(int excludedIndex) const
{
  const cv::Size image_size = current_frame_.images[0].size();

  const int gridSize = 10;
  const int xGridStep = image_size.width / gridSize;
  const int yGridStep = image_size.height / gridSize;
  const int stride = gridSize * gridSize;

  std::vector<int> pointsInCell(2 * stride);

  std::fill(pointsInCell.begin(), pointsInCell.end(), 0);

  for( int k = 0; k < matched_positions[0].size(); k++ )
    if( k != excludedIndex ) {

      for( int i = 0; i < 2; ++i ) {

        for( const auto &p : matched_positions[i][k] ) {

          int ii = (int) (p.x / xGridStep);
          int jj = (int) (p.y / yGridStep);

          pointsInCell[ii * gridSize + jj]++;
          pointsInCell[ii * gridSize + jj + stride]++;
        }
      }

    }

  cv::Scalar mean, stdDev;
  cv::meanStdDev(pointsInCell, mean, stdDev);

  return mean[0] / (stdDev[0] + 1e-7);
}

void c_rstereo_calibration_pipeline::filter_frames()
{
  const int nbframes =
      matched_positions[0].size();

  const int nbframesmax =
      std::max(1, std::max(calibration_options_.min_frames,
          calibration_options_.max_frames));

  accumulated_frames_ = nbframes;

  if( nbframes > nbframesmax ) {

    double worstValue = -HUGE_VAL;
    double maxQuality = estimate_grid_subset_quality(nbframes);

    int worstElemIndex = 0;

    const double alpha =
        calibration_options_.filter_alpha;

    for( size_t i = 0; i < nbframes; i++ ) {

      const double gridQDelta =
          estimate_grid_subset_quality(i) - maxQuality;

      const double currentValue =
          perViewErrors_.empty() ? gridQDelta :
              perViewErrors_[i] * alpha + gridQDelta * (1. - alpha);

      if( currentValue > worstValue ) {
        worstValue = currentValue;
        worstElemIndex = i;
      }
    }

    CF_DEBUG("XXX worstElemIndex=%d worstValue=%g", worstElemIndex, worstValue);

    matched_positions[0].erase(matched_positions[0].begin() + worstElemIndex);
    matched_positions[1].erase(matched_positions[1].begin() + worstElemIndex);

    if ( !perViewErrors_.empty() ) {
      perViewErrors_.erase(perViewErrors_.begin() + worstElemIndex);
    }

  }

  CF_DEBUG("nbframes=%d / %d", nbframes, nbframesmax);
}

void c_rstereo_calibration_pipeline::update_state()
{
}

bool c_rstereo_calibration_pipeline::save_current_camera_parameters() const
{
  return false;
}

bool c_rstereo_calibration_pipeline::initialize_pipeline()
{
  set_pipeline_stage(rstereo_calibration_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  //calibration_flags_ = calibration_options_.calibration_flags;
  stereo_intrinsics_initialized_ = false;

  /////////////////////////////////////////////////////////////////////////////


  if ( input_options_.left_stereo_source.empty() ) {
    CF_ERROR("ERROR: No left stereo source specified");
    return false;
  }

  if ( input_options_.right_stereo_source.empty() ) {
    CF_ERROR("ERROR: No right stereo source specified");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  input_sources_[0] =
      input_sequence_->source(input_options_.left_stereo_source);

  if ( !input_sources_[0] ) {
    CF_ERROR("ERROR: requested left stereo source not found in input sequence: %s",
        input_options_.left_stereo_source.c_str());
    return false;
  }

  input_sources_[1] =
      input_sequence_->source(input_options_.right_stereo_source);

  if ( !input_sources_[1] ) {
    CF_ERROR("ERROR: requested right stereo source not found in input sequence: %s",
        input_options_.right_stereo_source.c_str());
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  if( !(keypoints_extractor_ = create_sparse_feature_extractor(feature2d_options_.sparse_feature_extractor)) ) {
    CF_ERROR("ERROR: create_sparse_feature_extractor() fails");
    return false;
  }

  if( !(keypoints_matcher_ = create_sparse_feature_matcher(feature2d_options_.sparse_feature_matcher)) ) {
    CF_ERROR("ERROR: create_sparse_feature_matcher() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////


  return true;
}

void c_rstereo_calibration_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(rstereo_calibration_finishing);

  for ( int i = 0; i < 2; ++i ) {

    if ( input_sources_[i] ) {
      input_sources_[i]->close();
      input_sources_[i].reset();
    }

    current_frame_.images[i].release();
    current_frame_.masks[i].release();
    current_frame_.keypoints[i].clear();
    current_frame_.descriptors[i].release();
    current_frame_.matched_positions[i].clear();

    matched_positions[i].clear();
  }

  current_frame_.matches.clear();
  perViewErrors_.clear();
  rmap_.release();
  display_frame_.release();
  display_mask_.release();


  if( true ) {
    lock_guard lock(display_lock_);
    display_frame_.release();
    display_mask_.release();
  }

  keypoints_extractor_.reset();
  keypoints_matcher_.reset();

  base::cleanup_pipeline();

  set_pipeline_stage(rstereo_calibration_idle);
}

bool c_rstereo_calibration_pipeline::run_calibration()
{
  c_video_writer progress_writer;

  // FIXME: Hardcoded Camera Matrix
  // Extracted from P_rect_00 of KITTI calib_cam_to_cam.txt
  // .image_size = cv::Size(1242, 375),
  const cv::Matx33d camera_matrix_ =
      cv::Matx33d(
          7.215377e+02, 0.000000e+00, 6.095593e+02,
          0.000000e+00, 7.215377e+02, 1.728540e+02,
          0.000000e+00, 0.000000e+00, 1.000000e+00);


  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  if ( !open_input_streams() ) {
    CF_ERROR("open_input_streams() fails");
    return false;
  }

  if( canceled() ) {
    CF_ERROR("canceled");
    return false;
  }

  set_pipeline_stage(rstereo_calibration_in_progress);
  set_status_msg("RUNNING ...");

  bool fOK;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

    if ( canceled() ) {
      break;
    }

    filter_frames();

    if ( canceled() ) {
      break;
    }

    if( !read_stereo_frame() ) {
      CF_ERROR("read_next_stereo_pair() fails");
      break;
    }

    if( !detect_and_match_keypoints() ) {
      CF_ERROR("detect_and_match_keypoints() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if( current_frame_.matched_positions[0].size() < 5 ) {
      CF_ERROR("Not enough matched positions: %zu", current_frame_.matched_positions[0].size());
    }
    else {

      cv::Mat1b inliers;

      fOK =
          rectify_stereo_pose(
              camera_matrix_,
              current_frame_.matched_positions[1],
              current_frame_.matched_positions[0],
              EMM_LMEDS,
              &currentEulerAnges,
              &currentTranslationVector,
              &currentRotationMatrix,
              &currentEssentialMatrix,
              &currentFundamentalMatrix,
              &currentDerotationHomography,
              inliers);

      if( !fOK ) {
        CF_ERROR("rectify_stereo_pose() fails");
      }
      else {

        cv::Mat1d rhs;

        compute_distances_from_points_to_corresponding_epipolar_lines(rhs,
            currentFundamentalMatrix,
            current_frame_.matched_positions[1],
            current_frame_.matched_positions[0]);

        const double rmse =
            sqrt(cv::mean(rhs.mul(rhs), inliers)[0]);

        perViewErrors_.emplace_back(rmse);
        for( int i = 0; i < 2; ++i ) {
          matched_positions[i].emplace_back(current_frame_.matched_positions[i]);
        }

        CF_DEBUG("RMSE = %g", rmse);

        //
        // Compute epipoles from fundamental matrix
        //

        cv::Point2d current_epipoles_[2]; // current epipoles location
        cv::Point2d current_epipole_; // current average epipole location

        compute_epipoles(currentFundamentalMatrix,
            current_epipoles_);

        CF_DEBUG("EPIPOLES: E0:(%+g %+g) E1:(%+g %+g)",
            current_epipoles_[0].x, current_epipoles_[0].y,
            current_epipoles_[1].x, current_epipoles_[1].y);


        current_epipole_ =
            0.5 * (current_epipoles_[0] + current_epipoles_[1]);

        const double distance =
            hypot(current_epipoles_[0].x - current_epipoles_[1].x, current_epipoles_[0].y - current_epipoles_[1].y);

        if( distance > 1 ) {
          CF_WARNING("\nWARNING!\n"
              "Something looks POOR: Computed epipoles differ after derotation:\n"
              "E0 = {%g %g}\n"
              "E1 = {%g %g}\n"
              "Eavg={%g %g}\n",
              current_epipoles_[0].x, current_epipoles_[0].y,
              current_epipoles_[1].x, current_epipoles_[1].y,
              current_epipole_.x, current_epipole_.y);
        }

      }
    }

    update_display_image();

    if( true ) {

      if( !progress_writer.is_open() ) {

        const std::string output_file_name =
            generate_video_file_name("",
                "progress",
                ".avi");

        fOK =
            progress_writer.open(output_file_name,
                display_frame_.size(),
                display_frame_.channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("progress_writer.open('%s') fails", output_file_name.c_str());
          return false;
        }
      }

      if( !progress_writer.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
        CF_ERROR("c_video_writer: write(fails)");
        return false;
      }
    }
  }

  for ( int i = 0; i < 2; ++i ) {
    input_sources_[i]->close();
  }

////////////
  if( !canceled() && matched_positions[0].size() > 0 /* std::max(1, calibration_options_.min_frames)*/ ) {

    std::vector<cv::Point2f> matched_points[2];
    for( int i = 0; i < 2; ++i ) {
      for( const std::vector<cv::Point2f> &v : matched_positions[i] ) {
        matched_points[i].insert(matched_points[i].end(), v.begin(), v.end());
      }
      CF_DEBUG("matched_points[%d].size=%zu", i, matched_points[i].size());
    }

    fOK =
        rectify_stereo_pose(
            camera_matrix_,
            matched_points[1],
            matched_points[0],
            EMM_LMEDS,
            &currentEulerAnges,
            &currentTranslationVector,
            &currentRotationMatrix,
            &currentEssentialMatrix,
            &currentFundamentalMatrix,
            &currentDerotationHomography,
            cv::noArray());

    if( !fOK ) {
      CF_ERROR("rectify_stereo_pose() fails");
    }
    else {
      //
      // Compute epipoles from fundamental matrix
      //

      cv::Point2d current_epipoles_[2]; // current epipoles location
      cv::Point2d current_epipole_; // current average epipole location

      compute_epipoles(currentFundamentalMatrix,
          current_epipoles_);

      CF_DEBUG("EPIPOLES: E0:(%+g %+g) E1:(%+g %+g)",
          current_epipoles_[0].x, current_epipoles_[0].y,
          current_epipoles_[1].x, current_epipoles_[1].y);

      current_epipole_ =
          0.5 * (current_epipoles_[0] + current_epipoles_[1]);

      const double distance =
          hypot(current_epipoles_[0].x - current_epipoles_[1].x, current_epipoles_[0].y - current_epipoles_[1].y);

      if( distance > 1 ) {
        CF_WARNING("\nWARNING!\n"
            "Something looks POOR: Computed epipoles differ after derotation:\n"
            "E0 = {%g %g}\n"
            "E1 = {%g %g}\n"
            "Eavg={%g %g}\n",
            current_epipoles_[0].x, current_epipoles_[0].y,
            current_epipoles_[1].x, current_epipoles_[1].y,
            current_epipole_.x, current_epipole_.y);
      }

    }
  }

///////////
  return !canceled();
}

bool c_rstereo_calibration_pipeline::run_pipeline()
{

  if ( calibration_options_.enable_calibration && !run_calibration() ) {
    return false;
  }


  //////////////////////////////////////////////////////
  if( !canceled() && output_options_.save_rectified_images ) {

    set_status_msg("WRITE RECTIFIED VIDEO ...");

    c_video_writer writer;
    bool fOK;

    const std::string output_file_name =
        generate_video_file_name(output_options_.rectified_images_file_name,
            "rectified",
            ".avi");


    for( int i = 0; i < 2; ++i ) {
      if( !input_sources_[i]->open() ) {
        CF_ERROR("ERROR: can not open input source '%s'", input_sources_[i]->cfilename());
        return false;
      }
    }

    CF_DEBUG("Saving %s...", output_file_name.c_str());

    total_frames_ = input_sources_[0]->size();
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

      if( canceled() ) {
        break;
      }

      fOK = true;

      for( int i = 0; i < 2; ++i ) {

        cv::Mat & image = current_frame_.images[i];
        cv::Mat & mask = current_frame_.masks[i];

        if( !read_input_frame(input_sources_[i], image, mask) ) {
          CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
          fOK = false;
          break;
        }

        if( canceled() ) {
          break;
        }
      }

      if( !fOK || canceled() ) {
        break;
      }

      update_display_image();

      if( canceled() ) {
        break;
      }

      if( !writer.is_open() ) {

        fOK =
            writer.open(output_file_name,
                display_frame_.size(),
                display_frame_.channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("writer.open('%s') fails", output_file_name.c_str());
          return false;
        }
      }

      if( !writer.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
        CF_ERROR("c_video_writer: write(fails)");
        break;
      }

      if( canceled() ) {
        break;
      }
    }
  }


  set_status_msg("FINISHED ...");

  return true;
}

