/*
 * c_regular_stereo_pipeline.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#include "c_regular_stereo_pipeline.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/pixtype.h>
#include <core/proc/colormap.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/debug.h>


#define CATCH(fn) \
  catch( const cv::Exception &e ) { \
    CF_ERROR("OpenCV exception %s in %s():\n" \
        "%s\n" \
        "%s() : %d\n" \
        "file : %s\n", \
        fn, \
        __func__, \
        e.err.c_str(), \
        e.func.c_str(), \
        e.line, \
        e.file.c_str() \
        ); \
  } \
  catch( const std::exception &e ) { \
    CF_ERROR("std::exception %s in %s(): %s\n", fn, __func__, e.what()); \
  } \
  catch( ... ) { \
    CF_ERROR("Unknown exception %s in %s()\n", fn, __func__); \
  }




c_regular_stereo_pipeline::c_regular_stereo_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
  feature2d_options_.sparse_feature_extractor.detector.type = SPARSE_FEATURE_DETECTOR_AKAZE;
  feature2d_options_.sparse_feature_extractor.descriptor.type = SPARSE_FEATURE_DESCRIPTOR_AKAZE;
  feature2d_options_.sparse_feature_matcher.type = FEATURE2D_MATCHER_AUTO_SELECT;
}

c_regular_stereo_pipeline::~c_regular_stereo_pipeline()
{
  cancel();
}

c_regular_stereo_input_options & c_regular_stereo_pipeline::input_options()
{
  return input_options_;
}

const c_regular_stereo_input_options & c_regular_stereo_pipeline::input_options() const
{
  return input_options_;
}

c_regular_stereo_feature2d_options & c_regular_stereo_pipeline::feature2d_options()
{
  return feature2d_options_;
}

const c_regular_stereo_feature2d_options & c_regular_stereo_pipeline::feature2d_options() const
{
  return feature2d_options_;
}

c_regular_stereo_calibratie_options & c_regular_stereo_pipeline::calibration_options()
{
  return calibration_options_;
}

const c_regular_stereo_calibratie_options & c_regular_stereo_pipeline::calibration_options() const
{
  return calibration_options_;
}

c_regular_stereo_matching_options & c_regular_stereo_pipeline::stereo_matching_options()
{
  return stereo_matching_options_;
}

const c_regular_stereo_matching_options & c_regular_stereo_pipeline::stereo_matching_options() const
{
  return stereo_matching_options_;
}

c_regular_stereo_image_processing_options & c_regular_stereo_pipeline::image_processing_options()
{
  return image_processing_options_;
}

const c_regular_stereo_image_processing_options & c_regular_stereo_pipeline::image_processing_options() const
{
  return image_processing_options_;
}

c_regular_stereo_output_options & c_regular_stereo_pipeline::output_options()
{
  return output_options_;
}

const c_regular_stereo_output_options & c_regular_stereo_pipeline::output_options() const
{
  return output_options_;
}

bool c_regular_stereo_pipeline::serialize(c_config_setting settings, bool save)
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
    SERIALIZE_OPTION(section, save, input_options_, convert_to_grayscale);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
    SERIALIZE_OPTION(section, save, input_options_, cameraMatrix);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {
    SERIALIZE_OPTION(section, save, feature2d_options_, scale);
    SERIALIZE_OPTION(section, save, feature2d_options_, sparse_feature_extractor);
    SERIALIZE_OPTION(section, save, feature2d_options_, sparse_feature_matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "calibration_options")) ) {
    SERIALIZE_OPTION(section, save, calibration_options_, enable_calibration);
    SERIALIZE_OPTION(section, save, calibration_options_, apply_stereo_rectification);
    SERIALIZE_OPTION(section, save, calibration_options_, calibration_config_filename);
    SERIALIZE_OPTION(section, save, calibration_options_, min_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, max_frames);
    SERIALIZE_OPTION(section, save, calibration_options_, filter_alpha);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matching_options")) ) {
    SERIALIZE_OPTION(section, save, stereo_matching_options_, enable_stereo_matchning);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, max_disparity);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, max_scale);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, kernel_radius);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, kernel_sigma);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, save_debug_images);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, process_only_debug_frames);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, debug_frames);
    SERIALIZE_OPTION(section, save, stereo_matching_options_, debug_points);
  }


  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {

    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_calibration_config_file);
    SERIALIZE_OPTION(section, save, output_options_, calibration_config_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_rectified_videos);
    SERIALIZE_OPTION(section, save, output_options_, left_rectified_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, right_rectified_video_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_stereo_matches_video);
    SERIALIZE_OPTION(section, save, output_options_, stereo_matches_video_filename);


    SERIALIZE_OPTION(section, save, output_options_, save_motion_poses);
    SERIALIZE_OPTION(section, save, output_options_, motion_poses_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_stereo_match_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, stereo_match_progress_video_filename);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(settings, save, image_processing_options_, input_image_processor);
    SERIALIZE_IMAGE_PROCESSOR(settings, save, image_processing_options_, stereo_match_preprocessor);
    SERIALIZE_IMAGE_PROCESSOR(settings, save, image_processing_options_, output_image_processor);
  }

  return true;
}


bool c_regular_stereo_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  lock_guard lock(display_lock_);

  //update_display_image(applyHomography, drawmatches, stream_pos)

  display_frame_.copyTo(frame);
  mask.release();
  return true;
}
//
//void c_regular_stereo_pipeline::update_output_path()
//{
//  if( output_directory_.empty() ) {
//
//    std::string parent_directory =
//        input_sequence_->sources().empty() ? "." :
//            get_parent_directory(input_sequence_->source(0)->filename());
//
//    if( parent_directory.empty() ) {
//      parent_directory = ".";
//    }
//
//    output_path_ =
//        ssprintf("%s/rstereo",
//            parent_directory.c_str());
//
//  }
//  else if( !is_absolute_path(output_directory_) ) {
//
//    std::string parent_directory =
//        input_sequence_->sources().empty() ? "." :
//            get_parent_directory(input_sequence_->source(0)->filename());
//
//    if( parent_directory.empty() ) {
//      parent_directory = ".";
//    }
//
//    output_path_ =
//        ssprintf("%s/%s",
//            parent_directory.c_str(),
//            output_directory_.c_str());
//  }
//  else {
//    output_path_ =
//        output_directory_;
//  }
//
//}

bool c_regular_stereo_pipeline::open_input_streams()
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

bool c_regular_stereo_pipeline::read_stereo_frame()
{
  for( int i = 0; i < 2; ++i ) {

    if( canceled() ) {
      CF_ERROR("canceled");
      return false;
    }

    if( !read_input_frame(input_sources_[i], current_frame_->images[i], current_frame_->masks[i]) ) {
      CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
      return false;
    }
  }

  if( current_frame_->images[0].size() != current_frame_->images[1].size() ) {
    CF_ERROR("INPUT ERROR: Left (%dx%d) and right (%dx%d) image sizes not equal.\n"
        "Different image sizes are not yet supported",
        current_frame_->images[0].cols, current_frame_->images[0].rows,
        current_frame_->images[1].cols, current_frame_->images[1].rows);
    return false;
  }

  if( current_frame_->images[0].channels() != current_frame_->images[1].channels() ) {
    CF_ERROR("INPUT ERROR: Left (%d) and right (%d) number of image channels not equal.\n"
        "Different image types are not yet supported",
        current_frame_->images[0].channels(),
        current_frame_->images[1].channels());
    return false;
  }

  if( intrinsics_.image_size.empty() ) {
    intrinsics_.image_size =
        current_frame_->images[0].size();
  }
  else if( intrinsics_.image_size != current_frame_->images[0].size() ) {
    CF_ERROR("INPUT ERROR: Frame size change (%dx%d) -> (%dx%d) not supported\n",
        intrinsics_.image_size.width, intrinsics_.image_size.height,
        current_frame_->images[0].cols, current_frame_->images[0].rows);
    return false;
  }

  if( image_processing_options_.input_image_processor ) {
    for( int i = 0; i < 2; ++i ) {
      cv::Mat &image = current_frame_->images[i];
      cv::Mat &mask = current_frame_->masks[i];

      if( !image_processing_options_.input_image_processor->process(image, mask) ) {
        CF_ERROR("ERROR: input_image_processor->process() fails for stereo frame %d", i);
        return false;
      }
    }
  }

  return true;
}

bool c_regular_stereo_pipeline::detect_keypoints()
{
  for( int i = 0; i < 2; ++i ) {
    current_frame_->keypoints[i].clear();
    current_frame_->descriptors[i].release();
    //current_frame_->matched_positions[i].clear();
  }

  //
  // Extract keypoints
  //

  for( int i = 0; i < 2; ++i ) {

    if( canceled() ) {
      CF_ERROR("canceled");
      return false;
    }

    bool fOK =
        keypoints_extractor_->detectAndCompute(current_frame_->images[i],
            current_frame_->masks[i],
            current_frame_->keypoints[i],
            current_frame_->descriptors[i]);

    if( !fOK ) {
      CF_ERROR("keypoints_extractor_->detectAndCompute() fails");
      return false;
    }

    CF_DEBUG("[camera=%d] detectAndCompute: %zu keypoints descriptor: %dx%d type=%d", i,
        current_frame_->keypoints[i].size(),
        current_frame_->descriptors[i].rows,
        current_frame_->descriptors[i].cols,
        current_frame_->descriptors[i].type());
  }

  if( canceled() ) {
    CF_ERROR("canceled");
    return false;
  }


  return true;
}

bool c_regular_stereo_pipeline::read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const
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

  if( input_options_.convert_to_grayscale && output_image.channels() != 1 ) {
    cv::cvtColor(output_image, output_image, cv::COLOR_BGR2GRAY);
  }

  return true;
}


void c_regular_stereo_pipeline::update_calibration_display_image(bool applyHomography, bool drawmatches, int stream_pos)
{
  lock_guard lock(display_lock_);

  if ( current_frame_ && !current_frame_->images[0].empty() ) {


    const cv::Size sizes[2] =  {
        current_frame_->images[0].size(),
        current_frame_->images[1].size(),
    };

    const cv::Size size(sizes[0].width + sizes[1].width,
        sizes[0].height + sizes[1].height);


    const cv::Rect rc0(0, 0, sizes[0].width, sizes[0].height);      // tl image[0]
    const cv::Rect rc1(sizes[0].width, 0, sizes[1].width, sizes[1].height); // tr image[1]
    const cv::Rect rc2(0, sizes[0].height, sizes[1].width, sizes[1].height); // bl image[1]
    const cv::Rect rc3(sizes[0].width, sizes[0].height, sizes[1].width, sizes[1].height); // br blend

    display_frame_.create(size, current_frame_->images[0].type());

    cv::Mat tmp[2];

    if( !applyHomography || !haveRectificationHomography ) {
      tmp[0] = current_frame_->images[0];
      tmp[1] = current_frame_->images[1];
    }
    else {

      for ( int i = 0; i < 2; ++i ) {
        cv::warpPerspective(current_frame_->images[i], tmp[i],
            rectificationHomography[i],
            current_frame_->images[i].size(),
            cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
            cv::BORDER_CONSTANT);
      }

    }

    tmp[0].copyTo(display_frame_(rc0));
    tmp[1].copyTo(display_frame_(rc1));
    tmp[1].copyTo(display_frame_(rc2));
    cv::addWeighted(tmp[0], 0.5, tmp[1], 0.5, 0, display_frame_(rc3));

    if( drawmatches && stream_pos >= 0 ) {

      const auto pos =
          std::find_if(motion_poses_.begin(), motion_poses_.end(),
              [stream_pos](const c_motion_pose & pose) {
                return pose.stream_pos == stream_pos;
              });

      if ( pos != motion_poses_.end() ) {

        const c_motion_pose & pose =
            *pos;

        const bool doHomography =
            applyHomography && haveRectificationHomography;

        if ( display_frame_.channels() != 3 ) {
          cv::cvtColor(display_frame_, display_frame_,
              cv::COLOR_GRAY2BGR);
        }

        for ( uint i = 0, n = pose.matched_positions[0].size(); i  < n; ++i ) {

          cv::Point2f start, end;

          if( !doHomography ) {
            start = pose.matched_positions[0][i];
            end = pose.matched_positions[1][i] + cv::Point2f(sizes[0].width, 0);
          }
          else {

            const cv::Point2f &p0 =
                pose.matched_positions[0][i];

            const cv::Point2f &p1 =
                pose.matched_positions[1][i];

            const cv::Vec3d v0 =
                rectificationHomography[0] *
                    cv::Vec3d(p0.x, p0.y, 1);

            const cv::Vec3d v1 =
                rectificationHomography[1] *
                    cv::Vec3d(p1.x, p1.y, 1);

            start.x = v0(0) / v0(2);
            start.y = v0(1) / v0(2);

            end.x = v1(0) / v1(2) + sizes[0].width;
            end.y = v1(1) / v1(2);
          }

          const cv::Scalar clr(std::max(32, rand() % 255),
              std::max(32, rand() % 255),
              std::max(32, rand() % 255));

          cv::line(display_frame_, start, end, clr);;
        }
      }
    }
  }

 // on_accumulator_changed();
}


void c_regular_stereo_pipeline::update_disparity_map_display_image(const cv::Mat & currentFrame,
    const cv::Mat & referenceFrame,
    const cv::Mat1w & disparityMap,
    const cv::Mat1b & disparityMask,
    int max_disparity)
{

  static const auto convertToBGR =
      [](cv::InputArray src, cv::OutputArray & dst) {

        if ( src.depth() == CV_8U ) {
          if ( src.channels() == 1 ) {
            cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
          }
          else {
            src.copyTo(dst);
          }
        }
        else {

          double alpha, beta;

          get_scale_offset(src.depth(), CV_8U,
              &alpha,
              &beta);

          src.getMat().convertTo(dst, CV_8U, alpha, beta);

          if ( dst.channels() == 1 ) {
            cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
          }
        }
      };

  cv::Mat images[2];
  cv::Mat dispMap;

  cv::pyrDown(currentFrame, images[0]);
  cv::pyrDown(referenceFrame, images[1]);

  const cv::Size size0 =
      images[0].size();

  const cv::Size size1 =
      images[1].size();

  const cv::Size map_size =
      disparityMap.size();

  const cv::Size total_display_size(std::max(size0.width + size1.width, map_size.width),
      std::max(size0.height, size1.height) + map_size.height);

  const cv::Rect rc0(0, 0, size0.width, size0.height);
  const cv::Rect rc1(size0.width, 0, size1.width, size1.height);
  const cv::Rect rc2(0, rc1.height, map_size.width, map_size.height);

  disparityMap.convertTo(dispMap, CV_8U, 255. / max_disparity, 0);
  apply_colormap(dispMap, dispMap, COLORMAP_TURBO);
  dispMap.setTo(0, ~disparityMask);

  display_frame_.create(total_display_size, CV_8UC3);
  convertToBGR(images[0], display_frame_(rc0));
  convertToBGR(images[1], display_frame_(rc1));
  dispMap.copyTo(display_frame_(rc2));

  // on_accumulator_changed();
}


bool c_regular_stereo_pipeline::write_calibration_progress_video(c_output_frame_writer & progress_writer)
{
  if ( output_options_.save_progress_video ) {

    if( !progress_writer.is_open() ) {

      const std::string output_file_name =
          generate_output_file_name(output_options_.progress_video_filename,
              "progress",
              ".avi");

      bool fOK =
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
      CF_ERROR("c_output_frame_writer: write(fails)");
      return false;
    }
  }

  return true;
}

std::string c_regular_stereo_pipeline::get_calibraton_config_output_filename() const
{
  std::string output_file_name =
      calibration_options_.calibration_config_filename;

  if( output_file_name.empty() ) {
    output_file_name =
        ssprintf("%s/regular_stereto_calib.%s.yml",
            output_path_.c_str(),
            csequence_name());
  }
  else if( !is_absolute_path(output_file_name) ) {
    output_file_name =
        ssprintf("%s/%s",
            output_path_.c_str(),
            output_file_name.c_str());
  }

  return output_file_name ;
}

bool c_regular_stereo_pipeline::save_calibration_config_file() const
{
  std::string output_file_name =
      get_calibraton_config_output_filename();

  if( !create_path(get_parent_directory(output_file_name)) ) {
    CF_ERROR("create_path('%s') fails: %s ", output_file_name.c_str(), strerror(errno));
    return false;
  }

  cv::FileStorage fs(output_file_name, cv::FileStorage::WRITE);

  if( !fs.isOpened() ) {
    CF_ERROR("cv::FileStorage('%s') fails: %s ",
        output_file_name.c_str(),
        strerror(errno));
    return false;
  }

  time_t rawtime;
  time(&rawtime);
  char buf[256];
  strftime(buf, sizeof(buf) - 1, "%c", localtime(&rawtime));

  fs << "calibrationDate" << buf;

  fs << "cameraResolution" << intrinsics_.image_size;
  fs << "cameraMatrix" << intrinsics_.camera_matrix;
  fs << "dist_coeffs" << intrinsics_.dist_coeffs;
  fs << "rectificationHomography0" << rectificationHomography[0];
  fs << "rectificationHomography1" << rectificationHomography[1];

  fs.release();

  return true;


  return false;
}

bool c_regular_stereo_pipeline::load_calibration_config_file()
{
  std::string filename =
      calibration_options_.calibration_config_filename;

  if( filename.empty() && calibration_options_.enable_calibration ) {
    filename = get_calibraton_config_output_filename();
  }

  if ( filename.empty() ) {
    CF_ERROR("INPUT ERROR: No input calibration config file name specified");
    return false;
  }

  try {

    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if( !fs.isOpened() ) {
      CF_ERROR("Error: can not load extrinsic parameters "
          "from input file '%s'", filename.c_str());
    }
    else {

      fs["cameraResolution"] >> intrinsics_.image_size;
      fs["cameraMatrix"] >> intrinsics_.camera_matrix;
      fs["dist_coeffs"] >> intrinsics_.dist_coeffs;
      fs["rectificationHomography0"] >> rectificationHomography[0];
      fs["rectificationHomography1"] >> rectificationHomography[1];

      haveRectificationHomography = true;

      return true;
    }
  }
  CATCH(__func__);

  return false;
}


bool c_regular_stereo_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  output_path_ =
      create_output_path(output_options_.output_directory);

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
  if( calibration_options_.apply_stereo_rectification ) {
    if( !calibration_options_.enable_calibration && !load_calibration_config_file() ) {
      CF_ERROR("load_calibration_cofig_file() fails");
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  if( calibration_options_.enable_calibration ) {

    if( !(keypoints_extractor_ = create_sparse_feature_extractor(feature2d_options_.sparse_feature_extractor)) ) {
      CF_ERROR("ERROR: create_sparse_feature_extractor() fails");
      return false;
    }

    if( !(keypoints_matcher_ = create_sparse_feature_matcher(feature2d_options_.sparse_feature_matcher)) ) {
      CF_ERROR("ERROR: create_sparse_feature_matcher() fails");
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  current_frame_ = &input_frames_[0];
  previous_frame_ = &input_frames_[1];
  haveRectificationHomography = false;

  const cv::Matx23d & C =
      input_options_.cameraMatrix;

  intrinsics_.camera_matrix = cv::Matx33d(
      C(0, 0), C(0, 1), C(0, 2),
      C(1, 0), C(1, 1), C(1, 2),
      0, 0, 1);

  intrinsics_.image_size.width =
      intrinsics_.image_size.height = -1;

  if ( calibration_options_.enable_calibration && output_options_.save_motion_poses ) {

    std::string outfilename =
        generate_output_file_name(output_options_.motion_poses_filename,
            "poses",
            ".txt");

    if( !(posesfp_ = fopen(outfilename.c_str(), "w")) ) {
      CF_ERROR("Can not write '%s' : %s", outfilename.c_str(), strerror(errno));
      return false;
    }

    fprintf(posesfp_, "fidx"

        "\tE0.x\tE0.y"
        "\tE1.x\tE1.y"

        "\trmse0\trmse1"

        "\tT0.x\tT0.y\tT0.z"
        "\tT1.x\tT1.y\tT1.z"

        "\n");
  }


  return true;
}

void c_regular_stereo_pipeline::dump_motion_pose(const c_motion_pose & pose)
{
  if( posesfp_ ) {

    fprintf(posesfp_, "%6d"

        "\t%+9.3f\t%+9.3f"
        "\t%+9.3f\t%+9.3f"

        "\t%+9.3f\t%+9.3f"

        "\t%+9.3f\t%+9.3f\t%+9.3f"
        "\t%+9.3f\t%+9.3f\t%+9.3f"

        "\n",
        pose.stream_pos,

        pose.E[0].x, pose.E[0].y,
        pose.E[1].x, pose.E[1].y,

        pose.rmse[0], pose.rmse[1],

        pose.T[0](0), pose.T[0](1), pose.T[0](2),
        pose.T[1](0), pose.T[1](1), pose.T[1](2)
    );

  }

}


void c_regular_stereo_pipeline::cleanup_pipeline()
{
  if ( posesfp_ ) {
    fclose(posesfp_);
    posesfp_ = nullptr;
  }

  current_frame_ = nullptr;
  previous_frame_ = nullptr;

  motion_poses_.clear();

  for( int i = 0; i < 2; ++i ) {

    if( input_sources_[i] ) {
      input_sources_[i]->close();
      input_sources_[i].reset();
    }

    for( int j = 0; j < 2; ++j ) {

      input_frames_[i].images[j].release();
      input_frames_[i].masks[j].release();
      input_frames_[i].keypoints[j].clear();
      input_frames_[i].descriptors[j].release();
    }

  }

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
}

void c_regular_stereo_pipeline::reset_input_frames()
{
  for ( int i = 0; i < 2; ++i ) {

    //input_frames_[i].matches.clear();

    for ( int j = 0; j < 2; ++j ) {
      input_frames_[i].images[j].release();
      input_frames_[i].masks[j].release();
      input_frames_[i].keypoints[j].clear();
      input_frames_[i].descriptors[j].release();
    }
  }

  current_frame_ = & input_frames_[0];
  previous_frame_ = & input_frames_[1];
}


//
//double c_rstereo_calibration_pipeline::compute_stereo_pose(
//    const std::vector<cv::Point2f> & current_keypoints,
//    const std::vector<cv::Point2f> & reference_keypoints)
//{
//  cv::Mat1b inliers;
//  double rmse = -1;
//  bool fOK;
//
//  fOK =
//      rectify_stereo_pose(
//          cameraMatrix,
//          current_keypoints,
//          reference_keypoints,
//          EMM_LMEDS,
//          &currentEulerAnges,
//          &currentTranslationVector,
//          &currentRotationMatrix,
//          &currentEssentialMatrix,
//          &currentFundamentalMatrix,
//          &currentDerotationHomography,
//          inliers);
//
//  if( !fOK ) {
//    CF_ERROR("rectify_stereo_pose() fails");
//  }
//  else {
//
//    CF_DEBUG("currentEulerAnges: (%+.3f %+.3f %+.3f)",
//        currentEulerAnges(0) * 180 / CV_PI,
//        currentEulerAnges(1) * 180 / CV_PI,
//        currentEulerAnges(2) * 180 / CV_PI);
//
//
//    cv::Mat1d rhs;
//
//    compute_distances_from_points_to_corresponding_epipolar_lines(rhs,
//        currentFundamentalMatrix,
//        current_keypoints,
//        reference_keypoints);
//
//    rmse = sqrt(cv::mean(rhs.mul(rhs), inliers)[0]);
//  }
//
//  return rmse;
//}


bool c_regular_stereo_pipeline::compute_motion_pose(int camera_index, c_motion_pose * pose) const
{
  std::vector<cv::Point2f> current_positions;
  std::vector<cv::Point2f> previous_positions;
  std::vector<cv::DMatch> matches;
  cv::Mat1d rhs;
  cv::Mat1b inliers;
  cv::Matx33d FundamentalMatrix;
  cv::Point2d E[2];

  CF_DEBUG("descriptors[camera_index=%d]:\ncurrent: %dx%d type=%d previous: %dx%d type=%d",
      camera_index,

      current_frame_->descriptors[camera_index].rows,
      current_frame_->descriptors[camera_index].cols,
      current_frame_->descriptors[camera_index].type(),

      previous_frame_->descriptors[camera_index].rows,
      previous_frame_->descriptors[camera_index].cols,
      previous_frame_->descriptors[camera_index].type());


  const size_t num_matches =
      match_keypoints(keypoints_matcher_,

          current_frame_->keypoints[camera_index],
          current_frame_->descriptors[camera_index],

          previous_frame_->keypoints[camera_index],
          previous_frame_->descriptors[camera_index],

          &matches,
          &current_positions,
          &previous_positions);

  if ( num_matches < 8 ) {
    CF_DEBUG("[camera_index=%d] match_keypoints(): not enough (%zu < 8) matches found",
        camera_index, num_matches);
    return false;
  }

  bool fOK =
      estimate_camera_pose_and_derotation_homography(
          intrinsics_.camera_matrix,
          current_positions,
          previous_positions,
          EMM_LMEDS,
          nullptr,
          &pose->T[camera_index],
          nullptr,
          nullptr,
          &FundamentalMatrix,
          nullptr,
          inliers);

  if ( !fOK ) {
    CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
    return false;
  }

  compute_distances_from_points_to_corresponding_epipolar_lines(rhs,
      FundamentalMatrix,
      current_positions,
      previous_positions);

  pose->rmse[camera_index] =
      sqrt(cv::mean(rhs.mul(rhs), inliers)[0]);

  CF_DEBUG("[camera_index=%d] rmse=%g", camera_index, pose->rmse[camera_index]);

  if ( !(pose->rmse[camera_index]>=0) ) {
    CF_DEBUG("[camera_index=%d] bad rmse=%g", camera_index, pose->rmse[camera_index]);
    return false;
  }


  fOK =
      compute_epipoles(FundamentalMatrix,
          E);

  if ( !fOK ) {
    CF_ERROR("[camera_index=%d] compute_epipoles() fails", camera_index);
    return false;
  }

  if( hypot(E[0].x - E[1].x, E[0].y - E[1].y) > 1 ) {
    CF_ERROR("compute_epipoles() returns poor values: E0=(%+g %+g) E1=(%+g %+g)",
        E[0].x, E[0].y, E[1].x, E[1].y);
    return false;
  }

  pose->E[camera_index] = 0.5 * (E[0] + E[1]);

  return true;
}

bool c_regular_stereo_pipeline::detect_current_stereo_matches(c_motion_pose * pose)
{
  cv::Mat1d rhs;
  cv::Mat1b inliers;
  cv::Matx33d FundamentalMatrix;
  cv::Point2d E[2];

  pose->matched_positions[0].clear();
  pose->matched_positions[1].clear();

  const std::vector<cv::KeyPoint> &reference_keypoints =
      current_frame_->keypoints[0];

  const cv::Mat &reference_descriptors =
      current_frame_->descriptors[0];

  std::vector<cv::Point2f> &reference_positions =
      pose->matched_positions[0];

  // from right image
  const std::vector<cv::KeyPoint> &current_keypoints =
      current_frame_->keypoints[1];

  const cv::Mat &current_descriptors =
      current_frame_->descriptors[1];

  std::vector<cv::Point2f> &current_positions =
      pose->matched_positions[1];

  const size_t num_matches =
      match_keypoints(keypoints_matcher_,

          current_keypoints,
          current_descriptors,
          reference_keypoints,
          reference_descriptors,

          nullptr,
          &current_positions,
          &reference_positions);


  if ( num_matches < 8 ) {
    CF_DEBUG("stereo match: enough (%zu < 8) matches found", num_matches);
    return false;
  }


  return true;
}

bool c_regular_stereo_pipeline::run_calibration()
{
  if( !calibration_options_.enable_calibration ) {
    CF_WARNING("rstereo calibration is disabled, skipped");
    return true;
  }

  c_output_frame_writer progress_video_writer;
  bool fOK;

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

  set_status_msg("RUNNING ...");


  reset_input_frames();

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

    if ( canceled() ) {
      break;
    }

    if ( canceled() ) {
      break;
    }

    if( !read_stereo_frame() ) {
      CF_ERROR("read_stereo_frame() fails");
      break;
    }

    if( !detect_keypoints() ) {
      CF_ERROR("detect_keypoints() fails");
      break;
    }

    ///////////
    if( !previous_frame_->keypoints[0].empty() && !previous_frame_->keypoints[1].empty() ) {

      c_motion_pose pose;

      for( int i = 0; i < 2; ++i ) {

        if( !(fOK = compute_motion_pose(i, &pose)) ) {
          CF_ERROR("compute_motion_pose(camera=%d) fails", i);
          break;
        }
      }

      if ( canceled() ) {
        break;
      }

      if( !fOK ) {
        continue;
      }

      pose.stream_pos =
          processed_frames_;

      dump_motion_pose(pose);

      if ( canceled() ) {
        break;
      }

      if ( detect_current_stereo_matches(&pose) ) {
        motion_poses_.emplace_back(pose);
      }
    }


    ///////////

    if ( canceled() ) {
      break;
    }

    update_calibration_display_image();

    if ( canceled() ) {
      break;
    }

    if ( !write_calibration_progress_video(progress_video_writer) ) {
      CF_ERROR("write_progress_video() fails");
      return false;
    }

    std::swap(current_frame_, previous_frame_);
  }


  for ( int i = 0; i < 2; ++i ) {
    input_sources_[i]->close();
  }

  ////////////
  if(  !canceled() && motion_poses_.size() > 3 ) {

    static const auto meanstdev_poses =
        [](const std::vector<c_motion_pose> & poses, const std::vector<uint8_t> & inliers,
            cv::Point2d * dEm, cv::Point2d * dEs) -> bool {

              cv::Point2d dEmean, dEstdev;
              int m = 0;

              dEmean.x = dEmean.y = 0;
              dEstdev.x = dEstdev.y = 0;

              for ( uint i = 0, n = poses.size(); i < n; ++i ) {
                if ( inliers[i] ) {

                  const double dx =
                      poses[i].E[0].x - poses[i].E[1].x;

                  const double dy =
                      poses[i].E[0].y - poses[i].E[1].y;

                  dEmean.x += dx;
                  dEmean.y += dx;
                  dEstdev.x += dx * dx;
                  dEstdev.y += dx * dx;
                  ++m;
                }
              }

              if ( m > 3 ) {

                dEmean.x /= m;
                dEmean.y /= m;

                dEstdev.x =
                    sqrt(dEstdev.x / m - dEmean.x * dEmean.x);

                dEstdev.y =
                    sqrt(dEstdev.y / m - dEmean.y * dEmean.y);

                *dEm = dEmean;
                *dEs = dEstdev;

                return true;
              }

              return false;
            };

    // filter poses
    cv::Point2d dEmean, dEstdev;
    std::vector<uint8_t> inliers(motion_poses_.size(), true);
    std::vector<cv::Point2f> matched_stereo_positions[2];

    int pass = 0;

    for ( ; pass < 15; ++pass ) {

      if ( !meanstdev_poses(motion_poses_, inliers, &dEmean, &dEstdev) ) {
        CF_ERROR("[pass %d] meanstdev_poses() fails", pass);
        break;
      }

      const double threshold =
          8 * (dEstdev.x * dEstdev.x + dEstdev.y + dEstdev.y);

      int outliers = 0;

      for ( uint i = 0, n = motion_poses_.size(); i < n; ++i ) {
        if ( inliers[i] ) {

          const double dx =
              motion_poses_[i].E[0].x - motion_poses_[i].E[1].x;

          const double dy =
              motion_poses_[i].E[0].y - motion_poses_[i].E[1].y;

          const double r2 =
              dx * dx + dy * dy;

          if ( r2 > threshold ) {
            inliers[i] = false;
            ++outliers;
          }
        }
      }

      if ( !outliers ) {
        break;
      }
    }

    cv::Point2d motionPoles[2];
    int nmotionPoles = 0;

    for( uint j = 0, n = motion_poses_.size(); j < n; ++j ) {
      if( inliers[j] ) {

        const c_motion_pose &pose =
            motion_poses_[j];

        for ( int i = 0; i < 2; ++i ) {
          motionPoles[i] += pose.E[i];
        }

        ++nmotionPoles;
      }
    }

    for( int i = 0; i < 2; ++i ) {
      motionPoles[i] /= nmotionPoles;
    }


    CF_DEBUG("\n"
        "pass=%d inliers=%d/%zu dEmean = (%+.3f %+.3f) dEstdev = (%+.3f %+.3f) "
        "POLES= (%+g %+g) (%+g %+g)",
        pass,
        cv::countNonZero(inliers), inliers.size(),
        dEmean.x, dEmean.y, dEstdev.x, dEstdev.y,
        motionPoles[0].x, motionPoles[0].y, motionPoles[1].x, motionPoles[1].x);

    if( !canceled() ) {

      for( uint j = 0, m = motion_poses_.size(); j < m; ++j ) {
        if( inliers[j] ) {

          for ( int i = 0; i < 2; ++i ) {
            matched_stereo_positions[i].insert(matched_stereo_positions[i].end(),
                motion_poses_[j].matched_positions[i].begin(),
                motion_poses_[j].matched_positions[i].end());
          }
        }
      }
    }

    if ( !canceled() ) {

      cv::Vec3d A;
      cv::Vec3d T;
      cv::Matx33d R;
      cv::Matx33d Ebefore;
      cv::Matx33d Fbefore, Fafter;


      cv::Point2d EpipolesBefore[2];
      cv::Point2d EpipolesAfter[2];
      cv::Mat1b inliers;

      bool fOK =
          estimate_camera_pose_and_derotation_homography(
              intrinsics_.camera_matrix,
              matched_stereo_positions[0],
              matched_stereo_positions[1],
              EMM_LMEDS,
              &A,
              &T,
              &R,
              &Ebefore,
              &Fafter,
              nullptr,
              inliers);

      if ( !fOK ) {
        CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
        return false;
      }

      Fbefore =
          compose_fundamental_matrix(Ebefore,
              intrinsics_.camera_matrix);

      compute_epipoles(Fbefore, EpipolesBefore);
      compute_epipoles(Fafter, EpipolesAfter);

      CF_DEBUG("CAMERA_POSE:\n"
          "inliers = %d / %d:\n"
          "A = (%+g %+g %+g)\n"
          "T = (%+g %+g %+g) \n"
          "Eb = (%+g %+g) (%+g %+g)\n"
          "Ea = (%+g %+g) (%+g %+g)\n"
          "\n",
          cv::countNonZero(inliers), inliers.rows,
          A(0) * 180 / CV_PI, A(1) * 180 / CV_PI, A(2) * 180 / CV_PI,
          T(0), T(1), T(2),
          EpipolesBefore[0].x, EpipolesBefore[0].y, EpipolesBefore[1].x, EpipolesBefore[1].y,
          EpipolesAfter[0].x, EpipolesAfter[0].y, EpipolesAfter[1].x, EpipolesAfter[1].y
          );

      cv::Matx33d Rr =
          build_rotation(0., -asin(T(2)), 0.);

      const cv::Matx33d &C = intrinsics_.camera_matrix;
      const cv::Matx33d Ci = intrinsics_.camera_matrix.inv();

      cv::Matx33d H[2] = {
          C * Rr * R * Ci,
          C * Rr * Ci
      };

      const cv::Point2d deltaP =
          apply_homography(H[1], motionPoles[1]) -
              apply_homography(H[0], motionPoles[0]);

      CF_DEBUG("\nMOTION POLES deltaP = (%+g %+g)", deltaP.x, deltaP.y);

      cv::Matx33d HT(1, 0, deltaP.x,
          0, 1, 0,
          0, 0, 1);

      rectificationHomography[0] = HT * H[0];
      rectificationHomography[1] = H[1];
      haveRectificationHomography = true;

      if( !save_calibration_config_file() ) {
        CF_ERROR("ERROR: save_current_camera_parameters() fails");
        return false;
      }

      if( false ) {

        inliers.release();

        for( int i = 0; i < 2; ++i ) {
          cv::perspectiveTransform(matched_stereo_positions[i], matched_stereo_positions[i],
              rectificationHomography[i]);
        }

        fOK =
            estimate_camera_pose_and_derotation_homography(
                intrinsics_.camera_matrix,
                matched_stereo_positions[0],
                matched_stereo_positions[1],
                EMM_LMEDS,
                &A,
                &T,
                &R,
                &Ebefore,
                &Fafter,
                nullptr,
                inliers);

        if( !fOK ) {
          CF_ERROR("after test: estimate_camera_pose_and_derotation_homography() fails");
          return false;
        }

        Fbefore =
            compose_fundamental_matrix(Ebefore,
                intrinsics_.camera_matrix);

        compute_epipoles(Fbefore, EpipolesBefore);
        compute_epipoles(Fafter, EpipolesAfter);

        CF_DEBUG("AFER TEST CAMERA_POSE:\n"
            "inliers = %d / %d:\n"
            "A = (%+g %+g %+g)\n"
            "T = (%+g %+g %+g) \n"
            "Eb = (%+g %+g) (%+g %+g)\n"
            "Ea = (%+g %+g) (%+g %+g)\n"
            "\n",
            cv::countNonZero(inliers), inliers.rows,
            A(0) * 180 / CV_PI, A(1) * 180 / CV_PI, A(2) * 180 / CV_PI,
            T(0), T(1), T(2),
            EpipolesBefore[0].x, EpipolesBefore[0].y, EpipolesBefore[1].x, EpipolesBefore[1].y,
            EpipolesAfter[0].x, EpipolesAfter[0].y, EpipolesAfter[1].x, EpipolesAfter[1].y);

      }

    }
  }

  for( int i = 0; i < 2; ++i ) {
    input_sources_[i]->close();
  }

///////////
  return !canceled();
}

bool c_regular_stereo_pipeline::save_rectified_videos()
{
  if( output_options_.save_rectified_videos || output_options_.save_stereo_matches_video ) {

    c_output_frame_writer stereo_matches_video;

    c_output_frame_writer rectified_videos[2];

    std::string rectified_videos_output_filenames[2] = {
        output_options_.rectified_video_filenames[0],
        output_options_.rectified_video_filenames[1]
    };


    bool fOK;

    set_status_msg("WRITE OUTPUT VIDEOS ...");

    for( int i = 0; i < 2; ++i ) {
      if( !input_sources_[i]->open() ) {
        CF_ERROR("ERROR: can not open input source '%s'",
            input_sources_[i]->cfilename());
        return false;
      }
    }


    if ( output_options_.save_rectified_videos ) {

      static const std::string postfix[2] = {
          ".rectifed.left",
          ".rectifed.right"
      };

      for( int i = 0; i < 2; ++i ) {

        if( !rectified_videos_output_filenames[i].empty() ) {
          rectified_videos_output_filenames[i] =
              generate_output_file_name(rectified_videos_output_filenames[i],
                  postfix[i],
                  ".avi");
        }
        else {
          std::string filename;

          split_pathfilename(input_sources_[i]->filename(),
              nullptr,
              &filename,
              nullptr,
              true);

          rectified_videos_output_filenames[i] =
              ssprintf("%s/%s.%s.avi",
                  output_path_.c_str(),
                  filename.c_str(),
                  postfix[i].c_str());
        }

      }
    }

    if ( output_options_.save_stereo_matches_video ) {

    }


    total_frames_ = input_sources_[0]->size();
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    reset_input_frames();

    for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

      if( canceled() ) {
        break;
      }

      if ( !read_stereo_frame() ) {
        CF_ERROR("read_stereo_frame() fails");
        break;
      }

      if( canceled() ) {
        break;
      }

      fOK = true;


      if( output_options_.save_rectified_videos ) {

        cv::Mat tmp;

        for( int i = 0; i < 2; ++i ) {


          if( !calibration_options_.apply_stereo_rectification || !haveRectificationHomography ) {
            tmp = current_frame_->images[i];
          }
          else {
            cv::warpPerspective(current_frame_->images[i], tmp,
                rectificationHomography[i],
                current_frame_->images[i].size(),
                cv::INTER_LINEAR, // Note that cv::INTER_LINEAR can introduce some blur on kitti images
                cv::BORDER_CONSTANT);
          }


          if( !rectified_videos[i].is_open() ) {

            const cv::Size size =
                tmp.size();

            bool is_color =
                tmp.channels() > 1;

            if( !rectified_videos[i].open(rectified_videos_output_filenames[i], size, is_color) ) {
              CF_ERROR("recified_video_writer.open('%s') fails", rectified_videos_output_filenames[i].c_str());
              return false;
            }
          }


          if( !rectified_videos[i].write(tmp, cv::noArray(), false, processed_frames_) ) {
            CF_ERROR("rectified_video_writer.write() fails");
            return false;
          }

          if( canceled() ) {
            break;
          }
        }
      }

      if( output_options_.save_stereo_matches_video ) {

        update_calibration_display_image(false, false, processed_frames_);

        if( canceled() ) {
          break;
        }

        if( !stereo_matches_video.is_open() ) {

          const std::string output_file_name =
              generate_output_file_name(output_options_.stereo_matches_video_filename,
                  "stereo_matches",
                  ".avi");

          if( !stereo_matches_video.open(output_file_name, display_frame_.size(), display_frame_.channels() > 1) ) {
            CF_ERROR("stereo_matches_video_writer.open('%s') fails", output_file_name.c_str());
            return false;
          }
        }

        if( canceled() ) {
          break;
        }

        if( !stereo_matches_video.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
          CF_ERROR("stereo_matches_video_writer() fails");
          return false;
        }

        if( canceled() ) {
          break;
        }

        update_calibration_display_image(true, false, processed_frames_);

        if( !stereo_matches_video.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
          CF_ERROR("stereo_matches_video_writer() fails");
          return false;
        }

        if( canceled() ) {
          break;
        }

      }
    }

    for( int i = 0; i < 2; ++i ) {
      input_sources_[i]->close();
    }

  }

  return !canceled();
}

bool c_regular_stereo_pipeline::run_stereo_matching()
{

  if( !stereo_matching_options_.enable_stereo_matchning ) {
    return true;
  }

  c_sweepscan_stereo_matcher matcher;
  cv::Mat images[2];
  cv::Mat masks[2];

  cv::Mat1w disparityMap;
  cv::Mat1b disparityMask;

  c_output_frame_writer progress_video;

  bool fOK;

  matcher.set_max_disparity(stereo_matching_options_.max_disparity);
  matcher.set_max_scale(stereo_matching_options_.max_scale);
  matcher.set_kernel_radius(stereo_matching_options_.kernel_radius);
  matcher.set_kernel_sigma(stereo_matching_options_.kernel_sigma);
  matcher.set_max_scale(stereo_matching_options_.max_scale);
  matcher.set_debug_points(stereo_matching_options_.debug_points);


  double kernel_sigma_ = 1;
  int kernel_radius_ = 3;

  if ( !open_input_streams() ) {
    CF_ERROR("ERROR: open_input_streams() fails");
    return false;
  }

  reset_input_frames();

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_update() ) {

    if( canceled() ) {
      break;
    }

    if ( !read_stereo_frame() ) {
      CF_ERROR("read_stereo_frame() fails");
      break;
    }

    if( canceled() ) {
      return false;
    }

    if( stereo_matching_options_.process_only_debug_frames || stereo_matching_options_.save_debug_images ) {

      const auto pos =
          std::find(stereo_matching_options_.debug_frames.begin(),
              stereo_matching_options_.debug_frames.end(),
              processed_frames_);

      if( pos == stereo_matching_options_.debug_frames.end() ) {

        if( stereo_matching_options_.process_only_debug_frames ) {
          continue;
        }

        matcher.set_debug_directory("");
      }

      else if( stereo_matching_options_.save_debug_images ) {

        matcher.set_debug_directory(
            ssprintf("%s/debug/match%05d",
                output_path_.c_str(),
                processed_frames_));
      }

    }

    if( calibration_options_.apply_stereo_rectification ) {

      for( int i = 0; i < 2; ++i ) {

        if( canceled() ) {
          return false;
        }

        const cv::Size image_size =
            current_frame_->images[i].size();

        cv::warpPerspective(current_frame_->images[i], current_frame_->images[i],
            rectificationHomography[i],
            image_size,
            cv::INTER_LINEAR,
            cv::BORDER_REPLICATE);

        if( canceled() ) {
          return false;
        }

        cv::warpPerspective(cv::Mat1b(image_size, 255), current_frame_->masks[i],
            rectificationHomography[i],
            image_size,
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

        if( canceled() ) {
          return false;
        }

        cv::compare(current_frame_->masks[i], 255,
            current_frame_->masks[i],
            cv::CMP_GE);
      }

      if( canceled() ) {
        return false;
      }
    }

    if( !image_processing_options_.stereo_match_preprocessor ) {
      fOK =
          matcher.match(current_frame_->images[0], current_frame_->masks[0],
              current_frame_->images[1], current_frame_->masks[1],
              disparityMap,
              &disparityMask);
    }
    else {

      for( int i = 0; i < 2; ++i ) {
        current_frame_->images[i].copyTo(images[i]);
        current_frame_->masks[i].copyTo(masks[i]);
        if( !image_processing_options_.stereo_match_preprocessor->process(images[i], masks[i]) ) {
          CF_ERROR("stereo_match_preprocessor->process() fails for stereo frame %d", i);
          return false;
        }
      }

      fOK =
          matcher.match(images[0], masks[0],
              images[1], masks[1],
              disparityMap,
              &disparityMask);
    }

    if ( !fOK )  {
      CF_ERROR("matcher.match() fails");
      return false;
    }

    if( canceled() ) {
      return false;
    }

    update_disparity_map_display_image(current_frame_->images[0], current_frame_->images[1],
        disparityMap, disparityMask,
        std::max(10, matcher.max_disparity() * 3 / 4));

    if( canceled() ) {
      return false;
    }

    if ( output_options_.save_stereo_match_progress_video ) {

      std::string output_file_name =
          generate_output_file_name(output_options_.stereo_match_progress_video_filename,
              "match_progress",
              ".avi");

      if ( !progress_video.is_open() ) {
        if( !progress_video.open(output_file_name, display_frame_.size(), display_frame_.channels() > 1) ) {
          CF_ERROR("progress_video.open('%s') fails", output_file_name.c_str());
          return false;
        }
      }

      if ( !progress_video.write(display_frame_, cv::noArray(), false, processed_frames_) ) {
        CF_ERROR("progress_video.write() fails");
        return false;
      }
    }
  }

  return true;
}

bool c_regular_stereo_pipeline::run_pipeline()
{
  if( !run_calibration() ) {
    return false;
  }

  if( canceled() ) {
    return false;
  }

  if( calibration_options_.apply_stereo_rectification && !load_calibration_config_file() ) {
    CF_ERROR("load_calibration_config_file() fails");
    return false;
  }

  if( canceled() ) {
    return false;
  }

  if( !save_rectified_videos() ) {
    CF_ERROR("save_rectified_videos() fails");
    return false;
  }

  if ( !run_stereo_matching() ) {
    CF_ERROR("run_stereo_matching() fails");
    return false;
  }


  set_status_msg("FINISHED ...");

  return true;
}


