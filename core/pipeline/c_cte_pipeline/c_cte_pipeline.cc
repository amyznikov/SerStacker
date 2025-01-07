/*
 * c_cte_pipeline.cc
 *
 *  Created on: Dec 10, 2024
 *      Author: amyznikov
 */

#include "c_cte_pipeline.h"
#include <core/settings/camera_settings.h>
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/levmar.h>
#include <core/proc/c_lse2_estimate.h>
#include <thread>

#if HAVE_TBB
# include <tbb/tbb.h>
typedef tbb::blocked_range<int> tbb_range;
#endif

template<class _Tp>
static inline bool is_bad_number(_Tp x)
{
  return !(x >= 0 || x <= 0);
}

template<class _Tp>
static inline bool is_bad_number(const cv::Point_<_Tp> & v)
{
  return is_bad_number(v.x) || is_bad_number(v.y);
}

template<class _Tp>
static inline bool is_bad_number(const cv::Vec<_Tp, 3> & v)
{
  return is_bad_number(v[0]) || is_bad_number(v[1]) || is_bad_number(v[2]);
}

template<class _Tp>
static inline bool is_bad_number(const cv::Vec<_Tp, 2> & v)
{
  return is_bad_number(v[0]) || is_bad_number(v[1]);
}

static inline cv::Vec3d normalize(const cv::Vec3d & T)
{
  const double L = cv::norm(T);
  return L == 0 || L == 1 ? T : T / L;
}

static inline cv::Point2d compute_epipole_location(const cv::Matx33d & camera_matrix, const cv::Vec3d & T)
{
  const cv::Vec3d E =
      camera_matrix * T;

  const double E2abs =
      std::abs(E(2));

  const double S =
      E(2) >= 0 ? 1. / std::max(E2abs, 1e-9) :
          -1. / std::max(E2abs, 1e-9);

  return cv::Point2d(E[0] * S, E[1] * S);
}


static inline UVec3d compute_translation(const cv::Matx33d & camera_matrix_inv, const cv::Point2d & E)
{
  const cv::Vec3d T =
      camera_matrix_inv * cv::Vec3d(E.x, E.y, 1);

  return UVec3d(T);
}


static inline void draw_epipole(cv::Mat & image, const cv::Point2d & E)
{
  if( E.x >= 0 && E.y >= 0 && E.x < image.cols && E.y < image.rows ) {
    cv::ellipse(image, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
    cv::line(image, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }
}

static void copyimage(const cv::Mat & src, cv::Mat3b & dst)
{
  if( src.type() == CV_8UC3 ) {
    src.copyTo(dst);
  }
  else if( src.depth() == CV_8U ) {
    cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
  }
  else {

    double alpha, beta;

    get_scale_offset(src.depth(), CV_8U,
        &alpha,
        &beta);

    if( src.channels() == 3 ) {

      src.convertTo(dst, CV_8U,
          alpha,
          beta);
    }
    else {
      cv::Mat tmp;

      src.convertTo(tmp, CV_8U,
          alpha,
          beta);

      cv::cvtColor(tmp, dst,
          cv::COLOR_GRAY2BGR);

    }

  }
}

template<class _Tp1, class _Tp2>
static cv::Point_<_Tp1> warp(const cv::Point_<_Tp1> & p, const cv::Matx<_Tp2, 3, 3> & H)
{
  cv::Vec<_Tp1, 3> v = H * cv::Vec<_Tp1,3>(p.x, p.y, 1);
  if( v[2] != 0 && v[2] != 1 ) {
    v /= v[2];
  }
  return cv::Point_<_Tp1> (v[0], v[1]);
}

template<class _Tp1, class _Tp2>
static cv::Vec<_Tp1, 2> warpv(const cv::Point_<_Tp1> & p, const cv::Matx<_Tp2, 3, 3> & H)
{
  cv::Vec<_Tp1, 3> v = H * cv::Vec<_Tp1,3>(p.x, p.y, 1);
  if( v[2] != 0 && v[2] != 1 ) {
    v /= v[2];
  }
  return cv::Vec<_Tp1, 2> (v[0], v[1]);
}

template<class _Tp1, class _Tp2>
static cv::Vec<_Tp1, 3> warp(const cv::Vec<_Tp1, 3> & v, const cv::Matx<_Tp2, 3, 3> & H)
{
  const cv::Vec<_Tp1, 3> w =  H * v;
  return w / w[2];
}

template<class _Tp1, class _Tp2>
static cv::Vec<_Tp1, 2> warp(const cv::Vec<_Tp1, 2> & v, const cv::Matx<_Tp2, 3, 3> & H)
{
  const cv::Vec<_Tp2, 3> w = H * cv::Vec<_Tp2, 3>(v[0], v[1], 1);
  return cv::Vec<_Tp1, 2> (w[0]/w[2], w[1]/w[2]); // w / w[2];
}

c_cte_pipeline::c_cte_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

static void draw_keypoints(cv::InputOutputArray image, const std::vector<cv::KeyPoint> & keypoints)
{
  const cv::Scalar color = CV_RGB(24, 162, 24);
  const int w = 2;

  for( const cv::KeyPoint & kp : keypoints ) {

    const cv::Point2f & p =
        kp.pt;

    cv::rectangle(image, cv::Point2f(p.x - w, p.y - w), cv::Point2f(p.x + w, p.y + w),
        color, 1, cv::LINE_4);
  }
}


bool c_cte_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_epipolar_alignment_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->_camera_options = this->_camera_options;
  p->_feature2d = this->_feature2d;
  p->_context_options = this->_context_options;
  p->_pose_estimation = this->_pose_estimation;
  p->_output_options = this->_output_options;

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_cte_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Input Options", "");
      PIPELINE_CTL(ctrls, _input_options.read_step, "read step", "set > 1 to specify number of frames to skip");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Camera parameters", "");
      PIPELINE_CTL_CAMERA_INTRINSICS(ctrls, _camera_options.camera_intrinsics);
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Feature2D Options", "");
      PIPELINE_CTL(ctrls, _feature2d.image_scale, "image scale", "image scale");
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Detector", "");
        PIPELINE_CTL_FEATURE2D_DETECTOR_OPTIONS(ctrls, _feature2d.detector);
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Descriptor", "");
        PIPELINE_CTL_FEATURE2D_DESCRIPTOR_OPTIONS(ctrls, _feature2d.descriptor);
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Matcher", "");
        PIPELINE_CTL_FEATURE2D_MATCHER_OPTIONS(ctrls, _feature2d.matcher);
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Context Options...", "");
      PIPELINE_CTL(ctrls, _context_options.max_context_size, "Track context size", "Number of frames in context");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Pose Estimation", "Parameters for lm_refine_camera_pose2()");
      PIPELINE_CTL(ctrls, _pose_estimation.max_iterations, "max_iterations", "max iterations for outliers detection");
      PIPELINE_CTL(ctrls, _pose_estimation.max_levmar_iterations, "max_levmar_iterations", "max levmar iterations");
      PIPELINE_CTL(ctrls, _pose_estimation.levmar_epsf, "levmar_epsf", "levmar epsf");
      PIPELINE_CTL(ctrls, _pose_estimation.levmar_epsx, "levmar_epsx", "levmar epsx");
      PIPELINE_CTL(ctrls, _pose_estimation.robust_threshold, "robust_threshold", "robust_threshold");
      PIPELINE_CTL(ctrls, _pose_estimation.erfactor, "erfactor", "erfactor");
      PIPELINE_CTL(ctrls, _pose_estimation.ew, "ew", "ew");
    PIPELINE_CTL_END_GROUP(ctrls);


    ////////
    PIPELINE_CTL_GROUP(ctrls, "Output Options", "");
      PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");

      PIPELINE_CTL_GROUP(ctrls, "Save Progress video", "");
        PIPELINE_CTL(ctrls, _output_options.save_progress_video, "save_progress_video", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.progress_output_options,
            _this->_output_options.save_progress_video);
      PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
    ////////
  }

  return ctrls;
}

bool c_cte_pipeline::serialize(c_config_setting settings, bool save)
{

  static const auto get_group =
      [](c_config_setting setting, bool save, const std::string & name) {
        return save ? setting.add_group(name) : setting[name];
      };

  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, _input_options, read_step);
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_options")) ) {
    SERIALIZE_OPTION(section, save, _camera_options, camera_intrinsics);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {
    SERIALIZE_OPTION(section, save, _feature2d, image_scale);
    SERIALIZE_OPTION(get_group(section, save, "detector"), save, _feature2d, detector);
    SERIALIZE_OPTION(get_group(section, save, "descriptor"), save, _feature2d, descriptor);
    SERIALIZE_OPTION(get_group(section, save, "matcher"), save, _feature2d, matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "context_options")) ) {
    SERIALIZE_OPTION(section, save, _context_options, max_context_size);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "pose_estimation")) ) {
    SERIALIZE_OPTION(section, save, _pose_estimation, max_iterations);
    SERIALIZE_OPTION(section, save, _pose_estimation, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, _pose_estimation, levmar_epsf);
    SERIALIZE_OPTION(section, save, _pose_estimation, levmar_epsx);
    SERIALIZE_OPTION(section, save, _pose_estimation, robust_threshold);
    SERIALIZE_OPTION(section, save, _pose_estimation, erfactor);
    SERIALIZE_OPTION(section, save, _pose_estimation, ew);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "progress_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, progress_output_options);
    }
  }

  return true;
}


bool c_cte_pipeline::save_progress_video()
{
  if( _output_options.save_progress_video ) {

    INSTRUMENT_REGION("");

    cv::Mat display;

    lock_guard lock(mutex());

    //for( size_t i = 1; i < _frames.size(); ++i ) {
    if( _frames.size() > 1 ) {

      if( !create_display_image(1, display) ) {
        CF_WARNING("create_display_image() fails");
        return true;
      }

      if( canceled() ) {
        return false;
      }

      if( !_progress_writer.is_open() ) {

        bool fOk =
            add_output_writer(_progress_writer,
                _output_options.progress_output_options,
                "progress",
                ".avi");

        if( !fOk ) {
          CF_ERROR("add_output_writer('%s') fails",
              _progress_writer.filename().c_str());
          return false;
        }
      }

      if( !_progress_writer.write(display) ) {
        CF_ERROR("progress_writer_.write('%s') fails.",
            _progress_writer.filename().c_str());
        return false;
      }

      if( canceled() ) {
        return false;
      }
    }

    if( canceled() ) {
      return false;
    }
  }

  return true;
}

bool c_cte_pipeline::create_display_image(size_t current_frame_index, cv::OutputArray display_frame)
{
  if( _frames.size() > 1 && current_frame_index < _frames.size() ) {

    const c_cte_frame::uptr & reference_frame =
        _frames.front();

    const c_cte_frame::uptr & current_frame =
        _frames[current_frame_index];

    const cv::Size frame_size =
        reference_frame->image.size();

    const cv::Size display_size(frame_size.width * 2,
        frame_size.height * 3);

//    const cv::Vec3d dT =
//        current_frame->T - reference_frame->T;
//
//    const cv::Point2d E =
//        compute_epipole(_camera_options.camera_intrinsics.camera_matrix,
//            dT);

    const cv::Point2d & E =
        current_frame->E;

    const cv::Rect roi[3][2] = {

      { cv::Rect(0, 0, frame_size.width, frame_size.height),
          cv::Rect(frame_size.width, 0, frame_size.width, frame_size.height) },

      { cv::Rect(0, frame_size.height, frame_size.width, frame_size.height),
          cv::Rect(frame_size.width, frame_size.height, frame_size.width, frame_size.height) },

      { cv::Rect(0, 2 * frame_size.height, frame_size.width, frame_size.height),
          cv::Rect(frame_size.width, 2 * frame_size.height, frame_size.width, frame_size.height) },

    };

    display_frame.create(display_size, CV_8UC3);
    display_frame.setTo(cv::Scalar::all(0));

    cv::Mat & display =
        display_frame.getMatRef();

    cv::Mat3b display00 =
        display(roi[0][0]);

    cv::Mat3b display01 =
        display(roi[0][1]);

    cv::Mat3b display10 =
        display(roi[1][0]);

    cv::Mat3b display11 =
        display(roi[1][1]);

    cv::Mat3b display20 =
        display(roi[2][0]);

    cv::Mat3b display21 =
        display(roi[2][1]);

    copyimage(reference_frame->image,
        display00);

    copyimage(current_frame->image,
        display01);

    cv::addWeighted(display00, 0.5,
        display01, 0.5,
        0,
        display10);

    cv::warpPerspective(display01, display11, current_frame->H, frame_size,
        cv::INTER_LINEAR);

    cv::addWeighted(display11, 0.5,
        display00, 0.5,
        0,
        display11);

    if( true ) {

      std::vector<cv::KeyPoint> & rpts =
          reference_frame->keypoints;

      std::vector<cv::KeyPoint> & cpts =
          current_frame->keypoints;

      const std::vector<int32_t> & matches =
          reference_frame->matches[current_frame_index - 1];

      const cv::Mat1b & inliers =
          reference_frame->inliers[current_frame_index - 1];

      if( matches.size() != reference_frame->keypoints.size() ) {
        CF_ERROR("BUG");
        exit(1);
      }

      for( size_t k = 0, nk = rpts.size(); k < nk; ++k ) {

        if ( matches[k] < 0 ) {
          continue;
        }

        if( matches[k] >= cpts.size() ) {
          CF_ERROR("BUG");
          exit(1);
        }

        const cv::Point2f & rp =
            rpts[k].pt;

        const cv::Point2f & cp =
            cpts[matches[k]].pt;

        const cv::Scalar color =
            inliers[0][k] ? CV_RGB(250, 250, 32) :
                CV_RGB(250, 20, 20);

        cv::line(display20, rp, cp, color, 1,
            cv::LINE_8, 0);

        cv::rectangle(display20,
            cv::Point2f(rp.x - 1, rp.y - 1),
            cv::Point2f(rp.x + 1, rp.y + 1),
            CV_RGB(16, 160, 16),
            1,
            cv::LINE_4);


        const cv::Point2f cpw =
            warp(cp, current_frame->H);

        cv::line(display21, rp, cpw, color, 1,
            cv::LINE_8, 0);

        cv::rectangle(display21,
            cv::Point2f(rp.x - 1, rp.y - 1),
            cv::Point2f(rp.x + 1, rp.y + 1),
            CV_RGB(16, 160, 16),
            1,
            cv::LINE_4);

        if ( !inliers[0][k] ) {

          cv::line(display10, rp, cp, color, 1,
              cv::LINE_8, 0);

          cv::line(display11, rp, cpw, color, 1,
              cv::LINE_8, 0);

        }

      }
    }

    draw_epipole(display00, E);
    draw_epipole(display01, E);
    draw_epipole(display10, E);
    draw_epipole(display11, E);
    draw_epipole(display20, E);
    draw_epipole(display21, E);

    display.copyTo(lastDisplayImage);

    return true;
  }

  return false;

}

bool c_cte_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if( display_mask.needed() ) {
    display_mask.release();
  }

  lock_guard lock(mutex());

  if ( _output_options.save_progress_video ) {
    lastDisplayImage.copyTo(display_frame);
  }
  else if( _frames.size() > 1 ) {
    create_display_image(1, display_frame);
  }
  else {
    lastDisplayImage.copyTo(display_frame);
  }

  return true;
}


bool c_cte_pipeline::initialize()
{
  if ( !base::initialize() ) {
    CF_ERROR("c_cte_pipeline : base::initialize() fails");
    return false;
  }

  return true;
}

bool c_cte_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_cte_pipeline : base::initialize_pipeline() fails");
    return false;
  }

  cleanup_pipeline();
  _frames.clear();
  lastDisplayImage.release();

  _output_path =
      create_output_path(_output_options.output_directory);

  if( !(_keypoints_detector = create_sparse_feature_detector(_feature2d.detector)) ) {
    CF_ERROR("create_sparse_feature_detector() fails");
    return false;
  }

  if( !(_keypoints_descriptor = create_sparse_descriptor_extractor(_keypoints_detector, _feature2d.descriptor)) ) {
    CF_ERROR("create_sparse_descriptor_extractor() fails");
    return false;
  }

  return true;
}

void c_cte_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  _current_image.release();
  _current_mask.release();
}


bool c_cte_pipeline::run_pipeline()
{
  if ( !start_pipeline(_input_options.start_frame_index, _input_options.max_input_frames) ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  CF_DEBUG("_total_frames=%d", _total_frames);


  for( ; _processed_frames < _total_frames;  ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( _processed_frames > 0 && _input_options.read_step > 1 ) {
      if( !_input_sequence->seek(_input_sequence->current_pos() + _input_options.read_step - 1) ) {
        CF_FATAL("input_sequence->seek() fails\n");
        break;
      }
    }

    if( !_input_sequence->read(_current_image, &_current_mask) ) {
      CF_FATAL("input_sequence->read() fails\n");
      break;
    }

//    if ( !read_input_frame(_input_sequence, _input_options, _current_image, _current_mask, false, false) ) {
//      CF_ERROR("read_input_frame() fails");
//      break;
//    }

    if ( _current_image.empty() ) {
      continue;
    }

    if( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {

      lock_guard lock(mutex());

      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( true ) {
      lock_guard lock(mutex());
      _accumulated_frames = _processed_frames;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close_input_sequence();

  set_status_msg("FINISHED");


  return true;
}

bool c_cte_pipeline::process_current_frame()
{
  INSTRUMENT_REGION("");

  if( _feature2d.image_scale != 0 && _feature2d.image_scale != 1 ) {

    const double & scale =
        _feature2d.image_scale;

    cv::resize(_current_image, _current_image,
        cv::Size(), scale, scale,
        cv::INTER_AREA);

    if( !_current_mask.empty() ) {

      cv::resize(_current_mask, _current_mask,
          _current_image.size(), 0, 0,
          cv::INTER_AREA);

      cv::compare(_current_mask, 255, _current_mask,
          cv::CMP_GE);
    }
  }

  c_cte_frame::uptr current_frame(new c_cte_frame());

  if ( _keypoints_detector == _keypoints_descriptor ) {

    INSTRUMENT_REGION("detectAndCompute");
    _keypoints_detector->detectAndCompute(_current_image, _current_mask,
        current_frame->keypoints,
        current_frame->descriptors);
  }
  else {
    INSTRUMENT_REGION("detect_compute");
    _keypoints_detector->detect(_current_image, current_frame->keypoints, _current_mask);
    _keypoints_descriptor->compute(_current_image, current_frame->keypoints, current_frame->descriptors);
  }

  if( current_frame->descriptors.rows < 3 ) {
    CF_ERROR("[F %zu] Not enough keypoint descriptors extracted. Skip frame", _input_sequence->current_pos() - 1);
    return true;
  }

  if( !(current_frame->keypoints_matcher = create_sparse_feature_matcher(_keypoints_descriptor, _feature2d.matcher)) ) {
    CF_ERROR("create_sparse_feature_matcher() fails");
    return false;
  }

  current_frame->E.x =
      _current_image.cols / 2;

  current_frame->E.y =
      _current_image.rows / 2;

  current_frame->keypoints_matcher->train(current_frame->keypoints,
      current_frame->descriptors);

  _current_image.copyTo(current_frame->image);
  _current_mask.copyTo(current_frame->mask);

  if( !_frames.empty() ) {

    const c_cte_frame::uptr & back_frame =
        _frames.back();

    current_frame->E =
        back_frame->E;

    static const auto match_frames =
       [](c_cte_pipeline * cte, const c_cte_frame::uptr & current_frame, int rbeg, int rend) {

          INSTRUMENT_REGION("match_frames");

          std::vector<cv::DMatch> matches;

          for ( int i = rbeg; i < rend; ++i ) {

            const c_cte_frame::uptr & frame = cte->_frames[i];
            frame->inliers.emplace_back(cv::Mat1b(1, frame->keypoints.size(), (uint8_t) 255));

            frame->matches.emplace_back();
            std::vector<int32_t> & kpmatches = frame->matches.back();

            matches.clear();
            kpmatches.resize(frame->keypoints.size(), -1);

            current_frame->keypoints_matcher->match(frame->keypoints,
                frame->descriptors,
                matches);

            for ( const cv::DMatch & m : matches ) {
              kpmatches[m.queryIdx] = m.trainIdx;
            }
          }

        };

#if 0 // HAVE_TBB
    tbb::parallel_for(tbb_range(0, _frames.size()),
        [this, &current_frame](const tbb_range & range) {
          match_frames(this, current_frame, range.begin(), range.end());
        });

#else
    match_frames(this, current_frame, 0, _frames.size());
#endif
  }


  if ( true ) {
    lock_guard lock(mutex());
    _frames.emplace_back(std::move(current_frame));
  }

  const size_t max_context_size =
      std::max((size_t) 2, std::min((size_t) 128,
          _context_options.max_context_size));

  if( _frames.size() >= max_context_size ) {

    if ( !update_trajectory() ) {
      CF_ERROR("update_trajectory() fails");
      return false;
    }

    if( canceled() ) {
      return false;
    }

    if ( !save_progress_video() ) {
      CF_ERROR("save_progress_video() fails");
      return false;
    }

    if( true ) {
      lock_guard lock(mutex());

      c_cte_frame::uptr front_frame =
          std::move(_frames.front());

      _frames.pop_front();
    }
  }


  return true;
}

bool c_cte_pipeline::update_trajectory()
{
  using Vec2 =
      cv::Vec2d;

  static const auto pack_params =
      [](std::vector<double> & params, const cv::Vec3d & A) {
        if ( params.size() != 5 ) {
          params.resize(5);
        }
        params[0] = A[0];
        params[1] = A[1];
        params[2] = A[2];
      };

  static const auto unpack_params =
      [](const std::vector<double> & params, cv::Vec3d & A) {
        A[0] = params[0];
        A[1] = params[1];
        A[2] = params[2];
      };

  class c_levmar_solver_callback:
      public c_levmar_solver::callback
  {
    c_cte_pipeline * _cte;
    size_t _current_frame_index;
    size_t _reference_frame_index;
    cv::Matx33d _camera_matrix, _camera_matrix_inv;
    double _robust_threshold = 0; // (double) std::numeric_limits<float>::max();
    double _erfactor = 1.0;

    std::vector<float> rptsx;
    std::vector<float> rptsy;
    std::vector<float> cptsx;
    std::vector<float> cptsy;
    std::vector<uint32_t> ks;

    cv::Point2f E;

  public:

    c_levmar_solver_callback(c_cte_pipeline * cte, size_t current_frame_index, size_t reference_frame_index,
        const cv::Matx33d & camera_matrix, const cv::Matx33d & camera_matrix_inv) :
        _cte(cte),
        _current_frame_index(current_frame_index),
        _reference_frame_index(reference_frame_index),
        _camera_matrix(camera_matrix),
        _camera_matrix_inv(camera_matrix_inv)
    {

      const std::deque<c_cte_frame::uptr> & frames =
          cte->_frames;

      const c_cte_frame::uptr & current_frame =
          frames[current_frame_index];

      const c_cte_frame::uptr & reference_frame =
          frames[reference_frame_index];

      const std::vector<int32_t> & matches =
          reference_frame->matches[current_frame_index - reference_frame_index - 1];

      for( size_t k = 0, nk = matches.size(); k < nk; ++k ) {
        if( matches[k] >= 0 ) {

          const cv::Point2f & rp =
              reference_frame->keypoints[k].pt;

          const cv::Point2f & cp =
              current_frame->keypoints[matches[k]].pt;

          rptsx.emplace_back(rp.x);
          rptsy.emplace_back(rp.y);

          cptsx.emplace_back(cp.x);
          cptsy.emplace_back(cp.y);

          ks.emplace_back(k);
        }
      }
    }

    void set_robust_threshold(double v)
    {
      _robust_threshold = v;
    }

    double robust_threshold() const
    {
      return _robust_threshold;
    }

    void set_erfactor(double v)
    {
      _erfactor = v;
    }

    double erfactor() const
    {
      return _erfactor;
    }

    bool allow_tbb() const final
    {
      return true;
    }

    // test
    void set_epipole(const cv::Point2f & e )
    {
      E = e;
    }
    const cv::Point2f & epipole() const
    {
      return E;
    }

    // test
    bool estimate_epipole_location(const cv::Matx33f & H, cv::Point2f & E) const
    {
      c_lse2_estimate<float> lse;

      const int nj =
          ks.size();

      const cv::Mat1b & inliers =
          _cte->_frames[_reference_frame_index]->inliers[_current_frame_index - _reference_frame_index - 1];

      const float & h00 = H(0, 0);
      const float & h01 = H(0, 1);
      const float & h02 = H(0, 2);

      const float & h10 = H(1, 0);
      const float & h11 = H(1, 1);
      const float & h12 = H(1, 2);

      const float & h20 = H(2, 0);
      const float & h21 = H(2, 1);
      const float & h22 = H(2, 2);

      for( int j = 0; j < nj; ++j ) {
        if( inliers[0][ks[j]] ) {

          const float & rx = rptsx[j];
          const float & ry = rptsy[j];

          const float cz = (h20 * cptsx[j] + h21 * cptsy[j] + h22);
          const float cx = (h00 * cptsx[j] + h01 * cptsy[j] + h02) / cz;
          const float cy = (h10 * cptsx[j] + h11 * cptsy[j] + h12) / cz;

          lse.update(cy - ry, rx - cx, rx * cy - ry * cx);
        }
      }


      if( !lse.compute(E.x, E.y) ) {
        CF_ERROR("lse.compute() fails");
        return false;
      }


      return true;
    }

  protected:

    bool compute_reprojection_errors(const std::vector<double> & p, std::vector<double> & e)
    {
      //INSTRUMENT_REGION("");

      cv::Vec3d A;

      unpack_params(p, A);

      const cv::Matx33f H =
          _camera_matrix * build_rotation(A) * _camera_matrix_inv;

      const size_t nj =
          ks.size();

      std::vector<float> ervx(nj);
      std::vector<float> ervy(nj);
      std::vector<float> flowx(nj);
      std::vector<float> flowy(nj);
      std::vector<float> flowz(nj);

      const float & h20 = H(2, 0);
      const float & h21 = H(2, 1);
      const float & h22 = H(2, 2);

      const float & h00 = H(0, 0);
      const float & h01 = H(0, 1);
      const float & h02 = H(0, 2);

      const float & h10 = H(1, 0);
      const float & h11 = H(1, 1);
      const float & h12 = H(1, 2);

      for( size_t j = 0; j < nj; ++j ) {
        flowz[j] = h20 * cptsx[j] + h21 * cptsy[j] + h22;
      }

      for( size_t j = 0; j < nj; ++j ) {
        flowx[j] = (h00 * cptsx[j] + h01 * cptsy[j] + h02) / flowz[j] - rptsx[j];
        ervx[j] = rptsx[j] - (float) E.x;
      }

      for( size_t j = 0; j < nj; ++j ) {
        flowy[j] = (h10 * cptsx[j] + h11 * cptsy[j] + h12) / flowz[j] - rptsy[j];
        ervy[j] = rptsy[j] - (float) E.y;
      }


      e.resize(nj);

      for( size_t j = 0; j < nj; ++j ) {

        // epipolar distance
        const float er2 =
            ervx[j] * ervx[j] + ervy[j] * ervy[j];

        // displacement perpendicular to epipolar line
        //  elateral = flow.dot(Vec2(-erv[1], erv[0]))
        const float elateral =
            (ervx[j] * flowy[j] - ervy[j] * flowx[j]);

        // displacement along epipolar line
        //  eradial = flow.dot(erv)
        float eradial =
            (ervx[j] * flowx[j] + ervy[j] * flowy[j]);

        if( eradial > 0 ) {
          eradial /= (1. + er2 / (_erfactor * _erfactor));
        }

        e[j] =
            std::sqrt((elateral * elateral +
                eradial * eradial) / er2);

      }

      return true;
    }

  public:

    bool compute(const std::vector<double> & p, std::vector<double> & rhs,
        cv::Mat1d * J, bool * have_analytical_jac) final
    {
      const size_t nj =
          ks.size();

      const c_cte_frame::uptr & reference_frame =
         _cte->_frames[_reference_frame_index];

      const cv::Mat1b & inliers =
          reference_frame->inliers[_current_frame_index - _reference_frame_index - 1];

      compute_reprojection_errors(p, rhs);

      if( _robust_threshold <= 0 ) {
        for( size_t j = 0; j < nj; ++j ) {
          if( !inliers[0][ks[j]] ) {
            rhs[j] = 0;
          }
        }
      }
      else {

        const double scale =
            _robust_threshold / M_LN2;

        for( size_t j = 0; j < nj; ++j ) {
          rhs[j] = inliers[0][ks[j]] ? scale * std::log(1. + rhs[j] / _robust_threshold) : 0;
        }
      }

      return true;
    }

    int mark_outliers(const std::vector<double> & p, double thresh)
    {
      std::vector<double> e;

      const size_t nj =
          ks.size();

      const c_cte_frame::uptr & reference_frame =
         _cte->_frames[_reference_frame_index];

      cv::Mat1b & inliers =
          reference_frame->inliers[_current_frame_index - _reference_frame_index - 1];

      int num_outliers = 0;

      compute_reprojection_errors(p, e);

      for( size_t j = 0; j < nj; ++j ) {

        uint8_t & inlier =
            inliers[0][ks[j]];

        if ( e[j] < thresh ) {
          inlier = 255U;
        }
        else if( inlier ) {
          inlier = 0;
          ++num_outliers;
        }
      }

      return num_outliers;
    }

  };

  if( _frames.size() < 2 ) {
    return true;
  }

  INSTRUMENT_REGION("");

  c_levmar_solver lm(100, 1e-12);
  std::vector<double> p;
  cv::Vec3d A;
  cv::Point2f E;
  cv::Matx33f H;
  bool EOk = false;

  const cv::Matx33d & camera_matrix =
      _camera_options.camera_intrinsics.camera_matrix;

  const cv::Matx33d camera_matrix_inv =
      _camera_options.camera_intrinsics.camera_matrix.inv();

  if( _pose_estimation.max_levmar_iterations > 0 ) {
    lm.set_max_iterations(_pose_estimation.max_levmar_iterations);
  }

  const c_cte_frame::uptr & reference_frame =
        _frames.front();

  const c_cte_frame::uptr & current_frame =
      _frames[1];

//  for( auto & inliers : reference_frame->inliers ) {
//    inliers.setTo(255);
//  }

//  if( _frames.size() > 2 ) {
//
//    const size_t jmax =
//        _frames.size() - 1;
//
//    for( size_t k = 0, nk = reference_frame->keypoints.size(); k < nk; ++k ) {
//
//      int m =
//          reference_frame->matches[0][k];
//
//      if( m >= 0 ) {
//        for( size_t j = 1; j < jmax; ++j ) {
//          if( (m = _frames[j]->matches[0][m]) < 0 ) {
//            break;
//          }
//        }
//
//        if( m < 0 || m != reference_frame->matches[jmax - 1][k] ) {
//          reference_frame->inliers[0][0][k] = 0;
//        }
//      }
//    }
//  }


  CF_DEBUG("*");

  const int max_iterations =
      std::max(1, _pose_estimation.max_iterations);

  const cv::Vec3d & Ainitial =
      current_frame->A;

  const cv::Point2f & Einitial =
      current_frame->E;

  CF_DEBUG("INITIAL A=(%+g %+g %+g) E=(%+g %+g)",
      Ainitial(0) * 180 / CV_PI, Ainitial(1) * 180 / CV_PI, Ainitial(2) * 180 / CV_PI,
      Einitial.x, Einitial.y);

  for( int iteration = 0; iteration < max_iterations; ++iteration ) {

    if( canceled() ) {
      return false;
    }

    //if ( iteration < 2 )
    {
      pack_params(p, Ainitial);
    }

    c_levmar_solver_callback callback(this, 1, 0,
        camera_matrix, camera_matrix_inv);

    callback.set_epipole(Einitial);

    if( _pose_estimation.levmar_epsf >= 0 ) {
      lm.set_epsfn(_pose_estimation.levmar_epsf);
    }

    if( _pose_estimation.levmar_epsx >= 0 ) {
      lm.set_epsx(_pose_estimation.levmar_epsx);
    }

    if( _pose_estimation.erfactor > 0 ) {
      callback.set_erfactor(_pose_estimation.erfactor);
    }

    callback.set_robust_threshold(_pose_estimation.robust_threshold);

    CF_DEBUG("C lm.run[%d]", iteration);

    const int num_iterations =
        lm.run(callback, p);

    if( canceled() ) {
      return false;
    }

    const double rmse =
        lm.rmse();

    const int num_outliers =
        callback.mark_outliers(p, 3 * rmse);

    unpack_params(p, A);
    H = camera_matrix * build_rotation(A) * camera_matrix_inv;
    EOk = callback.estimate_epipole_location(H, E);

    if( true ) {

      CF_DEBUG("lm.run[%d]: iterations=%d rmse=%g num_outliers=%d A=(%g %g %g) E=(%g %g) EOk=%d",
          iteration,
          num_iterations,
          rmse,
          num_outliers,
          A[0] * 180 / CV_PI, A[1] * 180 / CV_PI, A[2] * 180 / CV_PI,
          E.x, E.y,
          EOk);
    }

    if( num_outliers < 1 ) {
      break;
    }

  }

  if( true ) {

    //unpack_params(p, A);
    //E = callback.estimate_epipole_location(camera_matrix * build_rotation(A) * camera_matrix_inv);

    lock_guard lock(mutex());

    current_frame->A = A;
    current_frame->H = H;

    if ( EOk ) {
      current_frame->E =
          (Einitial * _pose_estimation.ew + E) / (_pose_estimation.ew + 1);
    }


    CF_DEBUG("<UPDATE>");
    CF_DEBUG("[%zu] A=(%g %g %g) E=(%g %g) Eok=%d", 1, A[0] * 180 / CV_PI, A[1] * 180 / CV_PI, A[2] * 180 / CV_PI, E.x, E.y, EOk);
    CF_DEBUG("</UPDATE>");
  }


  CF_DEBUG("*");

  if( canceled() ) {
    return false;
  }

  return true;
}


