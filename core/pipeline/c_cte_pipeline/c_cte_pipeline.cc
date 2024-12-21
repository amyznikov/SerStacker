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


c_cte_pipeline::c_cte_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
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

    cv::Mat display;

    lock_guard lock(mutex());

    for( size_t i = 1; i < _frames.size(); ++i ) {

      if( !create_display_image(i, display) ) {
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

bool c_cte_pipeline::create_display_image(size_t back_frame_index, cv::OutputArray display_frame)
{
  if( _frames.size() > 1 && back_frame_index < _frames.size() ) {

    const c_cte_frame::uptr & front_frame =
        _frames.front();

    const c_cte_frame::uptr & back_frame =
        _frames[back_frame_index];

    const cv::Size frame_size =
        front_frame->image.size();

    const cv::Size display_size(frame_size.width * 2,
        frame_size.height * 3);

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

    cv::Mat3b front_image_display =
        display(roi[0][0]);

    cv::Mat3b back_image_display =
        display(roi[1][0]);

    cv::Mat3b epipoles_display =
        display(roi[0][1]);

    cv::Mat3b blend_display =
        display(roi[1][1]);

    cv::Mat3b flow_display =
        display(roi[2][0]);

    //    cv::Mat3b xx_display =
    //        display(roi[2][1]);

    copyimage(back_frame->image,
        back_image_display);

    copyimage(back_frame->image,
        epipoles_display);

    // cv::WARP_INVERSE_MAP|
    cv::warpPerspective(back_image_display, back_image_display, back_frame->H, frame_size,
        cv::INTER_LINEAR);

    copyimage(front_frame->image,
        front_image_display);

    cv::addWeighted(back_image_display, 0.5,
        front_image_display, 0.5,
        0,
        blend_display);

    cv::drawKeypoints(front_image_display, front_frame->keypoints,
        front_image_display,
        cv::Scalar::all(-1),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    if( true ) {

      std::vector<cv::KeyPoint> & Pi =
          front_frame->keypoints;

      std::vector<cv::KeyPoint> & Pj =
          back_frame->keypoints;

      const std::vector<cv::DMatch> & Mij =
          front_frame->matches[back_frame_index - 1];

      const cv::Mat1b & inliers =
          front_frame->inliers[back_frame_index - 1];

      for( size_t k = 0; k < Mij.size(); ++k ) {

        const cv::DMatch & m =
            Mij[k];

        const cv::Point2f pi =
            Pi[m.queryIdx].pt;

        const cv::Point2f pj =
            warp(Pj[m.trainIdx].pt, back_frame->H);


        const cv::Scalar color =
            inliers[0][k] ? CV_RGB(250, 250, 32) :
                CV_RGB(250, 20, 20);

        cv::line(display, pi, pj + cv::Point2f(0, frame_size.height), color, 1,
            cv::LINE_8, 0);

        cv::line(flow_display, pi, pj, color, 1,
            cv::LINE_8, 0);

      }
    }

    if( true ) {

      const cv::Vec3d dT =
          back_frame->T - front_frame->T;

      const cv::Point2d E =
          compute_epipole(_camera_options.camera_intrinsics.camera_matrix, dT);

      draw_epipole(epipoles_display, E);
      draw_epipole(blend_display, E);
      draw_epipole(flow_display, E);
    }

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

  if( _frames.size() < 2 || !create_display_image(_frames.size() - 1, display_frame) ) {
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

    _keypoints_detector->detectAndCompute(_current_image, _current_mask,
        current_frame->keypoints,
        current_frame->descriptors);
  }
  else {
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

  current_frame->keypoints_matcher->train(&current_frame->keypoints,
      current_frame->descriptors);


  if( !_frames.empty() ) {

#if HAVE_TBB
    tbb::parallel_for(tbb_range(0, _frames.size()),
        [this, &current_frame](const tbb_range & range) {

          for ( int i = range.begin(); i < range.end(); ++i ) {

            const c_cte_frame::uptr & frame =
                _frames[i];

            frame->matches.emplace_back();

            std::vector<cv::DMatch> & matches =
                frame->matches.back();

            current_frame->keypoints_matcher->match(&frame->keypoints,
                frame->descriptors,
                matches);

            frame->inliers.emplace_back(cv::Mat1b(1, matches.size(),
                    (uint8_t) 255U));

          }

        });

#else
    for( const c_cte_frame::uptr & frame : _frames ) {

      frame->matches.emplace_back();

      std::vector<cv::DMatch> & matches =
          frame->matches.back();

      current_frame->keypoints_matcher->match(&frame->keypoints,
          frame->descriptors,
          matches);

      frame->inliers.emplace_back(cv::Mat1b(1, matches.size(),
              (uint8_t) 255U));
    }
#endif
  }


  if ( true ) {

    lock_guard lock(mutex());

    if( !_frames.empty() ) {

      const c_cte_frame::uptr & back_frame =
          _frames.back();

      current_frame->A =
          back_frame->A;

      current_frame->T =
          back_frame->T; // + cv::Vec3d(0, 0, 1);
    }

    _current_image.copyTo(current_frame->image);
    _current_mask.copyTo(current_frame->mask);
    _frames.emplace_back(std::move(current_frame));
  }


  const size_t max_context_size =
      std::max((size_t) 2, std::min((size_t) 128,
          _context_options.max_context_size));

  if( _frames.size() >= max_context_size ) {

    CF_DEBUG("[%d] _frames.size()=%zu / %zu", _input_sequence->current_pos() - 1,  _frames.size(), max_context_size);

    for( size_t i = 0; i < _frames.size(); ++i ) {

      const c_cte_frame::uptr & F =
          _frames[i];

      const cv::Vec3d fT =
          F->T;

      CF_DEBUG("[%zu] matches=%zu A=(%+g %+g %+g) T=(%+g %+g %+g)", i,
          F->matches.size(),
          F->A[0], F->A[1], F->A[2],
          fT[0], fT[1], fT[2]);
    }

    CF_DEBUG("[%d] C update_trajectory() _frames.size()=%zu", _input_sequence->current_pos() - 1, _frames.size());
    if ( !update_trajectory() ) {
      CF_ERROR("update_trajectory() fails");
      return false;
    }
    CF_DEBUG("[%d] R update_trajectory() _frames.size()=%zu", _input_sequence->current_pos() - 1, _frames.size());

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
  static const auto pack_param =
      [](std::vector<double> & params, const cv::Vec3d & A, const UVec3d & T) {
        for( int i = 0; i < 3; ++i ) {
          params.emplace_back(A[i]);
        }
        params.emplace_back(T.phi());
        params.emplace_back(T.theta());
      };

  static const auto unpack_param =
      [](const std::vector<double> & params, size_t index, cv::Vec3d & A, UVec3d & T) {
        for ( size_t i = 0; i < 3; ++i ) {
          A[i] = params[5 * index + i];
        }
        T.phi() = params[5 * index + 3 + 0];
        T.theta() = params[5 * index + 3 + 1];
      };

  static const auto compute_projection_error =
      [](const cv::Point2d & cp, const cv::Point2d & rp,
          const cv::Matx33d & H, const cv::Point2d & E,
          EPIPOLAR_MOTION_DIRECTION direction) -> double
  {
    using Vec2 =
        cv::Vec2d;

    using Vec3 =
        cv::Vec3d;

    const Vec3 cv =
        H * Vec3(cp.x, cp.y, 1);

    if ( is_bad_number(cv)) {
      CF_ERROR("bad cv: [%g %g %g]", cv[0], cv[1], cv[2]);
    }


    // current point relative to epipole
    const Vec2 ecv(cv[0] / cv[2] - E.x,
        cv[1] / cv[2] - E.y);

    if ( is_bad_number(ecv) ) {
      CF_ERROR("bad ecv: [%g %g] cv=[%g %g %g] E=[%g %g]", ecv[0], ecv[1], cv[0], cv[1], cv[2], E.x, E.y);
    }

    // reference point relative to epipole
    const Vec2 erv(rp.x - E.x,
        rp.y - E.y);

    if ( is_bad_number(erv) ) {
      CF_ERROR("bad erv: [%g %g]", erv[0], erv[1]);
    }

    // displacement vector
    const Vec2 flow =
        (ecv - erv) / std::sqrt(erv[0] * erv[0] + erv[1] * erv[1]);

    if ( is_bad_number(flow) ) {
      CF_ERROR("bad flow: [%g %g]", flow[0], flow[1]);
    }

    // displacement perpendicular to epipolar line
    double rhs =
        std::abs(flow.dot(Vec2(-erv[1], erv[0])));

    if ( is_bad_number(rhs) ) {
      CF_ERROR("bad rhs: [%g]", rhs);
    }

    // displacement along epipolar line
    if ( 1 ) {
      switch (direction) {
        case EPIPOLAR_DIRECTION_FORWARD: {
          const double rflow =
              flow.dot(erv);
          if( rflow < 0 ) {
            rhs -= rflow;
          }
          break;
        }
        case EPIPOLAR_DIRECTION_BACKWARD: {
          const double rflow =
              flow.dot(erv);
          if( rflow > 0 ) {
            rhs += rflow;
          }
          break;
        }
      }
    }

    return rhs;
  };



  class c_levmar_solver_callback:
      public c_levmar_solver::callback
  {
    c_cte_pipeline * _cte;
    cv::Matx33d _camera_matrix, _camera_matrix_inv;
    double _robust_threshold = 1.0; // (double) std::numeric_limits<float>::max();

  public:

    c_levmar_solver_callback(c_cte_pipeline * cte) :
        _cte(cte)
    {
      _camera_matrix = _cte->_camera_options.camera_intrinsics.camera_matrix;
      _camera_matrix_inv = _cte->_camera_options.camera_intrinsics.camera_matrix.inv();
    }

    const cv::Matx33d & camera_matrix() const
    {
      return _camera_matrix;
    }

    const cv::Matx33d & camera_matrix_inv() const
    {
      return _camera_matrix_inv;
    }

    void set_robust_threshold(double v)
    {
      _robust_threshold = v;
    }

    double robust_threshold() const
    {
      return _robust_threshold;
    }

  protected:
    typedef std::function<bool (double, cv::Mat1b &, int k)> ComputeCallback;

    bool compute_rhs(const std::vector<double> & p, const ComputeCallback & callback)
    {
      const EPIPOLAR_MOTION_DIRECTION motion_direction =
          EPIPOLAR_DIRECTION_FORWARD;

      const std::deque<c_cte_frame::uptr> & frames =
          _cte->_frames;

      const c_cte_frame::uptr & front_frame =
          frames.front();

      const size_t num_frames =
          frames.size();

      const double robust_threshold =
          _cte->_pose_estimation.robust_threshold > 0 ? _cte->_pose_estimation.robust_threshold :
              (double) std::numeric_limits<float>::max();

      std::vector<cv::Vec3d> A(num_frames, cv::Vec3d::all(0));
      std::vector<UVec3d> T(num_frames);

      for( size_t i = 1; i < num_frames; ++i ) {
        unpack_param(p, i - 1, A[i], T[i]);
      }

//      CF_DEBUG("*");
//      for( size_t i = 0; i < num_frames; ++i ) {
//
//        const cv::Vec3d Tv =
//            T[i];
//
//        CF_DEBUG("XX [%zu] T=(%g %g %g) L=%g  A=(%g %g %g)", i,
//            Tv[0], Tv[1], Tv[2],
//            cv::norm(Tv),
//            A[i][0], A[i][1], A[i][2]);
//      }
//      CF_DEBUG("*");

      for( size_t i = 0; i < 1 /*num_frames - 1*/; ++i ) {

        const c_cte_frame::uptr & Fi =
            frames[i];

        //CF_DEBUG("H i=%zu Fi->matches.size()=%zu", i, Fi->matches.size());

        for( size_t j = i + 1; j < num_frames; ++j ) {

          const c_cte_frame::uptr & Fj =
              frames[j];

          const cv::Vec3d dA =
              A[j] - A[i];

          const cv::Vec3d dT =
              T[j] - T[i];

          const cv::Matx33d Hji =
              _camera_matrix * build_rotation(dA) * _camera_matrix_inv;

          const cv::Point2d Eji =
              compute_epipole(_camera_matrix, dT);

          const std::vector<cv::DMatch> & Mij =
              Fi->matches[j - i - 1];

          cv::Mat1b & inliers =
              Fi->inliers[j - i - 1];


//          CF_DEBUG("dT[%zu]=(%g %g %g) L=%g dA=(%g %g %g) Mij.size=%zu", i,
//              dT[0], dT[1], dT[2],
//              cv::norm(dT),
//              dA[0], dA[1], dA[2],
//              Mij.size());

          if( is_bad_number(Eji) ) {

            const cv::Vec3d Tvi =
                T[i];

            const cv::Vec3d Tvj =
                T[j];

            CF_ERROR("i=%zu j=%zu bad Eji=(%g %g) T[j]=(%g %g %g) T[i]=(%g %g %g) Mij.size=%zu",
                i, j,
                Eji.x, Eji.y,
                Tvj[0], Tvj[1], Tvj[2],
                Tvi[0], Tvi[1], Tvi[2],
                Mij.size());

            if( is_bad_number(Eji) ) {
              return false;
            }
          }


          for( size_t k = 0, nk = Mij.size(); k < nk; ++k ) {

            if( !inliers[0][k] ) {
              continue;
            }

            const cv::DMatch & m =
                Mij[k];

            const cv::Point2f & pj =
                Fj->keypoints[m.trainIdx].pt;

            if( is_bad_number(pj) ) {
              CF_ERROR("bad pj=(%g %g) at k=%zu", pj.x, pj.y, k);
              return false;
            }

            const cv::Point2f & pi =
                Fi->keypoints[m.queryIdx].pt;

            if( is_bad_number(pi) ) {
              CF_ERROR("bad pi=(%g %g) at k=%zu", pi.x, pi.y, k);
              return false;
            }

            const double err =
                compute_projection_error(pj, pi,
                    Hji, Eji,
                    motion_direction);

            if( is_bad_number(err) ) {
              CF_ERROR("bad data at k=%zu err=%g", k, err);
              return false;
            }

            callback(err, inliers, k);
          }
        }
      }

      return true;
    }

  public:
    bool compute(const std::vector<double> & p, std::vector<double> & rhs,
        cv::Mat1d * J, bool * have_analytical_jac) final
    {
      if( have_analytical_jac ) {
        *have_analytical_jac = false;
      }

      rhs.clear();

      const bool fOk =
          compute_rhs(p, [this, &rhs](double err, cv::Mat1b&, int) -> bool {
              // rhs.emplace_back(std::min(err, _robust_threshold));
              rhs.emplace_back(log(1 + err * _robust_threshold));
              return true;
            });

      return fOk;
    }

    int mark_outliers(const std::vector<double> & p, double rmse)
    {
      int num_outliers = 0;

      bool fOk =
          compute_rhs(p, [this, rmse, &num_outliers](double err, cv::Mat1b & inliers, int k) -> bool {

            if ( log(1 + err * _robust_threshold) > 3 * rmse ) {
              inliers[0][k] = 0;
              ++num_outliers;
            }

            return true;
          });

      return fOk ? num_outliers : -1;
    }

    void clear_outliers()
    {
      for( const c_cte_frame::uptr & Fi : _cte->_frames ) {
        for( auto & inliers : Fi->inliers ) {
          inliers.setTo(255);
        }
      }
    }


  };

  const c_cte_frame::uptr & F0 =
      _frames.front();

  c_levmar_solver lm(100, 1e-7);
  std::vector<double> p;
  //int num_iterations = 0;

  if( _pose_estimation.max_levmar_iterations > 0 ) {
    lm.set_max_iterations(_pose_estimation.max_levmar_iterations);
  }

  const c_cte_frame::uptr & front_frame =
      _frames.front();

  for( size_t j = 1, m = _frames.size(); j < m; ++j ) {

    const c_cte_frame::uptr & frame =
        _frames[j];

    pack_param(p, frame->A - front_frame->A,
        frame->T - front_frame->T);
  }


  const int max_iterations =
      std::max(1, _pose_estimation.max_iterations);

  for( int iteration = 0; iteration < max_iterations; ++iteration ) {

    if( canceled() ) {
      return false;
    }

    c_levmar_solver_callback callback(this);

    const int iteration_scale =
        (1 << (max_iterations - iteration));

    if ( _pose_estimation.robust_threshold > 0 ) {
      callback.set_robust_threshold(_pose_estimation.robust_threshold); // * iteration_scale
    }

    if( _pose_estimation.levmar_epsf >= 0 ) {
      lm.set_epsfn(_pose_estimation.levmar_epsf * iteration_scale);
    }

    if( _pose_estimation.levmar_epsx >= 0 ) {
      lm.set_epsx(_pose_estimation.levmar_epsx * iteration_scale);
    }

    CF_DEBUG("C lm.run[%d]", iteration);

    const int num_iterations =
        lm.run(callback, p);

    if( canceled() ) {
      return false;
    }

    const double rmse =
        lm.rmse();


    const int num_outliers =
        callback.mark_outliers(p, rmse);

    CF_DEBUG("lm.run[%d]: iterations=%d rmse=%g num_outliers=%d",
        iteration,
        num_iterations,
        rmse,
        num_outliers);

    if ( num_outliers < 1 ) {
      break;
    }
  }


  CF_DEBUG("*");
  if( true ) {

    const cv::Matx33d & camera_matrix =
        _camera_options.camera_intrinsics.camera_matrix;

    const cv::Matx33d camera_matrix_inv =
        _camera_options.camera_intrinsics.camera_matrix.inv();


    lock_guard lock(mutex());

    CF_DEBUG("<EXTRACT>");
    for( size_t j = 1; j < _frames.size(); ++j ) {

      const c_cte_frame::uptr & frame =
          _frames[j];

      unpack_param(p, j - 1, frame->A, frame->T);

      const cv::Vec3d & dA =
          frame->A;

      const cv::Vec3d dT =
          frame->T.vec();

      CF_DEBUG("[%zu] dT=(%g %g %g) dA=(%g %g %g)", j, dT[0], dT[1], dT[2], dA[0], dA[1], dA[2]);
    }
    CF_DEBUG("</EXTRACT>");

    CF_DEBUG("<UPDATE>");
    for( size_t j = 0; j < _frames.size(); ++j ) {

      const c_cte_frame::uptr & frame =
          _frames[j];

      if( j != 0 ) {
        frame->A = frame->A + front_frame->A;
        frame->T = frame->T + front_frame->T;
      }

      const cv::Vec3d Tv =
          frame->T.vec();

      const cv::Vec3d A =
          frame->A;

      const cv::Vec3d dA =
          frame->A - front_frame->A;

      const UVec3d dT =
          (frame->T - front_frame->T);

      const cv::Vec3d dTv =
          dT.vec();

      const double L =
          cv::norm(dTv);

      frame->H =
          camera_matrix * build_rotation(dA) * camera_matrix_inv;

      const cv::Point2d E =
          compute_epipole(_camera_options.camera_intrinsics.camera_matrix,
              dTv);

      CF_DEBUG("[%zu] "
          "T=(%g %g %g) "
          "dT=(%g %g %g) "
          "dL=%g "
          "A=(%g %g %g) "
          "dA=(%g %g %g) "
          "E=(%g %g)", j,
          Tv[0], Tv[1], Tv[2],
          dTv[0], dTv[1], dTv[2],
          L,
          A[0], A[1], A[2],
          dA[0], dA[1], dA[2],
          E.x, E.y);
    }
    CF_DEBUG("</UPDATE>");
  }
  CF_DEBUG("*");

  if( canceled() ) {
    return false;
  }

  return true;
}


