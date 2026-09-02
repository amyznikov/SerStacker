/*
 * c_canvas_average_pipeline.cc
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#include "c_canvas_average_pipeline.h"
#include <core/proc/unsharp_mask.h>
#include <core/proc/pixtype.h>
#include <core/io/load_image.h>
#include <core/io/save_image.h>
#include <core/readdir.h>

c_canvas_average_pipeline::c_canvas_average_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const std::string& c_canvas_average_pipeline::get_class_name() const
{
  return class_name();
}

const std::string& c_canvas_average_pipeline::class_name()
{
  static const std::string _classname = "canvas_average";
  return _classname;
}

const std::string& c_canvas_average_pipeline::tooltip()
{
  static const std::string _tooltip =
      "<strong>c_canvas_average_pipeline.</strong><br>"
          "test for canvas average<br>";
  return _tooltip;
}

bool c_canvas_average_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_image_stacking_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {

    SERIALIZE_OPTION(section, save, _registration_options, canvasSize);
    SERIALIZE_OPTION(section, save, _registration_options, eccUnsharpMaskSigma);
    SERIALIZE_OPTION(section, save, _registration_options, eccUnsharpMaskAlpha);

    SERIALIZE_OPTION(section, save, _registration_options, motion_type);
    SERIALIZE_OPTION(section, save, _registration_options, enable_star_registration);
    SERIALIZE_OPTION(section, save, _registration_options, enable_ecc_registration);
    SERIALIZE_OPTION(section, save, _registration_options, enable_eccflow_registration);

    if ( auto group = SERIALIZE_GROUP(section, save, "star_registration") ) {
      if( auto g = SERIALIZE_GROUP(group, save, "star_detection") ) {
        serialize_simple_star_detector_options(g, save, _registration_options.star_detection);
      }
      if( auto g = SERIALIZE_GROUP(group, save, "triangle_extractor") ) {
        serialize_triangle_extractor_options(g, save, _registration_options.triangle_extractor);
      }
      if( auto g = SERIALIZE_GROUP(group, save, "triangle_matcher") ) {
        serialize_triangle_matcher_options(g, save, _registration_options.triangle_matcher);
      }
    }

    if ( auto group = SERIALIZE_GROUP(section, save, "ecch") ) {
      serialize_ecch_options(group, save, _registration_options.ecch);
    }

    if ( auto group = SERIALIZE_GROUP(section, save, "eccflow") ) {
      serialize_eccflow_options(group, save, _registration_options.eccflow);
    }
  }

  if( (section = SERIALIZE_GROUP(settings, save, "average_options")) ) {

    if( auto group = SERIALIZE_GROUP(section, save, "sharpness_measure") ) {
      serialize_local_variance_map_options(group, save, _average_options.sharpness_measure);
    }

    if( auto group = SERIALIZE_GROUP(section, save, "update") ) {
      SERIALIZE_OPTION(group, save, _average_options.update, interpolation);
    }
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, autoSaveInterval);

    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, output_file_name);

    SERIALIZE_OPTION(section, save, _output_options, save_substacks);
    if( (subsection = SERIALIZE_GROUP(section, save, "substack_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, substack_frames);
      SERIALIZE_OPTION(subsection, save, _output_options, substack_output_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_input_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_input_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_input_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_progress_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_progress_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_reference_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_reference_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_reference_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_weights_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_weights_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_weights_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, display_scale);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_canvas_average_input_options> & ctx)
{
  using S = c_canvas_average_input_options;
  ctlbind(ctls, as_base<c_image_stacking_pipeline_base_input_options>(ctx));
  //ctlbind(ctls, "ecc_image_processor", ctx(&S::ecc_image_processor));
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_canvas_average_registration_options> & ctx)
{
  using S = c_canvas_average_registration_options;

  ctlbind(ctls, "canvasSize", ctx(&S::canvasSize), "");
  ctlbind(ctls, "eccUnsharpMaskSigma", ctx(&S::eccUnsharpMaskSigma), "");
  ctlbind(ctls, "eccUnsharpMaskAlpha", ctx(&S::eccUnsharpMaskAlpha), "");

  ctlbind(ctls, "motion_type", ctx(&S::motion_type), "");

  ctlbind_expandable_group(ctls, "Stars", "");
    ctlbind(ctls, "Enable star registration", ctx(&S::enable_star_registration), "");
    ctlbind_expandable_group(ctls, "Star Detection", "");
      ctlbind(ctls, ctx(&S::star_detection));
    ctlbind_end_group(ctls);
    ctlbind_expandable_group(ctls, "Triangle extraction", "");
      ctlbind(ctls, ctx(&S::triangle_extractor));
    ctlbind_end_group(ctls);
    ctlbind_expandable_group(ctls, "Triangle Matching", "");
      ctlbind(ctls, ctx(&S::triangle_matcher));
    ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "ECC", "");
    ctlbind(ctls, "Enable ecc", ctx(&S::enable_ecc_registration), "");
    ctlbind(ctls, ctx(&S::ecch));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "ECCFLOW", "");
    ctlbind(ctls, "Enable eccflow", ctx(&S::enable_eccflow_registration), "");
    ctlbind(ctls, ctx(&S::eccflow));
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_canvas_average_update_options> & ctx)
{
  using S = c_canvas_average_update_options;

  ctlbind_expandable_group(ctls, "Sharpness measure", "");
    ctlbind(ctls, CTL_CONTEXT(ctx, sharpness_measure));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Weighted average update", "");
    ctlbind(ctls, "interpolation", CTL_CONTEXT(ctx, update.interpolation));
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_canvas_average_output_options> & ctx)
{
  using S = c_canvas_average_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "display_scale", ctx(&S::display_scale), "");
  ctlbind_browse_for_directory(ctls, "output_directory", ctx(&S::output_directory), "");
  ctlbind_browse_for_file(ctls, "output_file_name", ctx(&S::output_file_name), "output_file_name");

  ctlbind(ctls, "autoSaveInterval:", ctx, ctx(&S::autoSaveInterval),
      std::function {[](const S * opts) {return !opts->save_substacks;}},
      "Save accumulator after processing each Interval frames");

  //  ctlbind_group(ctls, ctx, [](const S * opts) {return !opts->save_substacks;});
  //  ctlbind(ctls, "autoSaveInterval:", ctx(&S::autoSaveInterval), "Save accumulator after processing each Interval frames");
  //  ctlbind_end_group(ctls);

  ctlbind(ctls, "Save substacks", ctx(&S::save_substacks), "Enable save substacks every substack_frames");
  ctlbind(ctls, "Save input_video", ctx(&S::save_input_video), "");
  ctlbind(ctls, "Save progress_video", ctx(&S::save_progress_video), "");
  ctlbind(ctls, "Save reference_video", ctx(&S::save_reference_video), "");
  ctlbind(ctls, "Save weights_video", ctx(&S::save_weights_video), "");

  ctlbind_expandable_group(ctls, "Save SubStacks options...", ctx(&S::save_substacks));
    ctlbind(ctls, "substack_frames", ctx(&S::substack_frames),
        "Save substack every substack_frames accumulated and reset accumulator");
    ctlbind(ctls, ctx(&S::substack_output_options));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save input video options...", ctx(&S::save_input_video));
    ctlbind(ctls, ctx(&S::output_input_video_options));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save progress video options...", ctx(&S::save_progress_video));
    ctlbind(ctls, ctx(&S::output_progress_video_options));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save reference video options...", ctx(&S::save_reference_video));
    ctlbind(ctls, ctx(&S::output_reference_video_options));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save weights video options...", ctx(&S::save_weights_video));
    ctlbind(ctls, ctx(&S::output_weights_video_options));
  ctlbind_end_group(ctls);

}

const c_ctlist<c_canvas_average_pipeline> & c_canvas_average_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Image registration", "");
      ctlbind(ctls, ctx(&this_class::_registration_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Average options", "");
      ctlbind(ctls, ctx(&this_class::_average_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}

const c_enum_member * c_canvas_average_pipeline::get_preview_displays() const
{
  static const c_enum_member members[] = {
      { 0, "CANVAS", "" },
      { 1, "STARS", "" },
      { 0 },
  };

  return members;
}

bool c_canvas_average_pipeline::get_display_image(cv::OutputArray outputDisplayImage,
    cv::OutputArray outputDisplayMask)
{
  INSTRUMENT_REGION("");

  bool fOk = false;

  switch (_preview_display) {
    case 1: {
      // Current frame + Detected keypoints display, locked by mutex
      std::vector<cv::KeyPoint> current_keypoints;
      cv::Mat current_image;

      synchronized(_current_stars_lock, [&]() {
        _current_grayscale_image.copyTo(current_image);
        current_keypoints = _current_keypoints;
      });

      if ( !current_image.empty() ) {

        const int ddepth = outputDisplayImage.fixedType() ? outputDisplayImage.depth() :
            (_output_options.default_display_type >= 0 ? _output_options.default_display_type :
                current_image.depth());

        double dscale = 1.0, doffset = 0.0;
        getScaleOffset(current_image.depth(), ddepth, &dscale, &doffset);

        if( _output_options.display_scale != 0 ) {
          dscale *= _output_options.display_scale;
        }
        else {
          const double bpp_scale = (_input_bpp > 0) ? double(1 << _input_bpp) : 1.0;
          dscale *= bpp_scale;
          doffset *= bpp_scale;
        }

        const double maxColor = getMaxValForPixelDepth(current_image.depth()) * dscale + doffset;
//
//        CF_DEBUG("\n"
//            "current_image.depth()=%d _input_bpp=%d dscale=%g doffset=%g maxColor=%g",
//            current_image.depth(), _input_bpp, dscale, doffset, maxColor);

        current_image.convertTo(current_image, ddepth, dscale, doffset);
        cv::cvtColor(current_image, current_image, cv::COLOR_GRAY2BGR);

        cv::drawKeypoints(current_image,
            current_keypoints, current_image,
            cv::Scalar(0, 0, maxColor),
            cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

        fOk = true;
      }

      outputDisplayImage.move(current_image);
      outputDisplayMask.release();

      break;
    }

    default: {
      // Canvas display
      const PIXEL_DEPTH ddepth = _output_options.default_display_type;
      const double dscale = (_output_options.display_scale == 0 && _input_bpp > 0) ? (1 << _input_bpp) : 1;
      if( _average.compute(outputDisplayImage, outputDisplayMask, dscale, ddepth) ) {
        fOk = true;
      }
      break;
    }
  }

  return fOk;
}

bool c_canvas_average_pipeline::copy_parameters(const c_image_processing_pipeline::sptr & dst) const
{
  if ( !base::copy_parameters(dst) ) {
    CF_ERROR("c_canvas_average_pipeline::base::copyParameters() fails");
    return false;
  }

  const this_class::sptr p = std::dynamic_pointer_cast<this_class>(dst);
  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->_registration_options = this->_registration_options;
  p->_average_options = this->_average_options;
  p->_output_options = this->_output_options;
  p->_ecch.copy_parameters(_ecch);
  p->_eccflow.copy_parameters(_eccflow);

  return true;
}

std::string c_canvas_average_pipeline::generate_output_file_name(const std::string & suffix) const
{
  static const auto get_current_date_time_string =
      []() -> std::string
      {
        struct timespec t;
        struct tm *tm;

        int year;
        int month;
        int day;
        int hour;
        int min;
        int sec;

        clock_gettime(CLOCK_REALTIME, &t);
        tm = gmtime(&t.tv_sec);

        year = tm->tm_year + 1900;
        month = tm->tm_mon + 1;
        day = tm->tm_mday;
        hour = tm->tm_hour;
        min = tm->tm_min;
        sec = tm->tm_sec;
        // msec = t.tv_nsec / 1000000;

      return ssprintf("%0.4d%0.2d%0.2d_%0.2d%0.2d%0.2d_GMT",
          year, month, day, hour, min, sec);
    };

  std::string output_file_name_postfix = ".ACC";

  std::string output_file_name = _output_options.output_file_name;
  std::string output_suffix = suffix.empty() ? ".fits" : suffix;
  if( output_file_name.empty() ) {

    output_file_name =
        ssprintf("%s/%s%s.%s.32F%s",
            _output_path.c_str(),
            csequence_name(),
            output_file_name_postfix.c_str(),
            get_current_date_time_string().c_str(),
            output_suffix.c_str());
  }
  else {

    std::string path, name, sfx;

    split_pathfilename(output_file_name, &path, &name, &sfx);

    if( path.empty() ) {
      path = _output_path;
    }
    else if( !is_absolute_path(path) ) {
      path = ssprintf("%s/%s", _output_path.c_str(), path.c_str());
    }

    if( name.empty() ) {
      name = ssprintf("%s%s", csequence_name(),
          output_file_name_postfix.c_str());
    }

    if( sfx.empty() || sfx.back() == '.' ) {
      sfx = ".fits";
    }

    output_file_name =
        ssprintf("%s/%s.%s.32F%s",
            path.c_str(),
            name.c_str(),
            get_current_date_time_string().c_str(),
            sfx.c_str());
  }


  return output_file_name;
}


bool c_canvas_average_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path = create_output_path(_output_options.output_directory);

  _average.clear();
  _current_image.release();
  _current_mask.release();
  _apodizationWindow.release();
  _image_transform.reset();
  _star_extractor.clear();
  _triangle_extractor.clear();

  if ( !_input_options.darkbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.darkbayer_filename, _darkbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.darkbayer_filename.c_str());
      return false;
    }
  }

  if ( !_input_options.flatbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.flatbayer_filename, _flatbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.flatbayer_filename.c_str());
      return false;
    }
  }



  if ( !_input_options.missing_pixel_mask_filename.empty() ) {

    if ( !load_image(_input_options.missing_pixel_mask_filename, _missing_pixel_mask, cv::noArray()) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( _missing_pixel_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !_input_options.missing_pixels_marked_black ) {
      cv::invert(_missing_pixel_mask, _missing_pixel_mask);
    }
  }

  _average.setCanvasSize(_registration_options.canvasSize);

  const bool enable_registration =
      _registration_options.enable_star_registration ||
          _registration_options.enable_ecc_registration ||
          _registration_options.enable_eccflow_registration;

  if ( enable_registration ) {

    _image_transform = create_image_transform(_registration_options.motion_type);

    if ( _registration_options.enable_star_registration ) {
      _star_extractor.set_options(_registration_options.star_detection);
      _triangle_extractor.set_options(_registration_options.triangle_extractor);
      _triangle_matcher.set_options(_registration_options.triangle_matcher);
    }

    if ( _registration_options.enable_ecc_registration ) {
      _ecch.set_image_transform(_image_transform.get());
      _ecch.set_options(_registration_options.ecch);
    }

    if( _registration_options.enable_eccflow_registration ) {
      _eccflow.set_options(_registration_options.eccflow);
    }
  }


  CF_DEBUG("Output path='%s'", this->_output_path.c_str());


  return true;
}

void c_canvas_average_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  _ecch.clear();
  _image_transform.reset();
  //_average.clear();

  _current_image.release();
  _current_mask.release();
  _apodizationWindow.release();
}

bool c_canvas_average_pipeline::run_pipeline()
{
  INSTRUMENT_REGION("");

  if ( !start_pipeline(_input_options.start_frame_index, _input_options.max_input_frames) ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  bool fOK = true;
  for( ; _processed_frames < _total_frames; ++_processed_frames, ++_accumulated_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( is_bad_frame_index(_input_sequence->current_pos()) ) {
      CF_DEBUG("Skip frame %d as blacklisted", _input_sequence->current_pos());
      _input_sequence->seek(_input_sequence->current_pos() + 1);
      continue;
    }

    fOK = read_input_frame(_input_sequence, _input_options,
        _current_image, _current_mask,
        false,
        false);

    if( !fOK ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if ( _current_image.empty() ) {
      // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
      continue;
    }

    _input_bpp = _input_sequence->bpp();

    if( _input_options.input_image_processor && !_input_options.input_image_processor->empty() ) {
      INSTRUMENT_REGION("input_image_processor");
      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        fOK = false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame1() fails");
      fOK = false;
      break;
    }

    if( !_output_options.save_substacks || _output_options.substack_frames < 1 ) {
      if( _output_options.autoSaveInterval > 0 && ((_processed_frames + 1) % _output_options.autoSaveInterval) == 0 ) {
        save_averaged_image();
      }
    }
  }

  if( _output_options.save_substacks && _output_options.substack_frames > 0 ) {
    flush_substack_frame();
  }

  if( !_output_options.save_substacks || _output_options.substack_frames < 1 ) {
    save_averaged_image();
  }

  return fOK;
}

static cv::Rect computeNewCanvasBBox(const c_image_transform::sptr & transform,
    const cv::Rect & last_canvas_bbox,
    const cv::Size & current_image_size,
    const cv::Size & global_canvas_size)
{
  if( transform->invertible() ) {

    const cv::Mat1f inv_p = transform->invert(transform->parameters());

    std::vector<cv::Point2f> current_corners = {
        cv::Point2f(0.f, 0.f),
        cv::Point2f(static_cast<float>(current_image_size.width), 0.f),
        cv::Point2f(static_cast<float>(current_image_size.width), static_cast<float>(current_image_size.height)),
        cv::Point2f(0.f, static_cast<float>(current_image_size.height))
    };

    std::vector<cv::Point2f> ref_corners_local;

    if( transform->remap(inv_p, current_corners, ref_corners_local) && ref_corners_local.size() == 4 ) {

      cv::Point2f global_offset = last_canvas_bbox.tl();
      std::vector<cv::Point2f> ref_corners_global(4);
      for( size_t i = 0; i < 4; ++i ) {
        ref_corners_global[i] = ref_corners_local[i] + global_offset;
      }

      cv::RotatedRect rbox = cv::minAreaRect(ref_corners_global);
      cv::Rect new_global_bbox = rbox.boundingRect();
      return new_global_bbox & cv::Rect(0, 0, global_canvas_size.width, global_canvas_size.height);
    }
  }

  // FALLBACK for non-invertibla transformations
  int margin = 32;
  cv::Rect test_rect = cv::Rect(
      last_canvas_bbox.x - margin,
      last_canvas_bbox.y - margin,
      last_canvas_bbox.width + 2 * margin,
      last_canvas_bbox.height + 2 * margin
          ) & cv::Rect(0, 0, global_canvas_size.width, global_canvas_size.height);

  if( test_rect.empty() ) {
    return last_canvas_bbox;
  }

  cv::Mat2f test_rmap;
  transform->create_remap(test_rect.size(), test_rmap);

  int min_x = test_rect.width, max_x = 0;
  int min_y = test_rect.height, max_y = 0;
  bool found_any = false;
  const int step = 4;

  for( int y = 0; y < test_rmap.rows; y += step ) {
    const cv::Point2f * row_ptr = test_rmap.ptr<cv::Point2f>(y);
    for( int x = 0; x < test_rmap.cols; x += step ) {
      cv::Point2f src_pt = row_ptr[x];
      if( src_pt.x >= 0 && src_pt.x < current_image_size.width &&
          src_pt.y >= 0 && src_pt.y < current_image_size.height ) {
        if( x < min_x ) {
          min_x = x;
        }
        if( x > max_x ) {
          max_x = x;
        }
        if( y < min_y ) {
          min_y = y;
        }
        if( y > max_y ) {
          max_y = y;
        }
        found_any = true;
      }
    }
  }

  if( !found_any ) {
    return last_canvas_bbox;
  }

  min_x = std::max(0, min_x - step);
  min_y = std::max(0, min_y - step);
  max_x = std::min(test_rect.width - 1, max_x + step);
  max_y = std::min(test_rect.height - 1, max_y + step);

  cv::Rect new_global_bbox(
      test_rect.x + min_x,
      test_rect.y + min_y,
      max_x - min_x + 1,
      max_y - min_y + 1
          );

  return new_global_bbox & cv::Rect(0, 0, global_canvas_size.width, global_canvas_size.height);
}

bool c_canvas_average_pipeline::process_current_frame()
{
  INSTRUMENT_REGION("");

  if ( !write_input_video(_current_image, _current_mask) ) {
    CF_ERROR("write_input_video() fails");
    return false;
  }


  static const auto mkgrayscale = [](const cv::Mat & src, cv::Mat & dst) {
    if( src.channels() != 1 ) {
      cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    }
    else if ( &src != &dst ) {
      dst = src;
    }
  };

  cv::Mat current_binary_mask;
  cv::Mat current_weights;

  if ( !_current_mask.empty() ) {
    if ( _current_mask.depth() == CV_8U ) {
      current_binary_mask = _current_mask;
    }
    else {
      cv::compare(_current_mask, 0, current_binary_mask, cv::CMP_GT);
    }
  }

  if ( !_current_mask.empty() && _current_mask.depth() != CV_8U  ) {
    current_weights = _current_mask;
  }
  else {
    compute_weights(_current_image, current_binary_mask, current_weights);
  }

  const bool enable_registration =
      _registration_options.enable_star_registration ||
          _registration_options.enable_ecc_registration ||
          _registration_options.enable_eccflow_registration;


  if( !enable_registration || _average.accumulated_frames() < 1 ) {
    // Very first frame or no registration requested
    INSTRUMENT_REGION("initialize_accumulator");

    if ( !_average.add(_current_image, current_weights) ) {
      CF_ERROR("average_add() fails");
      return false;
    }
  }
  else {
    // Not a first frame

    INSTRUMENT_REGION("align_and_update");

    cv::Mat reference_image, reference_binary_mask;
    cv::Mat2f rmap, uv;

    if ( !_average.compute(reference_image, reference_binary_mask, 1, -1, _average.last_bbox()) ) {
      CF_ERROR("_average.compute() fails");
      return !canceled();
    }

    const cv::Rect bbox(0, 0, reference_image.cols, reference_image.rows);
    if( bbox.empty() ) {
      CF_ERROR("BAD Bounding Box from _average.compute()");
      return false;
    }

    mkgrayscale(reference_image, reference_image);
    mkgrayscale(_current_image, _current_grayscale_image);

    if ( _output_options.save_reference_video ) {

      const cv::Size frameSize(_current_grayscale_image.cols + 8, _current_grayscale_image.rows + 8);
      cv::Mat frame = cv::Mat::zeros(frameSize, reference_image.type());
      cv::Mat mask =  reference_binary_mask.empty() ? cv::Mat() : cv::Mat::zeros(frameSize, CV_8UC1);

      const cv::Size copySize(std::min(frameSize.width, bbox.width), std::min(frameSize.height, bbox.height));
      const cv::Rect copyBox(0,0, copySize.width, copySize.height);

      reference_image(copyBox).copyTo(frame(copyBox));
      if ( !reference_binary_mask.empty() ) {
        reference_binary_mask(copyBox).copyTo(mask(copyBox));
      }

      if ( !write_reference_video(frame, mask)) {
        CF_ERROR("write_reference_video() fails");
        return false;
      }
    }

    _image_transform->reset();

    if( _registration_options.enable_star_registration ) {
      INSTRUMENT_REGION("star_registration");

      synchronized(_current_stars_lock, [&]() {
        _star_extractor.detect(_current_grayscale_image, _current_keypoints, current_binary_mask);
      });
      if ( _current_keypoints.size() < 3 ) {
        CF_ERROR("_star_extractor.detect(_current_grayscale_image) fails: current_keypoints.size=%zu", _current_keypoints.size());
        return !canceled();
      }

      _star_extractor.detect(reference_image, _reference_keypoints, reference_binary_mask);
      if ( _reference_keypoints.size() < 3 ) {
        CF_ERROR("_star_extractor.detect(reference_image) fails: reference_keypoints.size=%zu", _reference_keypoints.size());
        return !canceled();
      }

      _triangle_extractor.compute(reference_image, _reference_keypoints, _reference_descriptors);
      _triangle_extractor.compute(_current_grayscale_image, _current_keypoints, _current_descriptors);

      _triangle_matcher.train(_reference_keypoints, _reference_descriptors);
      _triangle_matcher.match(_current_keypoints, _current_descriptors, _current_matches);
      if ( _current_matches.size() < 1 ) {
        CF_ERROR("_triangle_matcher.match() fails: triangle_matches.size()=%zu", _current_matches.size());
        return !canceled();
      }

      const bool transformEstimated =
          estimate_image_transform(_image_transform.get(),
              _current_keypoints, _reference_keypoints, _current_matches,
              _registration_options.transform_estimation);

      if( !transformEstimated ) {
        CF_ERROR("estimate_image_transform() fails");
        return !canceled();
      }
    }

    if( _registration_options.enable_ecc_registration ) {
      INSTRUMENT_REGION("ecc_registration");

      if ( _average.accumulated_frames() > 50 ) {
        const double sigma = _registration_options.eccUnsharpMaskSigma;
        const double alpha = _registration_options.eccUnsharpMaskAlpha;
        if ( sigma > 0 && alpha > 0 && alpha < 1 ) {
         INSTRUMENT_REGION("unsharp_mask");
         unsharp_mask(reference_image, reference_binary_mask, reference_image,
              sigma, alpha, 0, 1.5);
        }
      }

      /* This will compute _image_transform parameters as _ecch has the active pointer to _image_transform */
      _ecch.set_reference_image(reference_image, reference_binary_mask);
      if( !_ecch.align(_current_grayscale_image, current_binary_mask) ) {
        CF_ERROR("_ecch.align() fails");
        return false;
      }
    }

    if( _registration_options.enable_eccflow_registration ) {
      INSTRUMENT_REGION("eccflow_registration");
      // FIXME: This ugly stiff does not work yet as expected, just start the experimentation

      // create temporary rmap of reference image size for optflow computation

      bool fOk = false;
      if ( reference_image.size() == _current_grayscale_image.size() ) {
        _image_transform->create_remap(reference_image.size(), rmap);
        fOk = _eccflow.compute_uv(_current_grayscale_image, reference_image, rmap, current_binary_mask, reference_binary_mask);
      }
      else {
        const cv::Rect roi(0, 0, std::min(reference_image.cols, _current_grayscale_image.cols),
            std::min(reference_image.rows, _current_grayscale_image.rows));
        const cv::Mat cimage = _current_grayscale_image(roi);
        const cv::Mat cmask = current_binary_mask.empty() ? cv::Mat() : current_binary_mask(roi);
        const cv::Mat rimage = reference_image(roi);
        const cv::Mat rmask = reference_binary_mask.empty() ? cv::Mat() : reference_binary_mask(roi);

        _image_transform->create_remap(rimage.size(), rmap);
        fOk = _eccflow.compute_uv(cimage, rimage, rmap, cmask, rmask);
      }

      if( fOk ) {
        uv = _eccflow.current_uv();
      }
      else {
        CF_ERROR("_eccflow.compute_uv() fails");
      }
    }

    const cv::Rect newCanvasBBox =
        computeNewCanvasBBox(_image_transform,
            _average.last_bbox(),
            _current_image.size(),
            _average.accumulator_size());

    if( newCanvasBBox.empty() ) {
      CF_ERROR("BAD Bounding Box from computeNewCanvasBBox()");
      return false;
    }

    const cv::Point old_global_offset = _average.last_bbox().tl();
    const cv::Vec2f delta_T = cv::Vec2f(newCanvasBBox.x - old_global_offset.x,
        newCanvasBBox.y - old_global_offset.y);

    _image_transform->set_translation(_image_transform->translation() + delta_T);
    _image_transform->create_remap(newCanvasBBox.size(), rmap);

    if( !uv.empty() ) {
      INSTRUMENT_REGION("embed_uv");

      // Compute position of the old uv within the coordinates of the new rmap
      // delta_T = new_offset - old_offset -> old_offset = -delta_T
      cv::Rect old_uv_roi_in_new_rmap(
          static_cast<int>(-delta_T.val[0]),
          static_cast<int>(-delta_T.val[1]),
          uv.cols,
          uv.rows);

      // intersection to ensure don't miss the memory
      cv::Rect dst_roi = old_uv_roi_in_new_rmap & cv::Rect(0, 0, rmap.cols, rmap.rows);

      if (dst_roi.width > 0 && dst_roi.height > 0) {

        // corresponding box inside the uv matrix
        cv::Rect src_roi(
            dst_roi.x - old_uv_roi_in_new_rmap.x,
            dst_roi.y - old_uv_roi_in_new_rmap.y,
            dst_roi.width,
            dst_roi.height
        );

        // Add uv from ROI
        cv::Mat2f rmap_sub = rmap(dst_roi);
        cv::Mat2f uv_sub = uv(src_roi);
        cv::add(rmap_sub, uv_sub, rmap_sub);
      }
    }

    if ( !_average.add(_current_image, current_weights, rmap, newCanvasBBox.tl()) ) {
      CF_ERROR("average_add() fails");
      return false;
    }

    if ( !write_progress_video() ) {
      CF_ERROR("write_progress_video() fails");
      return false;
    }

    if ( !save_substack_frame() ) {
      CF_ERROR("save_substack_frame() fails");
      return false;
    }

  }

  return true;
}


void c_canvas_average_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst)
{
  INSTRUMENT_REGION("");

  static const auto createApodizationWindow = [](const cv::Size & size) -> cv::Mat {

    const int B = std::max(4, std::min(size.width, size.height) / 64);
    const float P = 2.0f;

    cv::Mat1f lut_x(1, size.width, 1.0f);
    cv::Mat1f lut_y(size.height, 1, 1.0f);

    for( int x = 0, xmax = std::min(B, size.width / 2); x < xmax; ++x ) {
      const float shaped = 1 - std::pow((float)(B - x) / B, P);
      const float factor = shaped;
      lut_x(0, x) = lut_x(0, size.width - 1 - x) = factor;
    }
    for( int y = 0, ymax = std::min(B, size.height / 2); y < ymax; ++y ) {
      const float shaped = 1 - std::pow((float)(B - y) / B, P);
      const float factor = shaped;
      lut_y(y, 0) = lut_y(size.height - 1 - y, 0) = factor;
    }

    return lut_y * lut_x;
  };

  if ( _average_options.sharpness_measure.kradius <= 0 ) {
    dst = srcmask;
  }
  else {
    const cv::Size src_size = src.size();

    compute_local_variance_map(src, _average_options.sharpness_measure, dst, false);
    if ( dst.size() != _apodizationWindow.size() ) {
      _apodizationWindow = createApodizationWindow(dst.size());
    }

    cv::multiply(dst, _apodizationWindow, dst);
    upscale_local_variance_map(dst, src_size);
    if ( !srcmask.empty() ) {
      dst.setTo(0, ~srcmask);
    }

    write_weights_video(dst, cv::noArray());
  }
}

bool c_canvas_average_pipeline::save_averaged_image()
{
  if ( _average.accumulated_frames() > 0 ) {
    cv::Mat avg;
    cv::Mat1f msk;

    if ( !_average.compute(avg, msk) ) {
      CF_ERROR("_average.compute() fails for output image");
    }
    else {
      const std::string output_file_name = generate_output_file_name(".fits");
      if ( save_image(avg, msk, output_file_name) ) {
        CF_DEBUG("Saved %s", output_file_name.c_str());
      }
      else {
        CF_ERROR("save_image() fails for %s", output_file_name.c_str());
      }
    }
  }

  return true;
}

bool c_canvas_average_pipeline::flush_substack_frame()
{
  if( _average.accumulated_frames() > 0 ) {

    cv::Mat avg;
    cv::Mat1f msk;

    if( !_average.compute(avg, msk) ) {
      CF_ERROR("_average.compute() fails for output image");
      return true; // ignore this error, just continue
    }

    if( !_substack_writer.is_open() ) {

      const bool fOk =
          add_output_writer(_substack_writer,
              _output_options.substack_output_options,
              "substack",
              ".ser"); // User .ser as default if not specified by user

      if( !fOk ) {
        CF_ERROR("add_output_writer('%s') fails", _substack_writer.filename().c_str());
        // This is critical error, may be disk full or incorrect user parameters
        return false;
      }
    }

    if( !_substack_writer.write(avg, msk) ) {
      CF_ERROR("_progress_writer.write('%s') fails.", _substack_writer.filename().c_str());
      // Critical error, may be disk full ?
      return false;
    }

    // Accumulator must be reset every substack_frames.
    // temporary block also GUI preview thread this moment
    synchronized([&]() {
      _average.reset();
    });
  }

  return true;
}


bool c_canvas_average_pipeline::save_substack_frame()
{
  if( _output_options.save_substacks && _output_options.substack_frames > 0 ) {
    if( _average.accumulated_frames() > _output_options.substack_frames ) {
      return flush_substack_frame();
    }
  }

  return true;
}


bool c_canvas_average_pipeline::write_input_video(cv::InputArray image, cv::InputArray mask)
{
  if( _output_options.save_input_video && !image.empty() ) {

    if( !_input_video_writer.is_open() ) {

      bool fOk =
          add_output_writer(_input_video_writer,
              _output_options.output_input_video_options,
              "input",
              ".ser");

      if( !fOk ) {
        CF_ERROR("add_output_writer('%s') fails",
            _input_video_writer.filename().c_str());
        return false;
      }
    }

    if( !_input_video_writer.write(image/*, mask*/) ) {
      CF_ERROR("_input_video_writer.write('%s') fails.",
          _input_video_writer.filename().c_str());
      return false;
    }
  }

  return true;
}

bool c_canvas_average_pipeline::write_progress_video()
{
  if( _output_options.save_progress_video && _average.accumulated_frames() > 0 ) {

    cv::Mat avg;
    cv::Mat1f msk;

    if( !_average.compute(avg, msk) ) {
      CF_ERROR("_average.compute() fails for output image");
      return true; // ignore this error
    }

    if( !_progress_writer.is_open() ) {

      const bool fOk =
          add_output_writer(_progress_writer,
              _output_options.output_progress_video_options,
              "progress",
              ".ser");

      if( !fOk ) {
        CF_ERROR("add_output_writer('%s') fails",
            _progress_writer.filename().c_str());
        return false;
      }
    }

    if( !_progress_writer.write(avg, msk) ) {
      CF_ERROR("_progress_writer.write('%s') fails.",
          _progress_writer.filename().c_str());
      return false;
    }
  }

  return true;
}

bool c_canvas_average_pipeline::write_reference_video(cv::InputArray image, cv::InputArray mask)
{
  if( _output_options.save_reference_video && !image.empty() ) {

    if( !_reference_video_writer.is_open() ) {

      bool fOk =
          add_output_writer(_reference_video_writer,
              _output_options.output_reference_video_options,
              "reference",
              ".ser");

      if( !fOk ) {
        CF_ERROR("add_output_writer('%s') fails",
            _reference_video_writer.filename().c_str());
        return false;
      }
    }

    if( !_reference_video_writer.write(image, mask) ) {
      CF_ERROR("_reference_video_writer.write('%s') fails.",
          _reference_video_writer.filename().c_str());
      return false;
    }
  }

  return true;
}

bool c_canvas_average_pipeline::write_weights_video(cv::InputArray image, cv::InputArray mask)
{
  if( _output_options.save_weights_video && !image.empty() ) {

    if( !_weights_video_writer.is_open() ) {

      bool fOk =
          add_output_writer(_weights_video_writer,
              _output_options.output_weights_video_options,
              "weights",
              ".ser");

      if( !fOk ) {
        CF_ERROR("add_output_writer('%s') fails",
            _weights_video_writer.filename().c_str());
        return false;
      }
    }

    if( !_weights_video_writer.write(image, mask) ) {
      CF_ERROR("_weights_video_writer.write('%s') fails.",
          _weights_video_writer.filename().c_str());
      return false;
    }
  }

  return true;
}
