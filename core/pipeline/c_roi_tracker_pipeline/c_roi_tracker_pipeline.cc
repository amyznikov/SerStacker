/*
 * c_roi_tracker_pipeline.cc
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#include "c_roi_tracker_pipeline.h"
#include <core/ssprintf.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

c_roi_tracker_pipeline::c_roi_tracker_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options() const
{
  return _input_options;
}

c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options()
{
  return _input_options;
}

const c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options() const
{
  return _tracker_options;
}

c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options()
{
  return _tracker_options;
}

const c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options() const
{
  return _output_options;
}

c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options()
{
  return _output_options;
}


bool c_roi_tracker_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "tracker_options")) ) {
    SERIALIZE_OPTION(section, save, _tracker_options, roi);
    ::serialize(_tracker_options.tracker, settings, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, save_progress_video);
    SERIALIZE_OPTION(section, save, _output_options, progress_video_filename);
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_input_options> & ctx)
{
  using S = c_roi_tracker_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
  ctlbind(ctls, "input_image_processor", ctx(&S::input_image_processor), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_pipeline_options> & ctx)
{
  using S = c_roi_tracker_pipeline_options;

  ctlbind(ctls, "ROI Rect X;Y;WxH;", ctx(&S::roi), "");
  ctlbind(ctls, ctx(&S::tracker));
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_tracker_output_options> & ctx)
{
  using S = c_roi_tracker_output_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));

  ctlbind(ctls, "save_progress_video", ctx(&S::save_progress_video), "");
  ctlbind_group(ctls, ctx(&S::save_progress_video));
    ctlbind_browse_for_file(ctls, "progress_video_filename", ctx(&S::progress_video_filename), "");
  ctlbind_end_group(ctls);
}



const c_ctlist<c_roi_tracker_pipeline> & c_roi_tracker_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "2. ROI tracking", "");
      ctlbind(ctls, ctx(&this_class::_tracker_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "3. Output options", "");
    ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}


bool c_roi_tracker_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _output_path =
      create_output_path(_output_options.output_directory);

  if( _tracker_options.tracker.enabled ) {

    if( (_objbox = _tracker_options.roi).empty() ) {
      CF_ERROR("INPUT ERROR: Tracking ROI is empty: x=%d y=%d w=%d h=%d ",
          _objbox.x, _objbox.y,
          _objbox.width, _objbox.height);
      return false;
    }

    if( !_tracker.initialize(_tracker_options.tracker) ) {
      CF_ERROR("tracker_.initialize() fails");
      return false;
    }
  }

  return true;
}

void c_roi_tracker_pipeline::cleanup_pipeline()
{
  if ( _input_sequence ) {
    _input_sequence->close();
  }

  if ( _progress_writer.is_open() ) {
    CF_DEBUG("Closing '%s'", _progress_writer.filename().c_str());
    _progress_writer.close();
  }

  _tracker.release();
}

bool c_roi_tracker_pipeline::run_pipeline()
{
  if( !_input_sequence ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if ( !_input_sequence->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      _input_sequence->is_live();

  if( is_live_sequence ) {
    _total_frames = INT_MAX;
    _processed_frames = 0;
    _accumulated_frames = 0;
  }
  else {

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);

    _total_frames = end_pos - start_pos;
    _processed_frames = 0;
    _accumulated_frames = 0;

    if( _total_frames < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          _total_frames,
          start_pos,
          end_pos,
          _input_sequence->size(),
          _input_options.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !_input_sequence->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !_input_sequence->read(_current_image, &_current_mask) ) {
        CF_DEBUG("input_sequence_->read() fails");
        break;
      }
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
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++_accumulated_frames;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}




bool c_roi_tracker_pipeline::process_current_frame()
{
  if ( _tracker_options.tracker.enabled ) {

    bool hasUpdates = false;

    if ( !_tracker.track(_current_image, _objbox, &hasUpdates) ) {
      CF_ERROR("tracker_.track() fails");
      return false;
    }

//    CF_DEBUG("hasUpdates: %d objbox_: %d;%d %dx%d", hasUpdates,
//        _objbox.x, _objbox.y, _objbox.width, _objbox.height);
  }

  if ( !write_progress_video() ) {
    CF_ERROR("write_progress_video() fails");
    return false;
  }

  return true;
}

bool c_roi_tracker_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  if ( _current_image.empty() ) {
    return false;
  }

  if ( _current_image.channels() == 1 ) {
    cv::cvtColor(_current_image, display_frame, cv::COLOR_GRAY2BGR);
  }
  else {
    _current_image.copyTo(display_frame);
  }

  if ( _tracker_options.tracker.enabled ) {
    cv::rectangle(display_frame.getMatRef(), _objbox,
        CV_RGB(32, 255, 64), 1, cv::LINE_8);
  }

  return true;
}

bool c_roi_tracker_pipeline::write_progress_video()
{
  if ( !_output_options.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !_progress_writer.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(_output_options.progress_video_filename,
            "progress",
            ".avi");

    bool fOK =
        _progress_writer.open(output_video_filename);

    if( !fOK ) {
      CF_ERROR("progress_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !_progress_writer.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("homography_video_writer_.write() fails: %s",
        _progress_writer.filename().c_str());
    return false;
  }


  return true;
}

