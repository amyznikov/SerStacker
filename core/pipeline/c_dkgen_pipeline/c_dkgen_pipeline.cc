/*
 * c_dkgen_pipeline.cc
 *
 *  Created on: Apr 10, 2026
 *      Author: amyznikov
 */

#include "c_dkgen_pipeline.h"
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <chrono>
#include <thread>
#include <core/debug.h>

static std::string createTimestamp()
{
  struct timespec t;
  struct tm *tm;

  int year, month, day, hour, min, sec;

  clock_gettime(CLOCK_REALTIME, &t);
  tm = gmtime(&t.tv_sec);

  year = tm->tm_year + 1900;
  month = tm->tm_mon + 1;
  day = tm->tm_mday;
  hour = tm->tm_hour;
  min = tm->tm_min;
  sec = tm->tm_sec;

  return ssprintf("%0.4d%0.2d%0.2d_%0.2d%0.2d%0.2d_GMT",
      year, month, day, hour, min, sec);
}


c_dkgen_pipeline::c_dkgen_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const std::string & c_dkgen_pipeline::get_class_name() const
{
  return class_name();
}

const std::string & c_dkgen_pipeline::class_name()
{
  static const std::string _class_name = "dkgen";
  return _class_name;
}

const std::string & c_dkgen_pipeline::tooltip()
{
  static const std::string tooltip_ =
      "<strong>c_dkgen_pipeline.</strong><br>"
      "Average input frames, useful for generating darks and flats<br>";
  return tooltip_;
}

bool c_dkgen_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    SERIALIZE_OPTION(section, save, _input_options, start_frame_index);
    SERIALIZE_OPTION(section, save, _input_options, max_input_frames);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, output_file_name);
    SERIALIZE_OPTION(section, save, _output_options, append_timestamp);
    SERIALIZE_OPTION(section, save, _output_options, append_imagesize);
    SERIALIZE_OPTION(section, save, _output_options, append_pixtype);
    SERIALIZE_OPTION(section, save, _output_options, output_depth);
  }

  return true;
}

const c_ctlist<c_dkgen_pipeline> & c_dkgen_pipeline::getcontrols()
{
  static const c_ctlist<this_class> _ctls = []() {

    c_ctlist<this_class> ctls;
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options",
        [&, ctx = ctx(&this_class::_input_options)]() {
          ctlbind(ctls, "start_frame", CTL_CONTEXT(ctx, start_frame_index), "");
          ctlbind(ctls, "max_input_frames", CTL_CONTEXT(ctx, max_input_frames), "");
        });

    ctlbind_expandable_group(ctls, "2. Output options",
        [&, ctx = ctx(&this_class::_output_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));
          ctlbind(ctls, "output file name:", CTL_CONTEXT(ctx, output_file_name));
          ctlbind(ctls, "output depth:", CTL_CONTEXT(ctx, output_depth));
          ctlbind(ctls, "append timestamp:", CTL_CONTEXT(ctx, append_timestamp));
          ctlbind(ctls, "append imagesize:", CTL_CONTEXT(ctx, append_imagesize));
          ctlbind(ctls, "append pixtype:", CTL_CONTEXT(ctx, append_pixtype));
        });

    return ctls;
  }();

  return _ctls;
}

bool c_dkgen_pipeline::copy_parameters(const base::sptr & dst) const
{
  if ( !base::copy_parameters(dst) ) {
    CF_ERROR("c_dkgen_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p = std::dynamic_pointer_cast<this_class>(dst);
  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->_output_options = this->_output_options;

  return true;
}

std::string c_dkgen_pipeline::generate_output_file_name(const std::string & filesuffix) const
{
  std::string output_file_name =
      _output_options.output_file_name;

  const std::string ts = _output_options.append_timestamp ?
      ssprintf(".%s", _timestamp.c_str()) : std::string("");

  if( output_file_name.empty() ) {
    output_file_name =
        ssprintf("%s/%s%s%s.tiff",
            _output_path.c_str(),
            csequence_name(),
            filesuffix.c_str(),
            ts.c_str());
  }
  else {

    std::string path, name, suffix;

    split_pathfilename(_output_options.output_file_name, &path, &name, &suffix);

    if( path.empty() ) {
      path = _output_path;
    }
    else if( !is_absolute_path(path) ) {
      path = ssprintf("%s/%s", _output_path.c_str(), path.c_str());
    }

    if( name.empty() ) {
      name = ssprintf("%s", csequence_name());
    }
    if( suffix.empty() || suffix.back() == '.' ) {
      suffix = ".tiff";
    }

    output_file_name =
        ssprintf("%s/%s%s%s%s",
            path.c_str(),
            name.c_str(),
            filesuffix.c_str(),
            ts.c_str(),
            suffix.c_str());
  }

  return output_file_name;
}

bool c_dkgen_pipeline::compute_average(cv::OutputArray avgframe, cv::OutputArray avgmask) const
{
  if( _avg_image.empty() ) {
    avgframe.release();
    avgmask.release();
    return false;
  }

  if( avgmask.needed() ) {
    cv::compare(_avg_mask, cv::Scalar::all(0), avgmask, cv::CMP_GT);
  }

  if( avgframe.needed() ) {
    cv::Mat tmp;
    cv::max(1.0, _avg_mask, tmp);
    if( _avg_image.channels() != tmp.channels() ) {
      const std::vector<cv::Mat> channels(_avg_image.channels(), tmp);
      cv::merge(channels, tmp);
    }

    cv::divide(_avg_image, tmp, avgframe, 255.0);
  }

  return true;
}

bool c_dkgen_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  return compute_average(display_frame, display_mask);
}

bool c_dkgen_pipeline::initialize_pipeline()
{
  cleanup_pipeline();

  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  _output_path = create_output_path(_output_options.output_directory);
  _timestamp = createTimestamp();

  return true;
}

void c_dkgen_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  _avg_image.release();
  _avg_mask.release();
  _white_mask.release();
}

bool c_dkgen_pipeline::run_pipeline()
{
  if( !_input_sequence ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if( !_input_sequence->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence = _input_sequence->is_live();
  if( is_live_sequence ) {
    _total_frames = INT_MAX;
  }
  else {

    const int start_pos = std::max(_input_options.start_frame_index, 0);

    const int end_pos = _input_options.max_input_frames < 1 ?
        _input_sequence->size() : std::min(_input_sequence->size(),
            _input_options.start_frame_index + _input_options.max_input_frames);

    _total_frames = end_pos - start_pos;

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

  int input_depth = -1;

  _processed_frames = 0;
  _accumulated_frames = 0;
  for( ; _processed_frames < _total_frames; ++_processed_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());

      if( !_input_sequence->read(_current_image, &_current_mask) ) {
        CF_DEBUG("input_sequence_->read() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }

      if( _avg_image.empty() ) {
        input_depth = _current_image.depth();
        const int ddepth = std::max(_current_image.depth(), CV_32F);
        const int dtype = CV_MAKETYPE(ddepth, _current_image.channels());
        _avg_image = cv::Mat::zeros(_current_image.size(), dtype);
        _avg_mask = cv::Mat::zeros(_current_image.size(), ddepth);
        _white_mask = cv::Mat1b(_current_image.size(), 255);
      }

      const auto & m = _current_mask.empty() ? _white_mask : _current_mask;

      cv::accumulate(_current_image, _avg_image, m);
      cv::accumulate(m, _avg_mask);

      ++_accumulated_frames;
    }

    if( canceled() ) {
      break;
    }

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  if( _accumulated_frames > 0 ) {

    cv::Mat avgimage, avgmask;
    if( compute_average(avgimage, avgmask) ) {

      if( cv::countNonZero(avgmask) == avgmask.size().area() ) {
        avgmask.release();
      }

      if ( _output_options.output_depth >= 0 && input_depth >= 0 ) {
        avgimage.convertTo(avgimage, _output_options.output_depth);
      }

      std::string avgsuffix = ".avg";
      if ( _output_options.append_imagesize ) {
        avgsuffix += ssprintf(".%dx%d", avgimage.cols, avgimage.rows);
      }
      if ( _output_options.append_pixtype ) {
        avgsuffix += ssprintf(".%s", pixtype2str(avgimage.depth()));
      }

      const std::string output_filename = generate_output_file_name(avgsuffix);
      CF_DEBUG("Saving %s ...", output_filename.c_str());

      if( !save_image(avgimage, avgmask, output_filename) ) {
        CF_ERROR("save_image() fails for %s", output_filename.c_str());
        return false;
      }
    }
  }

  return true;
}

