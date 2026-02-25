/*
 * c_live_stacking_pipeline.cc
 *
 *  Created on: Jul 15, 2023
 *      Author: amyznikov
 */

#include "c_live_stacking_pipeline.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<live_stacking_accumulation_type>()
{
  static const c_enum_member members[] = {

      { live_stacking_accumulation_disable, "disable",
          "Simple average" },

      { live_stacking_accumulation_average, "average",
          "Simple average" },

//      { live_stacking_accumulation_bayer_average, "bayer_average",
//          "Experimental code for bayer pattern average" },

      { live_stacking_accumulation_average},
  };

  return members;
}


c_live_stacking_pipeline::c_live_stacking_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_live_stacking_input_options& c_live_stacking_pipeline::input_options() const
{
  return _input_options;
}

c_live_stacking_input_options& c_live_stacking_pipeline::input_options()
{
  return _input_options;
}

const c_live_stacking_registration_options & c_live_stacking_pipeline::registration_options() const
{
  return _registration_options;
}

c_live_stacking_registration_options & c_live_stacking_pipeline::registration_options()
{
  return _registration_options;
}

const c_live_stacking_accumulation_options & c_live_stacking_pipeline::accumulation_options() const
{
  return _accumulation_options;
}

c_live_stacking_accumulation_options & c_live_stacking_pipeline::accumulation_options()
{
  return _accumulation_options;
}

const c_live_stacking_output_options & c_live_stacking_pipeline::output_options() const
{
  return _output_options;
}

c_live_stacking_output_options & c_live_stacking_pipeline::output_options()
{
  return _output_options;
}

bool c_live_stacking_pipeline::serialize(c_config_setting settings, bool save)
{
  static const auto get_group =
      [](c_config_setting setting, bool save, const std::string & name) {
        return save ? setting.add_group(name) : setting[name];
      };


  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {
    SERIALIZE_OPTION(section, save, _registration_options, enabled);
    SERIALIZE_OPTION(section, save, _registration_options, minimum_image_size);
    SERIALIZE_OPTION(section, save, _registration_options, min_rho);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "accumulation_options")) ) {
    SERIALIZE_OPTION(section, save, _accumulation_options, accumulation_type);
    SERIALIZE_OPTION(section, save, _accumulation_options, ignore_input_mask);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, save_accumulated_file);
    SERIALIZE_OPTION(section, save, _output_options, output_accumuated_file_name);

    SERIALIZE_OPTION(section, save, _output_options, save_accumulated_video);
    if( (subsection = get_group(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, display_scale);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_live_stacking_input_options> & ctx)
{
  using S = c_live_stacking_input_options;
  ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_live_stacking_registration_options> & ctx)
{
  using S = c_live_stacking_registration_options;

  ctlbind(ctls, "Enable registration", ctx(&S::enabled), "");
  ctlbind_group(ctls, ctx(&S::enabled));
    ctlbind(ctls, "minimum_image_size", ctx(&S::minimum_image_size), "");
    ctlbind(ctls, "min_rho", ctx(&S::min_rho), "");
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_live_stacking_accumulation_options> & ctx)
{
  using S = c_live_stacking_accumulation_options;

  ctlbind(ctls, "method", ctx(&S::accumulation_type), "");
  ctlbind(ctls, "ignore_input_mask", ctx(&S::ignore_input_mask), "");
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_live_stacking_output_options> & ctx)
{
  using S = c_live_stacking_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "display_scale", ctx(&S::display_scale), "");
  ctlbind(ctls, "output_directory", ctx(&S::output_directory), "");
  ctlbind(ctls, "save_accumuated_file", ctx(&S::save_accumulated_file), "");
  ctlbind(ctls, "accumuated_file_name", ctx(&S::output_accumuated_file_name), "");// "", _this->_output_options.save_accumulated_file);

  ctlbind_expandable_group(ctls, "Save accumulated video", "");
    ctlbind(ctls, "save_accumulated_video", ctx(&S::save_accumulated_video), "");
    ctlbind(ctls, ctx(&S::output_accumulated_video_options)); // _this->_output_options.save_accumulated_video
  ctlbind_end_group(ctls);

}

const c_ctlist<c_live_stacking_pipeline> & c_live_stacking_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "2. Image registration", "");
    ctlbind(ctls, ctx(&this_class::_registration_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "3. Accumulation options", "");
    ctlbind(ctls, ctx(&this_class::_accumulation_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "4. Output options", "");
    ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}
//
//const std::vector<c_image_processing_pipeline_ctrl>& c_live_stacking_pipeline::get_controls()
//{
//  static std::vector<c_image_processing_pipeline_ctrl> ctrls;
//
////  if( ctrls.empty() ) {
////
////    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
////    POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Image registration", "");
////    PIPELINE_CTL(ctrls, _registration_options.enabled, "enabled", "");
////    PIPELINE_CTL(ctrls, _registration_options.minimum_image_size, "minimum_image_size", "");
////    PIPELINE_CTL(ctrls, _registration_options.min_rho, "min_rho", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Accumulation options", "");
////    PIPELINE_CTL(ctrls, _accumulation_options.accumulation_type, "accumulation method", "");
////    PIPELINE_CTL(ctrls, _accumulation_options.ignore_input_mask, "ignore input mask", "");
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
////    PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
////    PIPELINE_CTL(ctrls, _output_options.display_scale, "display_scale", "");
////    PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
////    PIPELINE_CTL(ctrls, _output_options.save_accumulated_file, "save_accumuated_file", "");
////    PIPELINE_CTLC(ctrls, _output_options.output_accumuated_file_name, "accumuated_file_name", "", _this->_output_options.save_accumulated_file);
////
////
////    PIPELINE_CTL_GROUP(ctrls, "Save accumulated video", "");
////      PIPELINE_CTL(ctrls, _output_options.save_accumulated_video, "save_accumulated_video", "");
////      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_accumulated_video_options,
////          (_this->_output_options.save_accumulated_video));
////    PIPELINE_CTL_END_GROUP(ctrls);
////
////    PIPELINE_CTL_END_GROUP(ctrls);
////  }
//
//  return ctrls;
//}


bool c_live_stacking_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  double display_scale =
      _output_options.display_scale;

  if( display_scale <= 0 && (display_scale = _input_display_scale) <= 0 ) {
    display_scale = 1;
  }

  lock_guard lock(mutex());

  if( _frame_accumulation ) {
    return _frame_accumulation->compute(display_frame, display_mask, display_scale);
  }

  cv::Mat image, mask;

  if( _aligned_image.empty() ) {
    image = _current_image;
    mask = _current_mask;
  }
  else {
    image = _aligned_image;
    mask = _aligned_mask;
  }

  if( !image.empty() ) {

    if( display_scale == 1 ) {
      image.copyTo(display_frame);
    }
    else {
      image.convertTo(display_frame,
          image.depth(),
          display_scale);
    }

    mask.copyTo(display_mask);

    return true;
  }

  return false;
}


bool c_live_stacking_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path =
      create_output_path(_output_options.output_directory);

  _frame_accumulation.reset();
  _ecch.clear();
  _current_image.release();
  _reference_image.release();
  _aligned_image.release();

  _input_display_scale = -1;

  return true;
}

void c_live_stacking_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  if( _output_options.save_accumulated_file ) {

    cv::Mat image, mask;

    if( true ) {
      lock_guard lock(mutex());
      if( _frame_accumulation && _frame_accumulation->accumulated_frames() > 0 ) {
        if( !_frame_accumulation->compute(image, mask) ) {
          CF_ERROR("frame_accumulation_->compute() fails, can not save accumulator");
        }
      }
    }

    if( !image.empty() ) {

      const std::string output_file_name =
          generate_output_filename(_output_options.output_accumuated_file_name,
              "accumulator",
              ".tiff");

      if( save_image(image, output_file_name) ) {
        CF_DEBUG("Saved '%s'", output_file_name.c_str());
      }
      else {
        CF_DEBUG("save_image('%s') fails", output_file_name.c_str());
      }
    }
  }


}

bool c_live_stacking_pipeline::run_pipeline()
{
  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails, can not run");
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
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1. input_sequence_->size()=%d",
          _total_frames, _input_sequence->size());
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

    const bool fOk =
        read_input_frame(_input_sequence,
            _input_options,
            _current_image, _current_mask,
            false,
            false);

    if( !fOk ) {
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

    if( _input_display_scale <= 0 ) {
      _input_display_scale =
          (1 << _input_sequence->bpp());
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
  }

  return true;
}

bool c_live_stacking_pipeline::accumulate_image(const cv::Mat & current_image, const cv::Mat & current_mask)
{
  if( !_frame_accumulation ) {

    _frame_accumulation =
        create_frame_accumulation(_current_image.size(),
            _current_image.channels(),
            _accumulation_options.accumulation_type);

    if( !_frame_accumulation ) {
      CF_ERROR("create_frame_accumulation(current_image_.size = %dx%d) fails",
          _current_image.cols, _current_image.rows);
      return false;
    }
  }

  if( _accumulation_options.ignore_input_mask ) {

    if( !_frame_accumulation->add(_current_image, cv::noArray()) ) {
      CF_ERROR("frame_accumulation_->add(image.size = %dx%d) fails",
          _current_image.cols, _current_image.rows);
      return false;
    }

  }
  else {

    if( !_frame_accumulation->add(_current_image, _current_mask) ) {
      CF_ERROR("frame_accumulation_->add(image.size = %dx%d channels=%d mask.size=%dx%d channels=%d) fails",
          _current_image.cols, _current_image.rows, _current_image.channels(),
          _current_mask.cols, _current_mask.rows, _current_mask.channels());
      return false;
    }
  }

  return true;
}

bool c_live_stacking_pipeline::process_current_frame()
{

  cv::Mat image, mask;

  if( _registration_options.enabled ) {

    if( !_ecch.image_transform() ) {

      if( !(_image_transform = create_image_transfrom(_registration_options)) ) {
        CF_ERROR("create_image_transfrom() fails");
        return false;
      }

      _ecch.set_image_transform(_image_transform.get());
      _ecch.set_minimum_image_size(std::max(8, _registration_options.minimum_image_size));
      _ecch.set_maxlevel(-1);
    }


    if( _reference_image.empty() ) {

      if( _current_image.channels() == 3 ) {
        cv::cvtColor(_current_image, _reference_image, cv::COLOR_BGR2GRAY);
      }
      else {
        _current_image.copyTo(_reference_image);
      }

      _current_mask.copyTo(_reference_mask);
      _ecch.set_reference_image(_reference_image, _reference_mask);
    }

    else {

      if( _current_image.channels() == 1 ) {
        _ecch.align(_current_image, _current_mask);
      }
      else {
        cv::Mat gray;
        cv::cvtColor(_current_image, gray, cv::COLOR_BGR2GRAY);
        _ecch.align(gray, _current_mask);
      }

      if(  true ) {

        const cv::Mat2f current_remap =
            _ecch.create_remap();

        lock_guard lock(mutex());


        cv::remap(_current_image, _aligned_image, current_remap,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        if( _current_mask.empty() ) {
          cv::remap(cv::Mat1b(_current_image.size(), 255), _aligned_mask, current_remap,
              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else {
          cv::remap(_current_mask, _aligned_mask, current_remap,
              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }

        cv::compare(_aligned_mask, 254, _current_mask, cv::CMP_GE);
      }
    }

  }


  if( _accumulation_options.accumulation_type != live_stacking_accumulation_disable ) {

    lock_guard lock(mutex());

    if( !_frame_accumulation ) {

      _frame_accumulation =
          create_frame_accumulation(_current_image.size(),
              _current_image.channels(),
              _accumulation_options.accumulation_type);

      if( !_frame_accumulation ) {
        CF_ERROR("create_frame_accumulation(current_image_.size = %dx%d) fails",
            _current_image.cols, _current_image.rows);
        return false;
      }
    }



    if( !_registration_options.enabled || _frame_accumulation->accumulated_frames() < 1 ) {
      image = _current_image;
      mask = _current_mask;
    }
    else if( _registration_options.enabled ) {
      image = _aligned_image;
      mask = _aligned_mask;
    }

    if( !image.empty() ) {

      if( !accumulate_image(_current_image, _current_mask) ) {
        CF_ERROR("accumulate_image() fails");
        return false;
      }

      if ( _output_options.save_accumulated_video ) {

        const bool fOK =
            add_output_writer(_accumulated_video_writer,
                _output_options.output_accumulated_video_options,
                "acc",
                ".ser");

        if( !fOK ) {
          CF_ERROR("Can not open output writer '%s'",
              _accumulated_video_writer.filename().c_str());
          return false;
        }

      }

      _frame_accumulation->compute(image, mask);

      if ( !_accumulated_video_writer.write(image, mask) ) {
        CF_ERROR("accumulated_video_writer_.write() fails");
        return false;
      }

    }
  }

  return true;
}



c_image_transform::sptr c_live_stacking_pipeline::create_image_transfrom(const c_live_stacking_registration_options & opts)
{
  c_image_transform::sptr transform =
      ::create_image_transform(IMAGE_MOTION_TRANSLATION);

  return transform;
}

c_frame_accumulation::ptr c_live_stacking_pipeline::create_frame_accumulation(const cv::Size & image_size, int cn,
    live_stacking_accumulation_type type)
{
  switch (type) {
    case live_stacking_accumulation_average: {
      //return c_frame_weigthed_average::ptr(new c_frame_weigthed_average(image_size, CV_MAKETYPE(CV_32F, cn) , CV_32F));
      return c_frame_weigthed_average::ptr(new c_frame_weigthed_average());
    }
    default:
      CF_ERROR("Unsupported live_stacking_accumulation_type %d requested", type);
      break;
  }

  return nullptr;
}
