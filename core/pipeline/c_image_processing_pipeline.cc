/*
 * c_image_processing_pipeline.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#include "c_image_processing_pipeline.h"
#include <core/proc/inpaint.h>
#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/debug.h>
#include <chrono>
#include <thread>

std::vector<c_image_processing_pipeline::factory_item> c_image_processing_pipeline::_registered_classes;

c_image_processing_pipeline::factory_item::factory_item(const std::string & _class_name, const std::string & _tooltip,
    const c_image_processing_pipeline::instance_creator & _create_instance) :
    class_name(_class_name),
    tooltip(_tooltip),
    create_instance(_create_instance)
{
}

const std::vector<c_image_processing_pipeline::factory_item>& c_image_processing_pipeline::registered_classes()
{
  return _registered_classes;
}

const c_image_processing_pipeline::factory_item* c_image_processing_pipeline::find_class(const std::string & class_name)
{
  for( size_t i = 0, n = _registered_classes.size(); i < n; ++i ) {
    if( _registered_classes[i].class_name == class_name ) {
      return &_registered_classes[i];
    }
  }

  return nullptr;
}

const c_image_processing_pipeline::factory_item* c_image_processing_pipeline::find_class(const sptr & pipeline)
{
  return pipeline ? find_class(pipeline->get_class_name()) : nullptr;

}

bool c_image_processing_pipeline::register_class(const std::string & class_name, const std::string & tooltip,
    const c_image_processing_pipeline::instance_creator & create_instance)
{
  if( find_class(class_name) ) {
    CF_ERROR("c_image_processing_pipeline::class_factory: class '%s' already registered", class_name.c_str());
    return false;
  }

  _registered_classes.emplace_back(class_name, tooltip, create_instance);
  return true;
}

c_image_processing_pipeline::sptr c_image_processing_pipeline::create_instance(const std::string & class_name,
    const std::string & name, const c_input_sequence::sptr & input_sequence)
{
  const auto * item = find_class(class_name);
  if( !item ) {
    CF_ERROR("c_image_processing_pipeline::class_factory: class '%s' not registered", class_name.c_str());
    return nullptr;
  }

  c_image_processing_pipeline::sptr instance = item->create_instance(name, input_sequence);
  if( !instance ) {
    CF_ERROR("create_instance() fails for pipeline class  '%s'", class_name.c_str());
    return nullptr;
  }

  if( !instance->initialize() ) {
    CF_ERROR("instance->initialize() fails for pipeline class  '%s'", class_name.c_str());
    return nullptr;
  }

  return instance;
}


static void remove_bad_pixels(cv::Mat & image, double hot_pixels_variation_threshold, bool isbayer)
{
  INSTRUMENT_REGION("");

  cv::Mat medianImage, variationImage, meanVariationImage;
  double minimal_mean_variation_for_very_smooth_images;

  // threshold = estimate_noise(image);
  if ( image.depth() == CV_32F || image.depth() == CV_64F ) {
    minimal_mean_variation_for_very_smooth_images = 1e-3;
  }
  else {
    minimal_mean_variation_for_very_smooth_images = 1;
  }

  cv::medianBlur(image, medianImage, isbayer ? 3 : 5);
  cv::absdiff(image, medianImage, variationImage);

  static float K[3*3] = {
      1./8, 1./8, 1./8,
      1./8, 0.0,  1./8,
      1./8, 1./8, 1./8,
  };

  static const cv::Mat1f SE(3,3, K);

  cv::filter2D(variationImage, meanVariationImage, -1, SE);
  cv::max(meanVariationImage, minimal_mean_variation_for_very_smooth_images, meanVariationImage);

  medianImage.copyTo(image, variationImage > hot_pixels_variation_threshold * meanVariationImage);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_processing_pipeline::c_image_processing_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    _name(name),
    _input_sequence(input_sequence)
{
}

c_image_processing_pipeline::~c_image_processing_pipeline()
{
}

bool c_image_processing_pipeline::initialize()
{
  return true;
}

std::mutex & c_image_processing_pipeline::mutex()
{
  return _lock;
}


bool c_image_processing_pipeline::copyParameters(const sptr & dst) const
{
  if ( !dst ) {
    return false;
  }

  dst->_name = this->_name;

  return true;
}

void c_image_processing_pipeline::set_name(const std::string & name)
{
  _name = name;
}

const std::string& c_image_processing_pipeline::name() const
{
  return _name;
}

const char* c_image_processing_pipeline::cname() const
{
  return _name.c_str();
}

const char* c_image_processing_pipeline::csequence_name() const
{
  if( _input_sequence && !_input_sequence->name().empty() ) {
    return _input_sequence->cname();
  }
  return "live";
}


bool c_image_processing_pipeline::is_running() const
{
  return _is_running;
}

void c_image_processing_pipeline::set_running(bool v)
{
  if ( v != _is_running ) {
    _is_running = v;
    on_state_changed();
  }
}

void c_image_processing_pipeline::on_frame_processed()
{
  // give chance to GUI thread to call get_display_image()
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void c_image_processing_pipeline::on_state_changed()
{
}

void c_image_processing_pipeline::on_status_update()
{
}


bool c_image_processing_pipeline::has_master_frame() const
{
  return false;
}

void c_image_processing_pipeline::set_master_source(const std::string & /*master_source_path*/)
{
}

std::string c_image_processing_pipeline::master_source() const
{
  return "";
}

void c_image_processing_pipeline::set_master_frame_index(int /*v*/)
{
}

int c_image_processing_pipeline::master_frame_index() const
{
  return 0;
}

const c_input_sequence::sptr & c_image_processing_pipeline::input_sequence() const
{
  return _input_sequence;
}

void c_image_processing_pipeline::cancel(bool v)
{
  _canceled = v;
}

std::string c_image_processing_pipeline::generate_output_filename(const std::string & ufilename,
    const std::string & postfix,
    const std::string & suffix) const
{

  const bool live_stream =
      !_input_sequence ||
          _input_sequence->is_live();

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
        tm = localtime(&t.tv_sec);

        year = tm->tm_year + 1900;
        month = tm->tm_mon + 1;
        day = tm->tm_mday;
        hour = tm->tm_hour;
        min = tm->tm_min;
        sec = tm->tm_sec;
        // msec = t.tv_nsec / 1000000;

      return ssprintf("%0.4d%0.2d%0.2d_%0.2d%0.2d%0.2d",
          year, month, day, hour, min, sec);
    };


  std::string output_file_name =
      ufilename;

  if( output_file_name.empty() ) {

    if ( live_stream ) {
      output_file_name =
          ssprintf("%s/%s.%s.%s%s",
              _output_path.c_str(),
              csequence_name(),
              postfix.c_str(),
              get_current_date_time_string().c_str(),
              suffix.empty() ? ".avi" :
                  suffix.c_str());
    }
    else {
      output_file_name =
          ssprintf("%s/%s.%s%s",
              _output_path.c_str(),
              csequence_name(),
              postfix.c_str(),
              suffix.empty() ? ".avi" :
                  suffix.c_str());
    }
  }
  else {

    std::string file_directory;
    std::string file_name;
    std::string file_suffix;

    split_pathfilename(output_file_name,
        &file_directory,
        &file_name,
        &file_suffix);

    if( file_directory.empty() ) {
      file_directory = _output_path;
    }
    else if( !is_absolute_path(file_directory) ) {
      file_directory =
          ssprintf("%s/%s",
              _output_path.c_str(),
              file_directory.c_str());
    }

    if( file_name.empty() || file_name.front() == '.' ) {

      if ( live_stream ) {

        file_name =
            ssprintf("%s.%s.%s%s",
                csequence_name(),
                postfix.c_str(),
                get_current_date_time_string().c_str(),
                file_name.c_str());
      }
      else {
        file_name =
            ssprintf("%s.%s%s",
                csequence_name(),
                postfix.c_str(),
                file_name.c_str());
      }
    }

    if( file_suffix.empty() ) {
      file_suffix =
          suffix.empty() ? ".avi" :
              suffix;
    }

    output_file_name =
        ssprintf("%s/%s%s",
            file_directory.c_str(),
            file_name.c_str(),
            file_suffix.c_str());
  }

  return output_file_name;
}



bool c_image_processing_pipeline::canceled() const
{
  return _canceled;
}

void c_image_processing_pipeline::set_pipeline_stage(int newstage)
{
  const auto oldstage =
      _pipeline_stage;

  if( newstage != oldstage ) {
    _pipeline_stage = newstage;
    on_status_update(); // (oldstage, newstage);
  }
}

int c_image_processing_pipeline::pipeline_stage() const
{
  return _pipeline_stage;
}

void c_image_processing_pipeline::set_status_msg(const std::string & msg)
{
  if( true ) {
    lock_guard lock(_status_lock);
    _statusmsg = msg;
  }

  CF_DEBUG("STATUS: %s", msg.c_str());
  on_status_update();
}

std::string c_image_processing_pipeline::status_message() const
{
  std::string msg;
  if( true ) {
    lock_guard lock(_status_lock);
    msg = _statusmsg;
  }
  return msg;
}

int c_image_processing_pipeline::total_frames() const
{
  return _total_frames;
}

int c_image_processing_pipeline::processed_frames() const
{
  return _processed_frames;
}

int c_image_processing_pipeline::accumulated_frames() const
{
  return _accumulated_frames;
}

std::string c_image_processing_pipeline::create_output_path(const std::string & output_directory) const
{
  std::string output_path;

  if( output_directory.empty() ) {

    std::string parent_directory =
        _input_sequence->sources().empty() ? "." :
            get_parent_directory(_input_sequence->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            cname());

  }
  else if( !is_absolute_path(output_directory) ) {

    std::string parent_directory =
        _input_sequence->sources().empty() ? "." :
            get_parent_directory(_input_sequence->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            output_directory.c_str());
  }
  else {
    output_path =
        output_directory;
  }

  return output_path;
}

//void c_image_processing_pipeline::update_output_path()
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
//        ssprintf("%s/%s",
//            parent_directory.c_str(),
//            cname());
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

void c_image_processing_pipeline::gather_badframe_indexes()
{
  _badframes.clear();

  if( _input_sequence && !_input_sequence->is_live() ) {

    const bool was_open = _input_sequence->is_open();
    if( !was_open && !_input_sequence->open() ) {
      CF_ERROR("input_sequence_->open() fails");
      return;
    }

    const std::vector<c_input_source::sptr> &sources =
        _input_sequence->sources();

    for( uint source_index = 0, n = sources.size(); source_index < n; ++source_index ) {

      const c_input_source::sptr source =
          _input_sequence->source(source_index);

      if( source ) {

        const std::vector<uint> &bad_source_frames =
            source->badframes();

        for( uint source_frame_index : bad_source_frames ) {

          const int global_index =
              _input_sequence->global_pos(source_index,
                  source_frame_index);

          if( global_index >= 0 ) {
            _badframes.emplace_back(global_index);
          }
        }
      }
    }

    if( !was_open ) {
      _input_sequence->close(false);
    }
  }
}

bool c_image_processing_pipeline::is_bad_frame_index(int global_pos) const
{
  if( !_badframes.empty() ) {

    const std::vector<uint>::const_iterator pos =
        std::find(_badframes.begin(),
            _badframes.end(),
            global_pos);

    return pos != _badframes.end();
  }

  return false;
}

bool c_image_processing_pipeline::open_output_writer(c_output_frame_writer & writer, const c_output_frame_writer_options & opts,
    const std::string & postfix, const std::string & suffix) const
{
  if ( !writer.is_open() ) {

    const std::string filename =
        generate_output_filename(opts.output_filename,
            postfix,
            suffix);

    const bool fOK =
        writer.open(filename,
            opts.ffmpeg_opts,
            opts.output_image_processor,
            opts.output_pixel_depth,
            opts.save_frame_mapping);

    if( !fOK ) {
      CF_ERROR("writer.open('%s') fails",  filename.c_str());
      return false;
    }
  }

  return true;
}

bool c_image_processing_pipeline::add_output_writer(c_output_frame_writer & writer,
    const c_output_frame_writer_options & opts,
    const std::string & postfix,
    const std::string & suffix)
{
  if ( !writer.is_open() ) {

    const std::string filename =
        generate_output_filename(opts.output_filename,
            postfix,
            suffix);

    const bool fOK =
        writer.open(filename,
            opts.ffmpeg_opts,
            opts.output_image_processor,
            opts.output_pixel_depth,
            opts.save_frame_mapping);

    if( !fOK ) {
      CF_ERROR("writer.open('%s') fails",  filename.c_str());
      return false;
    }
  }

  const auto pos =
      std::find(_opened_writers.begin(), _opened_writers.end(),
          &writer);
  if( pos == _opened_writers.end() ) {
    _opened_writers.emplace_back(&writer);
  }

  return true;
}

bool c_image_processing_pipeline::add_output_writer(c_output_text_writer & writer,
    const std::string & filename)
{
  if( !writer.is_open() && !writer.open(filename) ) {
    CF_ERROR("writer.open('%s') fails", filename.c_str());
    return false;
  }

  const auto pos =
      std::find(_opened_text_writers.begin(), _opened_text_writers.end(),
          &writer);
  if( pos == _opened_text_writers.end() ) {
    _opened_text_writers.emplace_back(&writer);
  }

  return true;
}



bool c_image_processing_pipeline::serialize(c_config_setting setting, bool save)
{
  if( save ) {
    save_settings(setting, "class_name", get_class_name());
  }

  SERIALIZE_PROPERTY(setting, save, *this, name);
//  SERIALIZE_PROPERTY(setting, save, *this, master_source);
//  SERIALIZE_PROPERTY(setting, save, *this, master_frame_index);

  return true;
}

void c_image_processing_pipeline::set_display_type(int v)
{
  _display_type = v;
}

int c_image_processing_pipeline::display_type() const
{
  return _display_type;
}

const c_enum_member* c_image_processing_pipeline::get_display_types() const
{
  static const c_enum_member members[] = {
      { 0, "DEFAULT", "Default display type" },
      { 0 },
  };

  return members;
}

bool c_image_processing_pipeline::get_display_image(cv::OutputArray frame, cv::OutputArray mask)
{
  return false;
}

bool c_image_processing_pipeline::run(const c_input_sequence::sptr & input_sequence)
{
  CF_DEBUG("enter");
  cancel(false);
  set_running(true);

  const c_input_sequence::sptr backup_input_sequence =
      this->_input_sequence;

  if ( input_sequence ) {
    this->_input_sequence = input_sequence;
  }

  bool fOk = false;

  try {

    if ( !(fOk = initialize_pipeline()) ) {
      CF_ERROR("initialize() fails");
    }
    else if( !(fOk = run_pipeline()) ) {
      CF_ERROR("actual_run() fails");
    }

  }
  catch (const cv::Exception & e) {

    fOk = false;

    CF_ERROR("OpenCV Exception caught in c_image_processing_pipeline::run():\n"
        "%s\n"
        "%s\n"
        "%s() : %d\n"
        "file : %s\n",
        e.what(),
        e.err.c_str(), ///< error description
        e.func.c_str(),///< function name. Available only when the compiler supports getting it
        e.line,///< line number in the source file where the error has occurred
        e.file.c_str()///< source file name where the error has occurred
        );
  }
  catch (const std::exception & e) {

    fOk = false;
    CF_ERROR("std::exception catched in c_image_processing_pipeline::run(): %s\n", e.what());
  }
  catch (...) {
    fOk = false;
    CF_ERROR("Unknown exception catched in c_image_processing_pipeline::run()\n");
  }

  try {
    cleanup_pipeline();
  }
  catch (const cv::Exception & e) {

    fOk = false;

    CF_ERROR("OpenCV Exception catched in c_image_processing_pipeline::cleanup():\n"
        "%s\n"
        "%s() : %d\n"
        "file : %s\n",
        e.err.c_str(), ///< error description
        e.func.c_str(),///< function name. Available only when the compiler supports getting it
        e.line,///< line number in the source file where the error has occurred
        e.file.c_str()///< source file name where the error has occurred
        );
  }
  catch (const std::exception & e) {
    fOk = false;
    CF_ERROR("std::exception catched in c_image_processing_pipeline::cleanup(): %s\n", e.what());
  }
  catch (...) {
    fOk = false;
    CF_ERROR("Unknown exception catched in c_image_processing_pipeline::cleanup()\n");
  }

  if ( input_sequence ) {
    this->_input_sequence = backup_input_sequence;
  }

  set_running(false);
  CF_DEBUG("leave");
  return fOk;
}


bool c_image_processing_pipeline::initialize_pipeline()
{
  CF_DEBUG("Initializing '%s: %s'...", csequence_name(), cname());

  cancel(false);

  if ( !_input_sequence || _input_sequence->empty() ) {
    set_status_msg("ERROR: empty input sequence specified");
    return false;
  }

  _total_frames = 0;
  _processed_frames = 0;
  _accumulated_frames = 0;
  _statusmsg.clear();

  //  output_path_ =
  //      create_output_path(output_directory());

  gather_badframe_indexes();

  return true;
}

void c_image_processing_pipeline::cleanup_pipeline()
{
  if ( _input_sequence ) {
    _input_sequence->close();
  }

  for ( c_output_frame_writer * w :  _opened_writers ) {
    if ( w->is_open() ) {
      CF_DEBUG("Closing '%s'", w->filename().c_str());
      w->close();
    }
  }

  for ( c_output_text_writer * w :  _opened_text_writers ) {
    if ( w->is_open() ) {
      CF_DEBUG("Closing '%s'", w->filename().c_str());
      w->close();
    }
  }

  _opened_writers.clear();
  _opened_text_writers.clear();

  _darkbayer.release();
  _flatbayer.release();
  _missing_pixel_mask.release();
  _raw_bayer_image.release();

}

bool c_image_processing_pipeline::run_pipeline()
{
  CF_ERROR("c_image_processing_pipeline: Abstract run_pipeline() called");
  return false;
}


bool c_image_processing_pipeline::start_pipeline(int start_frame_index, int max_input_frames)
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  _processed_frames = 0;
  _accumulated_frames = 0;

  const bool is_live_sequence =
      _input_sequence->is_live();


  if( is_live_sequence ) {
    _total_frames = INT_MAX;
  }
  else {

    const int start_pos =
        std::max(start_frame_index, 0);

    const int end_pos =
        max_input_frames < 1 ?
            _input_sequence->size() :
            std::min(_input_sequence->size(),
                start_frame_index + max_input_frames);

    _total_frames = end_pos - start_pos;

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

  return true;
}

bool c_image_processing_pipeline::read_input_frame(const c_input_sequence::sptr & input_sequence,
    const c_image_processing_pipeline_input_options & input_options,
    cv::Mat & output_image, cv::Mat & output_mask,
    bool is_external_master_frame,
    bool save_raw_bayer) const
{
  //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  if( !is_external_master_frame ) {

    if( !_darkbayer.empty() ) {

      if( _darkbayer.size() != output_image.size() || _darkbayer.channels() != output_image.channels() ) {
        CF_FATAL("darkbayer (%dx%d*%d) and input frame (%dx%d*%d) not match",
            _darkbayer.cols, _darkbayer.rows, _darkbayer.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::subtract(output_image, _darkbayer,
          output_image);
    }

    if( !_flatbayer.empty() ) {

      if( _flatbayer.size() != output_image.size() || _flatbayer.channels() != output_image.channels() ) {
        CF_FATAL("flatbayer_ (%dx%d*%d) and input frame (%dx%d*%d) not match",
            _flatbayer.cols, _flatbayer.rows, _flatbayer.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::divide(output_image, _flatbayer,
          output_image, output_image.depth());
    }

    if ( input_options.enable_bground_normalization ) {
      nomalize_image_histogramm(output_image, output_mask, output_image,
          input_options.background_normalization_options,
          input_sequence->colorid());
    }
  }

  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }

    if ( input_options.filter_bad_pixels ) {
      remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, false);
    }

    if( output_image.depth() != CV_32F ) {
      output_image.convertTo(output_image, CV_32F,
          1. / ((1 << input_sequence->bpp())));
    }

  }
  else {

    const DEBAYER_ALGORITHM algo =
        input_options.debayer_method;

    if ( save_raw_bayer ) {

      _raw_bayer_colorid =
          input_sequence->colorid();

      if( output_image.depth() == CV_32F ) {
        output_image.copyTo(_raw_bayer_image);
      }
      else {
        output_image.convertTo(_raw_bayer_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      if( input_options.filter_bad_pixels && input_options.bad_pixels_variation_threshold > 0 ) {
        if( !bayer_denoise(_raw_bayer_image, input_options.bad_pixels_variation_threshold) ) {
          CF_ERROR("bayer_denoise() fails");
          return false;
        }
      }
    }


    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, input_sequence->colorid(), algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if ( input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, true);
        }
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }
        if( input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if( input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options.bad_pixels_variation_threshold, true);
        }

        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }

        if ( !nninterpolation(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toCString(algo));
        return false;
    }
  }

  if( input_options.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

//  if ( anscombe_.method() != anscombe_none ) {
//    anscombe_.apply(output_image, output_image);
//  }

  if ( !_missing_pixel_mask.empty() ) {

    if ( output_image.size() != _missing_pixel_mask.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          _missing_pixel_mask.cols, _missing_pixel_mask.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      _missing_pixel_mask.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, _missing_pixel_mask,
          output_mask);
    }
  }

  if ( !output_mask.empty() && input_options.inpaint_missing_pixels ) {
#if 1
    linear_interpolation_inpaint(output_image, output_mask,
        output_image);
#else
    average_pyramid_inpaint(output_image, output_mask,
        output_image);
#endif
  }


  return true;
}


bool c_image_processing_pipeline::open_input_sequence()
{
  if ( !_input_sequence ) {
    CF_ERROR("ERROR: input_sequence_ not set");
    return false;
  }

  if ( !_input_sequence->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }
  return true;
}

void c_image_processing_pipeline::close_input_sequence()
{
  if ( _input_sequence ) {
    _input_sequence->close();
  }
}

bool c_image_processing_pipeline::seek_input_sequence(int pos)
{
  if ( !_input_sequence->seek(pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(pos=%d) fails", pos);
    return false;
  }
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_sequence::c_image_sequence(const std::string & name)
{
  base::set_name(name);
}

std::string c_image_sequence::get_display_path() const
{
  std::string path;

  if( sources().size() > 0 ) {
    path = get_parent_directory(source(0)->filename());
  }
  //  else if( current_pipeline_ ) {
  //    path = current_pipeline_->output_directory();
  //  }

  return path;
}

void c_image_sequence::set_current_pipeline(const std::string & name)
{
  const auto pos =
      std::find_if(pipelines_.begin(), pipelines_.end(),
          [name](const c_image_processing_pipeline::sptr & pipeline) {
            return name == pipeline->name();
          });

  if( pos != pipelines_.end() ) {
    current_pipeline_ = *pos;
  }
}

void c_image_sequence::set_current_pipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  if( pipeline ) {

    const auto pos =
        std::find(pipelines_.begin(), pipelines_.end(), pipeline);

    if( pos == pipelines_.end() ) {
      pipelines_.emplace_back(pipeline);
    }

    current_pipeline_ = pipeline;
  }
}

const c_image_processing_pipeline::sptr& c_image_sequence::current_pipeline() const
{
  return current_pipeline_;
}

const std::vector<c_image_processing_pipeline::sptr>& c_image_sequence::pipelines() const
{
  return pipelines_;
}

void c_image_sequence::add_pipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  pipelines_.emplace_back(pipeline);

  if( !current_pipeline_ ) {
    current_pipeline_ = pipeline;
  }
}

void c_image_sequence::remove_pipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  const auto pos =
      std::find(pipelines_.begin(), pipelines_.end(), pipeline);

  if( pos != pipelines_.end() ) {

    if ( *pos == current_pipeline_ ) {
      current_pipeline_.reset();
    }

    pipelines_.erase(pos);
  }
}

void c_image_sequence::remove_pipeline(const std::string & name)
{
  const auto pos =
      std::find_if(pipelines_.begin(), pipelines_.end(),
          [name](const c_image_processing_pipeline::sptr & pipeline) {
            return name == pipeline->name();
          });

  if( pos != pipelines_.end() ) {

    if ( *pos == current_pipeline_ ) {
      current_pipeline_.reset();
    }

    pipelines_.erase(pos);
  }
}

c_image_processing_pipeline::sptr c_image_sequence::find_pipeline(const std::string & name) const
{
  const auto pos =
      std::find_if(pipelines_.begin(), pipelines_.end(),
          [name](const c_image_processing_pipeline::sptr & pipeline) {
            return name == pipeline->name();
          });

  return pos == pipelines_.end() ? nullptr : *pos;
}

bool c_image_sequence::pipeline_exists(const std::string & name) const
{
  const auto pos =
      std::find_if(pipelines_.begin(), pipelines_.end(),
          [name](const c_image_processing_pipeline::sptr & pipeline) {
            return name == pipeline->name();
          });

  return pos != pipelines_.end();
}


bool serialize_base_input_options(c_config_setting section, bool save, c_image_processing_pipeline_input_options & opts)
{
  c_config_setting subsection;

  SERIALIZE_OPTION(section, save, opts, debayer_method);
  SERIALIZE_OPTION(section, save, opts, darkbayer_filename);
  SERIALIZE_OPTION(section, save, opts, flatbayer_filename);
  SERIALIZE_OPTION(section, save, opts, missing_pixel_mask_filename);
  SERIALIZE_OPTION(section, save, opts, missing_pixels_marked_black);
  SERIALIZE_OPTION(section, save, opts, inpaint_missing_pixels);
  SERIALIZE_OPTION(section, save, opts, filter_bad_pixels);
  SERIALIZE_OPTION(section, save, opts, detect_bad_asi_frames);
  SERIALIZE_OPTION(section, save, opts, bad_pixels_variation_threshold);
  SERIALIZE_OPTION(section, save, opts, enable_color_maxtrix);
  SERIALIZE_OPTION(section, save, opts, start_frame_index);
  SERIALIZE_OPTION(section, save, opts, max_input_frames);

  SERIALIZE_OPTION(section, save, opts, enable_bground_normalization);
  if( (subsection = SERIALIZE_GROUP(section, save, "bground_normalization")) ) {
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, norm_type);
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, stretch);
    SERIALIZE_OPTION(subsection, save, opts.background_normalization_options, offset);
  }

  if( save ) {
    if( opts.input_image_processor ) {
      save_settings(section, "input_image_processor",
          opts.input_image_processor->name());
    }
  }
  else {
    std::string s;

    if( load_settings(section, "input_image_processor", &s) && !s.empty() ) {
      opts.input_image_processor =
          c_image_processor_collection::default_instance()->get(s);
    }
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
