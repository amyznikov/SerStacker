/*
 * c_stacking_pipeline.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"

#include <core/proc/estimate_noise.h>
#include <core/proc/extract_channel.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/autoclip.h>
#include <core/proc/normalize.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/fft_transfer_spectrum_profile.h>
#include <core/proc/smap.h>
#include <core/io/save_image.h>
#include <core/io/rgbamix.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/get_time.h>
#include <core/debug.h>



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    roi_selection_none = 0,
//    roi_selection_planetary_disk = 1

const struct roi_selection_method_desc roi_selection_methods[] = {
    {"none", roi_selection_none},
    {"planetary_disk", roi_selection_planetary_disk},
    {nullptr, roi_selection_none},
};

std::string toStdString(enum roi_selection_method v)
{
  for ( uint i = 0; roi_selection_methods[i].name; ++i ) {
    if ( roi_selection_methods[i].value == v ) {
      return roi_selection_methods[i].name;
    }
  }
  return "";
}

enum roi_selection_method fromStdString(const std::string & s, enum roi_selection_method defval )
{
  const char * cstr = s.c_str();

  for ( uint i = 0; roi_selection_methods[i].name; ++i ) {
    if ( strcasecmp(roi_selection_methods[i].name, cstr) == 0 ) {
      return roi_selection_methods[i].value;
    }
  }
  return defval;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const struct frame_registration_method_desc frame_registration_methods[] = {
    { "Feature Based Registration (SURF)", frame_registration_method_surf },
    { "Planetary Disk Registration", frame_registration_method_planetary_disk},
    { "Star Field Registration", frame_registration_method_star_field},
    { "Skip", frame_registration_method_skip},

    { nullptr, frame_registration_method_unknown }, // must be last
};

std::string toStdString(enum frame_registration_method v)
{
  for ( uint i = 0; frame_registration_methods[i].name; ++i ) {
    if ( frame_registration_methods[i].value == v ) {
      return frame_registration_methods[i].name;
    }
  }
  return "";
}

enum frame_registration_method fromStdString(const std::string & s, enum frame_registration_method defval)
{
  const char * cstr = s.c_str();

  for ( uint i = 0; frame_registration_methods[i].name; ++i ) {
    if ( strcasecmp(frame_registration_methods[i].name, cstr) == 0 ) {
      return frame_registration_methods[i].value;
    }
  }
  return defval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const struct frame_accumulation_method_desc frame_accumulation_methods[] = {
    { "average_masked", frame_accumulation_average_masked },
    { "average_weigted", frame_accumulation_average_weighted },
    { nullptr, frame_accumulation_method_unknown, },
};

std::string toStdString(enum frame_accumulation_method v)
{
  for ( uint i = 0; frame_accumulation_methods[i].name; ++i ) {
    if ( frame_accumulation_methods[i].value == v ) {
      return frame_accumulation_methods[i].name;
    }
  }
  return "";
}

enum frame_accumulation_method fromStdString(const std::string  & s,
    enum frame_accumulation_method defval )
{
  const char * cstr = s.c_str();

  for ( uint i = 0; frame_accumulation_methods[i].name; ++i ) {
    if ( strcasecmp(frame_accumulation_methods[i].name, cstr) == 0 ) {
      return frame_accumulation_methods[i].value;
    }
  }
  return defval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const struct frame_upscale_option_desc frame_upscale_options[] ={
    {"disabled", frame_upscale_disabled},
    {"before_align", frame_upscale_before_align},
    {"after_align", frame_upscale_after_align},
    {nullptr, frame_upscale_disabled},
} ;

std::string toStdString(enum frame_upscale_option v)
{
  for ( uint i = 0; frame_upscale_options[i].name; ++i ) {
    if ( frame_upscale_options[i].value == v ) {
      return frame_upscale_options[i].name;
    }
  }
  return "";
}

enum frame_upscale_option fromStdString(const std::string  & s,
    enum frame_upscale_option defval )
{
  const char * cstr = s.c_str();

  for ( uint i = 0; frame_upscale_options[i].name; ++i ) {
    if ( strcasecmp(frame_upscale_options[i].name, cstr) == 0 ) {
      return frame_upscale_options[i].value;
    }
  }
  return defval;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

// /home/data/scope/test_records/2020-10-05/Challenge/
class c_video_writer {

public:

  ~c_video_writer()
  {
    close();
  }

  bool open(const std::string & fileName, const cv::Size & frameSize, bool color)
  {
    const std::string suffix = get_file_suffix(fileName);
    if ( strcasecmp(suffix.c_str(), ".ser") == 0 ) {

      serVideo.create(fileName, frameSize.width, frameSize.height,
          color ? COLORID_BGR : COLORID_MONO,
              16);

      if ( !serVideo.is_open() ) {
        CF_ERROR("Can not write ser file '%s'", fileName.c_str());
        return false;
      }

    }
    else {

      aviVideo.open(fileName, cv::CAP_FFMPEG,
          cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'),
          10,
          frameSize,
          color);

      if ( !aviVideo.isOpened() ) {
        CF_ERROR("Can not write aligned video file '%s'", fileName.c_str());
        return false;
      }
    }

    return true;
  }


  bool isOpened() const
  {
    return aviVideo.isOpened() || serVideo.is_open();
  }

  bool write(const cv::Mat & currenFrame) {

    if ( aviVideo.isOpened() ) {
      currenFrame.convertTo(tmp, CV_8U, 255);
      aviVideo.write(tmp);
    }
    else if ( serVideo.is_open() ) {
      currenFrame.convertTo(tmp, CV_16U, 65535);
      serVideo.write(tmp);
    }
    else {
      CF_ERROR("ERROR: Output video file is not open");
      return false;
    }

    return true;
  }


  void close()
  {
    aviVideo.release();
    serVideo.close();
    tmp.release();
  }

protected:
  cv::VideoWriter aviVideo;
  c_ser_writer serVideo;
  cv::Mat tmp;
};


void write_aligned_video(const cv::Mat & currenFrame, c_video_writer & output_aligned_video,
    const c_image_stacking_output_options & output_options, const std::string & output_directory)
{
  if ( output_options.write_aligned_video && !output_options.output_aligned_video_filename.empty() ) {

    if ( !output_aligned_video.isOpened() ) {

      std::string pathfilename =
          output_options.output_aligned_video_filename;

      if ( !is_absolute_path(pathfilename)  ) {
        pathfilename = ssprintf("%s/%s", output_directory.c_str(), pathfilename.c_str());
      }

      if ( !create_path(get_parent_directory(pathfilename)) ) {
        CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
        return;
      }

      output_aligned_video.open(pathfilename,
          currenFrame.size(),
          currenFrame.channels() > 1);

      if ( !output_aligned_video.isOpened() ) {
        CF_ERROR("Can not write aligned video file '%s'",
            pathfilename.c_str());
      }
    }

    if ( output_aligned_video.isOpened() ) {
      output_aligned_video.write(currenFrame);
    }
  }
}


}


namespace {
//static bool write_image(cv::InputArray _image, cv::InputArray _mask, const std::string & fname)
//{
//  cv::Mat image_to_write;
//
//  if ( _mask.empty() || _image.channels() != 3 ) {
//    image_to_write = _image.getMat();
//  }
//  else {
//    cv::Mat alpha;
//
//    switch ( _image.depth() ) {
//    case CV_8U :
//      alpha = _mask.getMat();
//      break;
//    case CV_8S :
//      alpha = _mask.getMat();
//      break;
//    case CV_16U :
//      _mask.getMat().convertTo(alpha,  _image.depth(), UINT16_MAX / UINT8_MAX);
//      break;
//    case CV_16S :
//      _mask.getMat().convertTo(alpha, _image.depth(), INT16_MAX / (double) UINT8_MAX);
//      break;
//    case CV_32S :
//      _mask.getMat().convertTo(alpha, _image.depth(), INT32_MAX / (double) UINT8_MAX);
//      break;
//    case CV_32F :
//      _mask.getMat().convertTo(alpha, _image.depth(), 1.0 / UINT8_MAX);
//      break;
//    case CV_64F :
//      _mask.getMat().convertTo(alpha, _image.depth(), 1.0 / UINT8_MAX);
//      break;
//    }
//
//    image_to_write.create(_image.size(), CV_MAKETYPE(_image.depth(), _image.channels() + 1));
//
//    cv::Mat src[2] = {_image.getMat(), alpha };
//    const int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };
//    cv::mixChannels(src, 2, &image_to_write, 1, from_to, 4);
//  }
//
//  return save_image(image_to_write, fname);
//}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_stacking_options::c_image_stacking_options(const std::string & name)
  : name_(name)
{
}

c_image_stacking_options::ptr c_image_stacking_options::create(const std::string & name)
{
  return this_class::ptr(new this_class(name));
}

void c_image_stacking_options::set_name(const std::string & name)
{
  this->name_ = name;
}

const std::string & c_image_stacking_options::name() const
{
  return this->name_;
}

void c_image_stacking_options::set_input_sequence(const c_input_sequence::ptr & sequence)
{
  this->input_sequence_ = sequence;
}

const c_input_sequence::ptr & c_image_stacking_options::input_sequence() const
{
  return this->input_sequence_;
}

c_input_source::ptr c_image_stacking_options::add_input_source(const std::string & pathfilename)
{
  return input_sequence_ ? input_sequence_->add_source(pathfilename) : nullptr;
}

const std::vector<c_input_source::ptr> & c_image_stacking_options::input_sources() const
{
  return input_sequence_->sources();
}

bool c_image_stacking_options::add_input_sources(const std::vector<std::string> & pathfilenames)
{
  return input_sequence_ ? input_sequence_->add_sources(pathfilenames) : false;
}

bool c_image_stacking_options::remove_input_source(const c_input_source::ptr & source)
{
  return input_sequence_ ? input_sequence_->remove_source(source) : false;
}

c_image_stacking_input_options & c_image_stacking_options::input_options()
{
  return input_options_;
}

const c_image_stacking_input_options & c_image_stacking_options::input_options() const
{
  return input_options_;
}

c_stacking_master_frame_options & c_image_stacking_options::master_frame_options()
{
  return master_frame_options_;
}

c_roi_selection_options & c_image_stacking_options::roi_selection_options()
{
  return roi_selection_options_;
}

const c_roi_selection_options & c_image_stacking_options::roi_selection_options() const
{
  return roi_selection_options_;
}

c_feature_based_roi_selection::ptr c_image_stacking_options::create_roi_selection() const
{
  switch ( roi_selection_options_.method ) {
  case roi_selection_planetary_disk:
    return c_planetary_disk_selection::create(roi_selection_options_.crop_size);
    break;
  default :
    break;
  }
  return nullptr;
}


const c_stacking_master_frame_options & c_image_stacking_options::master_frame_options() const
{
  return master_frame_options_;
}

c_frame_registration_options & c_image_stacking_options::frame_registration_options()
{
  return frame_registration_options_;
}

const c_frame_registration_options & c_image_stacking_options::frame_registration_options() const
{
  return frame_registration_options_;
}

c_frame_registration::ptr c_image_stacking_options::create_frame_registration() const
{
  switch (frame_registration_options_.registration_method) {

    case frame_registration_method_surf:
      return c_feature_based_registration::create(frame_registration_options_.base_options,
          frame_registration_options_.feature_options);

    case frame_registration_method_planetary_disk:
      return c_planetary_disk_registration::create(frame_registration_options_.base_options,
          frame_registration_options_.planetary_disk_options);

    case frame_registration_method_star_field:
      return c_star_field_registration::create(frame_registration_options_.base_options,
          frame_registration_options_.star_field_options);

    default:
      break;
  }

  return nullptr;
}

c_frame_accumulation_options & c_image_stacking_options::accumulation_options()
{
  return accumulation_options_;
}

const c_frame_accumulation_options & c_image_stacking_options::accumulation_options() const
{
  return accumulation_options_;
}

c_frame_accumulation::ptr c_image_stacking_options::create_frame_accumulation() const
{
  switch ( accumulation_options_.accumulation_method ) {
  case frame_accumulation_average_masked :
    return c_frame_accumulation::ptr(new c_frame_accumulation_with_mask());
  case frame_accumulation_average_weighted :
    return c_frame_accumulation::ptr(new c_frame_accumulation_with_weights());
  default :
    break;
  }
  return nullptr;
}

c_image_stacking_output_options & c_image_stacking_options::output_options()
{
  return output_options_;
}

const c_image_stacking_output_options & c_image_stacking_options::output_options() const
{
  return output_options_;
}

std::string c_image_stacking_options::get_displaypatch() const
{
  std::string path;

  if ( input_sequence_&& input_sequence_->sources().size() > 0 ) {
    path = get_parent_directory(input_sequence_->source(0)->filename());
  }
  else if ( !output_options_.output_directory.empty() ) {
    path = output_options_.output_directory;
  }

  return path;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_stacks_collection :: c_image_stacks_collection()
{

}

c_image_stacks_collection::ptr c_image_stacks_collection::create()
{
  return this_class::ptr(new this_class());
}


size_t c_image_stacks_collection::size() const
{
  return this->stacks_.size();
}

const std::vector<c_image_stacking_options::ptr> & c_image_stacks_collection::items() const
{
  return this->stacks_;
}

c_image_stacking_options::ptr c_image_stacks_collection::item(size_t index) const
{
  return index < stacks_.size() ? stacks_[index] : nullptr;
}

c_image_stacking_options::ptr c_image_stacks_collection::item(const std::string & name) const
{
  ssize_t index = indexof(name);
  return index >= 0 ? stacks_[index] : nullptr;
}

void c_image_stacks_collection::add(const c_image_stacking_options::ptr & pipeline)
{
  stacks_.emplace_back(pipeline);
}

bool c_image_stacks_collection::remove(const c_image_stacking_options::ptr & pipeline)
{
  const size_t original_size = stacks_.size();
  if ( original_size > 0 ) {
    stacks_.erase(std::remove(stacks_.begin(), stacks_.end(), pipeline), stacks_.end());
  }
  return stacks_.size() < original_size;
}

ssize_t c_image_stacks_collection::indexof(const c_image_stacking_options::ptr & pipeline) const
{
  std::vector<c_image_stacking_options::ptr>::const_iterator ii =
      std::find(stacks_.begin(), stacks_.end(), pipeline);
  return ii == stacks_.end() ? -1 : ii - stacks_.begin();
}

ssize_t c_image_stacks_collection::indexof(const std::string & name) const
{
  std::vector<c_image_stacking_options::ptr>::const_iterator ii =
      std::find_if(stacks_.begin(), stacks_.end(),
          [&name](const c_image_stacking_options::ptr & pipeline) -> bool {
            return pipeline && strcasecmp(pipeline->name().c_str(), name.c_str()) == 0;
          });
  return ii == stacks_.end() ? -1 : ii - stacks_.begin();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const c_image_stacking_options::ptr & c_image_stacking_pipeline::stacking_options() const
{
  return stacking_options_;
}

void c_image_stacking_pipeline::set_canceled(bool canceled)
{
  canceled_ = canceled;
}

bool c_image_stacking_pipeline::canceled() const
{
  return canceled_;
}

int c_image_stacking_pipeline::total_frames() const
{
  return total_frames_;
}

int c_image_stacking_pipeline::processed_frames() const
{
  return processed_frames_;
}

int c_image_stacking_pipeline::accumulated_frames() const
{
  lock_guard lock(accumulator_lock_);
  return frame_accumulation_ ? frame_accumulation_->accumulated_frames() : 0;
}

std::string c_image_stacking_pipeline::status_message() const
{
  lock_guard lock(status_lock_);
  return statusmsg_;
}

void c_image_stacking_pipeline::set_status_msg(const std::string & msg)
{
  if ( true  ) {
    lock_guard lock(status_lock_);
    statusmsg_ = msg;
  }

  CF_DEBUG("STATUS: %s", msg.c_str());
  emit_status_changed();
}


bool c_image_stacking_pipeline::run(const c_image_stacking_options::ptr & options)
{

  cv::Mat tmp;
  c_video_writer output_aligned_video;
  std::string output_file_name;
  std::string output_directory;
  bool fOk;

  CF_DEBUG("Started '%s'", options->name().c_str());

  total_frames_ = 0;
  processed_frames_ = 0;
  set_canceled(false);

  set_status_msg("INITIALIZATION ...");


  c_input_sequence::ptr input_sequence =
      options->input_sequence();

  if ( !input_sequence || input_sequence->empty() ) {
    set_status_msg("ERROR: empty input sequence specified");
    return false;
  }


  if ( options->roi_selection_options().method == roi_selection_none ) {
    roi_selection_.reset();
  }
  else if ( !(roi_selection_ = options->create_roi_selection()) ) {
    set_status_msg("ERROR: create_roi_selection() fails");
    return false;
  }

  set_status_msg("PREPARE REFERENCE FRAME ...");

  if ( !load_or_generate_reference_frame(options) ) {
    set_status_msg("ERROR: select_and_load_reference_frame() fails");
    return false;
  }

  set_status_msg("PREPARE MAIN LOOP ...");

  const c_auto_close_input_sequence auto_close_on_exit(
      input_sequence);

  const c_image_stacking_input_options & input_options =
      options->input_options();

  const c_stacking_master_frame_options & master_frame_options =
      options->master_frame_options();

  const c_frame_accumulation_options & accumulation_options =
      options->accumulation_options();

  const c_frame_registration_options & registration_options =
      options->frame_registration_options();

  const c_image_stacking_output_options & output_options =
      options->output_options();

  frame_registration_ =
      options->create_frame_registration();

  if ( !frame_registration_ ) {
    set_status_msg("ERROR: create_frame_registration() fails");
    return false;
  }

  frame_accumulation_ =
      options->create_frame_accumulation();

  if ( !frame_accumulation_ ) {
    set_status_msg("ERROR: create_frame_accumulation() fails");
    return false;
  }

  if ( !(output_directory = output_options.output_directory).empty() ) {

    if ( !is_absolute_path(output_directory) ) {
      output_directory = ssprintf("%s/%s",
          get_parent_directory(input_sequence->source(0)->filename()).c_str(),
          output_directory.c_str());
    }

  }
  else {

    output_directory = get_parent_directory(
        input_sequence->source(0)->filename());

    if ( output_directory.empty() ) {
      output_directory = ".";
    }

  }


  if ( options->accumulation_options().upscale_option == frame_upscale_before_align ) {
    upscale(reference_frame_, reference_mask_, reference_frame_, reference_mask_);
  }

  CF_DEBUG("Reference frame : %dx%d depth=%d channels=%d",
      reference_frame_.cols, reference_frame_.rows,
      reference_frame_.depth(),
      reference_frame_.channels());

  CF_DEBUG("Reference mask : %dx%d depth=%d channels=%d",
      reference_mask_.cols, reference_mask_.rows,
      reference_mask_.depth(),
      reference_mask_.channels());

  if ( frame_registration_->ecc_normalization_scale() > 0 ) {
    frame_registration_->set_ecc_normalization_noise(ecc_normalization_noise_);
  }

  if ( !(fOk = frame_registration_->setup_referece_frame(reference_frame_, reference_mask_)) ) {
    set_status_msg("ERROR: setup_referece_frame() fails");
  }

  if ( !fOk || output_options.write_registartion_reference_frames ) {

    if ( !frame_registration_->reference_feature_image().empty() ) {
      save_image(frame_registration_->reference_feature_image(),
          ssprintf("%s/%s-feature-reference.tiff", output_directory.c_str(),
              options->name().c_str() ));
    }
    if ( !frame_registration_->ecc().reference_image().empty() ) {
      save_image(frame_registration_->ecc().reference_image(),
          ssprintf("%s/ecc-reference.tiff", output_directory.c_str(),
              options->name().c_str()));
    }
    if ( !frame_registration_->eccflow().reference_image().empty() ) {
      save_image(frame_registration_->eccflow().reference_image(),
          ssprintf("%s/eccflow-reference.tiff", output_directory.c_str(),
              options->name().c_str()));
    }
  }

  if ( !fOk ) {
    return false;
  }


  if ( !master_frame_options.generate_master_frame && master_source_index_ >= 0 ) {

    CF_DEBUG("\n\nATTENTION: ADDING MASTER TO ACCUMULATOR!!!!\n\n");


    //const cv::Rect ROI = frame_registration_->reference_ROI();
    const cv::Mat RF = reference_frame_; // ROI.empty() ? reference_frame_ : reference_frame_(ROI);
    const cv::Mat RM = reference_mask_; // reference_mask_.empty() ? reference_mask_ : (ROI.empty() ? reference_mask_ : reference_mask_(ROI));

    //CF_DEBUG("ROI: %dx%d x=%d y=%d ", ROI.width, ROI.height, ROI.x, ROI.y);
    CF_DEBUG("RF: %dx%d", RF.cols, RF.rows);
    CF_DEBUG("RM: %dx%d", RM.cols, RM.rows);

    if ( true ) {
      lock_guard lock(accumulator_lock_);

      switch ( accumulation_options.accumulation_method ) {
      case frame_accumulation_average_masked :

        if ( accumulation_options.upscale_option == frame_upscale_after_align ) {
          upscale(RF, RM, current_frame_, current_mask_);
          frame_accumulation_->add(current_frame_, current_mask_);
        }
        else {
          frame_accumulation_->add(RF, RM);
        }

        ++processed_frames_;
        break;

      case frame_accumulation_average_weighted :

        compute_weights(RF, RM, current_weights_);

        if ( accumulation_options.upscale_option == frame_upscale_after_align ) {
          upscale(RF, current_weights_, current_frame_, current_weights_);
          frame_accumulation_->add(current_frame_, current_weights_);
        }
        else {
          frame_accumulation_->add(RF, current_weights_);
        }

        ++processed_frames_;
        break;
      }
    }

    emit_accumulator_changed();
  }


  if ( !input_sequence->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }


  int n = input_sequence->size();
  int start_frame_index = 0;
  input_sequence->seek(start_frame_index);

  processed_frames_ = 0;
  total_frames_ = n - input_sequence->current_pos();
  set_status_msg("RUNNING...");

  for ( int i = 0; i < n; ++i, emit_status_changed() ) {

    double t0, t1, start_time, total_time;
    double time_read = 0;
    double time_upscale = 0;
    double time_register = 0;
    double time_remap = 0;
    double time_average = 0;
    double time_compute_weights = 0;

    if ( canceled() ) {
      break;
    }

    t0 = start_time = get_realtime_ms();
    fOk = read_input_frame(input_sequence, input_options, current_frame_, current_mask_);
    time_read = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( !fOk || canceled() ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    if ( canceled() ) {
      break;
    }

    if ( !master_frame_options.generate_master_frame || master_source_index_ < 0 || input_sequence->current_pos() != master_frame_index_ )  {
      ++processed_frames_;
    }

    CF_DEBUG("H:");
    if ( !select_image_roi(roi_selection_, current_frame_, current_mask_, current_frame_, current_mask_) ) {
      CF_DEBUG("H:");
      continue;
    }
    CF_DEBUG("H:");

    if ( canceled() ) {
      break;
    }

    if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
      compute_weights(current_frame_, current_mask_, current_weights_);
    }

    time_compute_weights = (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( accumulation_options.upscale_option == frame_upscale_before_align ) {
      upscale(current_frame_, current_mask_, current_frame_, current_mask_);
      if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
        upscale(current_weights_, cv::noArray(), current_weights_, cv::noArray());
      }
    }
    time_upscale = (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( frame_registration_ ) {
      if ( accumulation_options.upscale_option == frame_upscale_after_align ) {
        if ( !frame_registration_->register_frame(current_frame_, cv::noArray(), cv::noArray(), cv::noArray()) ) {
          CF_ERROR("[F %6d] reg->register_frame() fails\n", i);
          continue;
        }
      }
      else {
        if ( !frame_registration_->register_frame(current_frame_, current_frame_, cv::noArray(), cv::noArray()) ) {
          CF_ERROR("[F %6d] reg->register_frame() fails\n", i);
          continue;
        }
      }
    }

    time_register = (t1 = get_realtime_ms()) - t0, t0 = t1;

    if ( frame_registration_ ) {
      if ( accumulation_options.upscale_option == frame_upscale_after_align ) {

        //const cv::Rect ROI = frame_registration_->current_ROI();

        upscale(frame_registration_->current_remap(), cv::noArray(), tmp, cv::noArray());

        switch ( accumulation_options.accumulation_method ) {
        case frame_accumulation_average_masked :
          if ( !frame_registration_->custom_remap(current_frame_, current_frame_, /*ROI, */tmp, current_mask_, current_mask_) ) {
            CF_ERROR("[F %6d] reg->custom_remap(current_frame) fails\n", i);
            continue;
          }
          break;

        case frame_accumulation_average_weighted :
          if ( !frame_registration_->custom_remap(current_frame_, current_frame_, /*ROI,*/ tmp) ) {
            CF_ERROR("[F %6d] reg->custom_remap(current_frame) fails\n", i);
            continue;
          }
          if ( !frame_registration_->custom_remap(current_weights_, current_weights_, /*ROI,*/ tmp,
              cv::noArray(), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT) ) {
            CF_ERROR("[F %6d] reg->custom_remap(current_weights) fails\n", i);
            continue;
          }
          break;
        }

      }
      else {

        switch ( accumulation_options.accumulation_method ) {
        case frame_accumulation_average_masked :
          if ( !frame_registration_->remap(cv::noArray(), cv::noArray(), current_mask_, current_mask_) ) {
            CF_ERROR("[F %6d] reg->remap(current_frame) fails\n", i);
            continue;
          }
          break;

        case frame_accumulation_average_weighted :
          if ( !frame_registration_->remap(current_weights_, current_weights_,
              cv::noArray(), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT) ) {
            CF_ERROR("[F %6d] reg->remap(current_weights) fails\n", i);
            continue;
          }
          break;

        }

      }
    }

    time_remap = (t1 = get_realtime_ms()) - t0, t0 = t1;

    write_aligned_video(current_frame_,
        output_aligned_video,
        output_options,
        output_directory);


    if ( frame_accumulation_->accumulated_frames() > 0 && current_frame_.size() != frame_accumulation_->accumulator_size() ) {
      CF_ERROR("ERROR: current frame and accumulator sizes not match");
      break;
    }

    if ( !master_frame_options.generate_master_frame || master_source_index_ < 0 || input_sequence->current_pos() != master_frame_index_ )  {
      if ( true ) {
        lock_guard lock(accumulator_lock_);
        switch ( accumulation_options.accumulation_method ) {
        case frame_accumulation_average_masked :
          frame_accumulation_->add(current_frame_, current_mask_);
          break;
        case frame_accumulation_average_weighted :
          frame_accumulation_->add(current_frame_, current_weights_);
          break;
        }
      }
      emit_accumulator_changed();
    }

    time_average = (t1 = get_realtime_ms()) - t0, t0 = t1;


    total_time = get_realtime_ms() - start_time;

    CF_DEBUG("[F %5d / %5d / %6d] OK  %g ms\n"
        "read    : %g ms\n"
        "weights : %g ms\n"
        "upscale : %g ms\n"
        "register: %g ms\n"
        "remap   : %g ms\n"
        "average : %g ms\n"
        "-----------\n\n\n",
        i + start_frame_index, processed_frames_, n, total_time,

        time_read,
        time_compute_weights,
        time_upscale,
        time_register,
        time_remap,
        time_average);

    emit_accumulator_changed();
  }


  output_aligned_video.close();

  set_status_msg("FINISHING ...");

  if ( !compute_accumulated_image(current_frame_, current_mask_) ) {
    CF_ERROR("FATAL: compute_accumulated_image() fails");
    return false;
  }

  if ( output_file_name.empty() ) {

    output_file_name =
        ssprintf("%s/%s-32F.tiff",
            output_directory.c_str(),
            options->name().c_str());
  }


  bool transfer_fft_spectrum_power = true;
  if ( transfer_fft_spectrum_power ) {

    //const cv::Rect ROI = frame_registration_->reference_ROI();
    const cv::Mat RF = reference_frame_;//ROI.empty() ? reference_frame_ : reference_frame_(ROI);
    //const cv::Mat CM = ROI.empty() ? current_mask_ : current_mask_(ROI);

    if ( RF.size() == current_frame_.size() ) {

      cv::Mat SF;

      fft_transfer_spectrum_profile(RF,
          current_frame_,
          SF);

      clip_range(SF, 0, 1);

      std::string output_name =
          output_file_name;

      set_file_suffix(output_name,
          "-FFTS.tiff");

      CF_DEBUG("Saving '%s'", output_file_name.c_str());
      if ( !write_image(output_name, output_options, SF, current_mask_) ) {
        CF_ERROR("write_image('%s') fails", output_name.c_str());
      }
    }
  }

  normalize_minmax(current_frame_, current_frame_,
      0.01, 0.99,
      current_mask_,
      true);

  CF_DEBUG("Saving '%s'",  output_file_name.c_str());
  if ( !write_image(output_file_name, output_options, current_frame_, current_mask_) ) {
    CF_ERROR("write_image('%s') fails", output_file_name.c_str());
  }


  if ( options->frame_registration_options().registration_method != frame_registration_method_star_field ) {

    cv::Mat wbmask, wbimage;
    std::string outname;

    set_file_suffix(outname = output_file_name,
        "-WBH.tiff");

    cv::compare(current_frame_, 0.1, wbmask, cv::CMP_GT);

    if ( wbmask.channels() > 1 ) {
      cv::cvtColor(wbmask, wbmask, cv::COLOR_BGR2GRAY);
      cv::compare(wbmask, 255, wbmask, cv::CMP_GE);
    }

    cv::bitwise_and(current_mask_, wbmask, wbmask);

    if ( !histogram_white_balance(current_frame_, wbmask, wbimage, 5, 95) ) {
      CF_ERROR("histogram_white_balance(wbimage) fails");
      wbimage = current_frame_;
    }
    else {

      normalize_minmax(wbimage, wbimage,
          0.01, 0.99,
          current_mask_,
          true);
    }

    CF_DEBUG("Saving '%s'", outname.c_str());
    if ( !write_image(outname, output_options, wbimage, current_mask_) ) {
      CF_ERROR("write_image('%s') fails", outname.c_str());
    }
  }
  else {

    if ( !histogram_white_balance(current_frame_, current_mask_, current_frame_, 5, 95) ) {
      CF_ERROR("histogram_white_balance() fails");
    }
    else {

      normalize_minmax(current_frame_, current_frame_,
          0.01, 0.99,
          current_mask_,
          true);

      set_file_suffix(output_file_name,
          "-WBH.tiff");

      CF_DEBUG("SAVING '%s'", output_file_name.c_str());
      if ( !write_image(output_file_name, output_options, current_frame_, current_mask_) ) {
        CF_ERROR("write_image('%s') fails", output_file_name.c_str());
      }

    }
  }

  set_status_msg("FINISHED");

  return true;
}


bool c_image_stacking_pipeline::load_or_generate_reference_frame(const c_image_stacking_options::ptr & options)
{

  const c_stacking_master_frame_options & master_frame_options =
      options->master_frame_options();

  int num_total_frames = 0;

  c_input_sequence::ptr input_sequence;
  c_auto_close_input_sequence auto_close(input_sequence);
  bool fOk = false;

  master_source_index_ = -1;
  master_frame_index_ = -1;
  master_file_name_.clear();



  if ( master_frame_options.use_ffts_from_master_path ) {

    if ( master_frame_options.master_source_path.empty() ) {
      CF_ERROR("Master path is not specified for use_ffts_from_master_path=true");
      return false;
    }

    if ( is_absolute_path(master_frame_options.master_source_path) ) {

      master_file_name_ = ssprintf("%s/%s-32F-FFTS.tiff",
          master_frame_options.master_source_path.c_str(),
          options->name().c_str());

    }
    else if ( input_sequence->sources().size() > 0 ) {

      master_file_name_ = ssprintf("%s/%s/%s-32F-FFTS.tiff",
          get_parent_directory(input_sequence->source(0)->filename()).c_str(),
          master_frame_options.master_source_path.c_str(),
          options->name().c_str());

    }
    else {
      CF_ERROR("Can not generate master_file_name_ from given input");
      return false;
    }
  }
  else if ( (master_file_name_ = master_frame_options.master_source_path).empty() ) {
    master_file_name_ = options->input_sequence()->source(master_source_index_ = 0)->filename();
  }
  else {

    std::vector<c_input_source::ptr>::const_iterator source_pos =
        std::find_if(options->input_sequence()->sources().begin(), options->input_sequence()->sources().end(),
            [master_frame_options](const c_input_source::ptr & s ) -> bool {
              return s->filename() == master_frame_options.master_source_path;
            });

    if ( source_pos != options->input_sequence()->sources().end() ) {
      master_source_index_ = source_pos - options->input_sequence()->sources().begin();
    }
  }


  CF_DEBUG("USE master_file_name_ = %s", master_file_name_.c_str());

  if ( master_source_index_ >= 0 ) {
    input_sequence = options->input_sequence();
  }
  else if ( !(input_sequence = c_input_sequence::create(master_file_name_)) ) {
    CF_ERROR("ERROR: c_input_sequence::create(master_file_name_=%s) fails", master_file_name_.c_str());
    return false;
  }

  if ( !input_sequence->open() ) {
    CF_FATAL("ERROR: Can not open input source '%s'", master_file_name_.c_str());
    return false;
  }


  CF_DEBUG("master_frame_options.master_frame_index=%d", master_frame_options.master_frame_index);


  if ( master_frame_options.use_ffts_from_master_path ) {
    master_frame_index_ = 0;
  }
  else if ( (master_frame_index_ = master_frame_options.master_frame_index) < 0 ) {
    master_frame_index_ = 0;
  }
  else if ( master_source_index_ >= 0 ) {
    if ( master_frame_index_ >= input_sequence->source(master_source_index_)->size() ) {
      CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
          master_frame_index_, master_file_name_.c_str());
      return false;
    }
  }
  else if ( master_frame_index_ >= input_sequence->source(0)->size() ) {
    CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
        master_frame_index_, master_file_name_.c_str());
    return false;
  }

  CF_DEBUG("master_frame_options.master_frame_index=%d master_frame_index_=%d",
      master_frame_options.master_frame_index, master_frame_index_);


  num_total_frames = input_sequence->size();
  if ( master_source_index_ >= 0 ) {
    master_frame_index_ = input_sequence->global_pos(master_source_index_, master_frame_index_);
  }
  else {
    master_frame_index_ = input_sequence->global_pos(0, master_frame_index_);
  }

//  if ( master_source_index_ < 0 ) {
//    num_total_frames = input_sequence->source(0)->size();
//  }
//  else {
//    num_total_frames = input_sequence->source(master_source_index_)->size();
//    master_frame_index_ = input_sequence->global_pos(master_source_index_, master_frame_index_);
//  }

  CF_DEBUG("master_frame_options.master_frame_index=%d master_frame_index_=%d",
      master_frame_options.master_frame_index, master_frame_index_);

  if ( !input_sequence->seek(master_frame_index_) ) {
    CF_ERROR("ERROR: input_sequence->seek(local pos=%d) fails",
        master_frame_index_);
    return false;
  }

  if ( !master_frame_options.generate_master_frame || num_total_frames < 2 ) {

    if ( !read_input_frame(input_sequence, options->input_options(), reference_frame_, reference_mask_) ) {
      CF_FATAL("read_input_frame(reference_frame) fails");
      return false;
    }

    CF_DEBUG("H:");
    if ( !select_image_roi(roi_selection_, reference_frame_, reference_mask_, reference_frame_, reference_mask_) ) {
      CF_FATAL("select_image_roi(reference_frame) fails");
      return false;
    }
    CF_DEBUG("H:");

    fOk = true;
  }

  else { // Generate it !

    if ( true )  {
      lock_guard lock(accumulator_lock_);
      frame_accumulation_.reset(new c_frame_accumulation_with_mask());
    }

    if ( true )  {
      lock_guard lock(registration_lock_);
      frame_registration_ = options->create_frame_registration();
      // Forse disable some align features for reference frame generation
      frame_registration_->set_enable_eccflow(false);
      if ( frame_registration_->enable_ecc() ) {
        frame_registration_->ecc().set_reference_smooth_sigma(frame_registration_->ecc().input_smooth_sigma());
        if ( frame_registration_->motion_type() > ECC_MOTION_EUCLIDEAN ) {
          frame_registration_->set_motion_type(ECC_MOTION_EUCLIDEAN);
        }
      }
    }

    processed_frames_ = 0;
    total_frames_ =  std::min(master_frame_options.max_input_frames_to_generate_master_frame, num_total_frames - master_frame_index_);

    CF_DEBUG("H: num_source_frames=%d master_frame_index_=%d total_frames_=%d", num_total_frames, master_frame_index_, total_frames_);

    emit_status_changed();

    fOk = true;
    for ( ; processed_frames_ < total_frames_; ++processed_frames_, emit_status_changed() )  {

      if ( canceled() ) {
        break;
      }

      if ( !read_input_frame(input_sequence, options->input_options(), current_frame_, current_mask_) ) {
        set_status_msg("read_input_frame() fails");
        break;
      }

      if ( canceled() ) {
        break;
      }

      CF_DEBUG("H:");
      if ( !select_image_roi(roi_selection_, current_frame_, current_mask_, current_frame_, current_mask_) ) {
        CF_DEBUG("H:");
        continue;
      }
      CF_DEBUG("H:");

      if ( canceled() ) {
        break;
      }

      if ( frame_accumulation_->accumulated_frames() < 1 ) {

        if ( true ) {
          lock_guard lock(registration_lock_);
          if ( !(fOk = frame_registration_->setup_referece_frame(current_frame_, current_mask_)) ) {
            set_status_msg("ERROR: frame_registration_->setup_referece_frame() fails");
            break;
          }
        }

      }
      else if ( current_frame_.size() != frame_accumulation_->accumulator_size() ) {
        fOk = false;
        CF_ERROR("ERROR: input and accumulator frame sizes not match (input=%dx%d accumulator=%dx%d)",
            current_frame_.cols, current_frame_.rows,
            frame_accumulation_->accumulator_size().width, frame_accumulation_->accumulator_size().height);
        break;
      }

      if ( canceled() ) {
        break;
      }


      if ( true ) {
        lock_guard lock(registration_lock_);
        if ( !frame_registration_->register_frame(current_frame_,current_frame_, current_mask_, current_mask_) ) {
          CF_WARNING("WARNING: frame_registration_->register_frame(i=%d) fails", processed_frames_);
          continue;
        }
      }

      if ( canceled() ) {
        break;
      }

      if ( true )  {
        lock_guard lock(accumulator_lock_);
        if ( !(fOk = frame_accumulation_->add(current_frame_, current_mask_)) ) {
          CF_ERROR("ERROR: frame_accumulation_->add(current_frame) fails");
          break;
        }
      }

      if ( canceled() ) {
        break;
      }

      emit_accumulator_changed();
    }

    if ( fOk && !canceled() ) {
      if ( frame_accumulation_->accumulated_frames() > 0 ) {
        fOk = frame_accumulation_->compute(reference_frame_, reference_mask_);
      }
      else {
        fOk = false;
        set_status_msg("ERROR: No frames accumulated for reference frame");
      }
    }

    if ( true )  {
      lock_guard lock(registration_lock_);
      frame_registration_.reset();
    }

    if ( true )  {
      lock_guard lock(accumulator_lock_);
      frame_accumulation_.reset();
    }
  }

  if ( fOk && !canceled()) {
    ecc_normalization_noise_ =
        compute_image_noise(reference_frame_, reference_mask_,
            options->frame_registration_options().base_options.registration_channel);

    if ( master_source_index_ < 0 ) {
      ecc_normalization_noise_ *= 10;
    }

    CF_DEBUG("Set ecc_noise=%g", ecc_normalization_noise_);
  }

  return fOk && !canceled();
}



bool c_image_stacking_pipeline::read_input_frame(const c_input_sequence::ptr & input_sequence,
    const c_image_stacking_input_options & input_options,
    cv::Mat & output_image, cv::Mat & output_mask)
{
  input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  CF_DEBUG("input_sequence->read(): %dx%d channels=%d pixel_depth=%d",
      output_image.cols, output_image.rows,
      output_image.channels(),
      input_sequence->pixel_depth());

  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if ( input_options.enable_remove_bad_pixels ) {
      remove_bad_pixels(output_image);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->pixel_depth())));
  }
  else {

    extract_bayer_planes(output_image, output_image,
        input_sequence->colorid());

    if ( input_options.enable_remove_bad_pixels ) {
      remove_bad_pixels(output_image);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->pixel_depth())));

    gbinterpolation(output_image, output_image,
        input_sequence->colorid());

  }

  if ( input_options.enable_color_maxtrix && input_sequence->has_color_matrix() ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

  return true;
}

bool c_image_stacking_pipeline::select_image_roi(const c_feature_based_roi_selection::ptr & roi_selection,
    const cv::Mat & src, const cv::Mat & srcmask,
    cv::Mat & dst, cv::Mat & dstmask)
{
  if ( !roi_selection ) {
    if ( &src != &dst  ) {
      dst = src;
    }
    if ( &srcmask != &dstmask ) {
      dstmask = srcmask;
    }
    return true;
  }


  cv::Point2f feature_location;
  cv::Rect ROI;

  if ( !roi_selection->detect_object_roi(src, srcmask, feature_location, ROI) ) {
    CF_ERROR("roi_selection->detect_object_roi() fails");
    return false;
  }


  dst = src(ROI);

  if ( !srcmask.empty() ) {
    dstmask = srcmask(ROI);
  }
  else {
    dstmask.release();
  }

  return true;
}


bool c_image_stacking_pipeline::write_image(const std::string output_file_name,
    const c_image_stacking_output_options & output_options,
    const cv::Mat & output_image, const cv::Mat & output_mask)
{
  cv::Mat image_to_write;

  if ( !output_options.write_image_mask_as_alpha_channel || output_mask.empty() || output_image.channels() != 3  ) {
    image_to_write = output_image;
  }
  else {
    mergebgra(output_image, output_mask, image_to_write);
  }

  return save_image(image_to_write, output_file_name);
}


void c_image_stacking_pipeline::remove_bad_pixels(cv::Mat & image)
{
  cv::Mat medianImage, variationImage, meanVariationImage;
  cv::medianBlur(image, medianImage, 3);
  cv::absdiff(image, medianImage, variationImage);

  static float K[3*3] = {
      1./8, 1./8, 1./8,
      1./8, 0.0,  1./8,
      1./8, 1./8, 1./8,
  };
  cv::filter2D(variationImage, meanVariationImage, -1, cv::Mat1f(3, 3, K));
  medianImage.copyTo(image, variationImage > 5 * meanVariationImage);
}


void c_image_stacking_pipeline::upscale(cv::InputArray src, cv::InputArray srcmask, cv::OutputArray dst, cv::OutputArray dstmask)
{
  if ( !src.empty() && dst.needed() ) {
    cv::pyrUp(src, dst);
  }

  if ( !srcmask.empty() && dstmask.needed() ) {
    cv::pyrUp(srcmask, dstmask);

    if ( dstmask.depth() == CV_8U ) {
      cv::compare(dstmask.getMatRef(), 255, dstmask.getMatRef(), cv::CMP_GE);
    }
  }

}

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst)
{
  compute_smap(src, dst, 0.01, 0);
  if ( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }
}

double c_image_stacking_pipeline::compute_image_noise(const cv::Mat & image, const cv::Mat & mask,
    color_channel_type channel)
{
  double estimated_noise_level;

  if ( image.channels() == 1 ) {
    estimated_noise_level = estimate_noise(image, cv::noArray(), mask)[0];
  }
  else {
    cv::Mat tmp;
    extract_channel(image, tmp, cv::noArray(), cv::noArray(), channel >= 0 ? channel : color_channel_gray);
    estimated_noise_level = estimate_noise(tmp, cv::noArray(), mask)[0];
  }

  return estimated_noise_level;
}


bool c_image_stacking_pipeline::compute_accumulated_image(cv::OutputArray dst, cv::OutputArray dstmask) const
{
  lock_guard lock(accumulator_lock_);
  if ( frame_accumulation_ ) {
    return frame_accumulation_->compute(dst, dstmask);
  }
  return false;
}



