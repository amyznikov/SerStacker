/*
 * c_stacking_pipeline.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <tbb/tbb.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/extract_channel.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/autoclip.h>
#include <core/proc/normalize.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/fft.h>
#include <core/proc/smap.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
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
    { "None", frame_registration_none},
    { nullptr, frame_registration_none }, // must be last
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
    { "masked", frame_accumulation_average_masked },
    { "weigted", frame_accumulation_average_weighted },
    { "fft", frame_accumulation_fft },
    { "None", frame_accumulation_none },
    { nullptr, frame_accumulation_none, },
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
    {"none", frame_upscale_none},
    {"before_align", frame_upscale_before_align},
    {"after_align", frame_upscale_after_align},
    {nullptr, frame_upscale_none},
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


//static int countNaNs(const cv::Mat & image)
//{
//  int cnt = 0;
//
//  const int nc = image.channels();
//
//  for ( int y = 0; y < image.rows; ++y ) {
//
//    const float * p = image.ptr<const float>(y);
//
//    for ( int x = 0; x < image.cols * nc; ++x ) {
//      if ( isnan(p[x]) ) {
//        ++cnt;
//      }
//    }
//  }
//
//  return cnt;
//}

//static int countInfs(const cv::Mat & image)
//{
//  int cnt = 0;
//
//  const int nc = image.channels();
//
//  for ( int y = 0; y < image.rows; ++y ) {
//
//    const float * p = image.ptr<const float>(y);
//
//    for ( int x = 0; x < image.cols * nc; ++x ) {
//      if ( isinf(p[x]) ) {
//        ++cnt;
//      }
//    }
//  }
//
//  return cnt;
//}

//static cv::Mat2f remap2flow(const cv::Mat2f &rmap, const cv::Mat1b & mask)
//{
//  cv::Mat2f uv(rmap.size());
//
//  typedef tbb::blocked_range<int> range;
//  tbb::parallel_for(range(0, rmap.rows, 256),
//      [&rmap, &uv, &mask](const range &r) {
//        const int nx = rmap.cols;
//        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
//          for ( int x = 0; x < nx; ++x ) {
//
//            if ( mask[y][x] ) {
//              uv[y][x][0] = rmap[y][x][0] - x;
//              uv[y][x][1] = rmap[y][x][1] - y;
//            }
//            else {
//              uv[y][x][0] = 0;
//              uv[y][x][1] = 0;
//            }
//          }
//        }
//      });
//
//  return uv;
//}

static cv::Mat2f flow2remap(const cv::Mat2f &uv, const cv::Mat1b & mask)
{
  cv::Mat2f rmap(uv.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, rmap.rows, 256),
      [&rmap, &uv, &mask](const range &r) {
        const int nx = rmap.cols;
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < nx; ++x ) {
            if ( mask[y][x] ) {
              rmap[y][x][0] = -uv[y][x][0] + x;
              rmap[y][x][1] = -uv[y][x][1] + y;
            }
            else {
              rmap[y][x][0] = x;
              rmap[y][x][1] = y;
            }
          }
        }
      });

  return rmap;
}

//static cv::Mat2f iremap(const cv::Mat2f &map, const cv::Mat1b & mask)
//{
//  cv::Mat2f imap(map.size());
//
//  typedef tbb::blocked_range<int> range;
//  tbb::parallel_for(range(0, map.rows, 256),
//      [&map, &imap, &mask](const range &r) {
//        const int nx = map.cols;
//        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
//          for ( int x = 0; x < nx; ++x ) {
//
//            if ( mask[y][x] ) {
//              imap[y][x][0] = 2 * x - map[y][x][0];
//              imap[y][x][1] = 2 * y - map[y][x][1];
//            }
//            else {
//              imap[y][x][0] = x;
//              imap[y][x][1] = y;
//            }
//          }
//        }
//      });
//
//  return imap;
//}


static cv::Mat2f compute_turbulence_flow(
    ECC_MOTION_TYPE current_motion_type,
    const cv::Mat1f & current_transfrorm,
    const cv::Mat2f & current_remap,
    const cv::Mat1b & mask)
{

  cv::Mat2f uv;
  double tx = 0, ty = 0;

  // TODO: Extract also rotation component, because telescope mount sometimes could drift with FOV rotation as well.

  if ( !getTranslationComponent(current_motion_type, current_transfrorm, &tx, &ty) ) {
    CF_ERROR("getTranslationComponent(current_motion_type=%d) fails", current_motion_type);
    return uv;
  }

  uv.create(current_remap.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, current_remap.rows, 256),
      [&current_remap, &uv, &mask, tx, ty](const range &r) {
        const int nx = current_remap.cols;
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < nx; ++x ) {

            if ( mask[y][x] ) {
              uv[y][x][0] = current_remap[y][x][0] - x - tx;
              uv[y][x][1] = current_remap[y][x][1] - y - ty;
            }
            else {
              uv[y][x][0] = 0;
              uv[y][x][1] = 0;
            }
          }
        }
      });

  return uv;
}

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

c_input_options & c_image_stacking_options::input_options()
{
  return input_options_;
}

const c_input_options & c_image_stacking_options::input_options() const
{
  return input_options_;
}

c_master_frame_options & c_image_stacking_options::master_frame_options()
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


const c_master_frame_options & c_image_stacking_options::master_frame_options() const
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
  case frame_accumulation_fft :
    return c_frame_accumulation::ptr(new c_frame_accumulation_with_fft());
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
  std::string msg;
  if ( true ) {
    lock_guard lock(status_lock_);
    msg = statusmsg_;
  }
  return msg;
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
  c_video_writer output_aligned_video;
  std::string output_file_name;
  std::string output_directory;
  cv::Mat tmp;
  bool fOk;

  CF_DEBUG("Started '%s'", options->name().c_str());

  total_frames_ = 0;
  processed_frames_ = 0;

  current_frame_.release();
  reference_frame_.release();

  current_mask_.release();
  reference_mask_.release();

  current_weights_.release();
  reference_weights_.release();

  set_canceled(false);

  set_status_msg("INITIALIZATION ...");

  c_input_sequence::ptr input_sequence =
      options->input_sequence();

  if ( !input_sequence || input_sequence->empty() ) {
    set_status_msg("ERROR: empty input sequence specified");
    return false;
  }

  const c_input_options & input_options =
      options->input_options();

  const c_master_frame_options & master_frame_options =
      options->master_frame_options();

  const c_frame_accumulation_options & accumulation_options =
      options->accumulation_options();

  const c_frame_registration_options & registration_options =
      options->frame_registration_options();

  const c_image_stacking_output_options & output_options =
      options->output_options();


  ///

  if ( (output_directory = output_options.output_directory).empty() ) {

    output_directory = get_parent_directory(
        input_sequence->source(0)->filename());

  }
  else if ( !is_absolute_path(output_directory) ) {

    output_directory = ssprintf("%s/%s",
        get_parent_directory(input_sequence->source(0)->filename()).c_str(),
        output_directory.c_str());
  }

  if ( output_directory.empty() ) {
    output_directory = ".";
  }

  ///


  anscombe_.set_method(input_options.anscombe);

  if ( options->roi_selection_options().method == roi_selection_none ) {
    roi_selection_.reset();
  }
  else if ( !(roi_selection_ = options->create_roi_selection()) ) {
    set_status_msg("ERROR: create_roi_selection() fails");
    return false;
  }


  if( registration_options.registration_method != frame_registration_none ) {

    set_status_msg("PREPARE REFERENCE FRAME ...");

    if( !create_reference_frame(options, output_directory) ) {
      set_status_msg("ERROR: create_reference_frame() fails");
      return false;
    }

  }


  set_status_msg("PREPARE MAIN LOOP ...");

  const c_auto_close_input_sequence auto_close_on_exit(
      input_sequence);

  ///

  if ( true ) {
    lock_guard lock(registration_lock_);

    if ( registration_options.registration_method == frame_registration_none ) {
      frame_registration_.reset();
    }
    else if ( !(frame_registration_ = options->create_frame_registration()) ) {
      set_status_msg("ERROR: create_frame_registration() fails");
      return false;
    }
  }

  ///

  if ( true ) {
    lock_guard lock(accumulator_lock_);

    if ( accumulation_options.accumulation_method == frame_accumulation_none ) {
      frame_accumulation_.reset();
    }
    else if ( !(frame_accumulation_ = options->create_frame_accumulation()) ) {
      set_status_msg("ERROR: create_frame_accumulation() fails");
      return false;
    }
  }

  ///

  if ( frame_accumulation_ ) {

    if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
      compute_weights(reference_frame_, reference_mask_, reference_weights_);
    }

    if ( accumulation_options.upscale_option == frame_upscale_before_align ) {
      upscale(reference_frame_, reference_mask_, reference_frame_, reference_mask_);
      if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
        upscale(reference_weights_, cv::noArray(), reference_weights_, cv::noArray());
      }
    }
  }

  CF_DEBUG("Reference frame : %dx%d depth=%d channels=%d",
      reference_frame_.cols, reference_frame_.rows,
      reference_frame_.depth(),
      reference_frame_.channels());

  CF_DEBUG("Reference mask : %dx%d depth=%d channels=%d",
      reference_mask_.cols, reference_mask_.rows,
      reference_mask_.depth(),
      reference_mask_.channels());

  if ( frame_registration_ ) {
    lock_guard lock(registration_lock_);

    frame_registration_->set_ecc_normalization_noise(ecc_normalization_noise_);

    if ( !(fOk = frame_registration_->setup_referece_frame(reference_frame_, reference_mask_)) ) {
      set_status_msg("ERROR: setup_referece_frame() fails");
    }

    if ( !fOk || output_options.dump_reference_data_for_debug ) {

      if ( !frame_registration_->reference_feature_image().empty() ) {
        save_image(frame_registration_->reference_feature_image(),
            ssprintf("%s/%s-feature-reference.tiff", output_directory.c_str(),
                options->name().c_str()));
      }
      if ( !frame_registration_->ecc().reference_image().empty() ) {
        save_image(frame_registration_->ecc().reference_image(),
            ssprintf("%s/%s-ecc-reference.tiff", output_directory.c_str(),
                options->name().c_str()));
      }
      if ( !frame_registration_->eccflow().reference_image().empty() ) {
        save_image(frame_registration_->eccflow().reference_image(),
            ssprintf("%s/%s-eccflow-reference.tiff", output_directory.c_str(),
                options->name().c_str()));
      }
    }

    if ( !fOk ) {
      return false;
    }
  }


  ///

  if ( !input_sequence->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }



  CF_DEBUG("input_sequence->size()=%d", input_sequence->size());

  int start_frame_index = 0;
  if ( !input_sequence->seek(start_frame_index) ) {
    CF_ERROR("input_sequence->seek(start_frame_index=%d) fails", start_frame_index);
    set_status_msg("ERROR: input_sequence->seek(start_frame_index) fails");
    return false;
  }

  CF_DEBUG("input_sequence->current_pos()=%d", input_sequence->current_pos());

  total_frames_ = input_sequence->size() - input_sequence->current_pos();
  set_status_msg("RUNNING...");

  CF_DEBUG("total_frames_=%d", total_frames_);

  for ( processed_frames_ = 0; processed_frames_ < total_frames_; ++processed_frames_, emit_status_changed() ) {

    double t0, t1, start_time, total_time;
    double time_read = 0;
    double time_upscale = 0;
    double time_register = 0;
    double time_remap = 0;
    double time_accumulate = 0;
    double time_select_roi = 0;
    double time_compute_weights = 0;

    if ( canceled() ) {
      break;
    }

    t0 = start_time = get_realtime_ms();
    if ( !read_input_frame(input_sequence, input_options, current_frame_, current_mask_) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    time_read = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !select_image_roi(roi_selection_, current_frame_, current_mask_, current_frame_, current_mask_) ) {
      continue;
    }

    time_select_roi = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    if ( frame_accumulation_ && accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
      compute_weights(current_frame_, current_mask_, current_weights_);
    }

    time_compute_weights = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }


    if ( frame_accumulation_ && accumulation_options.upscale_option == frame_upscale_before_align ) {
      upscale(current_frame_, current_mask_, current_frame_, current_mask_);
      if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
        upscale(current_weights_, cv::noArray(), current_weights_, cv::noArray());
      }
    }

    time_upscale = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    ///////////////
    if ( frame_registration_ ) {

      if ( !frame_registration_->register_frame(current_frame_, current_frame_, current_mask_, current_mask_) ) {
        CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + start_frame_index);
        continue;
      }

      if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {
        if ( !frame_registration_->remap(current_weights_, current_weights_) ) {
          CF_ERROR("[F %6d] reg->remap(current_weights) fails\n", processed_frames_ + start_frame_index);
          continue;
        }
      }
    }

    time_register = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    write_aligned_video(current_frame_,
        output_aligned_video,
        output_options,
        output_directory);


    if ( canceled() ) {
      break;
    }

    if( output_options.frame_postprocessor ) {
      output_options.frame_postprocessor->process(current_frame_, current_mask_);
    }

    if( output_options.save_processed_frames ) {

      save_processed_frame(current_frame_, current_mask_,
          output_options, output_directory,
          options->name(),
          input_sequence->current_pos() - 1);
    }

    ///////////////

    if ( frame_accumulation_ ) {

      if ( accumulation_options.accumulation_method == frame_accumulation_average_weighted ) {

        compute_relative_weights(current_weights_, current_mask_, reference_weights_, current_weights_);

        if ( accumulation_options.upscale_option == frame_upscale_after_align ) {
          upscale(current_frame_, current_weights_, current_frame_, current_weights_);
        }

        if ( accumulation_options.accumulation_method != frame_accumulation_fft ) {
          if ( frame_accumulation_->accumulated_frames() > 0 && current_frame_.size() != frame_accumulation_->accumulator_size() ) {
            CF_ERROR("ERROR: current frame and accumulator sizes not match");
            break;
          }
        }

        if ( true ) {
          lock_guard lock(accumulator_lock_);

          frame_accumulation_->add(current_frame_, current_weights_);
        }

      }
      else {

        if ( accumulation_options.upscale_option == frame_upscale_after_align ) {
          upscale(current_frame_, current_mask_, current_frame_, current_mask_);
        }

        if ( accumulation_options.accumulation_method != frame_accumulation_fft ) {
          if ( frame_accumulation_->accumulated_frames() > 0 && current_frame_.size() != frame_accumulation_->accumulator_size() ) {
            CF_ERROR("ERROR: current frame and accumulator sizes not match");
            break;
          }
        }

        if ( true ) {
          lock_guard lock(accumulator_lock_);
          frame_accumulation_->add(current_frame_, current_mask_);
        }
      }

      emit_accumulator_changed();
    }

    time_accumulate = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    ///////////////

    total_time = get_realtime_ms() - start_time;

    CF_DEBUG("[F %d / %5d / %6d] OK  %g ms\n"
        "read    : %g ms\n"
        "roi     : %g ms\n"
        "weights : %g ms\n"
        "upscale : %g ms\n"
        "register: %g ms\n"
        "remap   : %g ms\n"
        "average : %g ms\n"
        "-----------\n\n\n",
        processed_frames_ + start_frame_index, processed_frames_, total_frames_, total_time,

        time_read,
        time_select_roi,
        time_compute_weights,
        time_upscale,
        time_register,
        time_remap,
        time_accumulate);

    emit_accumulator_changed();
  }


  output_aligned_video.close();

  if ( !fOk  ) {
    return false;
  }


  if( frame_accumulation_ ) {

    set_status_msg("FINISHING ...");

    if( !compute_accumulated_image(current_frame_, current_mask_) ) {
      CF_ERROR("FATAL: compute_accumulated_image() fails");
      return false;
    }

    if( anscombe_.method() != anscombe_none ) {
      anscombe_.inverse(current_frame_,
          current_frame_);
    }

    if( output_file_name.empty() ) {

      output_file_name =
          ssprintf("%s/%s-32F.tiff",
              output_directory.c_str(),
              options->name().c_str());
    }

    normalize_minmax(current_frame_, current_frame_,
        0.01, 0.99,
        current_mask_,
        true);

    CF_DEBUG("Saving '%s'", output_file_name.c_str());
    if( !write_image(output_file_name, output_options, current_frame_, current_mask_) ) {
      CF_ERROR("write_image('%s') fails", output_file_name.c_str());
    }

    const c_image_processor::ptr postprocessor = options->output_options().accumuated_image_postprocessor;

    CF_DEBUG("postprocessor: %s", postprocessor ?
        postprocessor->cname() : "NONE");

    if( postprocessor ) {

      std::string outname = output_file_name;
      set_file_suffix(outname, "-PP.tiff");

      if( !postprocessor->process(current_frame_, current_mask_) ) {
        CF_ERROR("postprocessor %s : process() fails", postprocessor->cname());
      }

      CF_DEBUG("Saving '%s'", outname.c_str());
      if( !write_image(outname, output_options, current_frame_, current_mask_) ) {
        CF_ERROR("ERROR: write_image('%s') fails", outname.c_str());
      }
    }

  }

  set_status_msg("FINISHED");

  return true;
}


bool c_image_stacking_pipeline::create_reference_frame(const c_image_stacking_options::ptr & options,
    const std::string & output_directory)
{

  const c_master_frame_options & master_frame_options =
      options->master_frame_options();

  int num_total_frames = 0;

  c_input_sequence::ptr input_sequence;
  c_auto_close_input_sequence auto_close(input_sequence);
  bool fOk = false;

  master_source_index_ = -1;
  master_frame_index_ = -1;
  master_file_name_.clear();



  if ( (master_file_name_ = master_frame_options.master_source_path).empty() ) {
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

  if ( (master_frame_index_ = master_frame_options.master_frame_index) < 0 ) {
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


  num_total_frames = input_sequence->size();
  if ( master_source_index_ >= 0 ) {
    master_frame_index_ = input_sequence->global_pos(master_source_index_, master_frame_index_);
  }
  else {
    master_frame_index_ = input_sequence->global_pos(0, master_frame_index_);
  }

  if ( !input_sequence->seek(master_frame_index_) ) {
    CF_ERROR("ERROR: input_sequence->seek(local pos=%d) fails",
        master_frame_index_);
    return false;
  }

  if ( master_frame_options.generate_master_frame && num_total_frames > 1 &&
      options->accumulation_options().accumulation_method != frame_accumulation_none ) {

    // Generate it !
    fOk = generate_reference_frame(input_sequence, options, output_directory);

  }

  else {
    // Read existing single frame from input sequence or external file
    if ( !read_input_frame(input_sequence, options->input_options(), reference_frame_, reference_mask_) ) {
      CF_FATAL("read_input_frame(reference_frame) fails");
      return false;
    }

    if ( !select_image_roi(roi_selection_, reference_frame_, reference_mask_, reference_frame_, reference_mask_) ) {
      CF_FATAL("select_image_roi(reference_frame) fails");
      return false;
    }

    fOk = true;
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


  if ( fOk && !canceled() ) {
    if ( options->output_options().dump_reference_data_for_debug ) {
      write_image(ssprintf("%s/%s-reference-frame.tiff", output_directory.c_str(),
          options->name().c_str()), options->output_options(), reference_frame_, reference_mask_);
    }
  }

  return fOk && !canceled();
}

bool c_image_stacking_pipeline::generate_reference_frame(const c_input_sequence::ptr & input_sequence,
    const c_image_stacking_options::ptr & options,
    const std::string & output_directory)
{
  const int input_sequence_size = input_sequence->size();
  bool fOk = false;

  cv::Mat masterflow;
  cv::Mat fftacc;
  float fftcnt = 0;

  if ( true )  {
    lock_guard lock(accumulator_lock_);
    frame_accumulation_.reset(new c_frame_accumulation_with_mask());
  }

  if ( options->frame_registration_options().registration_method != frame_registration_none ) {

    lock_guard lock(registration_lock_);

    if ( !(frame_registration_ = options->create_frame_registration()) ) {
      CF_FATAL("options->create_frame_registration() fails");
      set_status_msg("ERROR: frame_registration_->setup_referece_frame() fails");
      return false;
    }

    // Force some align options for reference frame generation

    if ( frame_registration_->enable_ecc() ) {

      frame_registration_->ecc().set_reference_smooth_sigma(
          frame_registration_->ecc().input_smooth_sigma());
    }


    frame_registration_->set_enable_eccflow(options->master_frame_options().allow_eccflow);
    if ( frame_registration_->enable_eccflow() && frame_registration_->eccflow().support_scale() < 5 ) {
      frame_registration_->eccflow().set_support_scale(5);
    }

    if ( options->master_frame_options().compensate_master_flow ) {
      //if ( frame_registration_->motion_type() > ECC_MOTION_EUCLIDEAN || frame_registration_->enable_eccflow() )
      {
        masterflow_accumulation_.reset(
            new c_frame_accumulation_with_mask());
      }
    }
  }


  fOk = true;
  processed_frames_ = 0;
  total_frames_ = std::min(options->master_frame_options().max_input_frames_to_generate_master_frame,
          input_sequence_size - master_frame_index_);

  emit_status_changed();

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

    if ( current_frame_.channels() > 1 ) {

      fOk = extract_channel(current_frame_, current_frame_, cv::noArray(), cv::noArray(),
          options->frame_registration_options().base_options.registration_channel);

      if ( !fOk ) {
        CF_ERROR("extract_channel(registration_channel=%d) fails",
            options->frame_registration_options().base_options.registration_channel);
        break;
      }

      if ( canceled() ) {
        break;
      }
    }

    if ( !select_image_roi(roi_selection_, current_frame_, current_mask_, current_frame_, current_mask_) ) {
      continue;
    }

    if ( canceled() ) {
      break;
    }

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


    if ( frame_registration_ ) {
      lock_guard lock(registration_lock_);

      if ( !frame_registration_->register_frame(current_frame_, current_frame_, current_mask_, current_mask_) ) {
        CF_WARNING("WARNING: frame_registration_->register_frame(i=%d) fails", processed_frames_);
        continue;
      }

      if ( masterflow_accumulation_ ) {

        const cv::Mat2f turbulence = compute_turbulence_flow(
            frame_registration_->motion_type(),
            frame_registration_->current_transform(),
            frame_registration_->current_remap(),
            current_mask_);

        if ( turbulence.empty() ) {
          CF_ERROR("compute_turbulence_flow() fails");
          set_status_msg("ERROR: compute_turbulence_flow() fails");
          break;
        }

        masterflow_accumulation_->add(turbulence, current_mask_);
      }

    }

    if ( canceled() ) {
      break;
    }

    if ( !(fOk = accumulate_fft_power_spectrum(current_frame_, fftacc, fftcnt)) ) {
      CF_ERROR("accumulate_fft_power_spectrum() fails");
      set_status_msg("ERROR: accumulate_fft_power_spectrum() fails");
      break;
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
    if ( !(fOk = (frame_accumulation_->accumulated_frames() > 0)) ) {
      set_status_msg("ERROR: No frames accumulated for reference frame");
    }
    else if ( !(fOk = frame_accumulation_->compute(reference_frame_, reference_mask_)) ) {
      set_status_msg("ERROR: frame_accumulation_->compute() fails");
    }
    else if ( masterflow_accumulation_ && masterflow_accumulation_->accumulated_frames() > 1 ) {
      masterflow_accumulation_->compute(masterflow);
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

  if ( fOk && !canceled() && reference_frame_.empty() ) {
    CF_ERROR("APP BUG IN frame_accumulation_->compute(): reference_frame_ is empty but status is OK");
    fOk =  false;
  }



  if ( fOk && !canceled() ) {

    if ( options->output_options().dump_reference_data_for_debug && !output_directory.empty() )  {
      write_image(ssprintf("%s/%s-initial-reference-frame.tiff", output_directory.c_str(),
              options->name().c_str()), options->output_options(), reference_frame_, reference_mask_);
    }

    cv::multiply(fftacc, 1. / fftcnt, fftacc);

    if ( !(fOk = swap_fft_power_spectrum(reference_frame_, fftacc, reference_frame_)) ) {
      CF_ERROR("ERROR: swap_power_spectrum() fails");
    }
    else {
      clip_range(reference_frame_, 0, 1, reference_mask_);
    }


    if ( options->master_frame_options().compensate_master_flow && !masterflow.empty() ) {

      if ( options->output_options().dump_reference_data_for_debug && !output_directory.empty() ) {

        save_image(masterflow_accumulation_->counter(),
            ssprintf("%s/%s-masterflow-counter.tiff", output_directory.c_str(),
                options->name().c_str()));

        save_image(masterflow,
            ssprintf("%s/%s-masterflow.flo", output_directory.c_str(),
                options->name().c_str()));
      }

      masterflow = flow2remap(masterflow,
            reference_mask_);

      // FIXME: BORDER !!!
      cv::remap(reference_frame_, reference_frame_, masterflow,
          cv::noArray(), cv::INTER_LANCZOS4, cv::BORDER_REPLICATE);

      cv::remap(reference_mask_, reference_mask_, masterflow,
          cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT);

      cv::compare(reference_mask_, 200, reference_mask_, cv::CMP_GE);

      masterflow.release();
    }

  }

  return  fOk && !canceled();
}



bool c_image_stacking_pipeline::read_input_frame(const c_input_sequence::ptr & input_sequence,
    const c_input_options & input_options,
    cv::Mat & output_image, cv::Mat & output_mask) const
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

    if ( input_options.remove_bad_pixels ) {
      remove_bad_pixels(output_image, input_options);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->pixel_depth())));

  }
  else {

    extract_bayer_planes(output_image, output_image,
        input_sequence->colorid());

    if ( input_options.remove_bad_pixels ) {
      remove_bad_pixels(output_image, input_options);
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

  if ( anscombe_.method() != anscombe_none ) {
    anscombe_.apply(output_image, output_image);
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




bool c_image_stacking_pipeline::write_image(const std::string & output_file_name,
    const c_image_stacking_output_options & output_options,
    const cv::Mat & output_image, const cv::Mat & output_mask)
{
  cv::Mat image_to_write;

  if ( !output_options.write_image_mask_as_alpha_channel || output_mask.empty() || (output_image.channels() != 3 && output_image.channels() != 1)  ) {
    image_to_write = output_image;
  }
  else {
    mergebgra(output_image, output_mask, image_to_write);
  }

  return save_image(image_to_write, output_file_name);
}


bool c_image_stacking_pipeline::save_processed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    const c_image_stacking_output_options & output_options,
    const std::string & output_directory,
    const std::string & sequence_name,
    int frame_index)
{

  const std::string output_file_name = ssprintf("%s/%s.%06d.tiff",
      output_directory.c_str(),
      sequence_name.c_str(),
      frame_index);

  return write_image(output_file_name, output_options,
      current_frame, current_mask);
}


void c_image_stacking_pipeline::remove_bad_pixels(cv::Mat & image,
    const c_input_options & input_optons)
{
  cv::Mat medianImage, variationImage, meanVariationImage;
  double minimal_mean_variation_for_very_smooth_images;

  // treshold = estimate_noise(image);
  if ( image.depth() == CV_32F || image.depth() == CV_64F ) {
    minimal_mean_variation_for_very_smooth_images = 1e-2;
  }
  else {
    minimal_mean_variation_for_very_smooth_images = 2;
  }

  cv::medianBlur(image, medianImage, 3);
  cv::absdiff(image, medianImage, variationImage);

  static float K[3*3] = {
      1./8, 1./8, 1./8,
      1./8, 0.0,  1./8,
      1./8, 1./8, 1./8,
  };
  cv::filter2D(variationImage, meanVariationImage, -1, cv::Mat1f(3, 3, K));
  cv::max(meanVariationImage, minimal_mean_variation_for_very_smooth_images, meanVariationImage);

  medianImage.copyTo(image, variationImage > input_optons.bad_pixels_variation_threshold * meanVariationImage);
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

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst)
{
  compute_smap(src, dst, 0.01, 0);

//  cv::Mat w;
//  compute_smap(src, w, 0.01, 0);
//  if ( !srcmask.empty() ) {
//    w.setTo(0, ~srcmask);
//  }
//  dst = std::move(w);
}

void c_image_stacking_pipeline::compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel)
{
  cv::divide(wc, wref, wrel);
  if ( mc.size() == wrel.size() ) {
    wrel.setTo(0, ~mc);
  }

//  int n;
//
//  if ( (n = countNaNs(wrel)) > 0 ) {
//    CF_DEBUG(" ******* NANS DETECTED : n= %d", n);
//    exit(1);
//  }
//  if ( (n = countInfs(wrel)) > 0 ) {
//    CF_DEBUG(" ******* INFS DETECTED : n= %d", n);
//    exit(1);
//  }


//  if ( mc.size() == wc.size() ) {
//    wrel.setTo(0, ~mc);
//  }
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


