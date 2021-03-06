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
#include <core/proc/inpaint.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/fft.h>
#include <core/proc/smap.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/ssprintf.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<>
const c_enum_member * members_of<roi_selection_method>()
{
  static constexpr c_enum_member members[] = {
      { roi_selection_none, "none", },
      { roi_selection_planetary_disk, "planetary_disk", },
      { roi_selection_rectange_crop, "rectangle", },
      { roi_selection_none, nullptr, },
  };
  return members;
}

template<>
const c_enum_member * members_of<frame_accumulation_method>()
{
  static constexpr c_enum_member members[] = {
      { frame_accumulation_weighted_average, "weighted_average", },
      { frame_accumulation_fft, "fft", },
      { frame_accumulation_none, "None", },
      { frame_accumulation_none, nullptr, },
  };
  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_stage>()
{
  static constexpr c_enum_member members[] = {
      { frame_upscale_after_align , "after_align", },
      { frame_upscale_before_align , "before_align", },
      { frame_upscale_stage_unknown , nullptr, },
  };
  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_option>()
{
  static constexpr c_enum_member members[] = {
      {frame_upscale_none, "none", },
      {frame_upscale_pyrUp, "x2.0", },
      {frame_upscale_x15, "x1.5", },
      {frame_upscale_x30, "x3.0", },
      {frame_upscale_none, nullptr, },
  };
  return members;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

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
              rmap[y][x][0] = x - uv[y][x][0];
              rmap[y][x][1] = y - uv[y][x][1];
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

static cv::Mat2f compute_turbulent_flow(
    ECC_MOTION_TYPE current_motion_type,
    const cv::Mat1f & current_transfrorm,
    const cv::Mat2f & current_remap
    /*,const cv::Mat1b & mask*/)
{

  cv::Mat2f uv;
  double tx = 0, ty = 0;

  // TODO: Extract also rotation component, because telescope mount sometimes could drift with FOV rotation as well.

  if ( !getTranslationComponents(current_motion_type, current_transfrorm, &tx, &ty) ) {
    CF_ERROR("getTranslationComponent(current_motion_type=%d) fails", current_motion_type);
    return uv;
  }

  uv.create(current_remap.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, current_remap.rows, 256),
      [&current_remap, &uv/*, &mask*/, tx, ty](const range &r) {
        const int nx = current_remap.cols;
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < nx; ++x ) {

//            if ( mask[y][x] ) {
//              uv[y][x][0] = current_remap[y][x][0] - x - tx;
//              uv[y][x][1] = current_remap[y][x][1] - y - ty;
//            }
//            else {
//              uv[y][x][0] = 0;
//              uv[y][x][1] = 0;
//            }

        if ( current_remap[y][x][0] >= 0 && current_remap[y][x][1] >= 0 ) {
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


// acc = (acc * n + cur ) / (n + 1);

//template<class T>
//static inline void average_add_(cv::Mat & acc, cv::Mat1f & cnt, const cv::Mat & cf, const cv::Mat1b & cm)
//{
//  const int cn = acc.channels();
//
//  if( cm.empty() ) {
//    for( int y = 0; y < acc.rows; ++y ) {
//      T *accp = acc.ptr<T>(y);
//      const T *srcp = cf.ptr<const T>(y);
//
//      for( int x = 0; x < acc.cols; ++x ) {
//        for( int c = 0; c < cn; ++c ) {
//          accp[x * cn + c] = (accp[x * cn + c] * cnt[y][x] + srcp[x * cn + c]) / (cnt[y][x] + 1);
//          ++cnt[y][x];
//        }
//      }
//    }
//  }
//  else {
//
//    for( int y = 0; y < acc.rows; ++y ) {
//      T *accp = acc.ptr<T>(y);
//      const T *srcp = cf.ptr<const T>(y);
//
//      for( int x = 0; x < acc.cols; ++x ) {
//        if( cm[y][x] ) {
//          for( int c = 0; c < cn; ++c ) {
//            accp[x * cn + c] = (accp[x * cn + c] * cnt[y][x] + srcp[x * cn + c]) / (cnt[y][x] + 1);
//            ++cnt[y][x];
//          }
//        }
//      }
//    }
//  }
//}
//
//static inline void average_add(cv::Mat & acc, cv::Mat1f & cntr, const cv::Mat & cf, const cv::Mat1b & cm)
//{
//  switch (acc.depth()) {
//    case CV_8U:
//      average_add_<uint8_t>(acc, cntr, cf, cm);
//      break;
//    case CV_8S:
//      average_add_<int8_t>(acc, cntr, cf, cm);
//      break;
//    case CV_16U:
//      average_add_<uint16_t>(acc, cntr, cf, cm);
//      break;
//    case CV_16S:
//      average_add_<int16_t>(acc, cntr, cf, cm);
//      break;
//    case CV_32S:
//      average_add_<int32_t>(acc, cntr, cf, cm);
//      break;
//    case CV_32F:
//      average_add_<float>(acc, cntr, cf, cm);
//      break;
//    case CV_64F:
//      average_add_<double>(acc, cntr, cf, cm);
//      break;
//  }
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

const char * c_image_stacking_options::cname() const
{
  return this->name_.c_str();
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

void c_image_stacking_options::remove_input_source(const c_input_source::ptr & source)
{
  if ( input_sequence_  ) {
    input_sequence_->remove_source(source);
  }
}

c_input_options & c_image_stacking_options::input_options()
{
  return input_options_;
}

const c_input_options & c_image_stacking_options::input_options() const
{
  return input_options_;
}

c_frame_upscale_options & c_image_stacking_options::upscale_options()
{
  return upscale_options_;
}

const c_frame_upscale_options & c_image_stacking_options::upscale_options() const
{
  return upscale_options_;
}

c_sparse_feature_extractor_options & c_image_stacking_options::sparse_feature_extractor_options()
{
  return frame_registration_options_.image_registration_options.feature_registration.sparse_feature_extractor;
}

const c_sparse_feature_extractor_options & c_image_stacking_options::sparse_feature_extractor_options() const
{
  return frame_registration_options_.image_registration_options.feature_registration.sparse_feature_extractor;
}

c_sparse_feature_detector_options & c_image_stacking_options::sparse_feature_detector_options()
{
  return sparse_feature_extractor_options().detector;
}

const c_sparse_feature_detector_options & c_image_stacking_options::sparse_feature_detector_options() const
{
  return sparse_feature_extractor_options().detector;
}

c_sparse_feature_descriptor_options & c_image_stacking_options::sparse_feature_descriptor_options()
{
  return sparse_feature_extractor_options().descriptor;
}

const c_sparse_feature_descriptor_options & c_image_stacking_options::sparse_feature_descriptor_options() const
{
  return sparse_feature_extractor_options().descriptor;
}

c_master_frame_options & c_image_stacking_options::master_frame_options()
{
  return frame_registration_options_.master_frame_options;
}

c_roi_selection_options & c_image_stacking_options::roi_selection_options()
{
  return roi_selection_options_;
}

const c_roi_selection_options & c_image_stacking_options::roi_selection_options() const
{
  return roi_selection_options_;
}

c_roi_selection::ptr c_image_stacking_options::create_roi_selection() const
{
  switch ( roi_selection_options_.method ) {
  case roi_selection_planetary_disk :
    return c_planetary_disk_selection::create(roi_selection_options_.planetary_disk_crop_size);

  case roi_selection_rectange_crop :
    return c_roi_rectangle_selection::create(roi_selection_options_.rectangle_roi_selection);

  default :
    break;
  }
  return nullptr;
}


const c_master_frame_options & c_image_stacking_options::master_frame_options() const
{
  return frame_registration_options_.master_frame_options;
}

c_frame_registration_options & c_image_stacking_options::frame_registration_options()
{
  return frame_registration_options_;
}

const c_frame_registration_options & c_image_stacking_options::frame_registration_options() const
{
  return frame_registration_options_;
}

c_frame_registration::sptr c_image_stacking_options::create_frame_registration(const c_image_registration_options & options) const
{
  return c_frame_registration::sptr(new c_frame_registration(options));
}

c_frame_registration::sptr c_image_stacking_options::create_frame_registration() const
{
  return create_frame_registration(frame_registration_options_.image_registration_options);
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
  case frame_accumulation_weighted_average:
    return c_frame_accumulation::ptr(new c_frame_weigthed_average());
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

c_image_processing_options & c_image_stacking_options::image_processing_options()
{
  return image_processing_options_;
}

const c_image_processing_options & c_image_stacking_options::image_processing_options() const
{
  return image_processing_options_;
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

void c_image_stacks_collection::add(const c_image_stacking_options::ptr & stack)
{
  stacks_.emplace_back(stack);
}

bool c_image_stacks_collection::remove(const c_image_stacking_options::ptr & stack)
{
  const size_t original_size = stacks_.size();
  if ( original_size > 0 ) {
    stacks_.erase(std::remove(stacks_.begin(), stacks_.end(), stack), stacks_.end());
  }
  return stacks_.size() < original_size;
}

void c_image_stacks_collection::set(int pos, const c_image_stacking_options::ptr & stack)
{
  if ( pos < 0 || pos >= (int)(stacks_.size()) ) {
    stacks_.emplace_back(stack);
  }
  else {
    stacks_[pos] = stack;
  }
}

ssize_t c_image_stacks_collection::indexof(const c_image_stacking_options::ptr & stack) const
{
  std::vector<c_image_stacking_options::ptr>::const_iterator ii =
      std::find(stacks_.begin(), stacks_.end(), stack);
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


class c_image_stacking_pipeline::c_video_writer {

public:

  ~c_video_writer()
  {
    close();
  }

  bool open(const std::string & filename, const cv::Size & frameSize, bool color)
  {
    output_file_name = filename;
    output_type = output_type_unknown;
    current_frame_index = 0;

    switch ( c_input_source::suggest_source_type(filename) ) {
    case c_input_source::SER : {

      serVideo.create(filename, frameSize.width, frameSize.height,
          color ? COLORID_BGR : COLORID_MONO,
          16);

      if ( !serVideo.is_open() ) {
        CF_ERROR("Can not write ser file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_ser;
      break;
    }

    case c_input_source::MOVIE : {

      aviVideo.open(filename, cv::CAP_FFMPEG,
          cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'),
          10,
          frameSize,
          color);

      if ( !aviVideo.isOpened() ) {
        CF_ERROR("Can not write aligned video file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_video;
      break;
    }

    case c_input_source::REGULAR_IMAGE : {

      output_type = output_type_images;
      break;
    }

    default : {
      CF_ERROR("NOOT SUPPORTED output format requested for file '%s'", filename.c_str());
      return false;
    }
    }

    return true;
  }


  bool isOpened() const
  {
    switch (output_type) {
      case output_type_images:
        return !output_file_name.empty();
      case output_type_ser:
        return serVideo.is_open();
      case output_type_video:
        return aviVideo.isOpened();
    }
    return false;
  }

  bool write(cv::InputArray currenFrame, cv::InputArray currentMask, bool with_alpha_mask = true)
  {
    switch ( output_type ) {
    case output_type_video :
      if ( aviVideo.isOpened() ) {
        currenFrame.getMat().convertTo(tmp, CV_8U, 255);
        aviVideo.write(tmp);
      }
      break;

    case output_type_ser :
      if ( serVideo.is_open() ) {
        currenFrame.getMat().convertTo(tmp, CV_16U, 65535);
        serVideo.write(tmp);
      }
      break;

    case output_type_images : {

      std::string fname =
          output_file_name;

      const std::string suffix =
          get_file_suffix(fname);

      set_file_suffix(fname, ssprintf("-%06d%s",
          current_frame_index,
          suffix.c_str()));

      if ( with_alpha_mask ) {
        if ( !save_image(currenFrame, currentMask, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }
      else {
        if ( !save_image(currenFrame, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }

      break;
    }

    default :
      CF_ERROR("ERROR: Output video file is not open");
      return false;
    }

    ++current_frame_index;
    return true;
  }

  void close()
  {
    aviVideo.release();
    serVideo.close();
    tmp.release();
    output_type = output_type_unknown;
  }

protected:
  cv::VideoWriter aviVideo;
  c_ser_writer serVideo;
  cv::Mat tmp;

  enum {
    output_type_unknown,
    output_type_video,
    output_type_ser,
    output_type_images,
  } output_type = output_type_unknown;

  std::string output_file_name;
  int current_frame_index = 0;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const c_image_stacking_options::ptr & c_image_stacking_pipeline::options() const
{
  return options_;
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

void c_image_stacking_pipeline::gather_badframe_indexes()
{
  badframes_.clear();

  if ( input_sequence_ ) {

    const bool was_open = input_sequence_->is_open();
    if( !was_open && !input_sequence_->open() ) {
      CF_ERROR("input_sequence_->open() fails");
      return;
    }

    const std::vector<c_input_source::ptr> & sources =
        input_sequence_->sources();

    for ( uint source_index  = 0, n = sources.size();  source_index < n; ++source_index ) {

      const c_input_source::ptr source =
          input_sequence_->source(source_index);

      if( source ) {

        const std::vector<uint> &bad_source_frames =
            source->load_badframes();

        for( uint source_frame_index : bad_source_frames ) {

          const int global_index =
              input_sequence_->global_pos(source_index,
                  source_frame_index);

          if ( global_index >= 0 ) {
            badframes_.emplace_back(global_index);
          }
        }
      }
    }

    if( !was_open ) {
      input_sequence_->close(false);
    }
  }
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


bool c_image_stacking_pipeline::run(const c_image_stacking_options::ptr & options )
{
  bool fOk = false;

  try {

    if ( !(fOk = initialize(options)) ) {
      CF_ERROR("initialize() fails");
    }
    else if ( !(fOk = actual_run()) ) {
      CF_ERROR("actual_run() fails");
    }

  }
  catch (const cv::Exception & e) {

    fOk = false;

    CF_ERROR("OpenCV Exception catched in c_image_stacking_pipeline::run():\n"
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

    CF_ERROR("std::exception catched in c_image_stacking_pipeline::run(): %s\n",
        e.what());
  }
  catch (...) {

    fOk = false;

    CF_ERROR("Unknown exception catched in c_image_stacking_pipeline::run()\n");
  }




  try {
    cleanup();
  }
  catch (const cv::Exception & e) {

    fOk = false;

    CF_ERROR("OpenCV Exception catched in c_image_stacking_pipeline::cleanup():\n"
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

    CF_ERROR("std::exception catched in c_image_stacking_pipeline::cleanup(): %s\n",
        e.what());
  }
  catch (...) {

    fOk = false;

    CF_ERROR("Unknown exception catched in c_image_stacking_pipeline::cleanup()\n");
  }

  return fOk;
}

bool c_image_stacking_pipeline::initialize(const c_image_stacking_options::ptr & options)
{

  CF_DEBUG("Initializing '%s'...",
      options->name().c_str());


  set_canceled(false);


  if (true ) {

    lock_guard lock(accumulator_lock_);

    output_directory_.clear();
    missing_pixel_mask_.release();
    reference_weights_.release();

    external_master_frame_ = false;
    master_frame_index_ = -1;
    ecc_normalization_noise_ = 0;

    total_frames_ = 0;
    processed_frames_ = 0;
    //reference_sharpness_ = 0;

    statusmsg_.clear();

    roi_selection_.reset();
    frame_registration_.reset();
    frame_accumulation_.reset();
    flow_accumulation_.reset();

    input_sequence_.reset();
  }

  /////////////////////////////////////////////////////////////////////////////

  if ( !(this->options_ = options) ) {
    CF_ERROR("No stacking options specified, can not initialize pipeline");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  if ( !(input_sequence_ = options_->input_sequence()) || input_sequence_->empty() ) {
    set_status_msg("ERROR: empty input sequence specified");
    return false;
  }

  gather_badframe_indexes();

  /////////////////////////////////////////////////////////////////////////////
  if ( (output_directory_ = options_->output_options().output_directory).empty() ) {

    std::string parent_directory =
        get_parent_directory(input_sequence_->source(0)->filename());

    if ( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_directory_ = ssprintf("%s/stacks",
        parent_directory.c_str());

  }
  else if ( !is_absolute_path(output_directory_) ) {

    std::string parent_directory =
        get_parent_directory(input_sequence_->source(0)->filename());

    if ( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_directory_ = ssprintf("%s/%s",
        parent_directory.c_str(),
        output_directory_.c_str());
  }

  if ( output_directory_.empty() ) {
    output_directory_ = "./stacks";
  }

  /////////////////////////////////////////////////////////////////////////////

  anscombe_.set_method(options_->input_options().anscombe);

  /////////////////////////////////////////////////////////////////////////////

  if ( options_->roi_selection_options().method != roi_selection_none ) {
    if ( !(roi_selection_ = options_->create_roi_selection()) ) {
      set_status_msg("ERROR: create_roi_selection() fails");
      return false;
    }
  }

  if ( !options_->input_options().missing_pixel_mask_filename.empty() ) {

    if ( !load_image(options_->input_options().missing_pixel_mask_filename, missing_pixel_mask_) ) {
      CF_ERROR("load_image('%s') fails.", options_->input_options().missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( missing_pixel_mask_.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          options_->input_options().missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !options_->input_options().missing_pixels_marked_black ) {
      cv::invert(missing_pixel_mask_, missing_pixel_mask_);
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  return true;
}


void c_image_stacking_pipeline::cleanup()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
    input_sequence_.reset();
  }

  roi_selection_.reset();

  if ( true ) {
    lock_guard lock(registration_lock_);
    frame_registration_.reset();
    flow_accumulation_.reset();
  }

  if ( true ) {
    lock_guard lock(accumulator_lock_);
    frame_accumulation_.reset();
    sharpness_norm_accumulation_.reset();
  }

  missing_pixel_mask_.release();
}


bool c_image_stacking_pipeline::actual_run()
{
  std::string output_file_name;
  cv::Mat2f upscaled_remap;
  cv::Mat tmp;
  bool fOk;


  CF_DEBUG("Starting '%s' ...",
      options_->cname());


  const c_input_options & input_options =
      options_->input_options();

  const c_frame_upscale_options & upscale_options =
      options_->upscale_options();

  const c_frame_accumulation_options & accumulation_options =
      options_->accumulation_options();

  const c_frame_registration_options & registration_options =
      options_->frame_registration_options();

  const c_image_processing_options & image_processing_options =
      options_->image_processing_options();

  const c_image_stacking_output_options & output_options =
      options_->output_options();


  options_->save(ssprintf("%s/%s.cfg",
      output_directory_.c_str(),
      options_->cname()));


  /////////////////////////////////////////////////////////////////////////////
  // SETUP FRAME REGISTRATION

  if ( registration_options.image_registration_options.enable_frame_registration ) {

    set_status_msg("CREATE REFERENCE FRAME ...");

    const c_master_frame_options & master_options =
        options_->master_frame_options();

    c_input_sequence::ptr input_sequence;
    int master_source_index = -1;
    int master_frame_index = -1;
    int max_frames_to_stack = 0;

    cv::Mat reference_frame;
    cv::Mat reference_mask;

    std::string master_file_name =
        master_options.master_source_path;

    CF_DEBUG("master_options.master_file_name=%s", master_file_name.c_str());

    if ( master_file_name.empty() ) {
      master_file_name =
          input_sequence_->
              source(master_source_index = 0)->filename();
    }
    else {

      std::vector<c_input_source::ptr>::const_iterator source_pos =
          std::find_if(options_->input_sequence()->sources().begin(), options_->input_sequence()->sources().end(),
              [master_options](const c_input_source::ptr & s ) -> bool {
                return s->filename() == master_options.master_source_path;
              });

      if ( source_pos != this->input_sequence_->sources().end() ) {
        master_source_index = source_pos - this->input_sequence_->sources().begin();
      }
    }

    if ( master_source_index >= 0 ) {
      input_sequence = this->input_sequence_;
      external_master_frame_ = false;
    }
    else if ( !(input_sequence = c_input_sequence::create(master_file_name)) ) {
      CF_ERROR("ERROR: c_input_sequence::create(master_file_name_=%s) fails", master_file_name.c_str());
      return false;
    }
    else {
      external_master_frame_ = true;
      master_source_index = 0;
    }

    if ( master_source_index >= (int) input_sequence->sources().size() ) {
      CF_FATAL("ERROR: master_source_index=%d exceeds input_sequence->sources().size()=%zu",
          master_source_index, input_sequence->sources().size());
      return false;
    }

    if ( !input_sequence->source(master_source_index)->enabled() ) {
      CF_FATAL("ERROR: master_source_index=%d is NOT enabled in input_sequence",
          master_source_index);
      return false;
    }

    if ( !input_sequence->open() ) {
      CF_FATAL("ERROR: Can not open master input source '%s'",
          master_file_name.c_str());
      return false;
    }

    if ( (master_frame_index = master_options.master_frame_index) < 0 ) {
      master_frame_index = 0;
    }
    else if ( master_frame_index >= input_sequence->source(master_source_index)->size() ) {
      CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
          master_frame_index, master_file_name.c_str());
      return false;
    }

    master_frame_index =
        input_sequence->global_pos(
            master_source_index,
            master_frame_index);

    if ( master_options.generate_master_frame ) {
      max_frames_to_stack =
          master_options.max_input_frames_to_generate_master_frame;
    }

    if ( !create_reference_frame(input_sequence, master_frame_index, max_frames_to_stack,
        reference_frame, reference_mask) ) {
      CF_FATAL("ERROR: create_reference_frame() fails");
      return false;
    }

    ecc_normalization_noise_ =
        compute_image_noise(reference_frame, reference_mask,
            options_->frame_registration_options().image_registration_options.registration_channel);

    if ( master_options.save_master_frame ) {
      write_image(ssprintf("%s/%s-master.tiff", output_directory_.c_str(), options_->cname()),
          output_options,
          reference_frame,
          reference_mask);
    }

    CF_DEBUG("Reference frame : %dx%d depth=%d channels=%d",
        reference_frame.cols, reference_frame.rows,
        reference_frame.depth(),
        reference_frame.channels());

    CF_DEBUG("Reference mask : %dx%d depth=%d channels=%d",
        reference_mask.cols, reference_mask.rows,
        reference_mask.depth(),
        reference_mask.channels());


    if ( weights_required() ) {
      compute_weights(reference_frame, reference_mask, reference_weights_);
    }

    if ( !(frame_registration_ = options_->create_frame_registration()) ) {
      CF_FATAL("options_->create_frame_registration() fails");
      return false;
    }

    if( image_processing_options.ecc_image_processor ) {

      const c_image_processor::ptr ecc_image_processor =
          image_processing_options.ecc_image_processor;

      frame_registration_->set_ecc_image_preprocessor(
          [ecc_image_processor](cv::InputOutputArray image, cv::InputOutputArray mask) {
            ecc_image_processor->process(image, mask);
          });
    }

    if( upscale_required(frame_upscale_before_align) ) {

      upscale_image(options_->upscale_options().upscale_option,
          reference_frame, reference_mask,
          reference_frame, reference_mask);

      if( !reference_weights_.empty() ) {
        upscale_image(options_->upscale_options().upscale_option,
            reference_weights_, cv::noArray(),
            reference_weights_, cv::noArray());

        CF_DEBUG("Reference weights: %dx%d depth=%d channels=%d",
            reference_weights_.cols, reference_weights_.rows,
            reference_weights_.depth(),
            reference_weights_.channels());
      }
    }

    if ( output_options.debug_frame_registration ) {
      frame_registration_->set_enable_debug(true);
      frame_registration_->set_debug_path(ssprintf("%s/debug-registration-reference",
          output_directory_.c_str()));
    }

    if ( !frame_registration_->setup_referece_frame(reference_frame, reference_mask) ) {
      CF_ERROR("ERROR: frame_registration_->setup_referece_frame() fails");
      return false;
    }


    if( upscale_required(frame_upscale_after_align) ) {

      upscale_image(options_->upscale_options().upscale_option,
          reference_weights_, cv::noArray(),
          reference_weights_, cv::noArray());

      CF_DEBUG("Reference weights: %dx%d depth=%d channels=%d",
          reference_weights_.cols, reference_weights_.rows,
          reference_weights_.depth(),
          reference_weights_.channels());
    }

    if ( output_options.debug_frame_registration && frame_registration_->options().eccflow.enabled  ) {

      const c_ecch_flow & eccflow =
          frame_registration_->eccflow();

      const std::vector<c_ecch_flow::pyramid_entry> & pyramid =
          eccflow.current_pyramid();

      const std::string debugpath =
          ssprintf("%s/eccdebug", output_directory_.c_str());

      for ( uint i = 0, n = pyramid.size(); i < n; ++i ) {
        save_image(pyramid[i].reference_image, ssprintf("%s/eccref.%02u.tiff", debugpath.c_str(), i));
      }
    }


    if ( output_options.debug_frame_registration ) {
      frame_registration_->set_enable_debug(false);
    }


    if( registration_options.accumulate_and_compensate_turbulent_flow ) {
      if( registration_options.image_registration_options.motion_type > ECC_MOTION_EUCLIDEAN ||
          registration_options.image_registration_options.eccflow.enabled ) {
        flow_accumulation_.reset(new c_frame_weigthed_average());
      }
    }
  }


  /////////////////////////////////////////////////////////////////////////////

  if ( accumulation_options.accumulation_method != frame_accumulation_none ) {

    lock_guard lock(accumulator_lock_);

    if ( !(frame_accumulation_ = options_->create_frame_accumulation()) ) {
      CF_ERROR("ERROR: create_frame_accumulation() fails");
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  if ( !input_sequence_->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }

  CF_DEBUG("input_sequence->size()=%d", input_sequence_->size());

  set_status_msg("RUNNING ...");



  const int start_pos =
      std::max(input_options.start_frame_index, 0);

  const int end_pos =
      input_options.max_input_frames < 1 ?
          input_sequence_->size() :
          std::min(input_sequence_->size(),
              input_options.start_frame_index + input_options.max_input_frames);

  if ( !(fOk = process_input_sequence(input_sequence_, start_pos, end_pos)) ) {
    CF_ERROR("process_input_sequence() fails");
    return false;
  }

  set_status_msg("FINISHING ...");


  // Read accumulators back

  if ( frame_accumulation_ ) {

    cv::Mat accumulated_image;
    cv::Mat1b accumulated_mask;

    if ( true ) {
      lock_guard lock(accumulator_lock_);

      if ( !frame_accumulation_->compute(accumulated_image, accumulated_mask) ) {
        CF_ERROR("ERROR: frame_accumulation_->compute() fails");
        return false;
      }

      average_pyramid_inpaint(accumulated_image,
          accumulated_mask,
          accumulated_image);
    }


    if ( anscombe_.method() != anscombe_none ) {
      anscombe_.inverse(accumulated_image,
          accumulated_image);
    }





    // Fix turbulent flow
    if ( flow_accumulation_ ) {

      cv::Mat accumulated_flow;
      double sharpenScale = 0;

      if ( !flow_accumulation_->compute(accumulated_flow) ) {
        CF_ERROR("ERROR: flow_accumulation_->compute() fails");
      }
      else {

        if ( options_->master_frame_options().accumulated_sharpen_factor > 0 ) {
          if ( frame_accumulation_->accumulated_frames() > 1 ) {

            if ( output_options.dump_reference_data_for_debug ) {
              write_image(ssprintf("%s/%s-before-sharpen.tiff",
                  output_directory_.c_str(), options_->cname()),
                  output_options,
                  accumulated_image,
                  accumulated_mask);
            }

            sharpenScale =
                options_->master_frame_options().accumulated_sharpen_factor
                    * sqrt(frame_accumulation_->accumulated_frames());

            CF_DEBUG("sharpenScale=%g accumulated_frames=%d",
                sharpenScale,
                frame_accumulation_->accumulated_frames());

            fftSharpenR1(accumulated_image, accumulated_image,
                sharpenScale, false);

          }
        }

        if ( accumulated_flow.size() != accumulated_image.size() ) {
          upscale_optflow(upscale_options.upscale_option,
              accumulated_flow,
              accumulated_flow);
        }

        if ( output_options.dump_reference_data_for_debug ) {

          save_image(flow_accumulation_->counter(),
              ssprintf("%s/%s-masterflow-counter.tiff", output_directory_.c_str(),
                  options_->cname()));

          save_image(accumulated_flow,
              ssprintf("%s/%s-masterflow.flo", output_directory_.c_str(),
                  options_->cname()));

          write_image(ssprintf("%s/%s-before-flow_compensation.tiff",
              output_directory_.c_str(), options_->cname()),
              output_options,
              accumulated_image,
              accumulated_mask);
        }

        accumulated_flow =
            flow2remap(accumulated_flow,
                accumulated_mask);

        // FIXME: BORDER !!!
        CF_DEBUG("H: accumulated_image=%dx%d", accumulated_image.cols, accumulated_image.rows);
        CF_DEBUG("H: accumulated_flow=%dx%d", accumulated_flow.cols, accumulated_flow.rows);
        cv::remap(accumulated_image, accumulated_image, accumulated_flow,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

        cv::remap(accumulated_mask, accumulated_mask, accumulated_flow,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        cv::compare(accumulated_mask, 255, accumulated_mask, cv::CMP_GE);

        if ( sharpenScale > 0 ) {

          fftSharpenR1(accumulated_image, accumulated_image,
              -sharpenScale, false);

        }
      }
    }


    if ( output_file_name.empty() ) {

      output_file_name =
          ssprintf("%s/%s-32F.tiff",
              output_directory_.c_str(),
              options_->cname());
    }




    CF_DEBUG("Saving '%s'", output_file_name.c_str());
    if ( !write_image(output_file_name, output_options, accumulated_image, accumulated_mask) ) {
      CF_ERROR("write_image('%s') fails", output_file_name.c_str());
    }


    if ( image_processing_options.accumulated_image_processor ) {

      if ( !image_processing_options.accumulated_image_processor->process(accumulated_image, accumulated_mask) ) {
        CF_ERROR("postprocessor %s : process() fails", image_processing_options.accumulated_image_processor->cname());
      }
      else {

        output_file_name =
            ssprintf("%s/%s-32F-PP.tiff",
                output_directory_.c_str(),
                options_->cname());

        CF_DEBUG("Saving '%s'", output_file_name.c_str());
        if ( !write_image(output_file_name, output_options, accumulated_image, accumulated_mask) ) {
          CF_ERROR("write_image('%s') fails", output_file_name.c_str());
        }
      }
    }

    if ( registration_options.image_registration_options.jovian_derotation.enabled ) {
      if ( registration_options.image_registration_options.jovian_derotation.rotate_jovian_disk_horizontally ) {

        const c_jovian_derotation & jovian_derotation =
            frame_registration_->jovian_derotation();

        const cv::RotatedRect &E =
            jovian_derotation.reference_ellipse();

        const cv::Rect &BB =
            jovian_derotation.reference_boundig_box();

        const cv::Mat T =
            createEuclideanTransform(E.center.x + BB.x, E.center.y + BB.y,
                //accumulated_image.cols / 2, accumulated_image.rows / 2,
                E.center.x + BB.x, E.center.y + BB.y,
                1.0,
                -E.angle * CV_PI / 180,
                CV_32F);

        const cv::Mat M =
            createRemap(ECC_MOTION_AFFINE,
                T,
                accumulated_image.size(),
                CV_32F);

          cv::remap(accumulated_image, accumulated_image,
              M, cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_REPLICATE);

          cv::remap(accumulated_mask, accumulated_mask,
              M, cv::noArray(),
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);

        cv::compare(accumulated_mask, 255,
            accumulated_mask,
            cv::CMP_GE);

        output_file_name =
            ssprintf("%s/%s-32F-PPR.tiff",
                output_directory_.c_str(),
                options_->cname());

        CF_DEBUG("Saving '%s'", output_file_name.c_str());
        if( !write_image(output_file_name, output_options, accumulated_image, accumulated_mask) ) {
          CF_ERROR("write_image('%s') fails", output_file_name.c_str());
        }
      }
    }
  }




  //////////////////

  set_status_msg("FINISHED");

  return true;
}

bool c_image_stacking_pipeline::create_reference_frame(const c_input_sequence::ptr & input_sequence,
    int master_frame_index, int max_frames_to_stack,
    cv::Mat & reference_frame,
    cv::Mat & reference_mask)
{
  const c_input_options & input_options =
      options_->input_options();

  const c_master_frame_options & master_options =
      options_->master_frame_options();

  const c_image_processing_options & image_processing_options =
      options_->image_processing_options();

  master_frame_generation_ = true;

  if ( !input_sequence->seek(master_frame_index) ) {
    CF_ERROR("ERROR: input_sequence->seek(master_frame_index=%d) fails", master_frame_index);
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if ( !read_input_frame(input_sequence, input_options, reference_frame, reference_mask) ) {
    CF_FATAL("read_input_frame(reference_frame) fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }


  if ( !select_image_roi(roi_selection_, reference_frame, reference_mask, reference_frame, reference_mask) ) {
    CF_FATAL("select_image_roi(reference_frame) fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if( master_options.apply_input_frame_processor && image_processing_options.input_image_processor ) {
    if( !image_processing_options.input_image_processor->process(reference_frame, reference_mask) ) {
      CF_ERROR("input_image_processor->process(reference_frame) fails");
      return false;
    }
  }

  if ( canceled() ) {
    return false;
  }

  if ( reference_frame.channels() > 1 ) {

    const color_channel_type master_channel =
        options_->frame_registration_options().
            image_registration_options.registration_channel;

    if ( !extract_channel(reference_frame, reference_frame, cv::noArray(), cv::noArray(), master_channel) ) {
      CF_ERROR("extract_channel(master_channel=%d) fails",master_channel);
      return false;
    }

    if ( canceled() ) {
      return false;
    }
  }

  // setup master index indicator
 this->master_frame_index_ = master_frame_index;

 if ( max_frames_to_stack < 2 || input_sequence->size() < 2 ) {

    // Use single frame as reference

    reference_sharpness_ =
        c_sharpness_norm_measure().measure(
            reference_frame,
            reference_mask);

  }
  else {

    // Generate from sequence
    c_image_registration_options master_registration_options =
        options_->frame_registration_options().image_registration_options;

    if ( master_options.eccflow_scale > 1 ) {
      master_registration_options.eccflow.enabled = true;
      master_registration_options.eccflow.support_scale = master_options.eccflow_scale;
    }

    if ( !(frame_registration_ = options_->create_frame_registration(master_registration_options)) ) {
      CF_FATAL("options_->create_frame_registration(master_registration_options) fails");
      return false;
    }

    if( image_processing_options.ecc_image_processor ) {

      const c_image_processor::ptr ecc_image_processor =
          image_processing_options.ecc_image_processor;

      frame_registration_->set_ecc_image_preprocessor(
          [ecc_image_processor](cv::InputOutputArray image, cv::InputOutputArray mask) {
            ecc_image_processor->process(image, mask);
          });
    }


    if ( !frame_registration_->setup_referece_frame(reference_frame, reference_mask) ) {
      CF_FATAL("frame_registration_->setup_referece_frame() fails");
      return false;
    }

    if ( weights_required() ) {
      compute_weights(reference_frame, reference_mask,
          reference_weights_);
    }

    if ( true ) {
      lock_guard lock(accumulator_lock_);

      frame_accumulation_.reset(new c_frame_weigthed_average());

      if ( master_options.master_sharpen_factor > 0 ) {
        sharpness_norm_accumulation_.reset(new c_sharpness_norm_measure());
      }
    }

    if ( canceled() ) {
      return false;
    }

    if ( max_frames_to_stack > input_sequence->size() ) {
      max_frames_to_stack = input_sequence->size();
    }

    const int startpos = std::max(0, master_frame_index - max_frames_to_stack / 2);
    const int endpos = startpos + max_frames_to_stack;

    if ( !process_input_sequence(input_sequence, startpos, endpos) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    // Reset master index indicator because master frame was generated from a sequence, not just a single frame

    if ( canceled() ) {
      return false;
    }

    // Read back accumulators
    if ( true ) {
      lock_guard lock(accumulator_lock_);

      if ( frame_accumulation_->accumulated_frames() < 1 ) {
        CF_ERROR("ERROR: No frames accumulated for reference frame");
        return false;
      }

      if ( !frame_accumulation_->compute(reference_frame, reference_mask) ) {
        CF_ERROR("ERROR: frame_accumulation_->compute() fails");
        return false;
      }

      average_pyramid_inpaint(reference_frame,
          reference_mask,
          reference_frame);

      // sharpen reference image
      if ( sharpness_norm_accumulation_ ) {

        reference_sharpness_ =
            sharpness_norm_accumulation_->average();

        const double current_sharpeness =
            sharpness_norm_accumulation_->measure(reference_frame,
                reference_mask);

        const double alpha =
            1 - master_options.master_sharpen_factor * current_sharpeness / reference_sharpness_;

        CF_DEBUG("XX MASTER: current sharpeness = %g averaged sharpeness = %g alpha=%g",
            current_sharpeness,
            reference_sharpness_,
            alpha);

        if ( options_->output_options().dump_reference_data_for_debug ) {

          save_image(reference_frame,
              ssprintf("%s/%s-initial_reference_frame.tiff",
                  output_directory_.c_str(),
                  options_->cname()));
        }

        if ( alpha < 1 ) {

          unsharp_mask(reference_frame, reference_frame,
              sharpness_norm_accumulation_->sigma(),
              alpha, 0, 1);

          if ( options_->output_options().dump_reference_data_for_debug ) {

            const double current_sharpeness =
                sharpness_norm_accumulation_->measure(reference_frame,
                    reference_mask);

            CF_DEBUG("AFTER UNSHARP: sharpeness = %g",
                current_sharpeness);

            if ( !master_options.save_master_frame ) {

              save_image(reference_frame, ssprintf("%s/%s-reference_frame_after_sharpenning.tiff",
                  output_directory_.c_str(),
                  options_->cname()));

            }
          }
        }
      }

    }

    if ( canceled() ) {
      return false;
    }
  }


  if ( true ) {
    lock_guard lock(accumulator_lock_);
    sharpness_norm_accumulation_.reset();
    flow_accumulation_.reset();
    frame_accumulation_.reset();
    frame_registration_.reset();
  }

  master_frame_generation_ = false;
  return true;
}


bool c_image_stacking_pipeline::process_input_sequence(const c_input_sequence::ptr & input_sequence, int startpos, int endpos)
{
  cv::Mat current_frame, current_mask;
  cv::Mat2f current_remap;
  cv::Mat1f current_weights;

  c_video_writer output_preprocessed_frames_writer;
  c_video_writer output_aligned_frames_writer;
  c_video_writer output_postprocessed_frames_writer;
  c_video_writer output_accumulation_masks_writer;

  const c_input_options & input_options =
      options_->input_options();

  const c_frame_upscale_options & upscale_options =
      options_->upscale_options();

  const c_master_frame_options & master_frame_options =
      options_->master_frame_options();

  const c_frame_accumulation_options & accumulation_options =
      options_->accumulation_options();

  const c_frame_registration_options & registration_options =
      options_->frame_registration_options();

  const c_image_processing_options & image_processing_options =
      options_->image_processing_options();

  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !input_sequence->seek(startpos) ) {
    CF_ERROR("input_sequence->seek(startpos=%d) fails", startpos);
    return false;
  }

  total_frames_ = endpos - startpos;
  CF_DEBUG("total_frames_=%d", total_frames_);

  int accumulated_frames_ = 0;
  for ( processed_frames_ = 0; processed_frames_ < total_frames_; ++processed_frames_, emit_status_changed() ) {

    double t0, t1, start_time, total_time;
    double time_read = 0;
    double time_upscale = 0;
    double time_register = 0;
    double time_remap = 0;
    double time_accumulate = 0;
    double time_select_roi = 0;

    if ( canceled() ) {
      break;
    }


    t0 = start_time = get_realtime_ms();

    if ( !badframes_.empty() ) {

      const std::vector<uint> :: const_iterator pos =
          std::find(badframes_.begin(),
              badframes_.end(),
              input_sequence->current_pos());

      if( pos != badframes_.end() ) {
        CF_DEBUG("Skip frame %d as blacklisted", input_sequence->current_pos());
        input_sequence->seek(input_sequence->current_pos() + 1);
        continue;
      }

    }


    if ( !read_input_frame(input_sequence, input_options, current_frame, current_mask) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    time_read = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !select_image_roi(roi_selection_, current_frame, current_mask, current_frame, current_mask) ) {
      continue;
    }

    time_select_roi = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    if( image_processing_options.input_image_processor ) {
      if( !image_processing_options.input_image_processor->process(current_frame, current_mask) ) {
        CF_ERROR("input_image_processor->process(current_frame) fails");
        continue;
      }
    }

    if ( canceled() ) {
      break;
    }

    if ( !master_frame_generation_ ) {

      save_preprocessed_frame(current_frame, current_mask,
          output_preprocessed_frames_writer);

      if ( canceled() ) {
        break;
      }
    }


    if ( master_frame_generation_ && current_frame.channels() > 1 ) {

      bool fOk = extract_channel(current_frame, current_frame,
          cv::noArray(), cv::noArray(),
          options_->frame_registration_options().image_registration_options.registration_channel);

      if ( !fOk ) {
        CF_ERROR("extract_channel(registration_channel=%d) fails",
            options_->frame_registration_options().image_registration_options.registration_channel);
        return false;
      }

      if ( canceled() ) {
        break;
      }
    }

    if( weights_required() ) {
      compute_weights(current_frame, current_mask, current_weights);
      if ( canceled() ) {
        break;
      }
    }

    if ( upscale_required(frame_upscale_before_align) ) {

      upscale_image(upscale_options.upscale_option,
          current_frame, current_mask,
          current_frame, current_mask);

      if ( canceled() ) {
        break;
      }

      if( !current_weights.empty() ) {
        upscale_image(options_->upscale_options().upscale_option,
            current_weights, cv::noArray(),
            current_weights, cv::noArray());
      }

      if ( canceled() ) {
        break;
      }
    }


    time_upscale = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    /////////////////////////////////////

    if ( sharpness_norm_accumulation_ ) {
      sharpness_norm_accumulation_-> add(current_frame, current_mask);
    }

    /////////////////////////////////////
    if ( frame_registration_ ) {

      if ( !external_master_frame_ && input_sequence->current_pos() == master_frame_index_ + 1 ) {

        if ( upscale_required(frame_upscale_after_align) ) {

          upscale_image(upscale_options.upscale_option,
              current_frame, current_mask,
              current_frame, current_mask);

        }

      }
      else {

        if ( output_options.debug_frame_registration ) {

          if ( !output_options.debug_frame_registration_frame_indexes.empty()  ) {

            const std::vector<int> :: const_iterator pos =
                std::find(output_options.debug_frame_registration_frame_indexes.begin(),
                    output_options.debug_frame_registration_frame_indexes.end(),
                    input_sequence->current_pos() - 1);

            if ( pos == output_options.debug_frame_registration_frame_indexes.end() ) {
              frame_registration_->set_enable_debug(false);
            }
            else {
              frame_registration_->set_enable_debug(true);
              frame_registration_->set_debug_path(ssprintf("%s/debug-registration-%d",
                  output_directory_.c_str(), *pos));
            }

          }

          if( canceled() ) {
            break;
          }
        }


        if ( !frame_registration_->register_frame(current_frame, current_mask) ) {
          CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + startpos);
          continue;
        }

        if( canceled() ) {
          break;
        }

        if ( flow_accumulation_ ) {

          const cv::Mat2f turbulence =
              compute_turbulent_flow(
                  frame_registration_->options().motion_type,
                  frame_registration_->current_transform(),
                  frame_registration_->current_remap()/*,
                   current_mask_*/);

          if ( turbulence.empty() ) {
            CF_ERROR("compute_turbulence_flow() fails");
          }
          else {
            flow_accumulation_->add(turbulence/*,
                current_mask_*/);
          }

          if( canceled() ) {
            break;
          }
        }

        if ( !upscale_required(frame_upscale_after_align) ) {
          current_remap = frame_registration_->current_remap();
        }
        else {
          current_remap.release();
          upscale_remap(upscale_options.upscale_option,
              frame_registration_->current_remap(),
              current_remap);
        }


        frame_registration_->custom_remap(current_remap,
            current_frame, current_frame,
            current_mask, current_mask,
            registration_options.image_registration_options.interpolation,
            master_frame_generation_ ?
                ECC_BORDER_REFLECT101 :
                registration_options.image_registration_options.border_mode,
            registration_options.image_registration_options.border_value);

        if( !current_weights.empty() ) {

          frame_registration_->custom_remap(current_remap,
              current_weights, current_weights,
              cv::noArray(), cv::noArray(),
              registration_options.image_registration_options.interpolation,
              ECC_BORDER_CONSTANT);

          cv::divide(current_weights, reference_weights_,
              current_weights);

          cv::multiply(current_mask, current_weights, current_weights,
              current_mask.depth() == CV_8U ?
                  1. / 255 : 1,
              CV_32F);

          current_mask = current_weights;
        }
      }


      if ( !master_frame_generation_ ) {

        save_aligned_frame(current_frame, current_mask,
            output_aligned_frames_writer);

        if ( canceled() ) {
          break;
        }
      }

      if( image_processing_options.aligned_image_processor ) {
        image_processing_options.aligned_image_processor->process(current_frame, current_mask);
        if ( canceled() ) {
          break;
        }
      }

      if( output_options.save_postprocessed_frames ) {

        save_postprocessed_frame(current_frame, current_mask,
            output_postprocessed_frames_writer);

        if ( canceled() ) {
          break;
        }
      }

    }

    time_register = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    /////////////////////////////////////////////////////////////////////////////////

    if ( output_options.save_accumulation_masks ) {

      save_accumulation_mask(current_frame, current_mask,
          output_accumulation_masks_writer);

      if ( canceled() ) {
        break;
      }
    }


    if ( frame_accumulation_ ) {

      if ( true ) {
        lock_guard lock(accumulator_lock_);

        if ( !frame_accumulation_->add(current_frame, current_mask) ) {
          CF_ERROR("frame_accumulation_->add(current_frame) fails");
          return false;
        }
      }

      ++accumulated_frames_;
      emit_accumulator_changed();
    }

    /////////////////////////////////////


    time_accumulate = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    ///////////////

    total_time = get_realtime_ms() - start_time;

    CF_DEBUG("[F %d / %5d / %5d / %6d] OK  %g ms\n"
        "read    : %g ms\n"
        "roi     : %g ms\n"
        "upscale : %g ms\n"
        "register: %g ms\n"
        "remap   : %g ms\n"
        "process : %g ms\n"
        "-----------\n\n\n",
        processed_frames_ + startpos, processed_frames_, accumulated_frames_, total_frames_, total_time,

        time_read,
        time_select_roi,
        time_upscale,
        time_register,
        time_remap,
        time_accumulate);

    emit_accumulator_changed();
  }

  return true;
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

    if ( input_options.filter_bad_pixels ) {
      remove_bad_pixels(output_image, input_options);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->pixel_depth())));

  }
  else {

    extract_bayer_planes(output_image, output_image,
        input_sequence->colorid());

    if ( input_options.filter_bad_pixels ) {
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

  if ( !output_mask.empty() && input_options.inpaint_missing_pixels ) {
    average_pyramid_inpaint(output_image, output_mask,
        output_image);
  }


  return true;
}

bool c_image_stacking_pipeline::select_image_roi(const c_roi_selection::ptr & roi_selection,
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


  cv::Rect ROI;

  if ( !roi_selection->select_roi(src, srcmask, ROI) || ROI.empty() ) {
    CF_ERROR("roi_selection->select_roi() fails");
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

  medianImage.copyTo(image, variationImage > input_optons.hot_pixels_variation_threshold * meanVariationImage);
}


void c_image_stacking_pipeline::upscale_remap(enum frame_upscale_option scale,
    cv::InputArray srcmap,
    cv::OutputArray dstmap)
{
  switch (scale) {
  case frame_upscale_x15:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3 / 2, srcmap.rows() * 3 / 2);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      cv::pyrUp(srcmap, dstmap);
    }
    break;
  }
  case frame_upscale_x30:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3, srcmap.rows() * 3);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    break;
  }
  default:
    if( !srcmap.empty() && dstmap.needed() ) {
      srcmap.copyTo(dstmap);
    }
    break;
  }
}

void c_image_stacking_pipeline::upscale_optflow(enum frame_upscale_option scale,
    cv::InputArray srcmap,
    cv::OutputArray dstmap)
{
  switch (scale) {
  case frame_upscale_x15:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3 / 2, srcmap.rows() * 3 / 2);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR);
      cv::multiply(dstmap, 3.0 / 2.0, dstmap);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      cv::pyrUp(srcmap, dstmap);
      cv::multiply(dstmap, 2.0, dstmap);
    }
    break;
  }
  case frame_upscale_x30:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3, srcmap.rows() * 3);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
      cv::multiply(dstmap, 3.0, dstmap);
    }
    break;
  }
  default:
    if( !srcmap.empty() && dstmap.needed() ) {
      srcmap.copyTo(dstmap);
    }
    break;
  }
}


void c_image_stacking_pipeline::upscale_image(enum frame_upscale_option scale,
    cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{

  switch (scale) {
  case frame_upscale_x15:
  {
    const cv::Size dstsize(src.cols() * 3 / 2, src.rows() * 3 / 2);
    if( !src.empty() && dst.needed() ) {
      cv::resize(src, dst, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::resize(srcmask, dstmask, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !src.empty() && dst.needed() ) {
      cv::pyrUp(src, dst);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::pyrUp(srcmask, dstmask);
    }
    break;
  }
  case frame_upscale_x30:
  {
    const cv::Size dstsize(src.cols() * 3, src.rows() * 3);
    if( !src.empty() && dst.needed() ) {
      cv::resize(src, dst, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::resize(srcmask, dstmask, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    break;
  }
  default:
    if( !src.empty() && dst.needed() ) {
      src.copyTo(dst);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      srcmask.copyTo(dstmask);
    }
    return;
  }

  if( !srcmask.empty() && dstmask.needed() ) {
    cv::compare(dstmask.getMatRef(), 255, dstmask.getMatRef(), cv::CMP_GE);
  }

}

bool c_image_stacking_pipeline::weights_required() const
{
  return options_->accumulation_options().accumulation_method == frame_accumulation_weighted_average &&
      options_->accumulation_options().lksize > 0;
}

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  const c_frame_accumulation_options & acc_options =
      options_->accumulation_options();

  compute_smap(src, dst,
      acc_options.lksize,
      acc_options.scale_size,
      acc_options.minv);
}

void c_image_stacking_pipeline::compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel)
{
  cv::divide(wc, wref, wrel);
  if ( mc.size() == wrel.size() ) {
    wrel.setTo(0, ~mc);
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

    if ( frame_accumulation_->accumulated_frames() < 1 ) {
      return false;
    }

    if ( frame_accumulation_ ) {
      return frame_accumulation_->compute(dst, dstmask);
    }
  }

  return false;
}

bool c_image_stacking_pipeline::upscale_required(frame_upscale_stage current_stage) const
{
  if ( master_frame_generation_  ) {
    return false;
  }

  const c_frame_upscale_options & opts =
      options_->upscale_options();

  return opts.upscale_option != frame_upscale_none  && current_stage == opts.upscale_stage;
}



bool c_image_stacking_pipeline::write_image(const std::string & output_file_name,
    const c_image_stacking_output_options & output_options,
    const cv::Mat & output_image, const cv::Mat & output_mask)
{
  cv::Mat image_to_write;

  if ( !output_options.write_image_mask_as_alpha_channel || output_mask.empty() || (output_image.channels() != 3 && output_image.channels() != 1)  ) {
    image_to_write = output_image;
  }
  else if ( !mergebgra(output_image, output_mask, image_to_write) ) {
    CF_ERROR("ERROR: mergebgra() fails");
    return false;
  }

  return save_image(image_to_write, output_file_name);
}


void c_image_stacking_pipeline::save_preprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer) const
{
  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !output_options.save_preprocessed_frames ) {
    return;
  }

  if ( !output_writer.isOpened() ) {

    std::string pathfilename =
        output_options.output_preprocessed_frames_filename;

    if ( pathfilename.empty() ) {
      pathfilename = ssprintf("%s-preproc.avi",
          options_->cname());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_directory_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1);

    if ( !output_writer.isOpened() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options.write_image_mask_as_alpha_channel);
}

void c_image_stacking_pipeline::save_aligned_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer) const
{
  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !output_options.save_aligned_frames ) {
    return;
  }

  if ( !output_writer.isOpened() ) {

    std::string pathfilename =
        output_options.output_aligned_frames_filename;

    if ( pathfilename.empty() ) {
      pathfilename = ssprintf("%s-aligned.avi",
          options_->cname());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_directory_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1);

    if ( !output_writer.isOpened() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options.write_image_mask_as_alpha_channel);
}

void c_image_stacking_pipeline::save_postprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer) const
{
  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !output_options.save_postprocessed_frames ) {
    return;
  }

  if ( !output_writer.isOpened() ) {

    std::string pathfilename =
        output_options.output_postprocessed_frames_filename;

    if ( pathfilename.empty() ) {
      pathfilename = ssprintf("%s-postproc.avi",
          options_->cname());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_directory_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1);

    if ( !output_writer.isOpened() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options.write_image_mask_as_alpha_channel);
}

void c_image_stacking_pipeline::save_accumulation_mask(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer) const
{
  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !output_options.save_accumulation_masks ) {
    return;
  }

  if ( !output_writer.isOpened() ) {

    std::string pathfilename =
        output_options.output_accumulation_masks_filename;

    if ( pathfilename.empty() ) {
      pathfilename = ssprintf("%s-masks.avi",
          options_->cname());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_directory_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        false);

    if ( !output_writer.isOpened() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  if ( !current_mask.empty() ) {
    output_writer.write(current_mask, cv::noArray());
  }
  else {
    output_writer.write(cv::Mat1b(current_frame.size(), 255), cv::noArray());
  }

}
