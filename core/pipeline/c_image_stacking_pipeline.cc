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
#include <core/ssprintf.h>
#include <core/get_time.h>
#include <core/debug.h>



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    { "Jovian Derotate", frame_registration_method_jovian_derotate},
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
    { "weigted_average", frame_accumulation_weighted_average },
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

const struct frame_upscale_stage_desc frame_upscale_stages[] = {
    { "after_align", frame_upscale_after_align },
    { "before_align", frame_upscale_before_align },
    { nullptr, frame_upscale_stage_unknown },
};

std::string toStdString(enum frame_upscale_stage v)
{
  for ( uint i = 0; frame_upscale_stages[i].name; ++i ) {
    if ( frame_upscale_stages[i].value == v ) {
      return frame_upscale_stages[i].name;
    }
  }
  return "";
}

enum frame_upscale_stage fromStdString(const std::string  & s,
    enum frame_upscale_stage defval )
{
  const char * cstr = s.c_str();

  for ( uint i = 0; frame_upscale_stages[i].name; ++i ) {
    if ( strcasecmp(frame_upscale_stages[i].name, cstr) == 0 ) {
      return frame_upscale_stages[i].value;
    }
  }
  return defval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const struct frame_upscale_option_desc frame_upscale_options[] ={
    {"none", frame_upscale_none},
    {"x2.0", frame_upscale_pyrUp},
    {"x1.5", frame_upscale_x15},
    {"x3.0", frame_upscale_x30},
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

c_frame_upscale_options & c_image_stacking_options::upscale_options()
{
  return upscale_options_;
}

const c_frame_upscale_options & c_image_stacking_options::upscale_options() const
{
  return upscale_options_;
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

    case frame_registration_method_jovian_derotate:
      return c_jovian_rotation_registration::create(frame_registration_options_.base_options,
          frame_registration_options_.planetary_disk_options,
          frame_registration_options_.jovian_derotation_options);

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

    CF_ERROR("std::exception catched in c_image_stacking_pipeline::run()\n",
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

    CF_ERROR("std::exception catched in c_image_stacking_pipeline::cleanup()\n",
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

    master_frame_index_ = -1;
    ecc_normalization_noise_ = 0;

    total_frames_ = 0;
    processed_frames_ = 0;
    reference_sharpness_ = 0;

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

  CF_DEBUG("H");
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

  const c_image_stacking_output_options & output_options =
      options_->output_options();


  /////////////////////////////////////////////////////////////////////////////
  // SETUP FRAME REGISTRATION

  if ( registration_options.registration_method != frame_registration_none ) {

    set_status_msg("CREATE REFERENCE FRAME ...");

    const c_master_frame_options & master_options =
        options_->master_frame_options();

    c_input_sequence::ptr input_sequence;
    int master_source_index = -1;
    int master_frame_index = -1;
    int max_frames_to_stack = 0;

    cv::Mat reference_frame;
    cv::Mat reference_mask;

    std::string master_file_name = master_options.master_source_path;

    CF_DEBUG("H");

    if ( master_file_name.empty() ) {
      master_file_name = this->input_sequence_->source(master_source_index = 0)->filename();
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
    }
    else if ( !(input_sequence = c_input_sequence::create(master_file_name)) ) {
      CF_ERROR("ERROR: c_input_sequence::create(master_file_name_=%s) fails", master_file_name.c_str());
      return false;
    }

    if ( !input_sequence->open() ) {
      CF_FATAL("ERROR: Can not open input source '%s'", master_file_name.c_str());
      return false;
    }

    if ( (master_frame_index = master_options.master_frame_index) < 0 ) {
      master_frame_index = 0;
    }
    else if ( master_source_index >= 0 ) {
      if ( master_frame_index >= input_sequence->source(master_source_index)->size() ) {
        CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
            master_frame_index, master_file_name.c_str());
        return false;
      }
    }
    else if ( master_frame_index >= input_sequence->source(0)->size() ) {
      CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
          master_frame_index, master_file_name.c_str());
      return false;
    }

    if ( master_source_index < 0 ) {
      master_frame_index = input_sequence->global_pos(0, master_frame_index);
    }
    else {
      master_frame_index = input_sequence->global_pos(master_source_index, master_frame_index);
    }


    if ( master_options.generate_master_frame ) {
      max_frames_to_stack = master_options.max_input_frames_to_generate_master_frame;
    }

    CF_DEBUG("H");
    if ( !create_reference_frame(input_sequence, master_frame_index, max_frames_to_stack,
        reference_frame, reference_mask) ) {
      CF_FATAL("ERROR: create_reference_frame() fails");
      return false;
    }
    CF_DEBUG("H");

    ecc_normalization_noise_ =
        compute_image_noise(reference_frame, reference_mask,
            options_->frame_registration_options().base_options.registration_channel);

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

    if ( !(frame_registration_ = options_->create_frame_registration()) ) {
      CF_FATAL("options_->create_frame_registration() fails");
      return false;
    }

    if ( options_->upscale_options().need_upscale_before_align() ) {
      upscale_image(options_->upscale_options().upscale_option,
          reference_frame, reference_mask,
          reference_frame, reference_mask);
    }

    if ( !frame_registration_->setup_referece_frame(reference_frame, reference_mask) ) {
      CF_ERROR("ERROR: frame_registration_->setup_referece_frame() fails");
      return false;
    }

    if ( false ) {

      if ( frame_registration_->enable_eccflow() ) {

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
    }


    if ( registration_options.accumulate_and_compensate_turbulent_flow ) {
      if ( frame_registration_->motion_type() > ECC_MOTION_EUCLIDEAN || frame_registration_->enable_eccflow() ) {
        flow_accumulation_.reset(new c_frame_weigthed_average());
      }
    }
    CF_DEBUG("H");
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

  master_frame_generation_ = false;

  if ( !(fOk = process_input_sequence(input_sequence_, 0, input_sequence_->size())) ) {
    CF_ERROR("process_input_sequence() fails");
    return false;
  }

  set_status_msg("FINISHING ...");


  // Read back accumulators

  if ( frame_accumulation_ ) {

    cv::Mat accumulated_frame;
    cv::Mat1b accumulated_mask;

    if ( true ) {
      lock_guard lock(accumulator_lock_);

      if ( !frame_accumulation_->compute(accumulated_frame, accumulated_mask) ) {
        CF_ERROR("ERROR: frame_accumulation_->compute() fails");
        return false;
      }

      average_pyramid_inpaint(accumulated_frame,
          accumulated_mask,
          accumulated_frame);
    }



    if( options_->master_frame_options().accumulated_sharpen_factor > 0 ) {

      c_sharpness_norm_measure sm;

      const double factor =
          options_->master_frame_options().accumulated_sharpen_factor;

      const double current_sharpeness =
          sm.measure(accumulated_frame, accumulated_mask);

      const double alpha =
          1.0 - factor * current_sharpeness / reference_sharpness_ / accumulated_frame.channels();

      CF_DEBUG("YYY accumulated sharpness = %g reference sharpness = %g alpha=%g",
          current_sharpeness,
          reference_sharpness_,
          alpha);

      if( options_->output_options().dump_reference_data_for_debug ) {

        save_image(accumulated_frame,
            ssprintf("%s/%s-initial_accumulated_frame.tiff",
                output_directory_.c_str(),
                options_->cname()));
      }

      if( alpha < 1 ) {

        unsharp_mask(accumulated_frame, accumulated_frame,
            sm.sigma() * upscale_options.image_scale(),
            alpha, 0, 1);

        const double current_sharpeness =
            sm.measure(accumulated_frame, accumulated_mask);

        CF_DEBUG("YYY final sharpeness = %g reference sharpeness = %g alpha=%g",
            current_sharpeness,
            reference_sharpness_,
            alpha);
      }
    }


    if ( anscombe_.method() != anscombe_none ) {
      anscombe_.inverse(accumulated_frame,
          accumulated_frame);
    }


    // Fix turbulent flow
    if ( flow_accumulation_ ) {

      cv::Mat accumulated_flow;

      if ( !flow_accumulation_->compute(accumulated_flow) ) {
        CF_ERROR("ERROR: flow_accumulation_->compute() fails");
      }
      else {

        CF_DEBUG("accumulated_flow: %dx%d", accumulated_flow.cols, accumulated_flow.rows);

        if ( accumulated_flow.size() != accumulated_frame.size() ) {
          upscale_remap(upscale_options.upscale_option,
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
              accumulated_frame,
              accumulated_mask);
        }

        accumulated_flow = flow2remap(accumulated_flow,
            accumulated_mask);

        // FIXME: BORDER !!!
        cv::remap(accumulated_frame, accumulated_frame, accumulated_flow,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

        cv::remap(accumulated_mask, accumulated_mask, accumulated_flow,
            cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT);

        cv::compare(accumulated_mask, 220, accumulated_mask, cv::CMP_GE);

        // reduce potential remap interpoolation artifacts
        //clip_range(accumulated_frame, 0, 1, accumulated_mask);
      }

    }

    if ( output_file_name.empty() ) {

      output_file_name =
          ssprintf("%s/%s-32F.tiff",
              output_directory_.c_str(),
              options_->cname());
    }

    CF_DEBUG("Saving '%s'", output_file_name.c_str());
    if ( !write_image(output_file_name, output_options, accumulated_frame, accumulated_mask) ) {
      CF_ERROR("write_image('%s') fails", output_file_name.c_str());
    }


    if ( output_options.accumuated_image_processor ) {

      if ( !output_options.accumuated_image_processor->process(accumulated_frame, accumulated_mask) ) {
        CF_ERROR("postprocessor %s : process() fails", output_options.accumuated_image_processor->cname());
      }
      else {

        output_file_name =
            ssprintf("%s/%s-32F-PP.tiff",
                output_directory_.c_str(),
                options_->cname());

        CF_DEBUG("Saving '%s'", output_file_name.c_str());
        if ( !write_image(output_file_name, output_options, accumulated_frame, accumulated_mask) ) {
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

  if ( master_options.apply_input_frame_processor && input_options.input_frame_processor ) {
    if ( !input_options.input_frame_processor->process(reference_frame, reference_mask) ) {
      CF_ERROR("input_frame_processor->process(reference_frame) fails");
      return false;
    }
  }

  if ( canceled() ) {
    return false;
  }

  if ( reference_frame.channels() > 1 ) {

    const color_channel_type master_channel =
        options_->frame_registration_options().
            base_options.registration_channel;

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

    if ( !(frame_registration_ = options_->create_frame_registration()) ) {
      CF_FATAL("options_->create_frame_registration() fails");
      return false;
    }

    if( frame_registration_->enable_ecc() ) {
      frame_registration_->base_options().ecc.reference_smooth_sigma =
          frame_registration_->base_options().ecc.input_smooth_sigma;
    }

    frame_registration_->set_enable_eccflow(master_options.eccflow_scale > 1);
    if ( frame_registration_->enable_eccflow() ) {

      frame_registration_->base_options().eccflow.support_scale =
          master_options.eccflow_scale;

      frame_registration_->base_options().eccflow.normalization_scale =
          master_options.eccflow_scale;
    }

    if ( !frame_registration_->setup_referece_frame(reference_frame, reference_mask) ) {
      CF_FATAL("frame_registration_->setup_referece_frame() fails");
      return false;
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

    master_frame_generation_ = true;
    if ( !process_input_sequence(input_sequence, startpos, endpos) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    // Reset master index indicator because master frame was generated from a sequence, not just a single frame
    this->master_frame_index_ = -1;

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

  return true;
}


bool c_image_stacking_pipeline::process_input_sequence(const c_input_sequence::ptr & input_sequence, int startpos, int endpos)
{
  cv::Mat current_frame, current_mask;
  cv::Mat2f current_remap;

  c_video_writer output_aligned_video;

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

  const c_image_stacking_output_options & output_options =
      options_->output_options();

  if ( !input_sequence->seek(startpos) ) {
    CF_ERROR("input_sequence->seek(startpos=%d) fails", startpos);
    return false;
  }

  total_frames_ = endpos - startpos;
  CF_DEBUG("total_frames_=%d", total_frames_);

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

    if ( input_options.input_frame_processor ) {
      if ( !input_options.input_frame_processor->process(current_frame, current_mask) ) {
        CF_ERROR("input_frame_processor->process(current_frame) fails");
        continue;
      }
    }

    if ( canceled() ) {
      break;
    }

    if ( master_frame_generation_ && current_frame.channels() > 1 ) {

      bool fOk = extract_channel(current_frame, current_frame,
          cv::noArray(), cv::noArray(),
          options_->frame_registration_options().base_options.registration_channel);

      if ( !fOk ) {
        CF_ERROR("extract_channel(registration_channel=%d) fails",
            options_->frame_registration_options().base_options.registration_channel);
        return false;
      }

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

      if ( input_sequence->current_pos() == master_frame_index_ + 1 ) {

        if ( upscale_required(frame_upscale_after_align) ) {

          upscale_image(upscale_options.upscale_option,
              current_frame, current_mask,
              current_frame, current_mask);

        }

      }
      else {

        if ( !frame_registration_->register_frame(current_frame, current_mask) ) {
          CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + startpos);
          continue;
        }

        if ( flow_accumulation_ ) {

          const cv::Mat2f turbulence = compute_turbulent_flow(
              frame_registration_->motion_type(),
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
            cv::INTER_LINEAR,
            cv::BORDER_REFLECT101);
      }
    }

    time_register = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    ///////////////

    if( output_options.frame_processor ) {
      output_options.frame_processor->process(current_frame, current_mask);
      if ( canceled() ) {
        break;
      }
    }

    ///////////////

    if( output_options.save_processed_frames ) {

      save_processed_frame(current_frame, current_mask,
          output_options, output_directory_,
          options_->name(),
          input_sequence);

      if ( canceled() ) {
        break;
      }
    }


    ///////////////

    if( !master_frame_generation_  && output_options.write_aligned_video ) {

      write_aligned_video(current_frame,
          output_aligned_video,
          output_options,
          output_directory_);

      if( canceled() ) {
        break;
      }
    }


    /////////////////////////////////////
    if ( frame_accumulation_ ) {

      if ( true ) {
        lock_guard lock(accumulator_lock_);

        if ( !frame_accumulation_->add(current_frame, current_mask) ) {
          CF_ERROR("frame_accumulation_->add(current_frame) fails");
          return false;
        }
      }

      emit_accumulator_changed();
    }

    /////////////////////////////////////


    time_accumulate = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      break;
    }

    ///////////////

    total_time = get_realtime_ms() - start_time;

    CF_DEBUG("[F %d / %5d / %6d] OK  %g ms\n"
        "read    : %g ms\n"
        "roi     : %g ms\n"
        "upscale : %g ms\n"
        "register: %g ms\n"
        "remap   : %g ms\n"
        "process : %g ms\n"
        "-----------\n\n\n",
        processed_frames_ + startpos, processed_frames_, total_frames_, total_time,

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

    if ( input_options.filter_hot_pixels ) {
      remove_bad_pixels(output_image, input_options);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->pixel_depth())));

  }
  else {

    extract_bayer_planes(output_image, output_image,
        input_sequence->colorid());

    if ( input_options.filter_hot_pixels ) {
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

  if ( output_mask.depth() != CV_8U ) {
    CF_ERROR("FIX THIS CODE: c_image_stacking_pipeline::write_image(). NON - CV_8UC1 MASK IS NOT SUPPORTED YET");
    return false;
  }

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
    const c_input_sequence::ptr & input_sequence)
{
  std::string source_name;

  const c_input_source::ptr current_source =
      input_sequence->current_source();

  const int global_pos =
      input_sequence->current_pos() - 1;

  const int local_pos =
      global_pos - current_source->global_pos();

  split_pathfilename(current_source->filename(),
      nullptr,
      &source_name,
      nullptr );


  const std::string output_file_name =
      input_sequence->sources().size() < 2 ?

          ssprintf("%s/%s-%06d.tiff",
              output_directory.c_str(),
              source_name.c_str(),
              global_pos) :

          ssprintf("%s/%s-%06d-%06d.tiff",
              output_directory.c_str(),
              source_name.c_str(),
              local_pos,
              global_pos);

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

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst)
{
  compute_smap(src, dst, 0.01, 0);
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


