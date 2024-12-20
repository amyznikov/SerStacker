/*
 * c_input_sequence.cc
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#include "c_input_sequence.h"
#include "load_image.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

c_input_sequence::~c_input_sequence()
{
  close(true);
}

c_input_sequence::sptr c_input_sequence::create()
{
  return c_input_sequence::sptr(new c_input_sequence());
}

c_input_sequence::sptr c_input_sequence::create(const std::string & sourcefilename)
{
  c_input_sequence::sptr obj ( new c_input_sequence() );

  if ( !sourcefilename.empty() && !obj->add_source(sourcefilename) ) {
    return nullptr;
  }
  return obj;
}

c_input_sequence::sptr c_input_sequence::create(const std::vector<std::string> & sourcefilenames)
{
  c_input_sequence::sptr obj(new c_input_sequence());

  if ( sourcefilenames.empty() ) {
    return obj;
  }

  bool sources_added = false;

  for ( const std::string & s : sourcefilenames ) {
    if ( obj->add_source(s) ) {
      sources_added = true;
    }
  }

  return sources_added ? obj : nullptr;
}

void c_input_sequence::set_name(const std::string & name)
{
  name_ = name;
}

const std::string& c_input_sequence::name() const
{
  return name_;
}

const char* c_input_sequence::cname() const
{
  return name_.c_str();
}

bool c_input_sequence::is_live() const
{
  return false;
}

bool c_input_sequence::serialize(c_config_setting settings, bool save)
{
  SERIALIZE_PROPERTY(settings, save, *this, name);
  // SERIALIZE_PROPERTY(settings, save, *this, auto_debayer);
  SERIALIZE_PROPERTY(settings, save, *this, auto_apply_color_matrix);

  if( save ) {

    c_config_setting sources_list =
        settings.add_list("sources");

    for ( const c_input_source::sptr & source : all_sources_ ) {
      if ( source )  {

        c_config_setting group =
            sources_list.add_group();

        group.set("enabled", source->enabled());
        group.set("file", source->filename());
      }
    }

  }
  else {

    c_config_setting sources_list =
        settings["sources"];

    if ( sources_list.isList() ) {

      const int n = sources_list.length();
      all_sources_.reserve(n);

      for ( int i = 0; i < n; ++i ) {

        std::string filename;
        bool enabled = true;

        c_config_setting e =
            sources_list.get_element(i);

        if ( !e.isGroup() ) {
          e.get(&filename);
        }
        else {
          e.get("enabled", &enabled);
          e.get("file", &filename);
        }


        if ( !filename.empty() ) {
          c_input_source::sptr source = c_input_source::create(filename);
          if ( !source ) {
            CF_ERROR("c_input_source::create(filename='%s') fails", filename.c_str());
          }
          else {
            source->set_enabled(enabled);
            all_sources_.emplace_back(source);
          }
        }
      }
    }
  }

  return true;
}
//
//void c_input_sequence::set_auto_debayer(enum DEBAYER_ALGORITHM algo)
//{
//  auto_debayer_ = algo;
//}
//
//enum DEBAYER_ALGORITHM c_input_sequence::auto_debayer() const
//{
//  return auto_debayer_;
//}

void c_input_sequence::set_auto_apply_color_matrix(bool v)
{
  auto_apply_color_matrix_ = v;
}

bool c_input_sequence::auto_apply_color_matrix() const
{
  return auto_apply_color_matrix_;
}

const std::vector<c_input_source::sptr> & c_input_sequence::sources() const
{
  return all_sources_;
}

const c_input_source::sptr & c_input_sequence::source(int index) const
{
  return all_sources_[index];
}


int c_input_sequence::indexof(const c_input_source::sptr & source,
    const std::vector<c_input_source::sptr> & list)
{
  std::vector<c_input_source::sptr>::const_iterator ii =
      std::find(list.begin(), list.end(), source);

  return ii == list.end() ? -1 : ii - list.begin();
}

int c_input_sequence::indexof(const std::string & pathfilename,
    const std::vector<c_input_source::sptr> & list)
{
  std::vector<c_input_source::sptr>::const_iterator ii =
      std::find_if(list.begin(), list.end(),
          [&pathfilename](const c_input_source::sptr & source) -> bool {
            return source->filename() == pathfilename;
          });

  return ii == list.end() ? -1 : ii - list.begin();
}


int c_input_sequence::indexof(const std::string & pathfilename) const
{
  return indexof(pathfilename, all_sources_);
}

int c_input_sequence::indexof(const c_input_source::sptr & source) const
{
  return indexof(source, all_sources_);
}


c_input_source::sptr c_input_sequence::source(const std::string & pathfilename) const
{
  const int pos = indexof(pathfilename);
  return pos >= 0 ? all_sources_[pos] : nullptr;
}


c_input_source::sptr c_input_sequence::add_source(const std::string & pathfilename, int pos)
{
  close();

  c_input_source::sptr source = c_input_source::create(pathfilename);
  if ( !source ) {
    CF_ERROR("c_input_source::create(pathfilename=%s) fails", pathfilename.c_str());
    return nullptr;
  }

  if ( pos < 0 || pos >= all_sources_.size() ) {
    all_sources_.emplace_back(source);
  }
  else {
    all_sources_.insert(all_sources_.begin() + pos, source);
  }

  return source;
}

bool c_input_sequence::add_sources(const std::vector<std::string> & pathfilenames)
{
  int num_sourcess_added = 0;
  for ( const std::string & pathname : pathfilenames ) {
    if ( add_source(pathname) ) {
      ++num_sourcess_added;
    }
  }
  return num_sourcess_added > 0;
}

void c_input_sequence::remove_source(int pos)
{
  if ( pos >= 0 && pos < all_sources_.size() ) {

    c_input_source::sptr p =
        all_sources_[pos];

    all_sources_.erase(all_sources_.begin() + pos);
  }
}


void c_input_sequence::remove_source(const c_input_source::sptr & source)
{
  if ( source ) {

    const std::vector<c_input_source::sptr>::iterator ii =
        std::find(all_sources_.begin(), all_sources_.end(), source);

    if ( ii != all_sources_.end() ) {
      all_sources_.erase(ii);
    }
  }
}

void c_input_sequence::remove_source(const std::string & sourcefilename)
{
  return remove_source(source(sourcefilename));
}


void c_input_sequence::clear()
{
  close();
  all_sources_.clear();
}

bool c_input_sequence::empty() const
{
  return all_sources_.empty();
}

bool c_input_sequence::open()
{
  close();

  total_frames_ = 0;

  for ( c_input_source::sptr & s : all_sources_ ) {
    if ( s->enabled() ) {
      enabled_sources_.emplace_back(s);
      s->set_global_pos(total_frames_);
      total_frames_ += s->size();
    }
  }

  if ( enabled_sources_.size() < 1 || !open_source(0)  ) {
    return false;
  }

  current_source_ = 0;
  current_global_pos_ = 0;

  return true;
}

bool c_input_sequence::is_open() const
{
  return current_global_pos_ >= 0;
}

void c_input_sequence::close(bool also_clear)
{
  if ( current_source_ >= 0 ) {
    close_source(current_source_);
  }

  total_frames_ = -1;
  current_source_ = -1;
  current_global_pos_ = -1;
  last_bpp_ = 0;
  last_colorid_ = COLORID_UNKNOWN;
  last_color_matrix_ = cv::Matx33f::eye();
  has_last_color_matrix_ = false;
  enabled_sources_.clear();

  if ( also_clear ) {
    clear();
  }

}

bool c_input_sequence::open_source(int source_index)
{
  if ( source_index < 0 || source_index >= (int) enabled_sources_.size() ) {
    return false;
  }

  if ( !enabled_sources_[source_index]->open() ) {
    CF_ERROR("enabled_[source_index=%d]->open('%s') fails", source_index,
        enabled_sources_[source_index]->filename().c_str());
    return false;
  }

  return true;
}

void c_input_sequence::close_source(int source_index)
{
  if ( source_index >= 0 && source_index < (int) enabled_sources_.size() ) {
    enabled_sources_[source_index]->close();
  }
}

bool c_input_sequence::seek_current_source(int source_pos)
{
  if ( current_source_ < 0 || current_source_ >= (int) enabled_sources_.size() ) {
    CF_DEBUG("c_input_sequence:seek_current_source(source_pos=%d) FAILS", source_pos);
    return false;
  }
  return enabled_sources_[current_source_]->seek(source_pos);
}

int c_input_sequence::size()
{
  return total_frames_;
}

bool c_input_sequence::seek(int global_pos)
{
  int source_pos = -1;
  int required_source_ = -1;

  for ( int i = 0, n = enabled_sources_.size(); i < n; ++i ) {
    const c_input_source::sptr & s = enabled_sources_[i];
    if ( global_pos >= s->global_pos() && global_pos < s->global_pos() + s->size() ) {
      required_source_ = i;
      source_pos = global_pos - s->global_pos();
      break;
    }
  }

  if ( required_source_ < 0 || required_source_ >= (int)enabled_sources_.size() )  {
    CF_ERROR("ERROR: required_source_=%d / %zu is invalid",
        required_source_, enabled_sources_.size());
    return  false;
  }

  if ( required_source_ != current_source_ ) {
    close_source(current_source_);
    if ( !open_source(current_source_ = required_source_) ) {
      CF_ERROR("open_source(%d) fails", current_source_);
      close();
      return false;
    }
  }

  if ( !seek_current_source(source_pos) ) {
    CF_ERROR("seek_current_source(source=%d, source_pos=%d) fails",
        current_source_, source_pos);
    close();
    return false;
  }


  current_global_pos_ = global_pos;

  return true;
}

int c_input_sequence::current_pos() const
{
  return current_global_pos_;
}

void c_input_sequence::update_current_pos()
{
  if ( current_source_ >= 0 && current_source_ < (int)enabled_sources_.size() ) {

    const c_input_source::sptr & s =
        enabled_sources_[current_source_];

    current_global_pos_ =
        s->global_pos() + s->curpos();
  }

}

c_input_source::sptr c_input_sequence::current_source() const
{
  return current_source_ >= 0 && current_source_ < (int) enabled_sources_.size() ? enabled_sources_[current_source_] : nullptr;
}

int c_input_sequence::global_pos(int source_index, int source_frame_index) const
{
  if ( !is_open() ) {
    CF_ERROR("c_input_sequence: IS NOT OPEN");
    return -1;
  }

  if ( source_index < 0 || source_index >= all_sources_.size() ) {
    CF_ERROR("Invalid source_index=%d / %zu", source_index, all_sources_.size());
    return -1;
  }

  int enabled_source_index = -1;
  for ( int i = 0, n = enabled_sources_.size(); i < n; ++i ) {
    if ( enabled_sources_[i] == all_sources_[source_index] ) {
      enabled_source_index = i;
      break;
    }
  }

  if ( enabled_source_index < 0 ) {
    CF_ERROR("source_index=%d is not enabled", source_index);
    return -1;
  }

  if ( source_frame_index < 0 || source_frame_index >= enabled_sources_[enabled_source_index]->size() ) {
    CF_ERROR("source_frame_index=%d exceeds the size=%d of source_index=%d ",
        source_frame_index, enabled_sources_[enabled_source_index]->size(), source_index);
    return -1;
  }

  return enabled_sources_[enabled_source_index]->global_pos() + source_frame_index;
}


bool c_input_sequence::read_current_source(cv::Mat & output_frame, cv::Mat * output_mask)
{
  last_bpp_ = 0;
  last_colorid_ = COLORID_UNKNOWN;
  has_last_color_matrix_ = false;

  c_input_source::sptr source = current_source();
  if ( !source ) {
    return false;
  }

  if ( !source->read(output_frame, &last_colorid_, &last_bpp_) ) {
    return false;
  }

//  if ( is_bayer_pattern(last_colorid_) ) {
//    if ( output_mask ) { // not clear the meaning of alpha mask with bayer pattern
//      output_mask->release();
//    }
//
//    if ( auto_debayer_ != DEBAYER_DISABLE && debayer(output_frame, output_frame, last_colorid_, auto_debayer_) ) {
//      last_colorid_ = COLORID_BGR;
//    }
//  }
//  else
  if ( output_mask  ) {

    if ( last_colorid_ == COLORID_OPTFLOW || (output_frame.channels() != 4 && output_frame.channels() != 2) ) {
      output_mask->release();
    }
    else if ( !splitbgra(output_frame, output_frame, output_mask) ) {
      output_mask->release();
      return false;
    }
  }


  if ( (has_last_color_matrix_ = source->has_color_matrix()) ) {
    last_color_matrix_ = source->color_matrix();
  }

  if( (_has_last_ts = source->has_last_ts()) ) {
    _last_ts = source->last_ts();
  }

  return true;
}

bool c_input_sequence::read(cv::Mat & output_frame, cv::Mat * output_mask)
{
  if ( current_source_ < 0 || current_source_ >= (int) enabled_sources_.size() ) {
    CF_DEBUG("return false: current_source_=%d", current_source_);
    return false;
  }

  while ( current_source_ < (int) enabled_sources_.size() ) {
    if ( read_current_source(output_frame, output_mask) ) {

      update_current_pos();
      return true;
    }

    close_source(current_source_);
    if ( ++current_source_ < (int) enabled_sources_.size() && !open_source(current_source_) ) {
      break;
    }
  }

  return false;
}

enum COLORID c_input_sequence::colorid() const
{
  return last_colorid_;
}

int c_input_sequence::bpp() const
{
  return last_bpp_;
}

const cv::Matx33f & c_input_sequence::color_matrix() const
{
  return last_color_matrix_;
}

bool c_input_sequence::has_color_matrix() const
{
  return has_last_color_matrix_;
}

bool c_input_sequence::has_last_ts() const
{
  return _has_last_ts;
}

double c_input_sequence::last_ts() const
{
  return _last_ts;
}
