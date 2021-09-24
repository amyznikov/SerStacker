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

c_input_sequence::ptr c_input_sequence::create()
{
  return c_input_sequence::ptr(new c_input_sequence());
}

c_input_sequence::ptr c_input_sequence::create(const std::string & sourcefilename)
{
  c_input_sequence::ptr obj ( new c_input_sequence() );

  if ( !sourcefilename.empty() && !obj->add_source(sourcefilename) ) {
    return nullptr;
  }
  return obj;
}

c_input_sequence::ptr c_input_sequence::create(const std::vector<std::string> & sourcefilenames)
{
  c_input_sequence::ptr obj(new c_input_sequence());

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


bool c_input_sequence::serialize(c_config_setting settings) const
{
  SAVE_PROPERTY(settings, *this, auto_debayer);
  SAVE_PROPERTY(settings, *this, auto_apply_color_matrix);

  c_config_setting sources_list =
      settings.add_list("sources");

  for ( const c_input_source::ptr & source : sources_ ) {
    if ( source )  {
      sources_list.add(source->filename());
    }
  }

  return true;
}

bool c_input_sequence::deserialize(c_config_setting settings)
{
  sources_.clear();

  LOAD_PROPERTY(settings, this, auto_debayer);
  LOAD_PROPERTY(settings, this, auto_apply_color_matrix);

  c_config_setting sources_list =
      settings["sources"];

  if ( sources_list.isList() ) {

    const int n = sources_list.length();
    sources_.reserve(n);

    for ( int i = 0; i < n; ++i ) {

      std::string filename;

      if ( sources_list.get_element(i).get(&filename) && !filename.empty() ) {
        c_input_source::ptr source = c_input_source::create(filename);
        if ( source ) {
          sources_.emplace_back(source);
        }
        else {
          CF_ERROR("c_input_source::create(filename='%s') fails", filename.c_str());
        }
      }
    }
  }


  return true;
}

void c_input_sequence::set_auto_debayer(enum DEBAYER_ALGORITHM algo)
{
  auto_debayer_ = algo;
}

enum DEBAYER_ALGORITHM c_input_sequence::auto_debayer() const
{
  return auto_debayer_;
}

void c_input_sequence::set_auto_apply_color_matrix(bool v)
{
  auto_apply_color_matrix_ = v;
}

bool c_input_sequence::auto_apply_color_matrix() const
{
  return auto_apply_color_matrix_;
}

const std::vector<c_input_source::ptr> & c_input_sequence::sources() const
{
  return sources_;
}

const c_input_source::ptr & c_input_sequence::source(int index) const
{
  return sources_[index];
}


int c_input_sequence::indexof(const c_input_source::ptr & source,
    const std::vector<c_input_source::ptr> & list)
{
  std::vector<c_input_source::ptr>::const_iterator ii =
      std::find(list.begin(), list.end(), source);

  return ii == list.end() ? -1 : ii - list.begin();
}

int c_input_sequence::indexof(const std::string & pathfilename,
    const std::vector<c_input_source::ptr> & list)
{
  std::vector<c_input_source::ptr>::const_iterator ii =
      std::find_if(list.begin(), list.end(),
          [&pathfilename](const c_input_source::ptr & source) -> bool {
            return source->filename() == pathfilename;
          });

  return ii == list.end() ? -1 : ii - list.begin();
}


int c_input_sequence::indexof(const std::string & pathfilename) const
{
  return indexof(pathfilename, sources_);
}

int c_input_sequence::indexof(const c_input_source::ptr & source) const
{
  return indexof(source, sources_);
}


c_input_source::ptr c_input_sequence::source(const std::string & pathfilename) const
{
  const int pos = indexof(pathfilename);
  return pos >= 0 ? sources_[pos] : nullptr;
}


c_input_source::ptr c_input_sequence::add_source(const std::string & pathfilename, int pos)
{
  close();

  c_input_source::ptr source = c_input_source::create(pathfilename);
  if ( !source ) {
    CF_ERROR("c_input_source::create(pathfilename=%s) fails", pathfilename.c_str());
    return nullptr;
  }

  if ( pos < 0 || pos >= sources_.size() ) {
    sources_.emplace_back(source);
  }
  else {
    sources_.insert(sources_.begin() + pos, source);
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
  if ( pos >= 0 && pos < sources_.size() ) {

    c_input_source::ptr p =
        sources_[pos];

    sources_.erase(sources_.begin() + pos);
  }
}


void c_input_sequence::remove_source(const c_input_source::ptr & source)
{
  if ( source ) {

    const std::vector<c_input_source::ptr>::iterator ii =
        std::find(sources_.begin(), sources_.end(), source);

    if ( ii != sources_.end() ) {
      sources_.erase(ii);
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
  sources_.clear();
}

bool c_input_sequence::empty() const
{
  return sources_.empty();
}

bool c_input_sequence::open()
{
  close();

  total_frames_ = 0;

  for ( c_input_source::ptr & s : sources_ ) {
    s->set_global_pos(total_frames_);
    total_frames_ += s->size();
  }

  if ( !open_source(0) ) {
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
  last_pixel_depth_ = 0;
  last_colorid_ = COLORID_UNKNOWN;
  last_color_matrix_ = cv::Matx33f::eye();
  has_last_color_matrix_ = false;

  if ( also_clear ) {
    clear();
  }

}

bool c_input_sequence::open_source(int source_index)
{
  if ( source_index < 0 || source_index >= (int) sources_.size() ) {
    return false;
  }

  if ( !sources_[source_index]->open() ) {
    CF_ERROR("sources_[source_index=%d]->open('%s') fails", source_index,
        sources_[source_index]->filename().c_str());
    return false;
  }

  return true;
}

void c_input_sequence::close_source(int source_index)
{
  if ( source_index >= 0 && source_index < (int) sources_.size() ) {
    sources_[source_index]->close();
  }
}

bool c_input_sequence::seek_current_source(int source_pos)
{
  if ( current_source_ < 0 || current_source_ >= (int) sources_.size() ) {
    return false;
  }
  return sources_[current_source_]->seek(source_pos);
}

int c_input_sequence::size()
{
  return total_frames_;
}

bool c_input_sequence::seek(int global_pos)
{
  int source_pos = -1;
  int required_source_ = -1;

  for ( int i = 0, n = sources_.size(); i < n; ++i ) {
    const c_input_source::ptr & s = sources_[i];
    if ( global_pos >= s->global_pos() && global_pos < s->global_pos() + s->size() ) {
      required_source_ = i;
      source_pos = global_pos - s->global_pos();
    }
  }

  if ( required_source_ < 0 || required_source_ >= (int)sources_.size() )  {
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

c_input_source::ptr c_input_sequence::current_source() const
{
  return current_source_ >= 0 && current_source_ < (int) sources_.size() ? sources_[current_source_] : nullptr;
}

int c_input_sequence::global_pos(int source_index, int source_frame_index) const
{
  if ( !is_open() ) {
    return -1;
  }

  if ( source_index < 0 || source_index >= sources_.size() ) {
    return -1;
  }

  if ( source_frame_index < 0 || source_frame_index >= sources_[source_index]->size() ) {
    return -1;
  }

  return sources_[source_index]->global_pos() + source_frame_index;
}


bool c_input_sequence::read_current_source(cv::Mat & output_frame, cv::Mat * output_mask)
{
  last_pixel_depth_ = 0;
  last_colorid_ = COLORID_UNKNOWN;
  has_last_color_matrix_ = false;

  c_input_source::ptr source = current_source();
  if ( !source ) {
    return false;
  }

  if ( !source->read(output_frame, &last_colorid_, &last_pixel_depth_) ) {
    return false;
  }


  if ( is_bayer_pattern(last_colorid_) ) {
    if ( output_mask ) { // not clear the meaning of alpha maskk with bayer pattern
      output_mask->release();
    }
    if ( auto_debayer_ != DEBAYER_DISABLE && debayer(output_frame, output_frame, last_colorid_, auto_debayer_) ) {
      last_colorid_ = COLORID_BGR;
    }
  }
  else if ( output_mask  ) {

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


  return true;
}

bool c_input_sequence::read(cv::Mat & output_frame, cv::Mat * output_mask)
{
  if ( current_source_ < 0 || current_source_ >= (int) sources_.size() ) {
    return false;
  }

  while ( current_source_ < (int) sources_.size() ) {
    if ( read_current_source(output_frame, output_mask) ) {
      ++current_global_pos_;
      return true;
    }

    close_source(current_source_);
    if ( ++current_source_ < (int) sources_.size() && !open_source(current_source_) ) {
      break;
    }
  }

  return false;
}

enum COLORID c_input_sequence::colorid() const
{
  return last_colorid_;
}

int c_input_sequence::pixel_depth() const
{
  return last_pixel_depth_;
}

const cv::Matx33f & c_input_sequence::color_matrix() const
{
  return last_color_matrix_;
}

bool c_input_sequence::has_color_matrix() const
{
  return has_last_color_matrix_;
}
