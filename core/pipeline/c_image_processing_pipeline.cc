/*
 * c_image_processing_pipeline.cc
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 */

#include "c_image_processing_pipeline.h"
#include <core/io/save_image.h>
#include <core/readdir.h>
#include <core/debug.h>


namespace {

bool get_data_range_for_pixel_depth(int ddepth, double * minval, double * maxval)
{
  switch (CV_MAT_DEPTH(ddepth)) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}

/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool get_scale_offset(int src_depth, int dst_depth, double * scale, double * offset)
{
  double srcmin, srcmax;
  double dstmin, dstmax;

  get_data_range_for_pixel_depth(src_depth, &srcmin, &srcmax);
  get_data_range_for_pixel_depth(dst_depth, &dstmin, &dstmax);

  *scale = (dstmax - dstmin) / (srcmax - srcmin);
  *offset = dstmin - *scale * srcmin;

  return true;
}

} // namespace
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<c_image_processing_pipeline::factory_item> c_image_processing_pipeline::registered_classes_;

c_image_processing_pipeline::factory_item::factory_item(const std::string & _class_name, const std::string & _tooltip,
    const c_image_processing_pipeline::instance_creator & _create_instance) :
    class_name(_class_name),
    tooltip(_tooltip),
    create_instance(_create_instance)
{
}

const std::vector<c_image_processing_pipeline::factory_item>& c_image_processing_pipeline::registered_classes()
{
  return registered_classes_;
}

const c_image_processing_pipeline::factory_item* c_image_processing_pipeline::find_class(const std::string & class_name)
{
  for( const auto &item : registered_classes_ ) {
    if( item.class_name == class_name ) {
      return &item;
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

  registered_classes_.emplace_back(class_name, tooltip, create_instance);
  return true;
}

c_image_processing_pipeline::sptr c_image_processing_pipeline::create_instance(const std::string & class_name,
    const std::string & name, const c_input_sequence::sptr & input_sequence)
{
  const auto *item = find_class(class_name);
  if( !item ) {
    CF_ERROR("c_image_processing_pipeline::class_factory: class '%s' not registered", class_name.c_str());
    return nullptr;
  }

  return item->create_instance(name, input_sequence);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_processing_pipeline::c_image_processing_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    name_(name),
    input_sequence_(input_sequence)
{
}

c_image_processing_pipeline::~c_image_processing_pipeline()
{
}

void c_image_processing_pipeline::set_name(const std::string & name)
{
  name_ = name;
}

const std::string& c_image_processing_pipeline::name() const
{
  return name_;
}

const char* c_image_processing_pipeline::cname() const
{
  return name_.c_str();
}

void c_image_processing_pipeline::set_sequence_name(const std::string & v)
{
  sequence_name_ = v;
}

const std::string& c_image_processing_pipeline::sequence_name() const
{
  return sequence_name_;
}

const char * c_image_processing_pipeline::csequence_name() const
{
  return sequence_name_.c_str();
}

void c_image_processing_pipeline::set_output_directory(const std::string & output_directory)
{
  output_directory_ = output_directory;
  update_output_path();
}

const std::string& c_image_processing_pipeline::output_directory() const
{
  return output_directory_;
}

void c_image_processing_pipeline::set_master_source(const std::string & master_source_path)
{
  master_source_ = master_source_path;
}

const std::string& c_image_processing_pipeline::master_source() const
{
  return master_source_;
}

void c_image_processing_pipeline::set_master_frame_index(int v)
{
  master_frame_index_ = v;
}

int c_image_processing_pipeline::master_frame_index() const
{
  return master_frame_index_;
}

const c_input_sequence::sptr& c_image_processing_pipeline::input_sequence() const
{
  return input_sequence_;
}

void c_image_processing_pipeline::cancel(bool v)
{
  canceled_ = v;
}

bool c_image_processing_pipeline::canceled() const
{
  return canceled_;
}

void c_image_processing_pipeline::set_pipeline_stage(int newstage)
{
  const auto oldstage =
      pipeline_stage_;

  if( newstage != oldstage ) {
    pipeline_stage_ = newstage;
    on_pipeline_stage_changed(oldstage, newstage);
  }
}

int c_image_processing_pipeline::pipeline_stage() const
{
  return pipeline_stage_;
}

void c_image_processing_pipeline::set_status_msg(const std::string & msg) const
{
  if( true ) {
    lock_guard lock(status_lock_);
    statusmsg_ = msg;
  }

  CF_DEBUG("STATUS: %s", msg.c_str());
  on_status_msg_changed(statusmsg_);
}

std::string c_image_processing_pipeline::status_message() const
{
  std::string msg;
  if( true ) {
    lock_guard lock(status_lock_);
    msg = statusmsg_;
  }
  return msg;
}

int c_image_processing_pipeline::total_frames() const
{
  return total_frames_;
}

int c_image_processing_pipeline::processed_frames() const
{
  return processed_frames_;
}

int c_image_processing_pipeline::accumulated_frames() const
{
  return accumulated_frames_;
}

void c_image_processing_pipeline::update_output_path()
{
  if( output_directory_.empty() ) {

    std::string parent_directory =
        get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            cname());

  }
  else if( !is_absolute_path(output_directory_) ) {

    std::string parent_directory =
        get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            output_directory_.c_str());
  }

  if( output_path_.empty() ) {
    output_path_ =
        ssprintf("./%s",
            cname());
  }

}

void c_image_processing_pipeline::gather_badframe_indexes()
{
  badframes_.clear();

  if( input_sequence_ ) {

    const bool was_open = input_sequence_->is_open();
    if( !was_open && !input_sequence_->open() ) {
      CF_ERROR("input_sequence_->open() fails");
      return;
    }

    const std::vector<c_input_source::sptr> &sources =
        input_sequence_->sources();

    for( uint source_index = 0, n = sources.size(); source_index < n; ++source_index ) {

      const c_input_source::sptr source =
          input_sequence_->source(source_index);

      if( source ) {

        const std::vector<uint> &bad_source_frames =
            source->load_badframes();

        for( uint source_frame_index : bad_source_frames ) {

          const int global_index =
              input_sequence_->global_pos(source_index,
                  source_frame_index);

          if( global_index >= 0 ) {
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

bool c_image_processing_pipeline::is_bad_frame_index(int global_pos) const
{
  if( !badframes_.empty() ) {

    const std::vector<uint>::const_iterator pos =
        std::find(badframes_.begin(),
            badframes_.end(),
            global_pos);

    return pos != badframes_.end();
  }

  return false;
}


bool c_image_processing_pipeline::serialize(c_config_setting setting, bool save)
{
  if( save ) {
    save_settings(setting, "class_name", get_class_name());
  }

  SERIALIZE_PROPERTY(setting, save, *this, name);
  SERIALIZE_PROPERTY(setting, save, *this, sequence_name);
  SERIALIZE_PROPERTY(setting, save, *this, output_directory);
  SERIALIZE_PROPERTY(setting, save, *this, master_source);
  SERIALIZE_PROPERTY(setting, save, *this, master_frame_index);

  return true;
}

bool c_image_processing_pipeline::run()
{

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

    CF_ERROR("OpenCV Exception catched in c_image_processing_pipeline::run():\n"
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

  return fOk;
}


bool c_image_processing_pipeline::initialize_pipeline()
{
  CF_DEBUG("Initializing '%s: %s'...", csequence_name(), cname());

  cancel(false);

  if ( !input_sequence_ || input_sequence_->empty() ) {
    set_status_msg("ERROR: empty input sequence specified");
    return false;
  }

  total_frames_ = 0;
  processed_frames_ = 0;
  accumulated_frames_ = 0;
  statusmsg_.clear();


  update_output_path();

  gather_badframe_indexes();

  return true;
}

void c_image_processing_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }
}

bool c_image_processing_pipeline::run_pipeline()
{
  CF_ERROR("c_image_processing_pipeline: Abstract run_pipeline() called");
  return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_sequence::c_image_sequence(const std::string & name) :
    name_(name)
{
}

void c_image_sequence::set_name(const std::string & name)
{
  name_ = name;

  for( const auto &pipeline : pipelines_ ) {
    if( pipeline ) {
      pipeline->set_sequence_name(name);
    }
  }
}

const std::string& c_image_sequence::name() const
{
  return name_;
}

const char* c_image_sequence::cname() const
{
  return name_.c_str();
}

std::string c_image_sequence::displaypatch() const
{
  std::string path;

  if( input_sequence_ && input_sequence_->sources().size() > 0 ) {
    path = get_parent_directory(input_sequence_->source(0)->filename());
  }
  else if( current_pipeline_ ) {
    path = current_pipeline_->output_directory();
  }

  return path;

}

const c_input_sequence::sptr& c_image_sequence::input_sequence() const
{
  return input_sequence_;
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

bool c_image_sequence::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  SERIALIZE_PROPERTY(settings, save, *this, name);

  if( save ) {

    if( input_sequence_ ) {
      input_sequence_->serialize(section =
          settings.add_group("input_sequence"));
    }

    if( current_pipeline_ ) {
      save_settings(settings, "current_pipeline",
          current_pipeline_->name());
    }

    if( !pipelines_.empty() ) {

      section = settings.add_list("pipelines");
      for( const c_image_processing_pipeline::sptr &pipeline : pipelines_ ) {
        if( pipeline ) {
          pipeline->serialize(section.add_group(), save);
        }
      }
    }

  }

  else {

    std::string class_name, object_name;

    pipelines_.clear();
    current_pipeline_.reset();

    if( (section = settings["input_sequence"]) ) {
      input_sequence_ = c_input_sequence::create();
      input_sequence_->deserialize(section);
    }

    CF_DEBUG("load pipelines");

    if( (section = settings["pipelines"]) && section.isList() ) {

      const int n =
          section.length();

      for( int i = 0; i < n; ++i ) {

        c_config_setting item =
            section[i];

        if( !item.isGroup() ) {
          CF_ERROR("pipeline item %d is not a libconfig group", i);
          continue;
        }

        if( !load_settings(item, "class_name", &class_name) || class_name.empty() ) {
          CF_ERROR("can not extract pipeline class name for libconfig item %d", i);
          continue;
        }

        if( !load_settings(item, "name", &object_name) || object_name.empty() ) {
          CF_ERROR("can not extract pipeline object name for libconfig item %d of class '%s'", i,
              class_name.c_str());
          continue;
        }

        c_image_processing_pipeline::sptr pipeline =
            c_image_processing_pipeline::create_instance(class_name, object_name, input_sequence_);

        if( !pipeline ) {
          CF_ERROR("c_image_processing_pipeline::create_instance(class_name='%s' object_name=%s) fails",
              class_name.c_str(), object_name.c_str());
          continue;
        }

        if( !pipeline->serialize(item, save) ) {
          CF_ERROR("pipeline->serialize(class='%s', name='%s', save=false) fails for item %d",
              class_name.c_str(), object_name.c_str(), i);
          continue;
        }

        pipelines_.emplace_back(pipeline);
      }
    }

    if( load_settings(settings, "current_pipeline", &object_name) && !object_name.empty() ) {
      set_current_pipeline(object_name);
    }

  }

  return true;
}

c_image_sequence::sptr c_image_sequence::load(const std::string & filename)
{
  CF_ERROR("FIXME: not implemented");
  return nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::string c_image_sequence_collection::default_config_filename_ =
    "~/.config/SerStacker/corrent_work.cfg";

c_image_sequence_collection::c_image_sequence_collection()
{
}

size_t c_image_sequence_collection::size() const
{
  return this->items_.size();
}

const std::vector<c_image_sequence::sptr>& c_image_sequence_collection::items() const
{
  return this->items_;
}

c_image_sequence::sptr c_image_sequence_collection::item(size_t index) const
{
  return index < items_.size() ? items_[index] : nullptr;
}

c_image_sequence::sptr c_image_sequence_collection::item(const std::string & name) const
{
  ssize_t index = indexof(name);
  return index >= 0 ? items_[index] : nullptr;
}

void c_image_sequence_collection::add(const c_image_sequence::sptr & sequence)
{
  items_.emplace_back(sequence);
}

bool c_image_sequence_collection::remove(const c_image_sequence::sptr & sequence)
{
  const size_t original_size = items_.size();
  if( original_size > 0 ) {
    items_.erase(std::remove(items_.begin(), items_.end(), sequence), items_.end());
  }
  return items_.size() < original_size;
}

void c_image_sequence_collection::set(int pos, const c_image_sequence::sptr & sequence)
{
  if( pos < 0 || pos >= (int) (items_.size()) ) {
    items_.emplace_back(sequence);
  }
  else {
    items_[pos] = sequence;
  }
}

ssize_t c_image_sequence_collection::indexof(const c_image_sequence::sptr & sequence) const
{
  std::vector<c_image_sequence::sptr>::const_iterator ii =
      std::find(items_.begin(), items_.end(), sequence);
  return ii == items_.end() ? -1 : ii - items_.begin();
}

ssize_t c_image_sequence_collection::indexof(const std::string & name) const
{
  std::vector<c_image_sequence::sptr>::const_iterator ii =
      std::find_if(items_.begin(), items_.end(),
          [&name](const c_image_sequence::sptr & pipeline) -> bool {
            return pipeline && strcasecmp(pipeline->name().c_str(), name.c_str()) == 0;
          });
  return ii == items_.end() ? -1 : ii - items_.begin();
}

const std::string& c_image_sequence_collection::default_config_filename()
{
  return default_config_filename_;
}

void c_image_sequence_collection::set_default_config_filename(const std::string & v)
{
  default_config_filename_ = v;
}

bool c_image_sequence_collection::save(const std::string & cfgfilename) const
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for c_image_processing_pipeline_collection::save()");
    return false;
  }

  CF_DEBUG("Saving '%s' ...", filename.c_str());

  c_config cfg(filename);

  time_t t = time(0);

  if( !save_settings(cfg.root(), "object_class", std::string("c_image_processing_pipeline_collection")) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  c_config_setting section =
      cfg.root().add_list("items");

  for( const c_image_sequence::sptr &sequence : items_ ) {
    if( sequence && !sequence->serialize(section.add_group(), true) ) {
      CF_ERROR("sequence->serialize() fails for sequence '%s'", sequence->cname());
    }
  }

  if( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  config_filename_ = filename;

  return true;
}

bool c_image_sequence_collection::load(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for c_image_processing_pipeline_collection::load()");
    return false;
  }

  // CF_DEBUG("Loading '%s' ...", filename.c_str());

  c_config cfg(filename);

  if( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return false;
  }

  std::string object_class;
  if( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("[%s] load_settings(object_class) fails", filename.c_str());
    return false;
  }

  if( object_class != "c_image_processing_pipeline_collection" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return false;
  }

  c_config_setting section =
      cfg.root().get("items");

  if( !section || !section.isList() ) {
    CF_FATAL("section 'items' is not found in file '%s''",
        filename.c_str());
    return false;
  }

  const int n =
      section.length();

  items_.clear();
  items_.reserve(n);

  for( int i = 0; i < n; ++i ) {

    c_config_setting item =
        section.get_element(i);

    if( item && item.isGroup() ) {

      c_image_sequence::sptr sequence(new c_image_sequence());
      if( !sequence->serialize(item, false) ) {
        CF_ERROR("sequence->serialize() fails for item index %d", i);
      }
      else {
        items_.emplace_back(sequence);
      }
    }
  }

  config_filename_ = filename;

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using c_video_writer =
    c_image_processing_pipeline::c_video_writer;

c_video_writer::~c_video_writer()
{
  close();
}

bool c_video_writer::is_open() const
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

bool c_video_writer::open(const std::string & filename, const cv::Size & frameSize, bool color,
    bool write_frame_mapping)
{
  output_file_name = filename;
  output_type = output_type_unknown;
  current_frame_index = 0;

  if ( filename.empty() ) {
    CF_ERROR("c_video_writer: No output file name specified");
    return false;
  }

  if( !create_path(get_parent_directory(filename)) ) {
    CF_ERROR("c_video_writer: create_path('%s') fails: %s",
        filename.c_str(),
        strerror(errno));
    return false;
  }

  switch (c_input_source::suggest_source_type(filename)) {
    case c_input_source::SER: {

      serVideo.create(filename, frameSize.width, frameSize.height,
          color ? COLORID_BGR : COLORID_MONO,
          16);

      if( !serVideo.is_open() ) {
        CF_ERROR("Can not write ser file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_ser;
      break;
    }

    case c_input_source::MOVIE: {

      aviVideo.open(filename, cv::CAP_FFMPEG,
          cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'),
          10,
          frameSize,
          color);

      if( !aviVideo.isOpened() ) {
        CF_ERROR("Can not write aligned video file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_video;
      break;
    }

    case c_input_source::REGULAR_IMAGE: {

      output_type = output_type_images;
      break;
    }

    default: {
      CF_ERROR("NOOT SUPPORTED output format requested for file '%s'", filename.c_str());
      return false;
    }
  }

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  if( write_frame_mapping ) {

    std::string mapfilename =
        ssprintf("%s.map.txt", filename.c_str());

    if( !(frame_mapping_fp = fopen(mapfilename.c_str(), "w")) ) {
      CF_ERROR("fopen('%s') fails : %s", strerror(errno));
    }
    else {
      fprintf(frame_mapping_fp, "seqidx\tfrmidx\n");
    }
  }

  return true;
}

bool c_video_writer::write(cv::InputArray currenFrame, cv::InputArray currentMask, bool with_alpha_mask, int seqindex)
{
  static const auto convert =
      [](cv::InputArray src, cv::OutputArray dst, int dst_depth) {
        double scale = 1;
        double offset = 0;
        get_scale_offset(src.depth(), dst_depth, &scale, &offset);
        src.getMat().convertTo(dst, dst_depth, scale, offset);
      };


  switch (output_type) {
    case output_type_video:
      if( aviVideo.isOpened() ) {
        if( currenFrame.depth() == CV_8U ) {
          aviVideo.write(currenFrame);
        }
        else {
          convert(currenFrame, tmp, CV_8U);
          aviVideo.write(tmp);
        }
      }
      break;

    case output_type_ser:
      if( serVideo.is_open() ) {
        if ( currenFrame.depth() == CV_16U ) {
          serVideo.write(currenFrame);
        }
        else {
          convert(currenFrame, tmp, CV_16U);
          serVideo.write(tmp);
        }
      }
      break;

    case output_type_images: {

      std::string fname =
          output_file_name;

      const std::string suffix =
          get_file_suffix(fname);

      set_file_suffix(fname, ssprintf("-%06d%s",
          current_frame_index,
          suffix.c_str()));

      if( with_alpha_mask ) {
        if( !save_image(currenFrame, currentMask, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }
      else {
        if( !save_image(currenFrame, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }

      break;
    }

    default:
      CF_ERROR("ERROR: Output video file is not open");
      return false;
  }

  if( frame_mapping_fp ) {
    fprintf(frame_mapping_fp, "%5d\t%d\n",
        seqindex, current_frame_index);
  }

  current_input_sequence_index = seqindex;
  ++current_frame_index;
  return true;
}

void c_video_writer::close()
{
  aviVideo.release();
  serVideo.close();
  tmp.release();

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  output_type = output_type_unknown;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
