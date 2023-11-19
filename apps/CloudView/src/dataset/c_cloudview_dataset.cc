/*
 * c_cloudview_dataset.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_dataset.h"
#include "vlo/c_vlo_dataset.h"
#include <mutex>
#include <core/debug.h>

namespace cloudview {

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace {


} // namespace






///////////////////////////////////////////////////////////////////////////////////////////////////

c_cloudview_dataset::c_cloudview_dataset(const type * dataset_type, const std::string & dataset_name) :
    dataset_type_(dataset_type),
    dataset_name_(dataset_name)
{
  input_sequence_.reset(new c_cloudview_input_sequence());
}

c_cloudview_dataset::~c_cloudview_dataset()
{
}

const c_cloudview_dataset::type * c_cloudview_dataset::dataset_type() const
{
  return dataset_type_;
}

const std::string & c_cloudview_dataset::file_filter() const
{
  return dataset_type_->file_filter();
}

void c_cloudview_dataset::set_name(const std::string & v)
{
  dataset_name_ = v;
}

const std::string & c_cloudview_dataset::name() const
{
  return dataset_name_;
}

const char * c_cloudview_dataset::cname() const
{
  return dataset_name_.c_str();
}

void c_cloudview_dataset::set_path(const std::string & v)
{
  dataset_path_ = v;
}

const std::string & c_cloudview_dataset::path() const
{
  return dataset_path_;
}

const char * c_cloudview_dataset::cpath() const
{
  return dataset_path_.c_str();
}

const c_cloudview_input_sequence::sptr & c_cloudview_dataset::input_sequence() const
{
  return input_sequence_;
}

const std::vector<c_cloudview_input_source::sptr> & c_cloudview_dataset::input_sources() const
{
  return input_sequence_->sources();
}

const std::vector<c_cloudview_dataset::type> & c_cloudview_dataset::supported_types()
{
  static std::mutex mtx;
  mtx.lock();

  static std::vector<type> types_;

  if( types_.empty() ) {

    types_.emplace_back("VLO",
        "VLO dataset to process VLO ADTF *.dat files",

        std::vector<std::string>({".dat", ".vsb"}),
          "VLO files (*.dat *.vsb) ;;"
          "All files (*.*)",

        [](const type * _type, const std::string & dataset_name) {
          return sptr(new c_vlo_dataset(_type, dataset_name));
        });
  }

  mtx.unlock();

  return types_;
}

c_cloudview_dataset::sptr c_cloudview_dataset::create(const std::string & dataset_type_name,
    const std::string & dataset_name )
{
  for( const c_cloudview_dataset::type &type : supported_types() ) {
    if( type.name() == dataset_type_name ) {
      return sptr(type.create_instance(dataset_name));
    }
  }

  return nullptr;
}

c_cloudview_input_source::sptr c_cloudview_dataset::add_input_source(const std::string & filename)
{
  c_cloudview_input_source::sptr source =
      create_input_source(filename);

  if ( !source ) {
    CF_ERROR("create_input_source('%s') fails", filename.c_str());
  }
  else {
    input_sequence_->add_source(source);
  }

  return source;
}



} /* namespace cloudview */
