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

c_cloudview_dataset::c_cloudview_dataset(const std::string & dataset_path) :
    dataset_path_(dataset_path)
{
}

c_cloudview_dataset::~c_cloudview_dataset()
{
}

const std::string & c_cloudview_dataset::dataset_path() const
{
  return dataset_path_;
}

const c_cloudview_input_sequence::sptr & c_cloudview_dataset::input_sequence() const
{
  return input_sequence_;
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
        []() {
          return sptr(new c_vlo_dataset());
        });
  }

  mtx.unlock();

  return types_;
}

c_cloudview_dataset::sptr c_cloudview_dataset::create(const std::string & dataset_type_name)
{
  for( const c_cloudview_dataset::type &type : supported_types() ) {
    if( type.name == dataset_type_name ) {
      return type.create_instance();
    }
  }

  return nullptr;
}


c_cloudview_dataset::sptr c_cloudview_dataset::open(const std::string & abspath)
{
  CF_ERROR("c_cloudview_dataset::open() function not implemented");
  return nullptr;
}



} /* namespace cloudview */
