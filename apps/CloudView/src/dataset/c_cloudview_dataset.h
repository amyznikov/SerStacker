/*
 * c_cloudview_dataset.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_dataset_h__
#define __c_cloudview_dataset_h__


#include "c_cloudview_input_sequence.h"
#include <functional>

namespace cloudview {

class c_cloudview_dataset
{
public:
  typedef c_cloudview_dataset this_class;
  typedef std::shared_ptr<this_class> sptr;

  class type
  {
  public:
    template<class Fn>
    type(const std::string & _name, const std::string & _tooltip,
        const std::vector<std::string> & _supported_suffixes,
        const std::string & _filter,
        const Fn & obj_creator) :
        name_(_name),
        tooltip_(_tooltip),
        supported_suffixes_(_supported_suffixes),
        filter_(_filter),
        create_instance_(obj_creator)
    {
    }

    const std::string & name() const
    {
      return name_;
    }

    const std::string & tooltip() const
    {
      return tooltip_;
    }

    const std::vector<std::string> & supported_suffixes() const
    {
      return supported_suffixes_;
    }

    const std::string & file_filter() const
    {
      return filter_;
    }

    sptr create_instance(const std::string & dataset_name) const
    {
      return create_instance_(this, dataset_name);
    }

  protected:
    std::string name_;
    std::string tooltip_;
    std::vector<std::string> supported_suffixes_;
    std::string filter_;
    std::function<sptr (const type *, const std::string &)> create_instance_;
  };


  virtual ~c_cloudview_dataset();

  const type * dataset_type() const;
  const std::string & file_filter() const;

  void set_name(const std::string & v);
  const std::string & name() const;
  const char * cname() const;

  void set_path(const std::string & v);
  const std::string & path() const;
  const char *cpath() const;

  static const std::vector<type> & supported_types();

  static sptr create(const std::string & dataset_type, const std::string & dataset_name = "");

  const c_cloudview_input_sequence::sptr & input_sequence() const;

  const std::vector<c_cloudview_input_source::sptr> & input_sources() const;

  c_cloudview_input_source::sptr add_input_source(const std::string & filename);

  virtual c_cloudview_input_source::sptr create_input_source(const std::string & filename) const = 0;

protected:
  c_cloudview_dataset(const type * dataset_type,
      const std::string & dataset_name);

protected:
  const type * dataset_type_ = nullptr;
  std::string dataset_name_;
  std::string dataset_path_;
  c_cloudview_input_sequence::sptr input_sequence_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_dataset_h__ */
