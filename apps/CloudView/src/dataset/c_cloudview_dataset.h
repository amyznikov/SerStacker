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

  struct type
  {
    std::string name;
    std::string tooltip;
    std::vector<std::string> supported_suffixes;
    std::function<sptr ()> create_instance;



    template<class Fn>
    type(const std::string & _name, const std::string & _tooltip,
        const std::vector<std::string> & _supported_suffixes,
        const Fn & obj_creator) :
        name(_name),
        tooltip(_tooltip),
        supported_suffixes(_supported_suffixes),
        create_instance(obj_creator)
    {
    }
  };


  virtual ~c_cloudview_dataset();

  static const std::vector<type> & supported_types();
  static sptr create(const std::string & dataset_type);

  static sptr open(const std::string & abspath);




  const std::string & dataset_path() const;
  const c_cloudview_input_sequence::sptr & input_sequence() const;



protected:
  c_cloudview_dataset(const std::string & dataset_path);

protected:
  std::string dataset_path_;
  c_cloudview_input_sequence::sptr input_sequence_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_dataset_h__ */
