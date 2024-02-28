/*
 * c_hdl_processor_routine.h
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_processor_routine_h__
#define __c_hdl_processor_routine_h__

#include <core/dataproc/c_data_frame_processor.h>
#include <core/io/hdl/c_hdl_data_frame.h>

class c_hdl_processor_routine :
    public c_data_frame_processor_routine
{
public:
  typedef c_hdl_processor_routine this_class;
  typedef c_data_frame_processor_routine base;
  typedef std::shared_ptr<this_class> sptr;

  void set_output_name(const std::string & v)
  {
    output_name_ = v;
  }

  const std::string& output_name() const
  {
    return output_name_;
  }

  bool process(c_data_frame::sptr & dataframe) override;
  virtual bool process(c_hdl_data_frame * vlo) = 0;

protected:
  c_hdl_processor_routine(const class_factory * _class_factory, bool enabled = true) :
    base(_class_factory, enabled)
  {
  }

protected:
  std::string output_name_;
};


#define DECLARE_HDL_PROCESSOR_CLASS_FACTORY(class_name, display_name, tooltip) \
    typedef class_name this_class; \
    typedef std::shared_ptr<this_class> sptr; \
    typedef c_hdl_processor_routine base; \
    struct c_class_factory : public base::class_factory { \
      c_class_factory() : \
        base::class_factory(#class_name, \
            display_name, \
            tooltip , \
            factory([]() {\
                return sptr(new this_class());\
         })) {} \
    }; \
    \
    static const c_class_factory* class_factory_instance() { \
      static c_class_factory class_factory_instance_; \
      return &class_factory_instance_; \
    } \
    class_name(bool enabled = false) : \
      base(class_factory_instance(), enabled) { \
    } \
    static sptr create(bool enabled = false) { \
        return sptr(new this_class(enabled)); \
    } \

#endif /* __c_hdl_processor_routine_h__ */
