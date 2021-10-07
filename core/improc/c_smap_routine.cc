/*
 * c_smap_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_smap_routine.h"

c_smap_routine::c_class_factory c_smap_routine::class_factory;

c_smap_routine::c_smap_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_smap_routine::ptr c_smap_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}

c_smap_routine::ptr c_smap_routine::create(int lksize, int scale_size, double minv, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_lksize(lksize);
  obj->set_scale_size(scale_size);
  obj->set_minv(minv);
  return obj;
}


void c_smap_routine::set_lksize(int v)
{
  lksize_ = v;
}

int c_smap_routine::lksize() const
{
  return lksize_;
}


void c_smap_routine::set_scale_size(int v)
{
  scale_size_ = v;
}

int c_smap_routine::scale_size() const
{
  return scale_size_;
}

void c_smap_routine::set_minv(double v)
{
  minv_ = v;
}

double c_smap_routine::minv() const
{
  return minv_;
}


bool c_smap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1f smap;
  compute_smap(image, smap, lksize_, scale_size_, minv_);
  image.move(smap);
  return true;
}

bool c_smap_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("lksize", &lksize_);
  settings.get("scale_size", &scale_size_);
  settings.get("minv", &minv_);

  return true;
}

bool c_smap_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }


  settings.set("lksize", lksize_);
  settings.set("scale_size", scale_size_);
  settings.set("minv", minv_);

  return true;
}
