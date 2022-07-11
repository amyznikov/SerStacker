/*
 * c_pyrdown_routine.cc
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#include "c_pyrdown_routine.h"

c_pyrdown_routine::c_class_factory c_pyrdown_routine::class_factory;

c_pyrdown_routine::c_pyrdown_routine(int count, bool enabled) :
    base(&class_factory, enabled), count_(count)
{
}


c_pyrdown_routine::ptr c_pyrdown_routine::create(int count, bool enabled)
{
  ptr obj(new this_class(count, enabled));
  return obj;
}

void c_pyrdown_routine::set_count(int v)
{
  count_ = v;
}

int c_pyrdown_routine::count() const
{
  return count_;
}

void c_pyrdown_routine::set_borderType(cv::BorderTypes v)
{
  borderType_ = v;
}

cv::BorderTypes c_pyrdown_routine::borderType() const
{
  return borderType_;
}

bool c_pyrdown_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("count", &count_);
  settings.get("borderType", &borderType_);

  return true;
}

bool c_pyrdown_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("count", count_);
  settings.set("borderType", borderType_);

  return true;
}

bool c_pyrdown_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( count_ > 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();


    for( int i = 0; i < count_ && std::min(image.cols(), image.rows()) > 3; ++i ) {

      cv::pyrDown(image, image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrDown(mask, mask, cv::Size(), borderType_);
      }
    }

    if( !mask.empty() ) {

      if( !trivialMask ) {
        cv::compare(mask, 255, mask, cv::CMP_GE);
      }
      else {
        cv::resize(mask, mask, image.size(), 0, 0, cv::INTER_NEAREST);
      }
    }

  }
  else if( count_ < 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    for( int i = 0; i < -count_ && std::max(image.cols(), image.rows()) < 16000; ++i ) {

      cv::pyrUp(image, image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrUp(mask, mask, cv::Size(), borderType_);
      }

    }

    if( !mask.empty() ) {

      if( !trivialMask ) {
        cv::compare(mask, 255, mask, cv::CMP_GE);
      }
      else {
        cv::resize(mask, mask, image.size(), 0, 0, cv::INTER_NEAREST);
      }
    }
  }

  return true;
}

