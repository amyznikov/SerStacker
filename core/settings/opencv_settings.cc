/*
 * opencv_settings.cc
 *
 *  Created on: Feb 12, 2022
 *      Author: amyznikov
 */

#include "opencv_settings.h"



bool load_settings(c_config_setting settings, cv::TermCriteria * t)
{
  c_config_setting p;

  if ( (p = settings["maxCount"]) ) {
    if ( load_settings(p, &t->maxCount) ) {
      t->type |= cv::TermCriteria::COUNT;
    }
    else {
      CF_ERROR("Syntax error in TermCriteria::maxCount value");
      return false;
    }
  }

  if ( (p = settings["eps"]) ) {
    if ( load_settings(p, &t->epsilon) ) {
      t->type |= cv::TermCriteria::EPS;
    }
    else {
      CF_ERROR("Syntax error in TermCriteria::epsilon value");
      return false;
    }
  }

  return true;
}

bool save_settings(c_config_setting settings, const cv::TermCriteria & t)
{
  if ( t.type & cv::TermCriteria::COUNT ) {
    save_settings(settings, "maxCount", t.maxCount);
  }

  if ( t.type & cv::TermCriteria::EPS ) {
    save_settings(settings, "eps", t.epsilon);
  }

  return true;
}
