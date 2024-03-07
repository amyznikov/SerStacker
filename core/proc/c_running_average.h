/*
 * c_running_average.h
 *
 *  Created on: Mar 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_running_average_h__
#define __c_running_average_h__

/**
 * The running mean estimation
 *
 * */
template<class T = double>
class c_running_average
{
public:

  void set_window_size(int v)
  {
    window_size_ = v;
  }

  int window_size() const
  {
    return window_size_;
  }

  T value() const
  {
    return value_;
  }

  void reset()
  {
    value_ = 0;
    pts_ = 0;
  }

  void update(T value)
  {
    if( pts_ < window_size_ ) {
      value_ = (value_ * pts_ + value) / (pts_ + 1);
    }
    else {
      value_ = (value_ * window_size_ + value) / (window_size_ + 1);
    }

    ++pts_;
  }


protected:
  T value_ = 0;
  int pts_ = 0;
  int window_size_ = 5;
};

#endif /* __c_running_average_h__ */
