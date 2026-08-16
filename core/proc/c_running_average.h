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
    _window_size = v;
  }

  int window_size() const
  {
    return _window_size;
  }

  T value() const
  {
    return _value;
  }

  void reset()
  {
    _value = 0;
    _pts = 0;
  }

  void update(T value)
  {
    if( _pts < _window_size ) {
      _value = (_value * _pts + value) / (_pts + 1);
    }
    else {
      _value = (_value * _window_size + value) / (_window_size + 1);
    }

    ++_pts;
  }


protected:
  T _value = 0;
  int _pts = 0;
  int _window_size = 5;
};

#endif /* __c_running_average_h__ */
