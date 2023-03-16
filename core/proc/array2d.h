/*
 * array2d.h
 *
 *  Created on: Mar 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __array2d_h__
#define __array2d_h__

#include <opencv2/opencv.hpp>

template<class T>
class c_array2d
{
public:
  typedef c_array2d this_class;
  typedef T elem_type;

  c_array2d()
  {
  }

  c_array2d(int rows, int cols)
  {
    create(rows, cols);
  }

  c_array2d(const cv::Size & s)
  {
    create(s);
  }

  c_array2d(const c_array2d & rhs)
  {
    this_class :: operator = (rhs);
  }

  ~c_array2d()
  {
    release();
  }

  c_array2d & operator = (const c_array2d & rhs)
  {
    release();
    create(rhs.size_);
    std::copy(rhs.p_, rhs.p_ + size_.width * size_.height, this->p_);
    return * this;
  }

  void create(int rows, int cols)
  {
    create(cv::Size(cols, rows));
  }

  void create(const cv::Size & s)
  {
    release();

    size_ = s;
    p_ = new elem_type[s.height * s.width];
    pp_ = new elem_type * [s.height];

    for ( int r = 0; r < s.height; ++r ) {
      pp_[r] = p_ + r * s.width;
    }
  }

  void release()
  {
    if ( p_  ) {
      delete [] p_, p_ = nullptr;
      delete [] pp_, pp_ = nullptr;
      size_.width = size_.height = 0;
    }
  }

  const elem_type * operator [](int row) const
  {
    return pp_[row];
  }

  elem_type * operator [](int row)
  {
    return pp_[row];
  }

  const cv::Size & size() const
  {
    return size_;
  }

  int rows() const
  {
    return size_.height;
  }

  int cols() const
  {
    return size_.width;
  }

  bool empty() const
  {
    return !p_;
  }

  elem_type & at(int r, int c)
  {
    return pp_[r][c];
  }

  const elem_type & at(int r, int c) const
  {
    return pp_[r][c];
  }

protected:
  cv::Size size_;
  elem_type * p_ = nullptr; // pointer to whole continuous array
  elem_type ** pp_ = nullptr; // pointers to individual rows
};




#endif /* __array2d_h__ */
