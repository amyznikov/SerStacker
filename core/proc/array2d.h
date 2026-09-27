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
    create(rhs._size);
    std::copy(rhs._p, rhs._p + _size.width * _size.height, this->_p);
    return * this;
  }

  void create(int rows, int cols)
  {
    create(cv::Size(cols, rows));
  }

  void create(const cv::Size & s)
  {
    if( !_p || _size != s ) {

      release();

      _size = s;
      _p = new elem_type[s.height * s.width];
      _pp = new elem_type*[s.height];

      for( int r = 0; r < s.height; ++r ) {
        _pp[r] = _p + r * s.width;
      }
    }
  }

  void release()
  {
    if ( _p  ) {
      delete [] _p, _p = nullptr;
      delete [] _pp, _pp = nullptr;
      _size.width = _size.height = 0;
    }
  }

  // pointer to row pointers
  const elem_type * const * ptr() const
  {
    return _pp;
  }

  // pointer to row pointers
  elem_type ** ptr()
  {
    return _pp;
  }

  const elem_type * operator [](int row) const
  {
    return _pp[row];
  }

  elem_type * operator [](int row)
  {
    return _pp[row];
  }

  elem_type * row(int row)
  {
    return _pp[row];
  }

  const elem_type * row(int row) const
  {
    return _pp[row];
  }

  const cv::Size & size() const
  {
    return _size;
  }

  int rows() const
  {
    return _size.height;
  }

  int cols() const
  {
    return _size.width;
  }

  bool empty() const
  {
    return !_p;
  }

  elem_type& operator ()(int r, int c)
  {
    return _pp[r][c];
  }

  const elem_type& operator ()(int r, int c) const
  {
    return _pp[r][c];
  }

  elem_type & at(int r, int c)
  {
    return _pp[r][c];
  }

  const elem_type & at(int r, int c) const
  {
    return _pp[r][c];
  }

  elem_type * data ()
  {
    return _p;
  }

  const elem_type * data () const
  {
    return _p;
  }

  int data_size() const
  {
    return _size.height * _size.width;
  }

protected:
  cv::Size _size = cv::Size(0, 0);
  elem_type * _p = nullptr; // pointer to whole continuous array
  elem_type ** _pp = nullptr; // pointers to individual rows
};




#endif /* __array2d_h__ */
