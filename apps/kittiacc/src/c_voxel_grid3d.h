/*
 * c_voxel_grid.h
 *
 *  Created on: Aug 7, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_voxel_grid_h__
#define __c_voxel_grid_h__

#include <algorithm>
#include <vector>
#include <memory>

template<class _ValueType>
class c_voxel_grid3d
{
public:
  typedef c_voxel_grid3d this_class;
  typedef _ValueType ValueType;

  template<class _It>
  static _It lower_bound(_It beg, _It end, int idx)
  {
    return std::lower_bound(beg, end, idx,
        [](const auto & a, int value) {
          return a->idx < value;
        });
  }

  struct ZDimension
  {
    typedef ZDimension this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;

    ZDimension(int _idx) :
      idx(_idx)
    {
    }

    _ValueType value;
    int idx;
  };

  struct YDimension
  {
    typedef YDimension this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;
    typedef typename ZDimension::uptr zptr;
    typedef typename std::vector<zptr>::iterator iterator;
    typedef typename std::vector<zptr>::const_iterator const_iterator;

    YDimension(int _idx) :
      idx(_idx)
    {
    }

    iterator begin()
    {
      return dim.begin();
    }

    const_iterator begin() const
    {
      return dim.begin();
    }

    iterator end()
    {
      return dim.end();
    }

    const_iterator end() const
    {
      return dim.end();
    }

    iterator lower_bound(int idx)
    {
      return c_voxel_grid3d::lower_bound(begin(), end(), idx);
    }

    std::vector<zptr> dim;
    int idx;
  };

  struct XDimension
  {
    typedef XDimension this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;
    typedef typename YDimension::uptr yptr;
    typedef typename std::vector<yptr>::iterator iterator;
    typedef typename std::vector<yptr>::const_iterator const_iterator;

    XDimension(int _idx) :
      idx(_idx)
    {
    }

    iterator begin()
    {
      return dim.begin();
    }

    const_iterator begin() const
    {
      return dim.begin();
    }

    iterator end()
    {
      return dim.end();
    }

    const_iterator end() const
    {
      return dim.end();
    }

    iterator lower_bound(int idx)
    {
      return c_voxel_grid3d::lower_bound(begin(), end(), idx);
    }


    std::vector<yptr> dim;
    int idx;
  };

  typedef typename XDimension::uptr xptr;
  typedef typename YDimension::uptr yptr;
  typedef typename ZDimension::uptr zptr;

  typedef typename std::vector<xptr>::iterator xiterator;
  typedef typename std::vector<xptr>::const_iterator const_xiterator;

  typedef typename XDimension::iterator yiterator;
  typedef typename XDimension::const_iterator const_yiterator;

  typedef typename YDimension::iterator ziterator;
  typedef typename YDimension::const_iterator const_ziterator;

  xiterator begin()
  {
    return dim.begin();
  }

  const_xiterator begin() const
  {
    return dim.begin();
  }

  xiterator end()
  {
    return dim.end();
  }

  const_xiterator end() const
  {
    return dim.end();
  }

  xiterator lower_bound(int idx)
  {
    return lower_bound(begin(), end(), idx);
  }

  const_xiterator lower_bound(int idx) const
  {
    return lower_bound(begin(), end(), idx);
  }

  const xptr & getitem(int ix)
  {
    xiterator pos = lower_bound(ix);
    if( pos == end() || (*pos)->idx != ix ) {
      pos = dim.insert(pos, xptr(new XDimension(ix)));
    }
    return *pos;
  }

  const yptr & getitem(const xptr & xp, int iy) const
  {
    yiterator pos = xp->lower_bound(iy);
    if( pos == xp->end() || (*pos)->idx != iy ) {
      pos = xp->dim.insert(pos, yptr(new YDimension(iy)));
    }
    return *pos;
  }

  const zptr & getitem(const yptr & yp, int iz) const
  {
    ziterator pos = yp->lower_bound(iz);
    if( pos == yp->end() || (*pos)->idx != iz ) {
      pos = yp->dim.insert(pos, zptr(new ZDimension(iz)));
    }
    return *pos;
  }

  template<class _Fn>
  bool insert(double x, double y, double z, const _Fn & update_value)
  {
    if ( x < _xmin || x > _xmax ) {
      return false;
    }

    if ( y < _ymin || y > _ymax ) {
      return false;
    }

    if ( z < _zmin || z > _zmax ) {
      return false;
    }

    const int ix =
        (int) ((x - _xmin) / _cellsize);

    const int iy =
        (int) ((y - _ymin) / _cellsize);

    const int iz =
        (int) ((z - _zmin) / _cellsize);


    update_value(getitem(getitem(getitem(ix), iy), iz)->value);

    //update_data(zz(yy(xx(ix), iy), iz)->data);

    return true;
  }



protected:
  double _xmin = -500;
  double _xmax = +500;
  double _ymin = -500;
  double _ymax = +500;
  double _zmin = -10;
  double _zmax = +20;
  double _cellsize = 0.1;
  std::vector<xptr> dim;

};

#endif /* __c_voxel_grid_h__ */
