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

template<class _Dt>
class c_voxel_grid3d
{
public:

  struct Z
  {
    typedef Z this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;

    _Dt data;
    int idx;
  };

  struct Y
  {
    typedef Y this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;

    std::vector<typename Z::uptr> a;
    int idx;
  };

  struct X
  {
    typedef X this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;
    std::vector<typename Y::uptr> a;
    int idx;
  };

  c_voxel_grid3d()
  {
  }


  typename std::vector<typename X::uptr>::iterator xbeg()
  {
    return _x.begin();
  }

  typename std::vector<typename X::uptr>::iterator xend()
  {
    return _x.end();
  }

  const typename X::uptr& xx(int ix)
  {
    typename std::vector<typename X::uptr>::iterator pos =
        _x.empty() ? _x.end() :
            std::lower_bound(_x.begin(), _x.end(), ix,
                [](const auto & a, int value) {
                  return a->idx < value;
                });

    if( pos == _x.end() || ((*pos)->idx != ix) ) {
      pos = _x.insert(pos, typename X::uptr(new X()));
      (*pos)->idx = ix;
    }

    return *pos;
  }

  const typename Y::uptr& yy(const typename X::uptr & xp, int iy)
  {
    typename std::vector<typename Y::uptr>::iterator pos =
        xp->a.empty() ? xp->a.end() :
            std::lower_bound(xp->a.begin(), xp->a.end(), iy,
                [](const auto & a, int value) {
                  return a->idx < value;
                });

    if( pos == xp->a.end() || ((*pos)->idx != iy) ) {
      pos = xp->a.insert(pos, typename Y::uptr( new Y()));
      (*pos)->idx = iy;
    }

    return *pos;
  }

  const typename Z::uptr& zz(const typename Y::uptr & yp, int iz)
  {
    typename std::vector<typename Z::uptr>::iterator pos =
        yp->a.empty() ? yp->a.end() :
            std::lower_bound(yp->a.begin(), yp->a.end(), iz,
                [](const auto & a, int value) {
                  return a->idx < value;
                });

    if( pos == yp->a.end() || ((*pos)->idx != iz) ) {
      pos = yp->a.insert(pos, typename Z::uptr(new Z()));
      (*pos)->idx = iz;
    }

    return *pos;
  }

  template<class _Fn>
  bool insert(double x, double y, double z, const _Fn & update_data)
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

    update_data(zz(yy(xx(ix), iy), iz)->data);

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
  std::vector<typename X::uptr> _x;

};

#endif /* __c_voxel_grid_h__ */
