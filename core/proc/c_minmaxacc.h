/*
 * c_minmaxacc.h
 *
 *  Created on: Nov 30, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_minmaxacc_h__
#define __c_minmaxacc_h__

#include <vector>
#include <algorithm>

template<class KeyType, class ValueType>
class c_minacc
{
public:
  struct item_type
  {
    KeyType key;
    ValueType value;
  };

  c_minacc(int maxitems) :
    maxitems_(maxitems)
  {
    items_.reserve(maxitems);
  }

  void reset()
  {
    items_.clear();
  }

  void update(const KeyType & key, const ValueType & value)
  {
    const auto pos =
        std::lower_bound(items_.begin(),
            items_.end(),
            key,
            [](const item_type & item, const KeyType & k) {
              return item.key < k;
            });

    if( pos == items_.end() ) {
      if( items_.size() < maxitems_ ) {
        items_.emplace_back(item_type{key, value});
      }
    }
    else {
      items_.insert(pos, item_type{key, value});
      if( items_.size() >= maxitems_ ) {
        items_.pop_back();
      }
    }
  }

  const std::vector<item_type> & items()
  {
    return items_;
  }

  double avgkey() const
  {
    double s = 0;
    for( const auto & item : items_ ) {
      s += item.key;
    }
    return s / items_.size();
  }

  double avgval() const
  {
    double s = 0;
    for( const auto & item : items_ ) {
      s += item.value;
    }
    return s / items_.size();
  }

protected:
 std::vector<item_type> items_;
 const size_t maxitems_;
};


template<class KeyType, class ValueType>
class c_maxacc
{
public:
  struct item_type
  {
    KeyType key;
    ValueType value;
  };

  c_maxacc(int maxitems) :
    maxitems_(maxitems)
  {
    items_.reserve(maxitems);
  }

  void reset()
  {
    items_.clear();
  }

  void update(const KeyType & key, const ValueType & value)
  {
    const auto pos =
        std::lower_bound(items_.begin(),
            items_.end(),
            key,
            [](const item_type & item, const KeyType & k) {
              return k < item.key;
            });

    if( pos == items_.end() ) {
      if( items_.size() < maxitems_ ) {
        items_.emplace_back(item_type{key, value});
      }
    }
    else {
      items_.insert(pos, item_type{key, value});
      if( items_.size() >= maxitems_ ) {
        items_.pop_back();
      }
    }
  }

  const std::vector<item_type> & items()
  {
    return items_;
  }

  double avgkey() const
  {
    double s = 0;
    for( const auto & item : items_ ) {
      s += item.key;
    }
    return s / items_.size();
  }

  double avgval() const
  {
    double s = 0;
    for( const auto & item : items_ ) {
      s += item.value;
    }
    return s / items_.size();
  }

protected:
 std::vector<item_type> items_;
 const size_t maxitems_;
};




#endif /* __c_minmaxacc_h__ */
