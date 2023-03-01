/*
 * notification.h
 *
 *  Created on: Nov 9, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_notification_h__
#define __c_notification_h__

#include <vector>
#include <algorithm>
#include <memory>
#include <functional>
#include <mutex>

template<typename _Signature>
class c_notification;
typedef std::shared_ptr<void> c_slotptr;

template<typename... _ArgTypes>
class c_notification<void(_ArgTypes...)>
{
public:
  typedef c_notification  this_class;
  typedef std::function<void(_ArgTypes...)> slot;
  typedef std::shared_ptr<slot> slotptr;

  template<class some_lambda>
  c_slotptr add(const some_lambda & fn)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    slots_.emplace_back(std::make_shared<slot>(fn));
    return slots_.back();
  }

  int remove(c_slotptr & _slot, bool reset = true)
  {
    int c = 0;

    if( _slot ) {

      std::lock_guard<std::mutex> lock(mtx_);

      auto pos = std::find(slots_.begin(), slots_.end(), _slot);
      for( ; pos != slots_.end(); ++c ) {
        slots_.erase(pos);
        pos = std::find(slots_.begin(), slots_.end(), _slot);
      }
    }

    if( reset && c ) {
      _slot.reset();
    }

    return c;
  }

  void operator () (_ArgTypes... args) const
  {
    for ( const auto & slot : slots_ ) {
      if ( *slot ) {
        (*slot)(args...);
      }
    }
  }

protected:
  std::vector<slotptr> slots_;
  mutable std::mutex mtx_;
};


#endif /* __c_notification_h__ */
