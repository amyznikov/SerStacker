/*
 * UpdateControls.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __UpdateControls_h__
#define __UpdateControls_h__

class HasUpdateControls
{
public:
  typedef HasUpdateControls ThisClass;

  void setUpdatingControls(bool v)
  {
    if( v ) {
      ++updatingControls_;
    }
    else if( updatingControls_ && --updatingControls_ < 0 ) {
      updatingControls_ = 0;
    }
  }

  bool updatingControls() const
  {
    return updatingControls_ > 0;
  }

  void updateControls()
  {
    setUpdatingControls(true);
    onupdatecontrols();
    setUpdatingControls(false);
  }


  virtual ~HasUpdateControls() = default;

protected:
  virtual void onupdatecontrols() {};

protected:
  int updatingControls_ = 0;
};

struct c_update_controls_lock
{
  HasUpdateControls * _this;
  c_update_controls_lock(HasUpdateControls * obj) : _this(obj) {
    if ( _this ) {
      _this->setUpdatingControls(true);
    }
  }
  ~c_update_controls_lock() {
    if ( _this ) {
      _this->setUpdatingControls(false);
    }
  }
};




#endif /* __UpdateControls_h__ */
