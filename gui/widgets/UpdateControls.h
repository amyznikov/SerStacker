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

  struct c_update_controls_lock {
    ThisClass * _this;
    c_update_controls_lock(ThisClass * obj) : _this(obj) {
      _this->setUpdatingControls(true);
    }
    ~c_update_controls_lock() {
      _this->setUpdatingControls(false);
    }
  };


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



#endif /* __UpdateControls_h__ */
