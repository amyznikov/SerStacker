/*
 * c_mm_registration.h
 *
 *  Created on: Sep 24, 2021
 *      Author: amyznikov
 */

#ifndef __c_mm_registration_h__
#define __c_mm_registration_h__

#include "c_feature_based_registration.h"

struct c_mm_registration_options {
};


class c_mm_registration
    : public c_feature_based_registration
{
public:
  typedef c_mm_registration this_class;
  typedef c_feature_based_registration base;
  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();
  static this_class::ptr create(const c_mm_registration_options & mm_options);
  static this_class::ptr create(const c_feature_based_registration_options & feature_options,
      const c_mm_registration_options & mm_options);
  static this_class::ptr create(const c_frame_registration_base_options & base_options,
      const c_feature_based_registration_options & feature_options,
      const c_mm_registration_options & mm_options);

  const c_mm_registration_options & mm_options() const;
  c_mm_registration_options & mm_options();

protected: // use create() instead
  c_mm_registration();
  c_mm_registration(const c_mm_registration_options & mm_options);
  c_mm_registration(const c_feature_based_registration_options & feature_options,
      const c_mm_registration_options & mm_options);
  c_mm_registration(const c_frame_registration_base_options & base_options,
      const c_feature_based_registration_options & feature_options,
      const c_mm_registration_options & mm_options);

protected:
  c_mm_registration_options mm_options_;
};

#endif /* __c_mm_registration_h__ */
