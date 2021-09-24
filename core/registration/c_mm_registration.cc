/*
 * c_mm_registration.cc
 *
 *  Created on: Sep 24, 2021
 *      Author: amyznikov
 */

#include "c_mm_registration.h"

c_mm_registration::c_mm_registration()
{
}
c_mm_registration::c_mm_registration(const c_mm_registration_options & mm_options)
  : mm_options_(mm_options)
{

}

c_mm_registration::c_mm_registration(const c_feature_based_registration_options & feature_options,
    const c_mm_registration_options & mm_options)
  : base(feature_options),
    mm_options_(mm_options)
{
}

c_mm_registration::c_mm_registration(const c_frame_registration_base_options & base_options,
    const c_feature_based_registration_options & feature_options,
    const c_mm_registration_options & mm_options)
  : base(base_options, feature_options),
    mm_options_(mm_options)
{
}

c_mm_registration::ptr c_mm_registration::create()
{
  return ptr (new c_mm_registration());
}

c_mm_registration::ptr c_mm_registration::create(const c_mm_registration_options & mm_options)
{
  return ptr (new c_mm_registration(mm_options));
}

c_mm_registration::ptr c_mm_registration::create(const c_feature_based_registration_options & feature_options,
    const c_mm_registration_options & mm_options)
{
  return ptr (new c_mm_registration(feature_options, mm_options));
}

c_mm_registration::ptr c_mm_registration::create(const c_frame_registration_base_options & base_options,
    const c_feature_based_registration_options & feature_options,
    const c_mm_registration_options & mm_options)
{
  return ptr (new c_mm_registration(base_options, feature_options, mm_options));
}

const c_mm_registration_options & c_mm_registration::mm_options() const
{
  return mm_options_;
}

c_mm_registration_options & c_mm_registration::mm_options()
{
  return mm_options_;
}
