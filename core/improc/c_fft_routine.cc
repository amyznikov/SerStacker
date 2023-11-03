/*
 * c_fft_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_fft_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_fft_routine::DisplayType>()
{

  static constexpr c_enum_member members[] = {
      {c_fft_routine::DisplayPower, "Power", "Display power spectrum"},
      {c_fft_routine::DisplayPhase, "Phase", "Display phase spectrum"},
      {c_fft_routine::DisplayReal, "Real", "Display Real part of spectrum"},
      {c_fft_routine::DisplayImag, "Imag", "Display Imaginary part of spectrum"},
      {c_fft_routine::DisplayPower},
  };

  return members;
}
