/*
 * c_fft_autosharp_routine.cc
 *
 *  Created on: Jun 5, 2026
 *      Author: amyznikov
 *
 */

#include "c_fft_autosharp_routine.h"
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/readdir.h>

void c_fft_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "centerDC", CTL_CONTEXT(ctx, _centerDC), "");
  ctlbind(ctls, CTL_CONTEXT(ctx, opts));
  ctlbind(ctls, "print_debug_info:", CTL_CONTEXT(ctx, debug_opts.print_debug_info), "");
  ctlbind(ctls, "write_debug_file:", CTL_CONTEXT(ctx, debug_opts.write_file), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, debug_opts.debug_file_name), "");
}

bool c_fft_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _centerDC);
    serialize_fft_autosharp_options(settings, save, opts);
    SERIALIZE_OPTION(settings, save, debug_opts, debug_file_name);
    // SERIALIZE_OPTION(settings, save, debug_opts, print_debug_info);
    // SERIALIZE_OPTION(settings, save, debug_opts, write_file);
    return true;
  }
  return false;
}

bool c_fft_autosharp_routine::initialize()
{
  return true;
}

void c_fft_autosharp_routine::state_changed()
{
  if ( !_enabled ) {
    // cleanup cached memory
    autosharp.clearCachedData();
  }
}


bool c_fft_autosharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  INSTRUMENT_REGION("fft_autosharp");

  if ( !autosharp.compute(opts, image, mask, image, mask, _display, &debug_opts) ) {
    CF_ERROR("autosharp.compute() fails");
    return false;
  }

  switch (_display) {
    case FFT_AUTOSHARP_DISPLAY_S_SPECTRUM:
    case FFT_AUTOSHARP_DISPLAY_P_SPECTRUM:
    case FFT_AUTOSHARP_DISPLAY_RESTORED_SPECTRUM:
      fftCCSSpectrumToPolar(image, image, _centerDC);
      break;
    case FFT_AUTOSHARP_DISPLAY_INVERSE_FILTER:
      if ( _centerDC ) {
        fftSwapQuadrants(image, image);
      }
      break;
    default:
      break;
  }

  return true;
}

