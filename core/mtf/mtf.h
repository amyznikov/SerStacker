/*
 * mtf.h
 *
 *  Created on: Mar 6, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __core_mtf_h__
#define __core_mtf_h__


#if 1
#include "c_rational_scurve_mtf.h"
using c_mtf = c_rational_scurve_mtf;
using c_mtf_options = c_rational_scurve_mtf::Options;
#else
#include "c_smooth_rational_mtf.h"
using c_mtf = c_smooth_rational_mtf;
using c_mtf_options = c_smooth_rational_mtf::Options;
#endif

#endif /* __core_mtf_h__ */
