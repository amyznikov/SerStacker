/*
 * focus.h
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __focus_h__
#define __focus_h__

#include "sharpness_measure/c_local_contrast_measure.h"
#include "sharpness_measure/c_lpg_sharpness_measure.h"


enum SHARPNESS_MEASURE {
  SHARPNESS_MEASURE_LCM = 0,
  SHARPNESS_MEASURE_LPG = 1,
};

#endif /* __focus_h__ */

