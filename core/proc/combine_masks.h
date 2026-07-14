/*
 * combine_masks.h
 *
 *  Created on: Jul 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __combine_masks_h__
#define __combine_masks_h__

#include <opencv2/opencv.hpp>

enum COMBINE_MASK_MODE {
  COMBINE_MASK_MODE_KEEP,
  COMBINE_MASK_MODE_REPLACE,
  COMBINE_MASK_MODE_AND,
  COMBINE_MASK_MODE_OR,
  COMBINE_MASK_MODE_XOR,
  COMBINE_MASK_MODE_NAND,
  COMBINE_MASK_MODE_NOR,
  COMBINE_MASK_MODE_NXOR,
  COMBINE_MASK_MODE_ANDN,
  COMBINE_MASK_MODE_ORN,
  COMBINE_MASK_MODE_XORN,
};

bool combine_masks(COMBINE_MASK_MODE mode,
    cv::InputArray newmask, cv::InputArray oldmask,
    cv::OutputArray output_combined_mask);


#endif /* __combine_masks_h__ */
