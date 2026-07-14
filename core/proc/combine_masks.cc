/*
 * combine_masks.cc
 *
 *  Created on: Jul 13, 2026
 *      Author: amyznikov
 */


#include "combine_masks.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<COMBINE_MASK_MODE>()
{
  static const c_enum_member members[] = {
      { COMBINE_MASK_MODE_KEEP, "KEEP", "" },
      { COMBINE_MASK_MODE_REPLACE, "REPLACE", "oldmask = newmask" },
      { COMBINE_MASK_MODE_AND, "AND", "oldmask = newmask & oldmask" },
      { COMBINE_MASK_MODE_OR, "OR", "oldmask = newmask | oldmask" },
      { COMBINE_MASK_MODE_XOR, "XOR", "oldmask = newmask ^ oldmask" },
      { COMBINE_MASK_MODE_NAND, "NAND", "oldmask = ~(newmask & oldmask)" },
      { COMBINE_MASK_MODE_NOR, "NOR", "oldmask = ~(newmask | oldmask)" },
      { COMBINE_MASK_MODE_NXOR, "NXOR", "oldmask = ~(newmask ^ oldmask)" },
      { COMBINE_MASK_MODE_ANDN, "ANDN", "oldmask = newmask & ~oldmask" },
      { COMBINE_MASK_MODE_ORN, "ORN", "oldmask = newmask | ~oldmask" },
      { COMBINE_MASK_MODE_XORN, "XORN", "oldmask = newmask ^ ~oldmask" },
      { COMBINE_MASK_MODE_REPLACE },
  };

  return members;
}

bool combine_masks(COMBINE_MASK_MODE mode,
    cv::InputArray newmask, cv::InputArray oldmask,
    cv::OutputArray output_mask)
{
  if( mode == COMBINE_MASK_MODE_REPLACE ) {
    newmask.copyTo(output_mask);
    return true;
  }

  if( newmask.empty() || mode == COMBINE_MASK_MODE_KEEP ) {
    oldmask.copyTo(output_mask);
    return true;
  }


  if( oldmask.empty() ) {
    newmask.copyTo(output_mask);
    switch (mode) {
      case COMBINE_MASK_MODE_NAND:
        case COMBINE_MASK_MODE_NOR:
        case COMBINE_MASK_MODE_NXOR:
        cv::bitwise_not(output_mask, output_mask);
        break;
    }
    return true;
  }

  if( newmask.size() != oldmask.size() || newmask.depth() != oldmask.depth() ) {
    CF_ERROR(
        "Invalid combined mask sizes/types: newmask: %dx%d channels=%d depth=%d oldmask: %dx%d channels=%d depth=%d",
        newmask.cols(), newmask.rows(), newmask.channels(), newmask.depth(),
        oldmask.cols(), oldmask.rows(), oldmask.channels(), oldmask.depth());
    return false;
  }

  const int cnew = newmask.channels();
  const int cold = oldmask.channels();
  if( cnew != 1 && cold != 1 && cnew != cold ) {
    CF_ERROR("Not supported combination of mask channels: "
        "newmask: %dx%d channels=%d depth=%d oldmask: %dx%d channels=%d depth=%d",
        newmask.cols(), newmask.rows(), newmask.channels(), newmask.depth(),
        oldmask.cols(), oldmask.rows(), oldmask.channels(), oldmask.depth());
    return false;
  }

  static const auto cdup =
      [](cv::InputArray src, int cn) -> cv::Mat {
        const std::vector<cv::Mat> channels(cn, src.getMat());
        cv::Mat tmp;
        cv::merge(channels, tmp);
        return tmp;
      };

  const cv::Mat newm =
      newmask.channels() >= oldmask.channels() ? newmask.getMat() :
          cdup(newmask, oldmask.channels());

  const cv::Mat oldm =
      oldmask.channels() >= newmask.channels() ? oldmask.getMat() :
          cdup(oldmask, newmask.channels());

  switch (mode) {
    case COMBINE_MASK_MODE_AND:
      case COMBINE_MASK_MODE_NAND:
      cv::bitwise_and(newm, oldm, output_mask);
      if( mode == COMBINE_MASK_MODE_NAND ) {
        cv::bitwise_not(output_mask, output_mask);
      }
      break;

    case COMBINE_MASK_MODE_OR:
      case COMBINE_MASK_MODE_NOR:
      cv::bitwise_or(newm, oldm, output_mask);
      if( mode == COMBINE_MASK_MODE_NOR ) {
        cv::bitwise_not(output_mask, output_mask);
      }
      break;

    case COMBINE_MASK_MODE_XOR:
      case COMBINE_MASK_MODE_NXOR:
      cv::bitwise_xor(newm, oldm, output_mask);
      if( mode == COMBINE_MASK_MODE_NXOR ) {
        cv::bitwise_not(output_mask, output_mask);
      }
      break;

    case COMBINE_MASK_MODE_ANDN:
      cv::bitwise_and(newm, ~oldm, output_mask);
      break;

    case COMBINE_MASK_MODE_ORN:
      cv::bitwise_or(newm, ~oldm, output_mask);
      break;

    case COMBINE_MASK_MODE_XORN:
      cv::bitwise_xor(newm, ~oldm, output_mask);
      break;

    case COMBINE_MASK_MODE_KEEP:
      break;

    default:
      CF_ERROR("APP BUG: Not handled mask combine mode %d", mode);
      return false;
  }

  return true;
}
