/*
 * c_edge_test_routine.cc
 *
 *  Created on: Dec 23, 2023
 *      Author: amyznikov
 */

#include "c_edge_test_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_edge_test_routine::DisplayID>()
{
  static const c_enum_member members[] = {
      { c_edge_test_routine::DisplayCornerTL, "TL", "" },
      { c_edge_test_routine::DisplayCornerTR, "TR", "" },
      { c_edge_test_routine::DisplayCornerBL, "BL", "" },
      { c_edge_test_routine::DisplayCornerBR, "BR", "" },

      { c_edge_test_routine::DisplayEdgeTLTR, "TLTR", "" },
      { c_edge_test_routine::DisplayEdgeBLBR, "BLBR", "" },

      { c_edge_test_routine::DisplayEdgeTLBL, "TLBL", "" },
      { c_edge_test_routine::DisplayEdgeTRBR, "TRBR", "" },

      { c_edge_test_routine::DisplayEdgeTLBR, "TLBR", "" },
      { c_edge_test_routine::DisplayEdgeTRBL, "TRBL", "" },

      { c_edge_test_routine::DisplayCornerTL },
  };

  return members;
}

void c_edge_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "kradius", ctx(&this_class::_kradius), "");
   ctlbind(ctls, "display", ctx(&this_class::_display_id), "");
}

bool c_edge_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_id);
    SERIALIZE_OPTION(settings, save, *this, _kradius);
    return true;
  }
  return false;
}

bool c_edge_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int r =
      std::max(1, _kradius);

  /*
   *  [] []    []    [] []
   *  [] []    []    [] []
   *
   *  [] []    []    [] []
   *
   *  [] []    []    [] []
   *  [] []    []    [] []
   */


  const cv::Point anhors[] = {
      cv::Point(r, r), // top left
      cv::Point(0, r), // top right

      cv::Point(r, 0), // bottom left
      cv::Point(0, 0), // bottom right
  };

  cv::Mat a[4];


  const cv::Size ksize(r + 1, r + 1);
  for ( int i = 0; i < 4; ++i ) {
    cv::boxFilter(image, a[i], -1, ksize, anhors[i], true, cv::BORDER_REPLICATE);
  }

  switch (_display_id) {
    case DisplayCornerTL:
      a[0].copyTo(image);
      break;
    case DisplayCornerTR:
      a[1].copyTo(image);
      break;
    case DisplayCornerBL:
      a[2].copyTo(image);
      break;
    case DisplayCornerBR:
      a[3].copyTo(image);
      break;
    case DisplayEdgeTLTR:
      cv::absdiff(a[0], a[1], image);
      break;
    case DisplayEdgeBLBR:
      cv::absdiff(a[2], a[3], image);
      break;
    case DisplayEdgeTLBL:
      cv::absdiff(a[0], a[2], image);
      break;
    case DisplayEdgeTRBR:
      cv::absdiff(a[1], a[3], image);
      break;
    case DisplayEdgeTLBR:
      cv::absdiff(a[0], a[3], image);
      break;
    case DisplayEdgeTRBL:
      cv::absdiff(a[1], a[2], image);
      break;

    default:
      break;
  }

  return true;
}
