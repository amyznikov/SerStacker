/*
 * c_edge_test_routine.h
 *
 *  Created on: Dec 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_edge_test_routine_h__
#define __c_edge_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_edge_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_edge_test_routine,
      "edge_test", "c_edge_test_routine");

  enum DisplayID {
    DisplayCornerTL,
    DisplayCornerTR,
    DisplayCornerBL,
    DisplayCornerBR,

    DisplayEdgeTLTR,
    DisplayEdgeBLBR,

    DisplayEdgeTLBL,
    DisplayEdgeTRBR,

    DisplayEdgeTLBR,
    DisplayEdgeTRBL,

    //DisplayEdgeType,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _kradius = 2;
  DisplayID _display_id = DisplayCornerTL;
};

#endif /* __c_edge_test_routine_h__ */
