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

  void set_kradius(int v)
  {
    kradius_ = v;
  }

  int kradius() const
  {
    return kradius_;
  }

  void set_display_id(DisplayID v)
  {
    display_id_ = v;
  }

  DisplayID display_id() const
  {
    return display_id_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int kradius_ = 2;
  DisplayID display_id_ = DisplayCornerTL;
};

#endif /* __c_edge_test_routine_h__ */
