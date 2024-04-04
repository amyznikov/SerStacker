/*
 * ctrlbind.cc
 *
 *  Created on: Apr 3, 2024
 *      Author: amyznikov
 */

#include "ctrlbind.h"
#include <core/feature2d/feature_extraction.h>

enum XXX
{
  XXX_MEMB1 = 0,
  XXX_MEMB2 = 1,
  XXX_MEMB3 = 2,
  XXX_MEMB4 = 4,
};

class c_ctrl_bind_test
{

public:

  double dprop() const
  {
    return dprop_;
  }

  void set_dprop(double v)
  {
    dprop_ = v;
  }

  int iprop() const
  {
    return iprop_;
  }

  void set_iprop(int v)
  {
    iprop_ = v;
  }

  bool bprop() const
  {
    return bprop_;
  }

  void set_bprop(bool v)
  {
    bprop_ = v;
  }

  const std::string & sprop() const
  {
    return sprop_;
  }

  void set_sprop(const std::string & v)
  {
    sprop_ = v;
  }

  XXX xprop() const
  {
    return xprop_;
  }

  void set_xprop(XXX v)
  {
    xprop_ = v;
  }

  int flagsprop() const
  {
    return flagsprop_;
  }

  void set_flagsprop(int v)
  {
    flagsprop_ = v;
  }

  c_sparse_feature_detector_options * feature2d_detector()
  {
    return &feature2d_detector_options_;
  }

  std::mutex & mutex()
  {
    return mutex_;
  }

  std::string chelp() const
  {
    return "this is help string";
  }

public:
  void get_parameters(std::vector<struct c_ctrl_bind> * ctls)
  {
    BIND_CTRL(ctls, iprop, "iprop", "tooltip for iprop");
    BIND_CTRL(ctls, bprop, "bprop", "tooltip for bprop");

    BIND_CTRL_BEGIN_GROUP(ctls, "Group 1", "");
      BIND_CTRL(ctls, sprop, "sprop", "tooltip for sprop");
      BIND_CTRL(ctls, xprop, "xprop", "tooltip for xprop");
      BIND_FLAGS_CTRL(ctls, flagsprop, XXX, "flagsprop", "tooltip for flagsprop");
    BIND_CTRL_END_GROUP(ctls);

    BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, sprop, "existing filename", "tooltip for existing filename");
    BIND_BROWSE_FOR_DIRECTORY_CTRL(ctls, sprop, "path to directory", "tooltip for path");
    BIND_MATH_EXPRESSION_CTRL(ctls, sprop, chelp, "formula", "tooltop for formula");
    BIND_SPINBOX_CTRL(ctls, iprop, 0, 10, 1, "iprop", "tooltop for spin");
    BIND_DOUBLE_SLIDER_CTRL(ctls, dprop, -10, 10, 0.1, "dprop", "tt for dprop");

    BIND_SPARSE_FEATURE_DETECTOR_CTRL(ctls, feature2d_detector, "feature2d_detector", "tt for feature2d_detector");
  }


protected:
  std::mutex mutex_;
  double dprop_ = 0;
  int iprop_ = 0;
  bool bprop_ = false;
  std::string sprop_;


  XXX xprop_ = XXX_MEMB2;
  int flagsprop_ = 0;

  c_sparse_feature_detector_options feature2d_detector_options_;
};


