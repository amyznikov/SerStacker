/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <cmath>
#include <array>
#include <core/ssprintf.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/levmar.h>
#include <core/proc/levmar3.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>
//#include <gui/widgets/QSettingsWidget.h>

#include <core/debug.h>

struct S1
{
  int a;
  int b;
};


class c_pp_base
{
public:

  using c_control_list = c_ctlist<c_pp_base> ;
  using ctlbind_context = c_ctlbind_context<c_pp_base>;

  virtual ~c_pp_base() = default;

  virtual void getcontrols(c_control_list & ctls)
  {
    c_pp_base::getcontrols(ctls, ctlbind_context());
  }

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
  }
};


class c_pp_derived : public c_pp_base
{
public:

  using ctlbind_context = c_ctlbind_context<c_pp_base, c_pp_derived>;

  virtual void getcontrols(c_control_list & ctls)
  {
    c_pp_derived::getcontrols(ctls, ctlbind_context());
  }

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
    c_pp_base::getcontrols(ctls, ctx);
  }
};


int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_pp_base::c_control_list ctls;

  c_pp_derived pp;
  pp.getcontrols(ctls);


  return 0;
}
