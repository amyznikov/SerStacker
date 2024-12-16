/*
 * fitEllipseLM.cc
 *
 *  Created on: Sep 17, 2023
 *      Author: amyznikov
 */

#include "fitEllipseLM.h"
#include <core/proc/levmar.h>
#include <core/debug.h>

bool fitEllipseLM1(const std::vector<cv::Point2f> & edge_points,
    double fixed_axis_ratio, // b / a
    double fixed_orientation, // radians
    cv::RotatedRect * rc)
{

  typedef c_levmar_solver
    c_lm_solver;

  class c_lm_solver_callback:
      public c_lm_solver::callback
  {
    double axis_ratio;
    double orientation;
    const std::vector<cv::Point2f> & points;

  public:

    c_lm_solver_callback(double _axis_ratio, double _orientation, const std::vector<cv::Point2f> & _points) :
        axis_ratio(_axis_ratio),
        orientation(_orientation),
        points(_points)
    {
    }

    bool compute(const std::vector<double> & params, std::vector<double> & rhs,
        cv::Mat1d * jac, bool * have_analytical_jac) final
    {
      const double x0 = params[0];
      const double y0 = params[1];
      const double a = params[2];

      const double b = a * axis_ratio;
      const double ca = cos(orientation);
      const double sa = sin(orientation);

      rhs.resize(points.size());

      for( int i = 0, n = points.size(); i < n; ++i ) {

        const cv::Point2f & p = points[i];

        const double xx = ((p.x - x0) * ca - (p.y - y0) * sa) / a;
        const double yy = ((p.x - x0) * sa + (p.y - y0) * ca) / b;

        rhs[i] = xx * xx + yy * yy - 1;
      }

      return true;
    }

  };


  cv::Scalar center, size;
  std::vector<double> params;

  cv::meanStdDev(edge_points, center, size);


  params.emplace_back(center[0]); // x0
  params.emplace_back(center[1]); // y0
  params.emplace_back(1.44 * std::max(size[0], size[1])); // a

  c_lm_solver_callback callback(fixed_axis_ratio, fixed_orientation, edge_points);
  c_lm_solver solver;

  const int iterations =
      solver.run(callback, params);

  CF_DEBUG("solver.run(): iterations=%d rmse=%g\n"
      "x0 = %g (%g) | y0 = %g (%g) | a = %g (%g)\n",
      iterations,
      solver.rmse(),
      params[0], center[0],
      params[1], center[1],
      params[2], std::max(size[0], size[1]));

  rc->center.x = params[0];
  rc->center.y = params[1];
  rc->size.width = 2 * params[2];
  rc->size.height = rc->size.width * fixed_axis_ratio;
  rc->angle = fixed_orientation * 180 / CV_PI;

  return true;
}
