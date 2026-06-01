/*
 * fitEllipseLM.cc
 *
 *  Created on: Sep 17, 2023
 *      Author: amyznikov
 */

#include "fitEllipseLM.h"
#include <core/proc/levmar.h>
#include <core/debug.h>

// cv::Point3f::x => x coordinate
// cv::Point3f::y => y coordinate
// cv::Point3f::z => weight of the point
bool fitEllipseLMW(const std::vector<cv::Point3f> & edge_points,
    double fixed_axis_ratio, // b / a
    double fixed_orientation, // radians
    cv::RotatedRect * rc)
{
  if (edge_points.empty()) {
    CF_ERROR("fitEllipseLMW: input points vector is empty");
    return false;
  }

  typedef c_levmard_solver c_lm_solver;

  class c_lm_solver_callback: public c_lm_solver::callback
  {
    double axis_ratio;
    double orientation;
    const std::vector<cv::Point3f> & points;

  public:
    c_lm_solver_callback(double _axis_ratio, double _orientation,
        const std::vector<cv::Point3f> & _points) :
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
      const double a  = params[2];
      const double b = a * axis_ratio;
      const double ca = std::cos(orientation);
      const double sa = std::sin(orientation);
      const int n = points.size();
      rhs.resize(n);

      double * __restrict rhsp = rhs.data();

      if (!jac ) {
        for( int i = 0; i < n; ++i ) {
          const cv::Point3f & p = points[i];
          const double dx = p.x - x0;
          const double dy = p.y - y0;
          const double xr =  dx * ca + dy * sa;
          const double yr = -dx * sa + dy * ca;
          const double R_meas = std::max(1e-6, std::sqrt(xr * xr + yr * yr));
          const double cos_phi = xr / R_meas;
          const double sin_phi = yr / R_meas;
          const double Rt_x = a * cos_phi;
          const double Rt_y = b * sin_phi;
          const double R_theor = std::sqrt(Rt_x * Rt_x + Rt_y * Rt_y);
          const double w = p.z;
          *rhsp++ = w * (R_meas - R_theor);
        }
      }
      else {

        jac->create(3, n);
        *have_analytical_jac = true;

        cv::Mat1d & J = *jac;

        double * __restrict j0 = J[0];
        double * __restrict j1 = J[1];
        double * __restrict j2 = J[2];

        for( int i = 0; i < n; ++i ) {
          const cv::Point3f & p = points[i];
          const double dx = p.x - x0;
          const double dy = p.y - y0;
          const double xr =  dx * ca + dy * sa;
          const double yr = -dx * sa + dy * ca;
          const double R_meas = std::max(1e-6, std::sqrt(xr * xr + yr * yr));
          const double cos_phi = xr / R_meas;
          const double sin_phi = yr / R_meas;
          const double Rt_x = a * cos_phi;
          const double Rt_y = b * sin_phi;
          const double R_theor = std::sqrt(Rt_x * Rt_x + Rt_y * Rt_y);
          const double w = p.z;
          *rhsp++ = w * (R_meas - R_theor);

          // dxr/dx0 = -ca, dxr/dy0 = -sa
          // dyr/dx0 =  sa, dyr/dy0 = -ca
          const double dRmeas_dx0 = -cos_phi * ca + sin_phi * sa;
          const double dRmeas_dy0 = -cos_phi * sa - sin_phi * ca;
          const double dRtheor_da = R_theor / a;
          *j0++ = w * dRmeas_dx0;   // d(err)/dx0
          *j1++ = w * dRmeas_dy0;   // d(err)/dy0
          *j2++ = -w * dRtheor_da;  // d(err)/da
        }
      }

      return true;
    }
  };

  cv::Scalar mean_center(0, 0, 0, 0);
  cv::Scalar std_dev;
  cv::meanStdDev(cv::Mat(edge_points), mean_center, std_dev);

  std::vector<double> initial_params;
  initial_params.emplace_back(mean_center[0]+100);
  initial_params.emplace_back(mean_center[1]+200);
  initial_params.emplace_back(1.44 * std::max(std_dev[0], std_dev[1]));

  std::vector<double> params = initial_params;
  c_lm_solver_callback callback(fixed_axis_ratio, fixed_orientation, edge_points);

  c_lm_solver solver;
  solver.set_epsfn(1e-12);
  solver.set_epsx(1e-12);
  solver.set_max_iterations(200);
  const int iterations = solver.run(callback, params);

  CF_DEBUG("solver.run(): iterations=%d rmse=%g converged=%d stop_reason=%d\n"
      "x0 = %g (initial=%g) | y0 = %g (initial=%g) | A = %g (initial=%g)\n"
      "edge_points.size=%zu\n",
      iterations, solver.rmse(), solver.converged(), solver.stop_reason(),
      params[0], initial_params[0],
      params[1], initial_params[1],
      params[2], initial_params[2],
      edge_points.size());

  rc->center.x = static_cast<float>(params[0]);
  rc->center.y = static_cast<float>(params[1]);
  rc->size.width = static_cast<float>(2.0 * params[2]);
  rc->size.height = static_cast<float>(rc->size.width * fixed_axis_ratio);
  rc->angle = static_cast<float>(fixed_orientation * 180.0 / CV_PI);

  return true;
}
