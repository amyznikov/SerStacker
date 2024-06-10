/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include <math.h>

#include "util/usettings.h"
#include "util/NumType.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "c_dso_display.h"

namespace dso
{
struct CalibHessian;
struct FrameHessian;

struct Pnt
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // index in jacobian. never changes (actually, there is no reason why).
  float u, v;

  // idepth / isgood / energy during optimization.
  float idepth;
  bool isGood;
  Vec2f energy;		// (UenergyPhotometric, energyRegularizer)
  bool isGood_new;
  float idepth_new;
  Vec2f energy_new;

  float iR;
  float iRSumNum;

  float lastHessian;
  float lastHessian_new;

  // max stepsize for idepth (corresponding to max. movement in pixel-space).
  float maxstep;

  // idx (x+y*w) of closest point one pyramid level above.
  int parent;
  float parentDist;

  // idx (x+y*w) of up to 10 nearest points in pixel space.
  int neighbours[10];
  float neighboursDist[10];

  float my_type;
  float outlierTH;
};

class CoarseInitializer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CoarseInitializer(int w, int h);
  ~CoarseInitializer();

  void setFirst(CalibHessian * HCalib, FrameHessian * newFrameHessian, const c_dso_display & display);
  bool trackFrame(FrameHessian * newFrameHessian, const c_dso_display & display);
  void calcTGrads(FrameHessian * newFrameHessian);

  int frameID = 0;
  bool fixAffine = false;
  bool printDebug = false;

  Pnt * points[PYR_LEVELS] = { nullptr };
  int numPoints[PYR_LEVELS] = { 0 };
  AffLight thisToNext_aff;
  SE3 thisToNext;

  FrameHessian * firstFrame = nullptr;
  FrameHessian * newFrame = nullptr;
  private:
  Mat33 K[PYR_LEVELS];
  Mat33 Ki[PYR_LEVELS];
  double fx[PYR_LEVELS] = { 0 };
  double fy[PYR_LEVELS] = { 0 };
  double fxi[PYR_LEVELS] = { 0 };
  double fyi[PYR_LEVELS] = { 0 };
  double cx[PYR_LEVELS] = { 0 };
  double cy[PYR_LEVELS] = { 0 };
  double cxi[PYR_LEVELS] = { 0 };
  double cyi[PYR_LEVELS] = { 0 };
  int w[PYR_LEVELS] = { 0 };
  int h[PYR_LEVELS] = { 0 };
  void makeK(CalibHessian * HCalib);

  bool snapped = false;
  int snappedAt = 0;

  // pyramid images & levels on all levels
  Eigen::Vector3f * dINew[PYR_LEVELS] = { nullptr };
  Eigen::Vector3f * dIFist[PYR_LEVELS] = { nullptr };

  Eigen::DiagonalMatrix<float, 8> wM;

  // temporary buffers for H and b.
  Vec10f * JbBuffer = nullptr;			// 0-7: sum(dd * dp). 8: sum(res*dd). 9: 1/(1+sum(dd*dd))=inverse hessian entry.
  Vec10f * JbBuffer_new = nullptr;

  Accumulator9 acc9;
  Accumulator9 acc9SC;

  Vec3f dGrads[PYR_LEVELS];

  float alphaK = 0;
  float alphaW = 0;
  float regWeight = 0;
  float couplingWeight = 0;

  Vec3f calcResAndGS(
      int lvl,
      Mat88f & H_out, Vec8f & b_out,
      Mat88f & H_out_sc, Vec8f & b_out_sc,
      const SE3 & refToNew, AffLight refToNew_aff,
      bool plot);
  Vec3f calcEC(int lvl); // returns OLD NERGY, NEW ENERGY, NUM TERMS.
  void optReg(int lvl);

  void propagateUp(int srcLvl);
  void propagateDown(int srcLvl);
  float rescale();

  void resetPoints(int lvl);
  void doStep(int lvl, float lambda, Vec8f inc);
  void applyStep(int lvl);

  void makeGradients(Eigen::Vector3f ** data);

  void debugPlot(int lvl, const c_dso_display & display);
  void makeNN();
};

struct FLANNPointcloud
{
  int num = 0;
  Pnt * points = nullptr;

  inline FLANNPointcloud()
  {
    num = 0;
    points = 0;
  }
  inline FLANNPointcloud(int n, Pnt * p) : num(n), points(p)
  {
  }
  inline size_t kdtree_get_point_count() const
  {
    return num;
  }
  inline float kdtree_distance(const float * p1, const size_t idx_p2, size_t /*size*/) const
  {
    const float d0 = p1[0] - points[idx_p2].u;
    const float d1 = p1[1] - points[idx_p2].v;
    return d0 * d0 + d1 * d1;
  }

  inline float kdtree_get_pt(const size_t idx, int dim) const
  {
    if( dim == 0 )
      return points[idx].u;
    else
      return points[idx].v;
  }
  template<class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const
  {
    return false;
  }
};

}

