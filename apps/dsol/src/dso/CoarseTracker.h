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
struct PointFrameResidual;

class CoarseTracker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CoarseTracker(int w, int h);
  ~CoarseTracker();

  bool trackNewestCoarse(FrameHessian * newFrameHessian,
      SE3 & lastToNew_out, AffLight & aff_g2l_out,
      int coarsestLvl, Vec5 minResForAbort,
      c_dso_display * display);

  void setCoarseTrackingRef(const std::vector<FrameHessian*> & frameHessians);

  void makeK(
      CalibHessian * HCalib);

  bool debugPrint, debugPlot;

  Mat33f K[PYR_LEVELS];
  Mat33f Ki[PYR_LEVELS];
  float fx[PYR_LEVELS] = { 0 };
  float fy[PYR_LEVELS] = { 0 };
  float fxi[PYR_LEVELS] = { 0 };
  float fyi[PYR_LEVELS] = { 0 };
  float cx[PYR_LEVELS] = { 0 };
  float cy[PYR_LEVELS] = { 0 };
  float cxi[PYR_LEVELS] = { 0 };
  float cyi[PYR_LEVELS] = { 0 };
  int w[PYR_LEVELS] = { 0 };
  int h[PYR_LEVELS] = { 0 };

  void debugPlotIDepthMap(float * minID, float * maxID, c_dso_display * display);
  void debugPlotIDepthMapFloat(c_dso_display * display);

  FrameHessian * lastRef = nullptr;
  AffLight lastRef_aff_g2l;
  FrameHessian * newFrame = nullptr;
  int refFrameID = 0;

  // act as pure ouptut
  Vec5 lastResiduals;
  Vec3 lastFlowIndicators;
  double firstCoarseRMSE = 0;

private:

  void makeCoarseDepthL0(const std::vector<FrameHessian*> & frameHessians);
  float * idepth[PYR_LEVELS] = { nullptr };
  float * weightSums[PYR_LEVELS] = { nullptr };
  float * weightSums_bak[PYR_LEVELS] = { nullptr };

  Vec6 calcResAndGS(int lvl, Mat88 & H_out, Vec8 & b_out, const SE3 & refToNew, AffLight aff_g2l, float cutoffTH);
  Vec6 calcRes(int lvl, const SE3 & refToNew, AffLight aff_g2l, float cutoffTH, c_dso_display * display);
  void calcGSSSE(int lvl, Mat88 & H_out, Vec8 & b_out, const SE3 & refToNew, AffLight aff_g2l);
  void calcGS(int lvl, Mat88 & H_out, Vec8 & b_out, const SE3 & refToNew, AffLight aff_g2l);

  // pc buffers
  float * pc_u[PYR_LEVELS] = { nullptr };
  float * pc_v[PYR_LEVELS] = { nullptr };
  float * pc_idepth[PYR_LEVELS] = { nullptr };
  float * pc_color[PYR_LEVELS] = { nullptr };
  int pc_n[PYR_LEVELS] = { 0 };

  // warped buffers
  float * buf_warped_idepth = nullptr;
  float * buf_warped_u = nullptr;
  float * buf_warped_v = nullptr;
  float * buf_warped_dx = nullptr;
  float * buf_warped_dy = nullptr;
  float * buf_warped_residual = nullptr;
  float * buf_warped_weight = nullptr;
  float * buf_warped_refColor = nullptr;
  int buf_warped_n = 0;

  std::vector<float*> ptrToDelete;

  Accumulator9 acc;
};

class CoarseDistanceMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CoarseDistanceMap(int w, int h);
  ~CoarseDistanceMap();

  void makeDistanceMap(const std::vector<FrameHessian*> & frameHessians, FrameHessian * frame);

  void makeInlierVotes(const std::vector<FrameHessian*> & frameHessians);

  void makeK(CalibHessian * HCalib);

  float * fwdWarpedIDDistFinal = nullptr;

  Mat33f K[PYR_LEVELS];
  Mat33f Ki[PYR_LEVELS];
  float fx[PYR_LEVELS] = { 0 };
  float fy[PYR_LEVELS] = { 0 };
  float fxi[PYR_LEVELS] = { 0 };
  float fyi[PYR_LEVELS] = { 0 };
  float cx[PYR_LEVELS] = { 0 };
  float cy[PYR_LEVELS] = { 0 };
  float cxi[PYR_LEVELS] = { 0 };
  float cyi[PYR_LEVELS] = { 0 };
  int w[PYR_LEVELS] = { 0 };
  int h[PYR_LEVELS] = { 0 };

  void addIntoDistFinal(int u, int v);

private:

  PointFrameResidual ** coarseProjectionGrid = nullptr;
  int * coarseProjectionGridNum = nullptr;
  Eigen::Vector2i * bfsList1 = nullptr;
  Eigen::Vector2i * bfsList2 = nullptr;

  void growDistBFS(int bfsNum);
};

}

