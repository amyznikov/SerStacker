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

//#include <iostream>
//#include <fstream>

#include <vector>
#include "util/globalCalib.h"
#include "util/NumType.h"
#include "util/globalFuncs.h"
#include "OptimizationBackend/RawResidualJacobian.h"

namespace dso
{
class PointHessian;
class FrameHessian;
class CalibHessian;

class EFResidual;

enum ResLocation
{
  ACTIVE = 0, LINEARIZED, MARGINALIZED, NONE
};
enum ResState
{
  IN = 0, OOB, OUTLIER
};

struct FullJacRowT
{
  Eigen::Vector2f projectedTo[MAX_RES_PER_POINT];
};

class PointFrameResidual
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static int instanceCounter;

  EFResidual * efResidual = nullptr;

  double state_energy = 0;
  double state_NewEnergy = 0;
  double state_NewEnergyWithOutlier = 0;
  ResState state_state = ResState::IN;
  ResState state_NewState = ResState::IN;

  void setState(ResState s)
  {
    state_state = s;
  }

  PointHessian * point = nullptr;
  FrameHessian * host = nullptr;
  FrameHessian * target = nullptr;
  RawResidualJacobian * J = nullptr;

  bool isNew = true;

  Eigen::Vector2f projectedTo[MAX_RES_PER_POINT];
  Vec3f centerProjectedTo;

  ~PointFrameResidual();
  PointFrameResidual();
  PointFrameResidual(PointHessian * point_, FrameHessian * host_, FrameHessian * target_);
  double linearize(CalibHessian * HCalib);

  void resetOOB()
  {
    state_NewEnergy = state_energy = 0;
    state_NewState = ResState::OUTLIER;

    setState(ResState::IN);
  }
  ;
  void applyRes(bool copyJacobians);

  void debugPlot();

  void printRows(std::vector<VecX> & v, VecX & r, int nFrames, int nPoints, int M, int res);
};
}

