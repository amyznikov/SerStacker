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

 
#include "HessianBlocks.h"
#include "util/NumType.h"

namespace dso
{


struct ImmaturePointTemporaryResidual
{
public:
  double state_energy = 0;
  double state_NewEnergy = 0;
	ResState state_state = ResState::IN;
  ResState state_NewState = ResState::IN;
	FrameHessian* target = nullptr;
};


enum ImmaturePointStatus {
	IPS_GOOD=0,					// traced well and good
	IPS_OOB,					// OOB: end tracking & marginalize!
	IPS_OUTLIER,				// energy too high: if happens again: outlier!
	IPS_SKIPPED,				// traced well and good (but not actually traced).
	IPS_BADCONDITION,			// not traced because of bad condition.
	IPS_UNINITIALIZED};			// not even traced once.


class ImmaturePoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// static values
	float color[MAX_RES_PER_POINT] = {0};
	float weights[MAX_RES_PER_POINT] = {0};


	Mat22f gradH;
	Vec2f gradH_ev;
	Mat22f gradH_eig;
	float energyTH = 0;
	float u = 0;
	float v = 0;
	FrameHessian* host = nullptr;
	int idxInImmaturePoints = 0;

	float quality = 0;

	float my_type = 0;

	float idepth_min = 0;
	float idepth_max = 0;

	ImmaturePoint(int u_, int v_, FrameHessian* host_, float type, CalibHessian* HCalib);
	~ImmaturePoint();

	ImmaturePointStatus traceOn(FrameHessian* frame, const Mat33f &hostToFrame_KRKi, const Vec3f &hostToFrame_Kt, const Vec2f &hostToFrame_affine, CalibHessian* HCalib, bool debugPrint=false);

	ImmaturePointStatus lastTraceStatus;
	Vec2f lastTraceUV;
	float lastTracePixelInterval = 0;

	float idepth_GT = 0;

	double linearizeResidual(CalibHessian *  HCalib, const float outlierTHSlack,
			ImmaturePointTemporaryResidual* tmpRes,
			float &Hdd, float &bd,
			float idepth);

	float getdPixdd(CalibHessian *  HCalib,
			ImmaturePointTemporaryResidual* tmpRes,
			float idepth);

	float calcResidual(CalibHessian *  HCalib, const float outlierTHSlack,
			ImmaturePointTemporaryResidual* tmpRes,
			float idepth);

private:
};

}

