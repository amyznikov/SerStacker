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
 
#include "util/NumType.h"
#include "c_dso_display.h"

namespace dso
{

enum PixelSelectorStatus {PIXSEL_VOID=0, PIXSEL_1, PIXSEL_2, PIXSEL_3};


class FrameHessian;

class PixelSelector
{
public:

  // int recursionsLeft=1, bool plot=false, float thFactor=1
	int makeMaps(const FrameHessian* const fh,
	    float* map_out, float density, int recursionsLeft, bool plot, float thFactor,
			c_dso_display * display);

	PixelSelector(int w, int h);
	~PixelSelector();
	int currentPotential = 0;


	bool allowFast = true;
	void makeHists(const FrameHessian* const fh);
private:

	Eigen::Vector3i select(const FrameHessian* const fh,
			float* map_out, int pot, float thFactor=1);


	unsigned char* randomPattern = nullptr;


	int* gradHist = nullptr;
	float* ths = nullptr;
	float* thsSmoothed = nullptr;
	int thsStep = 0;
	const FrameHessian* gradHistFrame = nullptr;
};




}

