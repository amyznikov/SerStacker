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

#define MAX_ACTIVE_FRAMES 100

#include <deque>
#include <vector>
#include <cmath>
#include <cstdio>

#include <dso/Residuals.h>
#include <dso/HessianBlocks.h>
#include <dso/ImmaturePoint.h>
#include <dso/PixelSelector2.h>
#include <dso/util/NumType.h>
#include <dso/util/globalCalib.h>
#include <dso/util/IndexThreadReduce.h>
#include <dso/OptimizationBackend/EnergyFunctional.h>
#include <dso/c_dso_display.h>


namespace dso {

class PixelSelector;
class PCSyntheticPoint;
class CoarseTracker;
struct FrameHessian;
struct PointHessian;
class CoarseInitializer;
struct ImmaturePointTemporaryResidual;
class c_image_and_exposure;
class CoarseDistanceMap;

class EnergyFunctional;

template<typename T> inline void deleteOut(std::vector<T*> &v, const int i)
{
	delete v[i];
	v[i] = v.back();
	v.pop_back();
}
template<typename T> inline void deleteOutPt(std::vector<T*> &v, const T* i)
{
	delete i;

	for(unsigned int k=0;k<v.size();k++)
		if(v[k] == i)
		{
			v[k] = v.back();
			v.pop_back();
		}
}
template<typename T> inline void deleteOutOrder(std::vector<T*> &v, const int i)
{
	delete v[i];
	for(unsigned int k=i+1; k<v.size();k++)
		v[k-1] = v[k];
	v.pop_back();
}
template<typename T> inline void deleteOutOrder(std::vector<T*> &v, const T* element)
{
	int i=-1;
	for(unsigned int k=0; k<v.size();k++)
	{
		if(v[k] == element)
		{
			i=k;
			break;
		}
	}
	assert(i!=-1);

	for(unsigned int k=i+1; k<v.size();k++)
		v[k-1] = v[k];
	v.pop_back();

	delete element;
}


inline bool eigenTestNan(const MatXX &m, std::string msg)
{
	bool foundNan = false;

  for( int y = 0; y < m.rows(); y++ ) {
    for( int x = 0; x < m.cols(); x++ ) {
      if( !std::isfinite((double) m(y, x)) ) {
        foundNan = true;
        break;
      }
    }
  }

  if( foundNan ) {
    printf("NAN in %s:\n", msg.c_str());
    // std::cout << m << "\n\n";
  }


	return foundNan;
}





class FullSystem
{
public:
  typedef FullSystem this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;


	FullSystem();
	virtual ~FullSystem();

	// adds a new frame, and creates point & residual structs.
	void addActiveFrame(const c_image_and_exposure & image, int id);

	// marginalizes a frame. drops / marginalizes points & residuals.
	void marginalizeFrame(FrameHessian* frame);
	void blockUntilMappingIsFinished();

	float optimize(int mnumOptIts);

	void printResult(std::string file);

	void debugPlot(std::string name);

	void printFrameLifetimes();
	// contains pointers to active frames

	c_dso_display * display = nullptr;


	bool isLost;
	bool initFailed;
	bool initialized;
	bool linearizeOperation;


	void setPhotometricGamma(const float BInv[256]);
	void setOriginalCalib(const VecXf &originalCalib, int originalW, int originalH);

  const std::vector<c_frame_shell::uptr> & getAllFrameHistory() const
  {
    return allFrameHistory;
  }


  virtual void onFrameHessianMakeImages(FrameHessian * fh)
  {
  }

private:

	CalibHessian Hcalib;




	// opt single point
	int optimizePoint(PointHessian* point, int minObs, bool flagOOB);
	PointHessian* optimizeImmaturePoint(ImmaturePoint* point, int minObs, ImmaturePointTemporaryResidual* residuals);

	double linAllPointSinle(PointHessian* point, float outlierTHSlack, bool plot);

	// mainPipelineFunctions
	Vec4 trackNewCoarse(FrameHessian* fh);
	void traceNewCoarse(FrameHessian* fh);
	void activatePoints();
	void activatePointsMT();
	void activatePointsOldFirst();
	void flagPointsForRemoval();
	void makeNewTraces(FrameHessian* newFrame, float* gtDepth);
	void initializeFromInitializer(FrameHessian* newFrame);
	void flagFramesForMarginalization(FrameHessian* newFH);


	void removeOutliers();


	// set precalc values.
	void setPrecalcValues();


	// solce. eventually migrate to ef.
	void solveSystem(int iteration, double lambda);
	Vec3 linearizeAll(bool fixLinearization);
	bool doStepFromBackup(float stepfacC,float stepfacT,float stepfacR,float stepfacA,float stepfacD);
	void backupState(bool backupLastStep);
	void loadSateBackup();
	double calcLEnergy();
	double calcMEnergy();
	void linearizeAll_Reductor(bool fixLinearization, std::vector<PointFrameResidual*>* toRemove, int min, int max, Vec10* stats, int tid);
	void activatePointsMT_Reductor(std::vector<PointHessian*>* optimized,std::vector<ImmaturePoint*>* toOptimize,int min, int max, Vec10* stats, int tid);
	void applyRes_Reductor(bool copyJacobians, int min, int max, Vec10* stats, int tid);

	void printOptRes(const Vec3 &res, double resL, double resM, double resPrior, double LExact, float a, float b);

	void debugPlotTracking();

	std::vector<VecX> getNullspaces(
			std::vector<VecX> &nullspaces_pose,
			std::vector<VecX> &nullspaces_scale,
			std::vector<VecX> &nullspaces_affA,
			std::vector<VecX> &nullspaces_affB);

	void setNewFrameEnergyTH();


	void printLogLine();
	void printEvalLine();
	void printEigenValLine();

//	std::ofstream* calibLog = nullptr;
//	std::ofstream* numsLog = nullptr;
//	std::ofstream* errorsLog = nullptr;
//	std::ofstream* eigenAllLog = nullptr;
//	std::ofstream* eigenPLog = nullptr;
//	std::ofstream* eigenALog = nullptr;
//	std::ofstream* DiagonalLog = nullptr;
//	std::ofstream* variancesLog = nullptr;
//	std::ofstream* nullspacesLog = nullptr;
//
//	std::ofstream* coarseTrackingLog = nullptr;

	// statistics
	long int statistics_lastNumOptIts = 0;
	long int statistics_numDroppedPoints = 0;
	long int statistics_numActivatedPoints = 0;
	long int statistics_numCreatedPoints = 0;
	long int statistics_numForceDroppedResBwd = 0;
	long int statistics_numForceDroppedResFwd = 0;
	long int statistics_numMargResFwd = 0;
	long int statistics_numMargResBwd = 0;
	float statistics_lastFineTrackRMSE = 0;







	// =================== changed by tracker-thread. protected by trackMutex ============
	std::mutex trackMutex;
	std::vector<c_frame_shell::uptr> allFrameHistory;
	CoarseInitializer* coarseInitializer = nullptr;
	Vec5 lastCoarseRMSE;


	// ================== changed by mapper-thread. protected by mapMutex ===============
	std::mutex mapMutex;
	std::vector<c_frame_shell*> allKeyFramesHistory;

	EnergyFunctional* ef = nullptr;
	// IndexThreadReduce<Vec10> treadReduce;

	float* selectionMap = nullptr;
	PixelSelector* pixelSelector = nullptr;
	CoarseDistanceMap* coarseDistanceMap = nullptr;

	std::vector<FrameHessian*> frameHessians;	// ONLY changed in marginalizeFrame and addFrame.
	std::vector<PointFrameResidual*> activeResiduals;
	float currentMinActDist;


	std::vector<float> allResVec;



	// mutex etc. for tracker exchange.
	std::mutex coarseTrackerSwapMutex;			// if tracker sees that there is a new reference, tracker locks [coarseTrackerSwapMutex] and swaps the two.
	CoarseTracker* coarseTracker_forNewKF = nullptr;			// set as as reference. protected by [coarseTrackerSwapMutex].
	CoarseTracker* coarseTracker = nullptr;					// always used to track new frames. protected by [trackMutex].
	float minIdJetVisTracker = 0;
	float maxIdJetVisTracker = 0;
	float minIdJetVisDebug = 0;
	float maxIdJetVisDebug = 0;


	// mutex for camToWorl's in shells (these are always in a good configuration).
	std::mutex shellPoseMutex;



/*
 * tracking always uses the newest KF as reference.
 *
 */

	void makeKeyFrame( FrameHessian* fh);
	void makeNonKeyFrame( FrameHessian* fh);
	void deliverTrackedFrame(FrameHessian* fh, bool needKF);
	void mappingLoop();

	// tracking / mapping synchronization. All protected by [trackMapSyncMutex].
	std::mutex trackMapSyncMutex;
	std::condition_variable trackedFrameSignal;
	std::condition_variable mappedFrameSignal;
	std::deque<FrameHessian*> unmappedTrackedFrames;
	int needNewKFAfter  = 0;	// Otherwise, a new KF is *needed that has ID bigger than [needNewKFAfter]*.
	std::thread mappingThread;
	bool runMapping = false;
	bool needToKetchupMapping = false;

	int lastRefStopID = 0;
};
}

