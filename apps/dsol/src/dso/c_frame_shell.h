/**
* This file is part derived from DSO sources.
*/


#pragma once
#ifndef __c_frame_shell_h___
#define __c_frame_shell_h___

#include "util/NumType.h"
#include <memory>

namespace dso {

struct c_frame_shell
{
  typedef c_frame_shell this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  this_class * trackingRef = nullptr;

	int id = 0; 			// INTERNAL ID, starting at zero.
	int incoming_id = 0;	// ID passed into DSO
  int marginalizedAt = -1;
	double timestamp = 0;		// timestamp passed into DSO.
  double movedByOpt = 0;

	// set once after tracking
	SE3 camToTrackingRef;

	// constantly adapted.
	SE3 camToWorld;				// Write: TRACKING, while frame is still fresh; MAPPING: only when locked [shellPoseMutex].

	AffLight aff_g2l;

	// statisitcs
	int statistics_outlierResOnThis = 0;
	int statistics_goodResOnThis = 0;

  bool poseValid = true;

  c_frame_shell()
  {
  }

  c_frame_shell(int _id, double _timestamp) :
    id(_id),
    incoming_id(_id),
    marginalizedAt(_id),
    timestamp(_timestamp)
  {
  }

  ~c_frame_shell()
  {
  }
};


}

#endif // __c_frame_shell_h___
