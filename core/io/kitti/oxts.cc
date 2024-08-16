/*
 * oxts.cc
 *
 *  <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
 */
#include "oxts.h"
#include <core/proc/pose.h>

///////////////////////////////////////////////////////////////////////////////
using namespace cv;
using namespace std;


//@brief Uses oxts_lower_bound() to find first oxts record  which oxts_out.ts >= ts
// Not sure this routine is really useful, but it is still used somewhere in code.
bool oxts_binary_search(const std::vector<oxts_data> & oxts, double ts,
    /* out*/struct oxts_data * oxts_out)
{
  if ( ts <= oxts.front().ts ) {
    * oxts_out = oxts.front();
    return false;
  }

  if ( ts >= oxts.back().ts ) {
    * oxts_out = oxts.back();
    return false;
  }

  *oxts_out = *oxts_lower_bound(oxts.begin(), oxts.end(), ts);
  return true;
}


//@brief Interpolate between oxts1 and oxts2 for requested time stamp ts,
//  and return interpolated values into oxtsi.
// Note that actually the extrapolation will made if ts is out of [oxts1..oxts2] timestamps range
void oxts_interpolate(const oxts_data & oxts1, const oxts_data & oxts2, double ts,
    /*out*/ oxts_data * oxtsi)
{
  oxtsi->ts = ts;

  //
  // Special cases for angle interpolations.
  //

  //  longitude of the oxts-unit [0..2pi]
  oxtsi->lon = wrap_angle_0_2pi(angle_interpolate(oxts1.ts, oxts1.lon, oxts2.ts, oxts2.lon, ts));

  // latitude of the oxts-unit [-pi/2..pi/2]
  // fixme: add some wrap to latitude interpolation ?
  oxtsi->lat = linint(oxts1.ts, oxts1.lat, oxts2.ts, oxts2.lat, ts);

  // roll (bank) angle,  0 = level, positive = left side up [-pi..pi]
  oxtsi->roll = wrap_angle_minus_pi_plus_pi(angle_interpolate(oxts1.ts, oxts1.roll, oxts2.ts, oxts2.roll, ts));

  // pitch angle, 0 = level, positive = front down [-pi/2..pi/2]
  oxtsi->pitch = wrap_angle_minus_pi_plus_pi(angle_interpolate(oxts1.ts, oxts1.pitch, oxts2.ts, oxts2.pitch, ts));

  // yaw (heading) angle, 0 = east,  positive = counter clockwise [-pi..pi]
  oxtsi->yaw = wrap_angle_minus_pi_plus_pi(angle_interpolate(oxts1.ts, oxts1.yaw, oxts2.ts, oxts2.yaw, ts));


  //
  // general case follows
  //
#define LININT(f) \
  oxtsi->f = linint(oxts1.ts, oxts1.f, oxts2.ts, oxts2.f, ts)

  LININT(alt);       //<!  altitude of the oxts-unit  [m]

  LININT(vn);        //<!  velocity towards north [m/s]
  LININT(ve);        //<!  velocity towards east [m/s]
  LININT(vu);        //<!  upward velocity, i.e. perpendicular to earth-surface [m/s]

  LININT(vf);        //<!  forward velocity, i.e. parallel to earth-surface [m/s]
  LININT(vl);        //<!  leftward velocity, i.e. parallel to earth-surface [m/s]

  LININT(ax);        //<!  acceleration in x, i.e. in direction of vehicle front (m/s^2)
  LININT(ay);        //<!  acceleration in y, i.e. in direction of vehicle left (m/s^2)
  LININT(az);        //<!  acceleration in z, i.e. in direction of vehicle top (m/s^2)

  LININT(af);        //<!  forward acceleration (m/s^2)
  LININT(al);        //<!  leftward acceleration (m/s^2)
  LININT(au);        //<!  upward acceleration (m/s^2)

  LININT(wx);        //<!  angular rate around x (rad/s)
  LININT(wy);        //<!  angular rate around y (rad/s)
  LININT(wz);        //<!  angular rate around z (rad/s)

  LININT(wf);        //<!  angular rate around forward axis (rad/s)
  LININT(wl);        //<!  angular rate around leftward axis (rad/s)
  LININT(wu);        //<!  angular rate around upward axis (rad/s)

  LININT(posacc);    //<!  position accuracy (m)
  LININT(velacc);    //<!  velocity accuracy (north/east in m/s)

#undef LININT
}





//@brief Search oxts by time stamp assuming oxts[] is sorted by increasing oxts.ts,
//   interpolate for specific ts, and return interpolated values into oxtsi.
//
// Note that actually the extrapolation will made if ts is out of [oxts1..oxts2] timestamps range
//
// This function returns false if requested ts is not in the range covered by oxts[] timestamps,
// therefore data stored in oxtsi are extrapolated (not interpolated)
bool oxts_interpolate_for_timestamp(const std::vector<oxts_data> & oxts, double ts,
    /*out*/ oxts_data * oxtsi)
{
  // typical 100 ms for KITTI
  const double OXTS_TIME_LAG  = 0.1;

//  size_t pos1, pos2;

  if ( oxts.empty() ) {
    return false;
  }

  if ( ts <= oxts.front().ts ) {
    if ( oxts.size() > 1 ) { // try extrapolate into past
      oxts_interpolate(oxts[0], oxts[1], ts, oxtsi);
    }
    else {
      *oxtsi = oxts.front();
    }
    return ts > oxts.front().ts - OXTS_TIME_LAG;
  }

  if ( ts >= oxts.back().ts ) {
    if ( oxts.size() > 1 ) { // try extrapolate into future
      oxts_interpolate(oxts[oxts.size() - 2], oxts[oxts.size() - 1], ts, oxtsi);
    }
    else {
      *oxtsi = oxts.back();
    }
    return ts < oxts.back().ts + OXTS_TIME_LAG;
  }




  typedef std::vector<oxts_data>::const_iterator
      OxtsArrayIterator;

  // first position at which pos2->ts > ts
  OxtsArrayIterator pos2 = oxts_upper_bound(oxts.begin(), oxts.end(), ts);

  // roll back to position where pos1->ts <= ts
  OxtsArrayIterator pos1 = pos2;
  while ( pos1 > oxts.begin() && pos1->ts > ts ) {
    --pos1;
  }

  if ( pos1->ts == ts ) { // exact match
    *oxtsi = *pos1;
    return true;
  }

  oxts_interpolate(*pos1, *pos2, ts, oxtsi);

  return true;
}




//
// OXTS PATH INTEGRATOR STUFF.
// NOT DOCUMENTED BECAUSE IS STILL EXPERIMENTAL,
// DO NOT REVIEW OR USE IT
//
static inline void oxts_integrator_update_translation_forward(
    const oxts_data * start_frame,
    const oxts_data * current_frame,
    const oxts_data * next_frame,
    /*in, out*/ cv::Vec3d * Translation)
{
  Matx33d L, T, Y;

  build_rotation((current_frame->roll - start_frame->roll),
      (current_frame->pitch - start_frame->pitch),
      (current_frame->yaw - start_frame->yaw),
      &L, &T, &Y);

  const double dT = 0.5 * (next_frame->ts - current_frame->ts);

  *Translation += dT * L * T * Y * Vec3d(
      current_frame->vf + next_frame->vf,
      current_frame->vl + next_frame->vl,
      current_frame->vu + next_frame->vu);
}

//
// OXTS PATH INTEGRATOR STUFF.
// NOT DOCUMENTED BECAUSE IS STILL EXPERIMENTAL,
// DO NOT REVIEW OR USE IT
//
static inline void oxts_integrator_update_translation_backward(
    const oxts_data * last_frame,
    const oxts_data * current_frame,
    const oxts_data * prev_frame,
    /*in, out*/ cv::Vec3d * Translation)
{
  Matx33d L, T, Y;

  build_rotation((prev_frame->roll - last_frame->roll),
      (prev_frame->pitch - last_frame->pitch),
      (prev_frame->yaw - last_frame->yaw),
      &L, &T, &Y);

  const double dT = 0.5 * (prev_frame->ts - current_frame->ts);

  *Translation += dT * L * T * Y * Vec3d(
      current_frame->vf + prev_frame->vf,
      current_frame->vl + prev_frame->vl,
      current_frame->vu + prev_frame->vu);
}


//
// OXTS PATH INTEGRATOR STUFF.
// NOT DOCUMENTED BECAUSE IS STILL EXPERIMENTAL,
// DO NOT REVIEW OR USE IT
//
void oxts_integrate(const oxts_data & oxts_start, const oxts_data & oxts_end,
    const std::vector<oxts_data> & oxts_path,
    /* out */cv::Vec3d * _CumulativeTranslation,
    /* out */cv::Matx33d * _CumulativeRotation)
{

  //
  // check if oxts_start and oxts_end are the same
  //
  if ( fabs(oxts_start.ts - oxts_end.ts) < FLT_EPSILON ) {
    if ( _CumulativeTranslation ) {
      *_CumulativeTranslation = Vec3d::all(0);
    }
    if ( _CumulativeRotation ) {
      *_CumulativeRotation = Matx33d::eye();
    }
    return;
  }


  //
  //  Compute total rotation
  //
  if ( _CumulativeRotation ) {
    cv::Matx33d L, T, Y;
    build_rotation(oxts_start.roll - oxts_end.roll,
        oxts_start.pitch - oxts_end.pitch,
        oxts_start.yaw - oxts_end.yaw,
        &L, &T, &Y);
    *_CumulativeRotation = L * T * Y;
  }




  //
  // Accumulate translations
  //
  if ( _CumulativeTranslation ) {

    typedef std::vector<oxts_data>::const_iterator
        OxtsArrayIterator;

    cv::Vec3d Translation = Vec3d::all(0);


    //
    // Check the direction for vector integration
    //

    if ( oxts_start.ts < oxts_end.ts ) {  // from past to future

      const oxts_data * first_frame = &oxts_start;
      const oxts_data * last_frame = &oxts_end;
      const oxts_data * current_frame = first_frame;

      // first position at which beg->ts > first_frame->ts
      const OxtsArrayIterator beg = oxts_upper_bound(oxts_path.begin(), oxts_path.end(),
          first_frame->ts);

      if ( beg < oxts_path.end() ) {

        // first position at which end->ts > last_frame->ts
        const OxtsArrayIterator end = oxts_upper_bound(beg, oxts_path.end(), last_frame->ts);

        for ( OxtsArrayIterator next_frame = beg; next_frame < end; ++next_frame ) {
          oxts_integrator_update_translation_forward(first_frame, current_frame, &*next_frame, &Translation);
          current_frame = &*next_frame;
        }
      }

      if ( current_frame->ts < last_frame->ts ) {
        oxts_integrator_update_translation_forward(first_frame, current_frame, last_frame, &Translation);
      }
    }
    else {  // from future to past

      const oxts_data * first_frame = &oxts_end;
      const oxts_data * last_frame = &oxts_start;
      const oxts_data * current_frame = last_frame;

      // returns first position at which end->ts > last_frame->ts
      const OxtsArrayIterator end = oxts_upper_bound(oxts_path.begin(), oxts_path.end(),
          last_frame->ts);

      if ( end > oxts_path.begin() ) {

        // first position at which beg->ts > first_frame->ts
        const OxtsArrayIterator beg = oxts_upper_bound(oxts_path.begin(), end, first_frame->ts);

        for ( OxtsArrayIterator prev_frame = end - 1; prev_frame > beg; --prev_frame ) {
          oxts_integrator_update_translation_backward(last_frame, current_frame, &*prev_frame, &Translation);
          current_frame = &*prev_frame;
        }
      }

      if ( current_frame->ts > first_frame->ts ) {
        oxts_integrator_update_translation_backward(last_frame, current_frame, first_frame, &Translation);
      }
    }

    *_CumulativeTranslation = Translation;
  }

}

///////////////////////////////////////////////////////////////////////////////

