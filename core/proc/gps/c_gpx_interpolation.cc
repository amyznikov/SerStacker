/*
 * c_gpx_interpolation.cc
 *
 *  Created on: Sep 15, 2024
 *      Author: amyznikov
 */

#include "c_gpx_interpolation.h"
#include <core/debug.h>

void c_gpx_interpolation::set_gpx_track(const c_gpx_track * track)
{
  _gpx_track = track;
}

const c_gpx_track * c_gpx_interpolation::gpx_track() const
{
  return _gpx_track;
}

c_gpx_interpolation::landmark_iterator c_gpx_interpolation::landmark_lower_bound(int videoFrameIndex)
{
  return std::lower_bound(_landmarks.begin(), _landmarks.end(), videoFrameIndex,
      [](const Landmark & landmark, int videoFrameIndex) {
        return landmark.videoFrameIndex < videoFrameIndex;
      });
}

c_gpx_interpolation::landmark_const_iterator c_gpx_interpolation::landmark_lower_bound(int videoFrameIndex) const
{
  return std::lower_bound(_landmarks.begin(), _landmarks.end(), videoFrameIndex,
      [](const Landmark & landmark, int videoFrameIndex) {
        return landmark.videoFrameIndex < videoFrameIndex;
      });
}


void c_gpx_interpolation::set_landmark(int videoFrameIndex, int gpxPointIndex)
{
  const auto pos =
      landmark_lower_bound(videoFrameIndex);

  if( pos != _landmarks.end() && pos->videoFrameIndex == videoFrameIndex ) {
    pos->gpxPointIndex = gpxPointIndex;
  }
  else {
    _landmarks.insert(pos, (Landmark ) { videoFrameIndex, gpxPointIndex });
  }
}

void c_gpx_interpolation::remove_landmark(int videoFrameIndex)
{
  const auto pos =
      landmark_lower_bound(videoFrameIndex);

  if( pos != _landmarks.end() && pos->videoFrameIndex == videoFrameIndex ) {
    _landmarks.erase(pos);
  }

}

//
//bool c_gpx_interpolation::interpolate_for_frame(int videoFrameIndex, c_gps_position * gps) const
//{
//  if ( _landmarks.size() < 2 ) {
//    CF_ERROR("APP BUG: at least 2 landmarks are required for linear interpolation");
//    return false;
//  }
//
//  double target_ts;
//
//  const double timescale =
//      (_gpx_track->pts[_landmarks.back().gpxPointIndex].timestamp
//          - _gpx_track->pts[_landmarks.front().gpxPointIndex].timestamp) /
//          (_landmarks.back().videoFrameIndex - _landmarks.front().videoFrameIndex);
//
//
//  // CF_DEBUG("FPS=%g", 1 / timescale);
//
//  const auto lpos =
//      landmark_lower_bound(videoFrameIndex);
//
//  if ( lpos == _landmarks.end() ) {
//
//    const size_t n =
//        _landmarks.size();
//
//    const int f0 =
//        _landmarks[n - 2].videoFrameIndex;
//
//    const int f1 =
//        _landmarks[n - 1].videoFrameIndex;
//
//    const double ts0 =
//        _gpx_track->pts[_landmarks[n - 2].gpxPointIndex].timestamp;
//
//    const double ts1 =
//        _gpx_track->pts[_landmarks[n - 1].gpxPointIndex].timestamp;
//
//    target_ts =
//        ts1 + (videoFrameIndex - f1) * timescale; //  * (ts1 - ts0) / (f1 - f0) ;
//
//  }
//  else if( lpos->videoFrameIndex == videoFrameIndex ) {
//    // exact match, don't interpolate
//    *gps = _gpx_track->pts[lpos->gpxPointIndex];
//    return true;
//  }
//  else if( lpos == _landmarks.begin() ) {
//
//    const int f0 =
//        _landmarks[0].videoFrameIndex;
//
////    const int f1 =
////        _landmarks[1].videoFrameIndex;
//
//    const double ts0 =
//        _gpx_track->pts[_landmarks[0].gpxPointIndex].timestamp;
//
////    const double ts1 =
////        _gpx_track->pts[_landmarks[1].gpxPointIndex].timestamp;
//
//    target_ts =
//        ts0 + (videoFrameIndex - f0) * timescale; //  * (ts1 - ts0) / (f1 - f0);
//
//  }
//  else {
//
//
//    const int lidx =
//        lpos - _landmarks.begin() - 1 ;
//
//    const int f0 =
//        _landmarks[lidx + 0].videoFrameIndex;
//
////    const int f1 =
////        _landmarks[lidx + 1].videoFrameIndex;
//
//
//    const double ts0 =
//        _gpx_track->pts[_landmarks[lidx + 0].gpxPointIndex].timestamp;
//
////    const double ts1 =
////        _gpx_track->pts[_landmarks[lidx + 1].gpxPointIndex].timestamp;
//
//    target_ts =
//        ts0 + (videoFrameIndex - f0) * timescale; // * (ts1 - ts0) / (f1 - f0);
//
//  }
//
//  const auto gpos =
//      std::lower_bound(_gpx_track->pts.begin(), _gpx_track->pts.end(), target_ts,
//          [](const c_gps_position & gps, double ts) {
//            return gps.timestamp < ts;
//          });
//
//  if( gpos == _gpx_track->pts.begin() ) {
//    *gps = _gpx_track->pts.front();
//  }
//  else if( gpos == _gpx_track->pts.end() ) {
//    *gps = _gpx_track->pts.back();
//  }
//  else if( gpos->timestamp == target_ts ) {
//    *gps = *gpos;
//  }
//  else {
//
//    const int gidx =
//        gpos - _gpx_track->pts.begin() - 1;
//
//    const c_gps_position & gps0 =
//        _gpx_track->pts[gidx + 0];
//
//    const c_gps_position & gps1 =
//        _gpx_track->pts[gidx + 1];
//
//    const double & ts0 =
//        gps0.timestamp;
//
//    const double & ts1 =
//        gps1.timestamp;
//
//    const double fraction =
//        (target_ts - ts0) / (ts1 - ts0);
//
//    gps->latitude =
//        gps0.latitude + (gps1.latitude - gps0.latitude) * fraction;
//
//    gps->longitude =
//        gps0.longitude + (gps1.longitude - gps0.longitude) * fraction;
//
//    gps->altitude =
//        gps0.altitude + (gps1.altitude - gps0.altitude) * fraction;
//
//  }
//
//  return true;
//}


bool c_gpx_interpolation::interpolate_for_frame(int videoFrameIndex, c_gps_position * gps) const
{
  if ( _landmarks.size() < 2 ) {
    CF_ERROR("APP BUG: at least 2 landmarks are required for linear interpolation");
    return false;
  }

  double target_ts;

  const auto lpos =
      landmark_lower_bound(videoFrameIndex);

  if ( lpos == _landmarks.end() ) {

    const size_t n =
        _landmarks.size();

    const int f0 =
        _landmarks[n - 2].videoFrameIndex;

    const int f1 =
        _landmarks[n - 1].videoFrameIndex;

    const double ts0 =
        _gpx_track->pts[_landmarks[n - 2].gpxPointIndex].timestamp;

    const double ts1 =
        _gpx_track->pts[_landmarks[n - 1].gpxPointIndex].timestamp;

    target_ts =
        ts1 + (videoFrameIndex - f1) * (ts1 - ts0) / (f1 - f0) ;

  }
  else if( lpos->videoFrameIndex == videoFrameIndex ) {
    // exact match, don't interpolate
    *gps = _gpx_track->pts[lpos->gpxPointIndex];
    return true;
  }
  else if( lpos == _landmarks.begin() ) {

    const int f0 =
        _landmarks[0].videoFrameIndex;

    const int f1 =
        _landmarks[1].videoFrameIndex;

    const double ts0 =
        _gpx_track->pts[_landmarks[0].gpxPointIndex].timestamp;

    const double ts1 =
        _gpx_track->pts[_landmarks[1].gpxPointIndex].timestamp;

    target_ts =
        ts0 + (videoFrameIndex - f0) * (ts1 - ts0) / (f1 - f0);

  }
  else {


    const int lidx =
        lpos - _landmarks.begin() - 1 ;

    const int f0 =
        _landmarks[lidx + 0].videoFrameIndex;

    const int f1 =
        _landmarks[lidx + 1].videoFrameIndex;


    const double ts0 =
        _gpx_track->pts[_landmarks[lidx + 0].gpxPointIndex].timestamp;

    const double ts1 =
        _gpx_track->pts[_landmarks[lidx + 1].gpxPointIndex].timestamp;

    target_ts =
        ts0 + (videoFrameIndex - f0) * (ts1 - ts0) / (f1 - f0);

  }

  const auto gpos =
      std::lower_bound(_gpx_track->pts.begin(), _gpx_track->pts.end(), target_ts,
          [](const c_gps_position & gps, double ts) {
            return gps.timestamp < ts;
          });

  if( gpos == _gpx_track->pts.begin() ) {
    *gps = _gpx_track->pts.front();
  }
  else if( gpos == _gpx_track->pts.end() ) {
    *gps = _gpx_track->pts.back();
  }
  else if( gpos->timestamp == target_ts ) {
    *gps = *gpos;
  }
  else {

    const int gidx =
        gpos - _gpx_track->pts.begin() - 1;

    const c_gps_position & gps0 =
        _gpx_track->pts[gidx + 0];

    const c_gps_position & gps1 =
        _gpx_track->pts[gidx + 1];

    const double & ts0 =
        gps0.timestamp;

    const double & ts1 =
        gps1.timestamp;

    const double fraction =
        (target_ts - ts0) / (ts1 - ts0);

    gps->latitude =
        gps0.latitude + (gps1.latitude - gps0.latitude) * fraction;

    gps->longitude =
        gps0.longitude + (gps1.longitude - gps0.longitude) * fraction;

    gps->altitude =
        gps0.altitude + (gps1.altitude - gps0.altitude) * fraction;

  }

  return true;
}
