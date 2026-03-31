/*
 * c_hdl_ground_test_routine.cc
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#include "c_hdl_ground_test_routine.h"
#include<core/debug.h>

static void detectRoad(const cv::Mat1f & Z, const cv::Mat1b & M,
    cv::Mat1b & outputRoadMask,
    float H0,
    float Zroof,
    float gtol)
{
  const int ROWS = Z.rows;
  const int COLS = Z.cols;

  outputRoadMask.create(Z.size());
  outputRoadMask.setTo(0);

  const float Z_GROUND_TARGET = -H0;
  const int MAX_GAP_PX = 4;

  for( int c = 0; c < COLS; ++c ) {
    int state = 0;
    float lastValidZ = Z_GROUND_TARGET;
    int gapCount = 0;

    for( int r = 0; r < ROWS; ++r ) {
      bool isValid = M(r, c) > 0;
      float curZ = Z(r, c);

      if( state == 0 ) {
        if( isValid && curZ < Zroof - 0.1f ) {
          state = 1;
        }
        else {
          continue;
        }
      }

      if( state == 1 ) {
        if( isValid ) {
          if( std::abs(curZ - Z_GROUND_TARGET) < 0.25f ) {
            state = 2;
            lastValidZ = curZ;
            outputRoadMask(r, c) = 255;
          }
        }
        continue;
      }

      if( state == 2 ) {
        if( isValid ) {
#if 1

          float diff = curZ - lastValidZ;

          if( (diff >= 0 && diff < gtol) || (diff < 0 && diff > -2 * gtol) ) {
            outputRoadMask(r, c) = 255;
            lastValidZ = curZ;
            gapCount = 0;
          }
          else {
            gapCount++;
          }
#else
          float diff = std::abs(curZ - lastValidZ);

          if( diff < gtol ) {
            outputRoadMask(r, c) = 255;
            lastValidZ = curZ;
            gapCount = 0;
          }
          else {
            gapCount++;
          }
#endif

        }
        else {
          gapCount++;
        }

        if( gapCount > MAX_GAP_PX ) {
          state = 1;
          gapCount = 0;
        }
      }
    }
  }
}


void c_hdl_ground_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "H0", ctx(&this_class::H0), "Sensor height in meters");
  ctlbind(ctls, "Zroof", ctx(&this_class::Zroof), "Z-coordinate of proper car roof in meters");
  ctlbind(ctls, "gtol", ctx(&this_class::gtol), "Ground tolerance in meters - max allowed variation of the ground between consecutive points");
}

bool c_hdl_ground_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, H0);
    SERIALIZE_OPTION(settings, save, *this, Zroof);
    SERIALIZE_OPTION(settings, save, *this, gtol);
//    SERIALIZE_OPTION(settings, save, *this, updateSelectionMask);
    return true;
  }
  return false;
}

bool c_hdl_ground_test_routine::process(c_hdl_data_frame * hdl)
{
  const c_hdl_range_image & range_image = hdl->range_image();

  cv::Mat1f Z;
  cv::Mat1b M, Mroad;

  range_image.build_z(hdl->current_frame()->points, Z, &M);

  detectRoad(Z, M, Mroad, H0, Zroof, gtol);

  std::vector<cv::Vec3f> points, colors;

  for ( const auto & p : hdl->current_frame()->points ) {
    int r, c;
    if ( range_image.project(p, &r, &c) ) {
      const float isRoad = Mroad(r,c) ? 1 : 0;
      points.emplace_back(compute_cartesian(p));
      colors.emplace_back(cv::Vec3f(isRoad, isRoad, isRoad));
    }
  }

  hdl->add_point_cloud("GND_Z",
      points,
      colors,
      cv::noArray());

  cv::rotate(Z, Z, cv::ROTATE_90_COUNTERCLOCKWISE);
  //cv::rotate(M, M, cv::ROTATE_90_COUNTERCLOCKWISE);
  cv::rotate(Mroad, Mroad, cv::ROTATE_90_COUNTERCLOCKWISE);
  hdl->add_image("GND_Z", Z, Mroad);



//  cv::Mat1f ZM = Z.clone();
//  ZM.setTo(0, ~Mroad);
//  ZM.setTo(1, Mroad);
//  hdl->add_image("GND_ZM", ZM, Mroad);

//  if( updateSelectionMask ) {
//    hdl->update_selection(Mroad, c_data_frame::SELECTION_MASK_AND);
//  }


  return true;
}


//bool c_hdl_ground_test_routine::process(c_hdl_data_frame * hdl)
//{
//  const c_hdl_range_image & range_image = hdl->range_image();
//
//  cv::Mat1f D, RD, E, dRD, dE, dRDE, dRDEX;
//  cv::Mat1b M;
//
//  range_image.build_distances(hdl->current_frame()->points, D, &M);
//  range_image.build_ray_elevations(E);
//  E.forEach([](float &val, const int * position) {
//      val = std::sin(val);
//  });
//
//  cv::divide(1.0, D, RD);
//  RD.setTo(0, D == 0);
//
//  static float Kc[2] = { -1, 1 };
//  static const cv::Mat1f K(1, 2, Kc);
//
//  cv::filter2D(RD, dRD, CV_32F, K, cv::Point(1, 0), 0, cv::BORDER_REPLICATE);
//  cv::filter2D(E, dE, CV_32F, -K, cv::Point(1, 0), 1e-5, cv::BORDER_REPLICATE);
//  cv::divide(dRD, dE, dRDE);
//
//  dRDEX.create(dRDE.rows, dRDE.cols);
//  for( int y = 0; y < dRDE.rows; ++y ) {
//    const float * srcp = dRDE[y];
//    float * dstp = dRDEX[y];
//    for( int x = 0; x < dRDE.cols; ++x ) {
//      dstp[x] = x * srcp[x];
//    }
//  }
//
//
//  cv::rotate(D, D, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(RD, RD, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(E, E, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(dRD, dRD, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(dE, dE, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(dRDE, dRDE, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(dRDEX, dRDEX, cv::ROTATE_90_COUNTERCLOCKWISE);
//  cv::rotate(M, M, cv::ROTATE_90_COUNTERCLOCKWISE);
//
//  hdl->add_image("GND_D", D, M);
//  hdl->add_image("GND_RD", RD, M);
//  hdl->add_image("GND_SE", E, M);
//  hdl->add_image("GND_dRD", dRD, M);
//  hdl->add_image("GND_dE", dE, M);
//  hdl->add_image("GND_dRDE", dRDE, M);
//  hdl->add_image("GND_dRDEX",dRDEX, M);
//
//  return true;
//}
