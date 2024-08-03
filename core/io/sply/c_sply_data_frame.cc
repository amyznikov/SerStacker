/*
 * c_sply_data_frame.cc
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#include "c_sply_data_frame.h"
#include <core/ssprintf.h>
#include <core/debug.h>

static bool setItems(const char * output_name, cv::OutputArrayOfArrays a, const std::vector<cv::Mat> & vec)
{
//    CF_DEBUG("%s: needed=%d fixedType=%d fixedSize=%d kind=%d (%s)",
//        output_name,
//        a.needed(),
//        a.fixedType(),
//        a.fixedSize(),
//        a.kind(),
//        toCString(a.kind()));

  if( a.needed() ) {

    if( vec.empty() ) {
      a.release();
    }
    else {

      switch (a.kind()) {
        case cv::_InputArray::MAT:

          if ( !a.fixedType() ) {
            a.assign(vec[0]);
          }
          else if ( vec[0].channels() == a.channels() ) {
            vec[0].convertTo(a, a.type());
          }
          else if ( vec[0].channels() == 1  ) {
            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = vec[0];
            }

            cv::Mat tmp;
            cv::merge(channels, tmp);

            if ( a.depth() != tmp.depth() ) {
              a.assign(tmp);
            }
            else {
              tmp.convertTo(a, a.depth());
            }
          }
          else {
            CF_ERROR("Not supported output array channels for '%s': fixedType=%d fixedSize=%d kind=%d (%s) src.channels=%d dst.channels=%d",
                output_name,
                a.fixedType(),
                a.fixedSize(),
                a.kind(),
                toCString(a.kind()),
                vec[0].channels(),
                a.channels());

            return false;
          }

          break;

        case cv::_InputArray::STD_VECTOR_MAT:
          a.createSameSize(vec, vec[0].type());
          a.assign(vec);
          break;

        case cv::_InputArray::STD_VECTOR:
          if ( vec[0].channels() == a.channels() ) {
            vec[0].convertTo(a, a.type());
          }
          else if ( vec[0].channels() == 1  ) {

            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = vec[0];
            }

            cv::Mat tmp;
            cv::merge(channels, tmp);
            tmp.convertTo(a, a.type());
          }
          else {
            CF_ERROR("Not supported output array channels for '%s': fixedType=%d fixedSize=%d kind=%d (%s) src.channels=%d dst.channels=%d",
                output_name,
                a.fixedType(),
                a.fixedSize(),
                a.kind(),
                toCString(a.kind()),
                vec[0].channels(),
                a.channels());

            return false;
          }

          break;

        default:
          CF_ERROR("Not supported output array kind for '%s': fixedType=%d fixedSize=%d kind=%d (%s)",
              output_name,
              a.fixedType(),
              a.fixedSize(),
              a.kind(),
              toCString(a.kind()));

          return false;
      }

    }

  }


  return true;
}



c_sply_data_frame::c_sply_data_frame()
{
  display_types_.emplace(DisplayType_PointCloud);

  add_display_channel("COLOR",
      "Point Color",
      -1, -1);

  add_display_channel("DISTANCE",
      "Distance to each point",
      -1, -1);

  add_display_channel("DEPTH",
      "3D Point depth",
      0, 300);

  add_display_channel("HEIGHT",
      "3D Point height",
      -5, 15);

//  add_display_channel("AZIMUTH",
//      "3D Point azimuth angle",
//      0, 2 * M_PI);

//  add_display_channel("ELEVATION",
//      "3D Point elevation angle",
//      -25 * M_PI, 25 * M_PI);

//  add_display_channel("GSLOPE",
//      "Local surface horizontal slope",
//      -M_PI, +M_PI);

  add_display_channel("X",
      "X coordinate",
      -1, -1);

  add_display_channel("Y",
      "Y coordinate",
      -1, -1);

  add_display_channel("Z",
      "Z coordinate",
      -1, -1);

}


bool c_sply_data_frame::get_point_cloud(const std::string & display_name,
    cv::OutputArrayOfArrays output_points,
    cv::OutputArrayOfArrays output_colors,
    cv::OutputArrayOfArrays output_mask)
{
  if( output_points.needed() ) {
    setItems("_points", output_points, _points);
  }

  if( output_colors.needed() ) {

    if( display_name == "DISTANCE" ) {

//      cv::Mat1f distances(points_.size(), 1);
//
//      for ( int i = 0, n = points_.size(); i < n; ++i ) {
//        distances[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
//            points_[i][1] * points_[i][1] +
//            points_[i][2] * points_[i][2]);
//      }
//
//      output_colors.move(distances);
    }
    else if( display_name == "DEPTH" ) {

//      cv::Mat1f depths(points_.size(), 1);
//
//      for ( int i = 0, n = points_.size(); i < n; ++i ) {
//        depths[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
//            points_[i][1] * points_[i][1] +
//            points_[i][2] * points_[i][2]);
//      }
//
//      output_colors.move(depths);
    }

    else if( display_name == "X" ) {
      //cv::extractChannel(cv::Mat(points_), output_colors, 0);
    }
    else if( display_name == "Y" ) {
      //cv::extractChannel(cv::Mat(points_), output_colors, 1);
    }
    else if( display_name == "Z" ) {
      //cv::extractChannel(cv::Mat(points_), output_colors, 2);
    }
    else  {
      //cv::Mat(_colors).copyTo(output_colors);
      setItems("_colors", output_colors, _colors);
    }
  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}

