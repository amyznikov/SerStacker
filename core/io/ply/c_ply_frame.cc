/*
 * c_ply_frame.cc
 *
 *  Created on: Jan 24, 2024
 *      Author: amyznikov
 */

#include "c_ply_frame.h"
#include <core/ssprintf.h>
#include <core/debug.h>

static bool setItems(const char * output_name, cv::OutputArrayOfArrays a, const cv::Mat & vec)
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
            a.assign(vec);
          }
          else if ( vec.channels() == a.channels() ) {
            vec.convertTo(a, a.type());
          }
          else if ( vec.channels() == 1  ) {
            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = vec;
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
                vec.channels(),
                a.channels());

            return false;
          }

          break;

        case cv::_InputArray::STD_VECTOR_MAT: {
          std::vector<cv::Mat> v(1, vec);
          a.createSameSize(v, vec.type());
          a.assign(v);
          break;
        }

        case cv::_InputArray::STD_VECTOR:
          if ( vec.channels() == a.channels() ) {
            vec.convertTo(a, a.type());
          }
          else if ( vec.channels() == 1  ) {

            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = vec;
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
                vec.channels(),
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


c_ply_frame::c_ply_frame()
{
  display_types_.emplace(DisplayType_TextFile);
  display_types_.emplace(DisplayType_PointCloud);

  add_display_channel("COLORS",
      "Point Color",
      -1, -1);

  add_display_channel("DISTANCES",
      "3-channel 2D Image with distances to points",
      -1, -1);

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

std::string c_ply_frame::get_filename()
{
  return filename_;
}

bool c_ply_frame::get_point_cloud(const std::string & display_name,
      cv::OutputArrayOfArrays output_points,
      cv::OutputArrayOfArrays output_colors,
      cv::OutputArrayOfArrays output_mask)
{
  if( output_points.needed() ) {
    setItems("points_", output_points, cv::Mat(points_));
  }

  if( output_colors.needed() ) {

    if( display_name == "DISTANCES" ) {

      cv::Mat1f distances(points_.size(), 1);

      for ( int i = 0, n = points_.size(); i < n; ++i ) {
        distances[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
            points_[i][1] * points_[i][1] +
            points_[i][2] * points_[i][2]);
      }

      setItems("distances", output_colors, distances);
      //output_colors.move(distances);
    }
    else if( display_name == "X" ) {
      cv::Mat tmp;
      cv::extractChannel(cv::Mat(points_), tmp, 0);
      setItems("X", output_colors, tmp);
    }
    else if( display_name == "Y" ) {
      cv::Mat tmp;
      cv::extractChannel(cv::Mat(points_), tmp, 1);
      setItems("Y", output_colors, tmp);
    }
    else if( display_name == "Z" ) {
      cv::Mat tmp;
      cv::extractChannel(cv::Mat(points_), tmp, 2);
      setItems("Z", output_colors, tmp);
    }
    else  {
      setItems("colors_", output_colors, cv::Mat(colors_));
    }
  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}
