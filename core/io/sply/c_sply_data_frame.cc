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
  _display_types.emplace(DisplayType_PointCloud);

  add_image_display("COLOR",
      "Point Color",
      -1, -1);

  add_image_display("DISTANCE",
      "Distance to each point",
      -1, -1);

  add_image_display("DEPTH",
      "3D Point depth",
      0, 300);

  add_image_display("HEIGHT",
      "3D Point height",
      -5, 15);

//  add_image_display("AZIMUTH",
//      "3D Point azimuth angle",
//      0, 2 * M_PI);

//  add_image_display("ELEVATION",
//      "3D Point elevation angle",
//      -25 * M_PI, 25 * M_PI);

//  add_image_display("GSLOPE",
//      "Local surface horizontal slope",
//      -M_PI, +M_PI);

  add_image_display("X",
      "X coordinate",
      -1, -1);

  add_image_display("Y",
      "Y coordinate",
      -1, -1);

  add_image_display("Z",
      "Z coordinate",
      -1, -1);

}


template<class T>
static bool extract_distances_(const cv::Mat & _pts, cv::Mat & _distances)
{
  const cv::Mat_<cv::Vec<T, 3>> pts = _pts;

  _distances.create(pts.size().area(), 1, pts.depth());

  cv::Mat_<T> distances =
      _distances;

  int i = 0;

  for( int y = 0; y < pts.rows; ++y ) {
    for( int x = 0; x < pts.cols; ++x ) {

      const cv::Vec<T, 3> & v =
          pts[y][x];

      distances[i++][0] =
          std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
  }

  return true;
}

static bool extract_distances(const cv::Mat & pts, cv::Mat & distances)
{
  switch (pts.type()) {
    case CV_32FC3:
      return extract_distances_<float>(pts, distances);
    case CV_64FC3:
      return extract_distances_<double>(pts, distances);
  }

  return false;
}

template<class T>
static bool extract_depth_(const cv::Mat & _pts, cv::Mat & _depth)
{
  const cv::Mat_<cv::Vec<T, 3>> pts = _pts;

  _depth.create(pts.size().area(), 1, pts.depth());

  cv::Mat_<T> depth =
      _depth;

  int i = 0;

  for( int y = 0; y < pts.rows; ++y ) {
    for( int x = 0; x < pts.cols; ++x ) {

      const cv::Vec<T, 3> & v =
          pts[y][x];

      depth[i++][0] =
          std::sqrt(v[0] * v[0] + v[1] * v[1]);
    }
  }

  return true;
}

static bool extract_depth(const cv::Mat & pts, cv::Mat & distances)
{
  switch (pts.type()) {
    case CV_32FC3:
      return extract_distances_<float>(pts, distances);
    case CV_64FC3:
      return extract_distances_<double>(pts, distances);
  }

  return false;
}

bool c_sply_data_frame::get_point_cloud(const std::string & display_name,
    cv::OutputArray output_points,
    cv::OutputArray output_colors,
    cv::OutputArray output_mask,
    std::vector<uint64_t> * output_pids)
{
  if( output_points.needed() ) {
    setItems("_points", output_points, _points);
  }

  if( output_colors.needed() ) {

    if( display_name == "DISTANCE" ) {

      std::vector<cv::Mat> m(_points.size());

      for ( size_t i = 0, n = _points.size(); i < n; ++i ) {
        extract_distances(_points[i], m[i]);
      }

      setItems("_distances", output_colors, m);

    }
    else if( display_name == "DEPTH" ) {

      std::vector<cv::Mat> m(_points.size());

      for ( size_t i = 0, n = _points.size(); i < n; ++i ) {
        extract_depth(_points[i], m[i]);
      }

      setItems("_depths", output_colors, m);
    }

    else if( display_name == "X" ) {

      std::vector<cv::Mat> m(_points.size());

      for ( size_t i = 0, n = _points.size(); i < n; ++i ) {
        cv::extractChannel(_points[i], m[i], 0);
      }

      setItems("X", output_colors, m);
    }
    else if( display_name == "Y" ) {

      std::vector<cv::Mat> m(_points.size());

      for ( size_t i = 0, n = _points.size(); i < n; ++i ) {
        cv::extractChannel(_points[i], m[i], 1);
      }

      setItems("Y", output_colors, m);
    }
    else if( display_name == "Z" ) {

      std::vector<cv::Mat> m(_points.size());

      for ( size_t i = 0, n = _points.size(); i < n; ++i ) {
        cv::extractChannel(_points[i], m[i], 2);
      }

      setItems("Z", output_colors, m);
    }
    else  {

      setItems("_colors", output_colors, _colors);
    }
  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}

