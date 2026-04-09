/*
 * c_hdl_data_frame.cc
 *
 *  Created on: Feb 25, 2024
 *      Author: amyznikov
 */

#include "c_hdl_data_frame.h"
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


c_hdl_data_frame::c_hdl_data_frame()
{
  cleanup();
}

void c_hdl_data_frame::set_current_lidar(const c_hdl_specification & lidar)
{
  _current_lidar = lidar;
  _range_image.set_lidar_specifcation(&_current_lidar);
}


void c_hdl_data_frame::setup_default_channels()
{
  _image_displays.clear();

  add_image_display("RAY_ELEVATIONS",
      "Raw laser ray elevations in radians (no offset correction)",
      -CV_PI / 2, +CV_PI / 2);

  add_image_display("DEPTH",
      "3D Point depth",
      0, 300);

  add_image_display("INTENSITY",
      "3D Point intensity",
      0, 1);

  add_image_display("DISTANCES",
      "3D Point distance",
      0, 300);

  add_image_display("X",
      "3D Point X",
      -5, 150);

  add_image_display("Y",
      "3D Point Y",
      -150, 150);

  add_image_display("Z",
      "3D Point Z",
      -5, 15);

  add_image_display("HEIGHT",
      "3D Point height",
      -5, 15);

  add_image_display("AZIMUTH",
      "3D Point azimuth angle",
      0, 2 * M_PI);

  add_image_display("ELEVATION",
      "3D Point elevation angle",
      -25 * M_PI, 25 * M_PI);

  add_image_display("LASER_ID",
      "3D Point laser_id",
      0, 128);

  add_image_display("LASER_RING",
      "3D Point laser ring",
      0, 128);

  add_image_display("DATABLOCK",
      "3D Point Lidar Data block id",
      0, 12);

  add_image_display("TIMESTAMP",
      "3D Point time stamp relative to start of rotation",
      0, 100);

  add_image_display("GSLOPES",
      "Local surface horizontal slope",
      -M_PI, +M_PI);


//  add_display_channel("SELECTION_MASK",
//      "1- or 3-channel binary 2D Image representing current selection mask",
//      0, 255);

  _display_types.clear();
  _display_types.emplace(DisplayType_Image);
  _display_types.emplace(DisplayType_PointCloud);

  _range_image.set_azimuthal_resolution(c_hdl_range_image::default_azimuthal_resolution());
  _range_image.set_start_azimuth(c_hdl_range_image::default_start_azimuth());
}

bool c_hdl_data_frame::get_image(const std::string & display_name,
    cv::OutputArray output_image,
    cv::OutputArray output_mask,
    cv::OutputArray output_data)
{
  if (base::get_image(display_name, output_image, output_mask, output_data)) {
    return true;
  }

  cv::Mat1f im1;
  cv::Mat1b im2;
  cv::Mat1b m;

  bool fOk = true;

  //_range_image.set_lidar_specifcation(&_current_lidar);


  if ( display_name == "RAY_ELEVATIONS" ) {
    _range_image.build_ray_elevations(im1);
  }
  else if ( display_name == "INTENSITY" ) {
    _range_image.build_intensity(_current_frame->points, im1, &m);
  }
  else if( display_name == "DISTANCES" ) {
    _range_image.build_distances(_current_frame->points, im1, &m);
  }
  else if( display_name == "X" ) {
    _range_image.build_x(_current_frame->points, im1, &m);
  }
  else if( display_name == "Y" ) {
    _range_image.build_y(_current_frame->points, im1, &m);
  }
  else if( display_name == "Z" ) {
    _range_image.build_z(_current_frame->points, im1, &m);
  }
  else if( display_name == "HEIGHT" ) {
    _range_image.build_heights(_current_frame->points, im1, &m);
  }
  else if( display_name == "AZIMUTH" ) {
    _range_image.build_azimuths(_current_frame->points, im1, &m);
  }
  else if( display_name == "ELEVATION" ) {
    _range_image.build_elevations(_current_frame->points, im1, &m);
  }
  else if( display_name == "LASER_ID" ) {
    _range_image.build_lazerids(_current_frame->points, im2, &m);
  }
  else if( display_name == "LASER_RING" ) {
    _range_image.build_lazer_rings(_current_frame->points, im2, &m);
  }
  else if( display_name == "DATABLOCK" ) {
    _range_image.build_datablocks(_current_frame->points, im2, &m);
  }
  else if( display_name == "TIMESTAMP" ) {
    _range_image.build_timestamps(_current_frame->points, im1, &m);
  }
  else if( display_name == "GSLOPES" ) {
    _range_image.build_gslopes(_current_frame->points, im1, &m);
  }
  else if( display_name == "DEPTH" ) {
    _range_image.build_depths(_current_frame->points, im1, &m);
  }
  else {
    fOk = false;
  }

  if( !im1.empty() ) {
    output_image.move(im1);
  }
  else if( !im2.empty() ) {
    output_image.move(im2);
  }

  if ( !output_image.empty() ) {
    cv::rotate(output_image.getMat(),  output_image,
        cv::ROTATE_90_COUNTERCLOCKWISE);
  }

  if ( output_mask.needed() ) {

    if( !m.empty() ) {
      cv::rotate(m, m, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    copy_output_mask(m, output_mask);
  }

  return fOk;
}


bool c_hdl_data_frame::get_point_cloud(const std::string & display_name,
    cv::OutputArray output_points,
    cv::OutputArray output_colors,
    cv::OutputArray output_mask,
    std::vector<uint64_t> * output_pids)
{

  if ( output_pids ) {
    output_pids->clear();
  }

  if ( base::get_point_cloud(display_name, output_points, output_colors, output_mask) ) {
    return true;
  }


  std::vector<cv::Vec3f> _points;

  convert_to_cartesian(_current_frame->points, _points);

  if( output_points.needed() ) {

    cv::Mat(_points).copyTo(output_points);

//    std::vector<cv::Mat> positions(1, cv::Mat(_points));
//    setItems("_points", output_points, positions);
  }



  if( output_colors.needed() ) {

    std::vector<float> colors(_current_frame->points.size());

    //    CF_DEBUG("display_name=%s", display_name.c_str());

    if( display_name == "INTENSITY" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].intensity;
      }

    }
    else if( display_name == "DISTANCES" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].distance;
      }

    }
    else if( display_name == "X" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _points[i][0];
      }

    }
    else if( display_name == "Y" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _points[i][1];
      }
    }
    else if( display_name == "Z" || display_name == "HEIGHT"  ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _points[i][2];
      }
    }

    else if( display_name == "AZIMUTH" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].azimuth;
      }

    }
    else if( display_name == "ELEVATION" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].elevation;
      }
    }
    else if( display_name == "LASER_ID" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].laser_id;
      }
    }
    else if( display_name == "LASER_RING" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].laser_ring;
      }
    }
    else if( display_name == "DATABLOCK" ) {

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].datablock;
      }
    }
    else if( display_name == "TIMESTAMP" ) {

      double tsmin = DBL_MAX;
      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        const double ts = _current_frame->points[i].timestamp;
        if( ts < tsmin ) {
          tsmin = ts;
        }
      }

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = _current_frame->points[i].timestamp - tsmin;
      }
    }
    else if( display_name == "GSLOPES" ) {

      cv::Mat1f im1;
      cv::Mat1b m;
      int r, c;

      //_range_image.set_lidar_specifcation(&_current_lidar);
      _range_image.build_gslopes(_current_frame->points, im1, &m);

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        if( _range_image.project(_current_frame->points[i], &r, &c) ) {
          colors[i] = im1[r][c];
        }
      }

    }
    else if ( display_name == "RAY_ELEVATIONS" ) {

      const auto & lidar = _current_lidar;

      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        const auto & p = _current_frame->points[i];
        if ( p.laser_id >= 0 && p.laser_id < lidar.lasers.size() ) {
          colors[i] = lidar.lasers[p.laser_id].vert_correction * CV_PI / 180;
        }
      }
    }

    else // if( display_name == "DEPTH" )
    {
      for( int i = 0, n = _current_frame->points.size(); i < n; ++i ) {
        colors[i] = compute_depth(_current_frame->points[i]);
      }
    }

    cv::Mat(colors).copyTo(output_colors);
  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}




