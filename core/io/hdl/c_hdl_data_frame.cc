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

void c_hdl_data_frame::cleanup()
{
  setup_default_channels();
}

void c_hdl_data_frame::setup_default_channels()
{
  data_displays_.clear();

  add_display_channel("DEPTH",
      "3D Point depth",
      0, 300);

  add_display_channel("INTENSITY",
      "3D Point intensity",
      0, 1);

  add_display_channel("DISTANCES",
      "3D Point distance",
      0, 300);

  add_display_channel("X",
      "3D Point X",
      -5, 150);

  add_display_channel("Y",
      "3D Point Y",
      -150, 150);

  add_display_channel("Z",
      "3D Point Z",
      -5, 15);

  add_display_channel("HEIGHT",
      "3D Point height",
      -5, 15);

  add_display_channel("AZIMUTH",
      "3D Point azimuth angle",
      0, 2 * M_PI);

  add_display_channel("ELEVATION",
      "3D Point elevation angle",
      -25 * M_PI, 25 * M_PI);

  add_display_channel("LASER_ID",
      "3D Point laser_id",
      0, 128);

  add_display_channel("LASER_RING",
      "3D Point laser ring",
      0, 128);

  add_display_channel("DATABLOCK",
      "3D Point Lidar Data block id",
      0, 12);

  add_display_channel("TIMESTAMP",
      "3D Point time stamp relative to start of rotation",
      0, 100);

  add_display_channel("GSLOPES",
      "Local surface horizontal slope",
      -M_PI, +M_PI);


//  add_display_channel("SELECTION_MASK",
//      "1- or 3-channel binary 2D Image representing current selection mask",
//      0, 255);

  display_types_.clear();
  display_types_.emplace(DisplayType_Image);
  display_types_.emplace(DisplayType_PointCloud);

  range_image_.set_azimuthal_resolution(c_hdl_range_image::default_azimuthal_resolution());
  range_image_.set_start_azimuth(c_hdl_range_image::default_start_azimuth());
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

  range_image_.set_lidar_specifcation(&current_lidar_);


  if ( display_name == "INTENSITY" ) {
    range_image_.build_intensity(current_frame_->points, im1, &m);
  }
  else if( display_name == "DISTANCES" ) {
    range_image_.build_distances(current_frame_->points, im1, &m);
  }
  else if( display_name == "X" ) {
    range_image_.build_x(current_frame_->points, im1, &m);
  }
  else if( display_name == "Y" ) {
    range_image_.build_y(current_frame_->points, im1, &m);
  }
  else if( display_name == "Z" ) {
    range_image_.build_z(current_frame_->points, im1, &m);
  }
  else if( display_name == "HEIGHT" ) {
    range_image_.build_heights(current_frame_->points, im1, &m);
  }
  else if( display_name == "AZIMUTH" ) {
    range_image_.build_azimuths(current_frame_->points, im1, &m);
  }
  else if( display_name == "ELEVATION" ) {
    range_image_.build_elevations(current_frame_->points, im1, &m);
  }
  else if( display_name == "LASER_ID" ) {
    range_image_.build_lazerids(current_frame_->points, im2, &m);
  }
  else if( display_name == "LASER_RING" ) {
    range_image_.build_lazer_rings(current_frame_->points, im2, &m);
  }
  else if( display_name == "DATABLOCK" ) {
    range_image_.build_datablocks(current_frame_->points, im2, &m);
  }
  else if( display_name == "TIMESTAMP" ) {
    range_image_.build_timestamps(current_frame_->points, im1, &m);
  }
  else if( display_name == "GSLOPES" ) {
    range_image_.build_gslopes(current_frame_->points, im1, &m);
  }
  else if( display_name == "DEPTH" ) {
    range_image_.build_depths(current_frame_->points, im1, &m);
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
    std::vector<std::vector<uint64_t>> * output_pids)
{

  std::vector<cv::Vec3f> _points;

  convert_to_cartesian(current_frame_->points, _points);

  if( output_points.needed() ) {

    std::vector<cv::Mat> positions(1, cv::Mat(_points));
    setItems("_points", output_points, positions);
  }

  if( output_colors.needed() ) {

    std::vector<float> colors(current_frame_->points.size());

    //    CF_DEBUG("display_name=%s", display_name.c_str());

    if( display_name == "INTENSITY" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].intensity;
      }

    }
    else if( display_name == "DISTANCES" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].distance;
      }

    }
    else if( display_name == "X" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = _points[i][0];
      }

    }
    else if( display_name == "Y" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = _points[i][1];
      }
    }
    else if( display_name == "Z" || display_name == "HEIGHT"  ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = _points[i][2];
      }
    }

    else if( display_name == "AZIMUTH" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].azimuth;
      }

    }
    else if( display_name == "ELEVATION" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].elevation;
      }
    }
    else if( display_name == "LASER_ID" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].laser_id;
      }
    }
    else if( display_name == "LASER_RING" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].laser_ring;
      }
    }
    else if( display_name == "DATABLOCK" ) {

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].datablock;
      }
    }
    else if( display_name == "TIMESTAMP" ) {

      double tsmin = DBL_MAX;
      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        const double ts = current_frame_->points[i].timestamp;
        if( ts < tsmin ) {
          tsmin = ts;
        }
      }

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = current_frame_->points[i].timestamp - tsmin;
      }
    }
    else if( display_name == "GSLOPES" ) {

      cv::Mat1f im1;
      cv::Mat1b m;
      int r, c;

      range_image_.set_lidar_specifcation(&current_lidar_);
      range_image_.build_gslopes(current_frame_->points, im1, &m);

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        if( range_image_.project(current_frame_->points[i], &r, &c) ) {
          colors[i] = im1[r][c];
        }
      }

    }
//    else if( display_name == "SELECTION_MASK" ) {
//
//      if( selection_mask_.rows != colors.size() ) {
//
//        std::fill(colors.begin(), colors.end(), 255);
//      }
//      else {
//        for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
//
//        }
//
//      }
//    }
    else // if( display_name == "DEPTH" )
    {
      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i] = compute_depth(current_frame_->points[i]);
      }
    }

    std::vector<cv::Mat> _colors(1, cv::Mat(colors));
    setItems("_colors", output_colors, _colors);

  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}

