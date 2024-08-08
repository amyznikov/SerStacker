/*
 * c_kitti_dataset.h
 *
 *  Created on: Aug 4, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_kitti_dataset_h__
#define __c_kitti_dataset_h__


#include <opencv2/opencv.hpp>
#include <memory>
#include "oxts_track.h"

class c_kitti_frame
{
public:
  typedef c_kitti_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_kitti_frame(const std::string & filename = "") :
      _filename(filename)
  {
  }


  static uptr create_uptr(const std::string & filename)
  {
    return uptr(new this_class(filename));
  }

  static sptr create_sptr(const std::string & filename)
  {
    return sptr(new this_class(filename));
  }

  // velodyne points (xxx.bin) path file name
  void set_filename(const std::string & v)
  {
    _filename = v;
  }

  // velodyne points (xxx.bin) path file name
  const std::string & filename() const
  {
    return _filename;
  }

  // velodyne points (xxx.bin) path file name
  const char *cfilename() const
  {
    return _filename.c_str();
  }

  // start frame time stamps floating-point seconds
  void set_tss(double v)
  {
    _tss = v;
  }

  // start frame time stamps floating-point seconds
  double tss() const
  {
    return _tss;
  }

  // end frame time stamps floating-point seconds
  void set_tse(double v)
  {
    _tse = v;
  }

  // end frame time stamps floating-point seconds
  double tse() const
  {
    return _tse;
  }

  // facing-forward frame time stamps floating-point seconds
  void set_tsf(double v)
  {
    _tsf = v;
  }

  // facing-forward frame time stamps floating-point seconds
  double tsf() const
  {
    return _tsf;
  }


  bool load_points(std::vector<cv::Vec3f> & pts, std::vector<float> & reflectances, std::vector<double> & timestamps);


protected:
  friend class c_kitti_dataset;
  std::string _filename;
  double _tss = 0;
  double _tse = 0;
  double _tsf = 0;
};

class c_kitti_dataset
{
public:
  typedef c_kitti_dataset this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_kitti_dataset(const std::string & input_directory = "") :
    _input_directory(input_directory)
  {
  }

  const std::string & input_directory() const
  {
    return _input_directory;
  }

  const std::vector<c_kitti_frame::uptr> & frames()
  {
    return  _frames;
  }

  const c_oxts_track & oxts_track() const
  {
    return _oxts_track;
  }

  bool load(const std::string & input_directory  = "");

  void clear();

protected:
  std::string _input_directory;
  std::string _oxts_directory;
  std::string _velodyne_directory;
  std::string _cam2cam_file_name;
  std::string _velo2cam_file_name;
  std::string _imu2velo_file_name;

  std::vector<c_kitti_frame::uptr> _frames;
  c_oxts_track _oxts_track;
};

#endif /* __c_kitti_dataset_h__ */
