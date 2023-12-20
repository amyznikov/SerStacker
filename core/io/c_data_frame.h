/*
 * c_cloudview_data_frame.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_data_frame_h__
#define __c_cloudview_data_frame_h__

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <set>

enum DataViewType {
  DataViewType_Image,
  DataViewType_PointCloud,
};

struct DataDisplayChannel
{
  std::string name;
  std::string tooltip;
  double minval, maxval;
};

class c_data_frame
{
public:
  typedef c_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_data_frame() = default;
  virtual ~c_data_frame() = default;

  const std::set<DataViewType> & get_supported_view_types() const
  {
    return viewTypes_;
  }

  const std::map<int, DataDisplayChannel> & get_display_channels(DataViewType selectedViewType)
  {
    return displayChanenls_;
  }

  virtual bool get_display_data(DataViewType * selectedViewType, int selectedDisplayId,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask)
  {
    return false;
  }

  virtual std::string get_filename()
  {
    return "";
  }

  virtual bool get_text(int id, std::string & text)
  {
    return false;
  }

  virtual bool get_image(int id, cv::OutputArray image,
      cv::OutputArray mask = cv::noArray())
  {
    return false;
  }

  virtual bool get_point_cloud(int id, cv::OutputArray points,
      cv::OutputArray colors)
  {
    return false;
  }

  virtual bool get_structured_point_cloud(int id, cv::OutputArray points,
      cv::OutputArray colors)
  {
    return false;
  }

  virtual void cleanup()
  {
  }

protected:
  void add_display_channel(int id, const std::string & name,
      const std::string & tooltip,
      double minval,
      double maxval);

protected:
  std::set<DataViewType> viewTypes_;
  std::map<int, DataDisplayChannel> displayChanenls_;
};

#endif /* __c_cloudview_data_frame_h__ */
