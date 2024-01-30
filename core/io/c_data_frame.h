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

enum DataViewType
{
  DataViewType_Image,
  DataViewType_PointCloud,
  DataViewType_TextFile,
};

struct DataDisplayChannel
{
  std::string tooltip;
  double minval, maxval;
  cv::Mat image, data, mask;
};


class c_data_frame
{
public:
  typedef c_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::map<std::string, DataDisplayChannel> DisplayMap;

  enum SELECTION_MASK_MODE
  {
    SELECTION_MASK_REPLACE,
    SELECTION_MASK_AND,
    SELECTION_MASK_OR,
    SELECTION_MASK_XOR,
  };


  c_data_frame() = default;
  virtual ~c_data_frame() = default;

  const std::set<DataViewType> & get_supported_view_types() const
  {
    return viewTypes_;
  }

  const DisplayMap & displayChannels() const
  {
    return displayChannels_;
  }

  virtual bool get_data(DataViewType * viewType,
      const std::string & channelName,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask)
  {
    return false;
  }

  virtual bool set_data(DataViewType viewType,
      const std::string & channelName,
      cv::InputArray image,
      cv::InputArray data,
      cv::InputArray mask);

  virtual std::string get_filename()
  {
    return "";
  }

  virtual void cleanup()
  {
  }

protected:
  void add_display_channel(const std::string & name,
      const std::string & tooltip,
      double minval,
      double maxval);

protected:
  std::set<DataViewType> viewTypes_;
  DisplayMap displayChannels_;
};

#endif /* __c_cloudview_data_frame_h__ */
