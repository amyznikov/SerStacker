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
#include <map>

enum DisplayType
{
  DisplayType_Image,
  DisplayType_PointCloud,
  DisplayType_TextFile,
};

struct DisplayData
{
  std::string tooltip;
  std::vector<cv::Mat> images, data, masks;
  double minval, maxval;
};

class c_data_frame
{
public:
  typedef c_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::map<std::string, DisplayData, std::less<std::string>> DisplayMap;

  enum SELECTION_MASK_MODE
  {
    SELECTION_MASK_REPLACE,
    SELECTION_MASK_AND,
    SELECTION_MASK_OR,
    SELECTION_MASK_XOR,
  };


  c_data_frame() = default;
  virtual ~c_data_frame() = default;

  const std::set<DisplayType, std::less<DisplayType>> & get_available_display_types() const
  {
    return display_types_;
  }

  const DisplayMap & get_available_data_displays() const
  {
    return data_displays_;
  }

  virtual bool get_image(const std::string & display_name,
      cv::OutputArray output_image,
      cv::OutputArray output_mask,
      cv::OutputArray output_data);

  virtual bool get_point_cloud(const std::string & display_name,
      cv::OutputArray output_points,
      cv::OutputArray output_colors,
      cv::OutputArray output_mask);

  virtual std::string get_filename();

  virtual void add_image(const std::string & display_name,
      cv::InputArray image,
      cv::InputArray mask,
      cv::InputArray data);

  virtual void add_images(const std::string & display_name,
      const std::vector<cv::Mat> & images,
      const std::vector<cv::Mat> & masks = std::vector<cv::Mat>(),
      const std::vector<cv::Mat> & data = std::vector<cv::Mat>());

  virtual void add_images(const std::string & display_name,
      size_t count,
      const cv::Mat images[/*count*/],
      const cv::Mat masks[/*count*/],
      const cv::Mat data[/*count*/]);

  virtual void add_point_cloud(const std::string & display_name,
      cv::InputArray points,
      cv::InputArray colors,
      cv::InputArray mask);

  virtual void cleanup()
  {
  }

  void update_selection(cv::InputArray mask,
      SELECTION_MASK_MODE mode);

protected:
  DisplayMap::iterator add_display_channel(const std::string & name,
      const std::string & tooltip = "",
      double minval = -1,
      double maxval = -1 );

protected:
  std::set<DisplayType, std::less<DisplayType>> display_types_;
  DisplayMap data_displays_;
  cv::Mat selection_mask_;
};

#endif /* __c_cloudview_data_frame_h__ */
