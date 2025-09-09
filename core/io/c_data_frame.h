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

struct ImageDisplay
{
  std::string tooltip;
  double minval, maxval;
  std::vector<cv::Mat> images, data, masks;
};

struct CloudDisplay
{
  std::string tooltip;
  double minval, maxval;
  std::vector<cv::Mat> points, colors, masks;
};

class c_data_frame
{
public:
  typedef c_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::map<std::string, ImageDisplay, std::less<std::string>> ImageDisplays;
  typedef std::map<std::string, CloudDisplay, std::less<std::string>> CloudDisplays;


  enum SELECTION_MASK_MODE
  {
    SELECTION_MASK_DISABLE,
    SELECTION_MASK_REPLACE,
    SELECTION_MASK_AND,
    SELECTION_MASK_OR,
    SELECTION_MASK_XOR,
  };


  c_data_frame() = default;
  virtual ~c_data_frame() = default;

  const std::set<DisplayType, std::less<DisplayType>> & get_available_display_types() const
  {
    return _display_types;
  }

  const ImageDisplays & get_available_image_displays() const
  {
    return _image_displays;
  }

  const CloudDisplays & get_available_cloud_displays() const
  {
    return _cloud_displays;
  }

  virtual bool get_image(const std::string & display_name,
      cv::OutputArray output_image,
      cv::OutputArray output_mask,
      cv::OutputArray output_data);

  virtual bool get_point_cloud(const std::string & display_name,
      cv::OutputArray output_points,
      cv::OutputArray output_colors,
      cv::OutputArray output_mask,
      std::vector<uint64_t> * output_pids = nullptr);

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

  virtual void clean_artifacts()
  {
    _selection_mask.release();
  }

  const cv::Mat & selection_mask() const
  {
    return _selection_mask;
  }

  void update_selection(cv::InputArray mask,
      SELECTION_MASK_MODE mode);

  static void copy_output_mask(cv::InputArray src,
      cv::OutputArray dst);


  ///
  virtual bool supports_point_annotations()
  {
    return false;
  }

  virtual bool has_point_annotations()
  {
    return false;
  }

  virtual void set_point_annotation(uint64_t pid, int cmap, uint8_t label)
  {
  }

  virtual uint8_t point_annotation(uint64_t pid, int cmap)
  {
    return 0;
  }
  ///

protected:
  ImageDisplays::iterator add_image_display(const std::string & name,
      const std::string & tooltip = "",
      double minval = -1,
      double maxval = -1 );

  CloudDisplays::iterator add_cloud_display(const std::string & name,
      const std::string & tooltip = "",
      double minval = -1,
      double maxval = -1 );

protected:
  std::set<DisplayType, std::less<DisplayType>> _display_types;
  ImageDisplays _image_displays;
  CloudDisplays _cloud_displays;
  cv::Mat _selection_mask;
};

#endif /* __c_cloudview_data_frame_h__ */
