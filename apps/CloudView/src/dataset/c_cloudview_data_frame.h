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
#include <vector>
#include <memory>

namespace cloudview {

class c_cloudview_data_item
{
public:
  enum Type
  {
    image,
    structured_cloud3d,
    unstructured_cloud3d,
  };

  const std::string & name() const
  {
    return name_;
  }

  Type type() const
  {
    return image;
  }

  virtual ~c_cloudview_data_item()
  {
  }

protected:
  std::string name_;
//  Type type_
};

class c_cloudview_data_frame
{
public:
  typedef c_cloudview_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_data_frame() = default;
  virtual ~c_cloudview_data_frame() = default;

  const std::vector<c_cloudview_data_item> & items() const
  {
    return items_;
  }

  const c_cloudview_data_item * item(const std::string & name) const
  {
    return nullptr;
  }

  virtual bool get_image(const std::string & name, cv::OutputArray image, cv::OutputArray mask)
  {
    return false;
  }

  virtual bool get_structured_cloud3d(const std::string & name, cv::OutputArray points, cv::OutputArray colors)
  {
    return false;
  }

  virtual bool get_unstructured_cloud3d(const std::string & name, cv::OutputArray points, cv::OutputArray colors)
  {
    return false;
  }

protected:
  std::vector<c_cloudview_data_item> items_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_data_frame_h__ */
