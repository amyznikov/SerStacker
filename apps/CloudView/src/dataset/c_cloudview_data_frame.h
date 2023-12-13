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

namespace cloudview {


enum ViewType {
  ViewType_Image,
  ViewType_PointCloud,
  //CloudView_Text,
  //CloudView_Table,
};

struct DisplayChannel
{
  std::string name;
  std::string tooltip;
  double minval, maxval;
};
//
//class c_cloudview_data_item
//{
//public:
//  enum Type
//  {
//    image,
//    text,
//    filename,
//    point_cloud_3d,
//    structured_point_cloud_3d,
//  };
//
//  c_cloudview_data_item(const std::string & name, Type type, int dataid, const std::string & tooltip) :
//      name_(name),
//      tooltip_(tooltip),
//      type_(type),
//      dataid_(dataid)
//  {
//  }
//
//  const std::string & name() const
//  {
//    return name_;
//  }
//
//  const char * cname() const
//  {
//    return name_.c_str();
//  }
//
//  const std::string & tooltip() const
//  {
//    return tooltip_;
//  }
//
//  const char * ctooltip() const
//  {
//    return tooltip_.c_str();
//  }
//
//  Type type() const
//  {
//    return type_;
//  }
//
//  int dataid() const
//  {
//    return dataid_;
//  }
//
//
//protected:
//  std::string name_;
//  std::string tooltip_;
//  Type type_;
//  int dataid_;
//};

class c_cloudview_data_frame
{
public:
  typedef c_cloudview_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_data_frame() = default;
  virtual ~c_cloudview_data_frame() = default;

  virtual void getSupportedViewTypes(std::set<ViewType> * viewTypes)
  {
    viewTypes->clear();
  }

  const std::map<int, DisplayChannel> & getDisplayChannels(ViewType selectedViewType)
  {
    return displayChanenls_;
  }

  virtual bool getViewData(ViewType * selectedViewType, int selectedDisplayId,
      cv::OutputArray image,
      cv::OutputArray data,
      cv::OutputArray mask)
  {
    return false;
  }


//  const std::vector<c_cloudview_data_item> & items() const
//  {
//    return items_;
//  }
//
//  const c_cloudview_data_item* item(const std::string & name) const
//  {
//    const auto pos =
//        std::find_if(items_.begin(), items_.end(),
//            [&name](const auto & item) {
//              return name == item.name();
//            });
//    return pos == items_.end() ? nullptr : &*pos;
//  }
//
//  const c_cloudview_data_item* item(int id) const
//  {
//    const auto pos =
//        std::find_if(items_.begin(), items_.end(),
//            [id](const auto & item) {
//              return id == item.dataid();
//            });
//    return pos == items_.end() ? nullptr : &*pos;
//  }

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


protected:
  void add_display_channel(int id, const std::string & name,
      const std::string & tooltip,
      double minval,
      double maxval);

protected:
  std::map<int, DisplayChannel> displayChanenls_;
  // std::vector<c_cloudview_data_item> items_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_data_frame_h__ */
