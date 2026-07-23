/*
 * c_cloudview_data_frame.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_data_frame.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<DisplayType>()
{
  static const c_enum_member members[] = {
      { DisplayType_Image, "Image", "Image" },
      { DisplayType_PointCloud, "PointCloud", "PointCloud" },
      { DisplayType_TextFile, "TextFile", "TextFile" },
      { DisplayType_Image },
  };

  return members;
}

void c_data_frame::copy_output_mask(cv::InputArray src, cv::OutputArray dst)
{
  src.copyTo(dst);
}

c_data_frame::ImageDisplays::iterator c_data_frame::add_image_display(const std::string & display_name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  auto pos = _image_displays.find(display_name);
  if ( pos == _image_displays.end() ) {

    const ImageDisplay c = {
        .tooltip = tooltip,
        .minval = minval,
        .maxval = maxval
    };

    pos = _image_displays.emplace(display_name, c).first;
  }

  pos->second.image.release();
  pos->second.mask.release();
  pos->second.data.release();

  return pos;
}

c_data_frame::CloudDisplays::iterator c_data_frame::add_cloud_display(const std::string & display_name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  auto pos = _cloud_displays.find(display_name);
  if ( pos == _cloud_displays.end() ) {

    const CloudDisplay c = {
        .tooltip = tooltip,
        .minval = minval,
        .maxval = maxval
    };

    pos = _cloud_displays.emplace(display_name, c).first;
  }

  CloudDisplay & display = pos->second;

  display.points.release();
  display.colors.release();
  display.mask.release();

  return pos;

}

void c_data_frame::add_image(const std::string &display_name,
    cv::InputArray image,
    cv::InputArray mask,
    cv::InputArray data)
{
  auto &display = add_image_display(display_name)->second;
  display.image = image.getMat().clone();
  display.data = data.getMat().clone();
  display.mask = mask.getMat().clone();
  _display_types.emplace(DisplayType_Image);
}

void c_data_frame::set_image(const std::string & display_name, cv::Mat && image, cv::Mat && mask)
{
  auto &display = add_image_display(display_name)->second;

  display.image = std::move(image);
  display.mask = std::move(mask);
  display.data = std::move(cv::Mat());

  _display_types.emplace(DisplayType_Image);
}

void c_data_frame::add_point_cloud(const std::string & display_name,
    cv::InputArray points,
    cv::InputArray colors,
    cv::InputArray mask)
{
  auto &display = add_cloud_display(display_name)->second;
  display.points = points.getMat().clone();
  display.colors = colors.getMat().clone();
  display.mask = mask.getMat().clone();
  _display_types.emplace(DisplayType_PointCloud);
}

bool c_data_frame::get_image(const std::string &display_name,
    cv::OutputArray output_image,
    cv::OutputArray output_mask,
    cv::OutputArray output_data)
{
  bool fOk = false;

  const auto pos = _image_displays.find(display_name);

  if (pos != _image_displays.end()) {

    const auto &display = pos->second;

    if (output_image.needed() && !display.image.empty()) {

      if (_selection_mask.size() == output_image.size() && _selection_mask.channels() == 3) {
        CF_DEBUG("WARNING: Special case for selection_mask_ not handled");
      }

      display.image.copyTo(output_image);
      fOk = true;
    }
    if (output_mask.needed() && !display.mask.empty()) {
      copy_output_mask(display.mask, output_mask);
      fOk = true;
    }
    if (output_data.needed() && !display.data.empty()) {
      display.data.copyTo(output_data);
      fOk = true;
    }
  }

  return fOk;
}


bool c_data_frame::get_point_cloud(const std::string & display_name,
    cv::OutputArray points,
    cv::OutputArray colors,
    cv::OutputArray mask,
    std::vector<uint64_t> * output_pids)
{
  bool fOk = false;

  //CF_DEBUG("request for cloud '%s'", display_name.c_str());
  const auto pos = _cloud_displays.find(display_name);

  if (pos == _cloud_displays.end()) {
    // CF_DEBUG("cloud '%s' not found", display_name.c_str());
  }
  else {

    const auto &display = pos->second;
    //CF_DEBUG("cloud '%s' is found", display_name.c_str());

    if ( points.needed() && !display.points.empty()) {
      //CF_DEBUG("points '%s' copied", display_name.c_str());
      display.points.copyTo(points);
    }

    if ( colors.needed() && !display.colors.empty()) {
      //CF_DEBUG("colors '%s' copied", display_name.c_str());
      display.colors.copyTo(colors);
    }

    if ( mask.needed() && !display.mask.empty()) {
      //CF_DEBUG("masks '%s' copied", display_name.c_str());
      display.mask.copyTo(mask);
    }

    //CF_DEBUG("cloud '%s' leave", display_name.c_str());
    return true;
  }
  return false;
}

std::string c_data_frame::get_filename()
{
  return "";
}


void c_data_frame::update_selection(cv::InputArray mask, COMBINE_MASK_MODE mode)
{
  if ( mode != COMBINE_MASK_MODE_KEEP ) {

    if ( !mask.empty()  ) {
      combine_masks(mode, mask, _selection_mask, _selection_mask);
    }
    else if ( mode == COMBINE_MASK_MODE_REPLACE ) {
      mask.copyTo(_selection_mask);
    }
  }
}

