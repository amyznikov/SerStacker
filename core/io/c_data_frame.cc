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


template<>
const c_enum_member* members_of<c_data_frame::SELECTION_MASK_MODE>()
{
  static const c_enum_member members[] = {
      { c_data_frame::SELECTION_MASK_DISABLE, "DISABLE", "DISABLE" },
      { c_data_frame::SELECTION_MASK_REPLACE, "REPLACE", "REPLACE" },
      { c_data_frame::SELECTION_MASK_AND, "AND", "AND" },
      { c_data_frame::SELECTION_MASK_OR, "OR", "OR" },
      { c_data_frame::SELECTION_MASK_XOR, "XOR", "XOR" },
      { c_data_frame::SELECTION_MASK_REPLACE }
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
  auto pos =
      _image_displays.find(display_name);

  if ( pos == _image_displays.end() ) {

    const ImageDisplay c = {
        .tooltip = tooltip,
        .minval = minval,
        .maxval = maxval
    };

    pos = _image_displays.emplace(display_name, c).first;
  }

  pos->second.images.clear();
  pos->second.data.clear();
  pos->second.masks.clear();

  return pos;
}

c_data_frame::CloudDisplays::iterator c_data_frame::add_cloud_display(const std::string & display_name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  auto pos =
      _cloud_displays.find(display_name);

  if ( pos == _cloud_displays.end() ) {

    const CloudDisplay c = {
        .tooltip = tooltip,
        .minval = minval,
        .maxval = maxval
    };

    pos =
        _cloud_displays.emplace(display_name, c).first;
  }

  CloudDisplay & display =
      pos->second;

  display.points.clear();
  display.colors.clear();
  display.masks.clear();

  return pos;

}

void c_data_frame::add_image(const std::string &display_name,
    cv::InputArray image,
    cv::InputArray mask,
    cv::InputArray data)
{

  auto &display =
      add_image_display(display_name)->second;

  if (!image.empty()) {
    display.images.emplace_back(image.getMat().clone());
  }
  if (!data.empty()) {
    display.data.emplace_back(data.getMat().clone());
  }
  if (!mask.empty()) {
    display.masks.emplace_back(mask.getMat().clone());
  }

  _display_types.emplace(DisplayType_Image);
}


void c_data_frame::add_images(const std::string & display_name,
    const std::vector<cv::Mat> & images,
    const std::vector<cv::Mat> & masks,
    const std::vector<cv::Mat> & data)
{
  auto &display =
      add_image_display(display_name)->second;

  if (!images.empty()) {
    display.images = images;
  }
  if (!data.empty()) {
    display.data = data;
  }
  if (!masks.empty()) {
    display.masks = masks;
  }

  _display_types.emplace(DisplayType_Image);
}

void c_data_frame::add_images(const std::string &display_name,
    size_t count,
    const cv::Mat images[/*count*/],
    const cv::Mat masks[/*count*/],
    const cv::Mat data[/*count*/])
{
  auto &display =
      add_image_display(display_name)->second;

  for ( size_t i = 0; i < count; ++i ) {

    if ( images ) {
      display.images.emplace_back(images[i]);
    }

    if ( masks ) {
      display.masks.emplace_back(masks[i]);
    }

    if ( data ) {
      display.data.emplace_back(data[i]);
    }
  }

  _display_types.emplace(DisplayType_Image);
}

void c_data_frame::add_point_cloud(const std::string & display_name,
    cv::InputArray points,
    cv::InputArray colors,
    cv::InputArray mask)
{
  auto &display =
      add_cloud_display(display_name)->second;

  if ( !points.empty() ) {
    display.points.emplace_back(points.getMat().clone());
  }

  if ( !colors.empty() ) {
    display.colors.emplace_back(colors.getMat().clone());
  }
  if (!mask.empty()) {
    display.masks.emplace_back(mask.getMat().clone());
  }

  _display_types.emplace(DisplayType_PointCloud);
}

bool c_data_frame::get_image(const std::string &display_name,
    cv::OutputArray output_image,
    cv::OutputArray output_mask,
    cv::OutputArray output_data)
{
  const auto pos =
      _image_displays.find(display_name);

  bool fOk = false;

  if (pos != _image_displays.end()) {

    const auto &display =
        pos->second;

    if (output_image.needed() && !display.images.empty()) {

      if (_selection_mask.size() == output_image.size() && _selection_mask.channels() == 3) {
        CF_DEBUG("WARNING: Special case for selection_mask_ not handled");
        display.images[0].copyTo(output_image);
      }
      else {
        display.images[0].copyTo(output_image);
      }

      fOk = true;
    }
    if (output_mask.needed() && !display.masks.empty()) {
      copy_output_mask(display.masks[0], output_mask);
      fOk = true;
    }
    if (output_data.needed() && !display.data.empty()) {
      display.data[0].copyTo(output_data);
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

  const auto pos =
      _cloud_displays.find(display_name);

  if (pos != _cloud_displays.end()) {

    const auto &display =
        pos->second;

    if ( points.needed() && !display.points.empty()) {
      display.points[0].copyTo(points);
    }

    if ( colors.needed() && !display.colors.empty()) {
      display.colors[0].copyTo(colors);
    }

    if ( mask.needed() && !display.masks.empty()) {
      display.masks[0].copyTo(mask);
    }

    return true;
  }
  return false;
}

std::string c_data_frame::get_filename()
{
  return "";
}


void c_data_frame::update_selection(cv::InputArray mask, SELECTION_MASK_MODE mode)
{
  if ( mode != SELECTION_MASK_DISABLE ) {

    static const auto dup_channels =
      [](const cv::Mat & src, cv::Mat & dst, int cn) -> bool {

        std::vector<cv::Mat> channels(cn);

        for( int i = 0; i < cn; ++i ) {
          channels[i] = src;
        }

        cv::merge(channels, dst);
        return true;
      };


    if( _selection_mask.empty() || mode == SELECTION_MASK_REPLACE ) {
      mask.getMat().copyTo(_selection_mask);
    }
    else if ( !mask.empty() ) {

      cv::Mat cmask;

      if ( _selection_mask.channels() == mask.channels() ) {
        cmask = mask.getMat();
      }
      else if ( _selection_mask.channels() == 1 ) {
        dup_channels(_selection_mask, _selection_mask, mask.channels());
        cmask = mask.getMat();
      }
      else if ( mask.channels() == 1 ) {
        dup_channels(mask.getMat(), cmask, _selection_mask.channels());
      }
      else {

        CF_ERROR("Unsupported combination of mask channels:\n"
            "selection_mask: %dx%d channels=%d\n"
            "mask: %dx%d channels=%d\n"
            "",
            _selection_mask.cols, _selection_mask.rows, _selection_mask.channels(),
            mask.cols(), mask.rows(), mask.channels());

        return;
      }

      switch (mode) {
        case SELECTION_MASK_AND:
          cv::bitwise_and(cmask, _selection_mask, _selection_mask);
          break;
        case SELECTION_MASK_OR:
          cv::bitwise_or(cmask, _selection_mask, _selection_mask);
          break;
        case SELECTION_MASK_XOR:
          cv::bitwise_xor(cmask, _selection_mask, _selection_mask);
          break;
        default:
          CF_ERROR("Not implemented mask operation detected: mode=%d", mode);
          break;
      }
    }
  }
}

