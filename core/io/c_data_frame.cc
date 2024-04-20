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
      { c_data_frame::SELECTION_MASK_REPLACE, "REPLACE", "REPLACE" },
      { c_data_frame::SELECTION_MASK_AND, "AND", "AND" },
      { c_data_frame::SELECTION_MASK_OR, "OR", "OR" },
      { c_data_frame::SELECTION_MASK_XOR, "XOR", "XOR" },
      { c_data_frame::SELECTION_MASK_REPLACE }
  };

  return members;
}

namespace {

static void reduce_color_channels(cv::InputArray src, cv::OutputArray dst, enum cv::ReduceTypes rtype, int dtype = -1)
{
  cv::Mat s, tmp;

  if (src.isContinuous()) {
    s = src.getMat();
  }
  else {
    src.copyTo(s);
  }

  const int src_rows = s.rows;
  cv::reduce(s.reshape(1, s.total()), tmp, 1, rtype, dtype);
  tmp.reshape(0, src_rows).copyTo(dst);
}

} // namespace

void c_data_frame::copy_output_mask(cv::InputArray src, cv::OutputArray dst)
{
  if ( src.empty() ) {
    dst.release();
  }
  else if ( src.channels() == 1 ) {
    src.copyTo(dst);
  }
  else {
    reduce_color_channels(src, dst, cv::
        REDUCE_MAX);
  }
}

c_data_frame::DisplayMap::iterator c_data_frame::add_display_channel(const std::string & display_name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  auto pos =
      data_displays_.find(display_name);

  if ( pos == data_displays_.end() ) {

    DisplayData c;
    c.tooltip = tooltip;
    c.minval = minval;
    c.maxval = maxval;

    pos =
        data_displays_.emplace(display_name,
            std::move(c)).first;
  }

  pos->second.images.clear();
  pos->second.data.clear();
  pos->second.masks.clear();

  return pos;
}

void c_data_frame::add_image(const std::string &display_name,
    cv::InputArray image,
    cv::InputArray mask,
    cv::InputArray data)
{

  auto &display =
      add_display_channel(display_name)->second;

  if (!image.empty()) {
    display.images.emplace_back(image.getMat().clone());
  }
  if (!data.empty()) {
    display.data.emplace_back(data.getMat().clone());
  }
  if (!mask.empty()) {
    display.masks.emplace_back(mask.getMat().clone());
  }

  display_types_.emplace(DisplayType_Image);
}


void c_data_frame::add_images(const std::string & display_name,
    const std::vector<cv::Mat> & images,
    const std::vector<cv::Mat> & masks,
    const std::vector<cv::Mat> & data)
{
  auto &display =
      add_display_channel(display_name)->second;

  if (!images.empty()) {
    display.images = images;
  }
  if (!data.empty()) {
    display.data = data;
  }
  if (!masks.empty()) {
    display.masks = masks;
  }

  display_types_.emplace(DisplayType_Image);
}

void c_data_frame::add_images(const std::string &display_name,
    size_t count,
    const cv::Mat images[/*count*/],
    const cv::Mat masks[/*count*/],
    const cv::Mat data[/*count*/])
{
  auto &display =
      add_display_channel(display_name)->second;

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

  display_types_.emplace(DisplayType_Image);
}

void c_data_frame::add_point_cloud(const std::string & display_name,
    cv::InputArray points,
    cv::InputArray colors,
    cv::InputArray mask)
{
  CF_ERROR("FIXME: c_data_frame::add_point_cloud() not implemented");
}

bool c_data_frame::get_image(const std::string &display_name,
    cv::OutputArray output_image,
    cv::OutputArray output_mask,
    cv::OutputArray output_data)
{
  const auto pos =
      data_displays_.find(display_name);

  bool fOk = false;

  if (pos != data_displays_.end()) {

    const auto &display =
        pos->second;

    if (output_image.needed() && !display.images.empty()) {
      display.images[0].copyTo(output_image);
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
    cv::OutputArray mask)
{
  return false;
}

std::string c_data_frame::get_filename()
{
  return "";
}


void c_data_frame::update_selection(cv::InputArray mask, SELECTION_MASK_MODE mode)
{
  static const auto dup_channels =
      [](const cv::Mat & src, cv::Mat & dst, int cn) -> bool {

        std::vector<cv::Mat> channels(cn);

        for( int i = 0; i < cn; ++i ) {
          channels[i] = src;
        }

        cv::merge(channels, dst);
        return true;
      };


  if( selection_mask_.empty() || mode == SELECTION_MASK_REPLACE ) {
    mask.getMat().copyTo(selection_mask_);
  }
  else if ( !mask.empty() ) {

    cv::Mat cmask;

    if ( selection_mask_.channels() == mask.channels() ) {
      cmask = mask.getMat();
    }
    else if ( selection_mask_.channels() == 1 ) {
      dup_channels(selection_mask_, selection_mask_, mask.channels());
      cmask = mask.getMat();
    }
    else if ( mask.channels() == 1 ) {
      dup_channels(mask.getMat(), cmask, selection_mask_.channels());
    }
    else {

      CF_ERROR("Unsupported combination of mask channels:\n"
          "selection_mask: %dx%d channels=%d\n"
          "mask: %dx%d channels=%d\n"
          "",
          selection_mask_.cols, selection_mask_.rows, selection_mask_.channels(),
          mask.cols(), mask.rows(), mask.channels());

      return;
    }

    switch (mode) {
      case SELECTION_MASK_AND:
        cv::bitwise_and(cmask, selection_mask_, selection_mask_);
        break;
      case SELECTION_MASK_OR:
        cv::bitwise_or(cmask, selection_mask_, selection_mask_);
        break;
      case SELECTION_MASK_XOR:
        cv::bitwise_xor(cmask, selection_mask_, selection_mask_);
        break;
      default:
        CF_ERROR("Not implemented mask operation detected: mode=%d", mode);
        break;
    }
  }
}

