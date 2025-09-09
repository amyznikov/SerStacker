/*
 * c_data_annotation_labels.cc
 *
 *  Created on: Oct 24, 2024
 *      Author: gandriim
 */

#include "c_data_annotation_labels.h"
#include <core/settings/opencv_settings.h>
#include <core/ssprintf.h>
#include <core/readdir.h>

namespace {

struct c_data_annotation_enum_member
{
  int value = 0;
  std::string name;
  cv::Vec4b color = cv::Vec4b::all(255U);
  std::string comment;
};

static const c_data_annotation_enum_member * default_data_annotation_object_types()
{
  static const c_data_annotation_enum_member members[] = {
      { DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_CLEAR, "CLEAR", cv::Vec4b(0,0,0, 255), "" },
      { DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_GROUND, "GROUND", cv::Vec4b(10,170,255, 128), "" },
      { DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_OBSTACLE, "OBSTACLE", cv::Vec4b(0,0,0, 255), "" },
      { DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_CLEAR },
  };

  return members;
}

static const c_data_annotation_enum_member * default_data_annotation_point_types()
{
  static const c_data_annotation_enum_member members[] = {
      { DEFAULT_DATA_ANNOTATION_POINT_TYPE_CLEAR, "CLEAR", cv::Vec4b(0,0,0, 255), "" },
      { DEFAULT_DATA_ANNOTATION_POINT_TYPE_NOISE, "NOISE", cv::Vec4b(255, 85, 255, 255), "" },
      { DEFAULT_DATA_ANNOTATION_POINT_TYPE_CLEAR }
  };

  return members;
}

}


const std::string & default_data_annotation_config_filename()
{
  static const std::string config_filename = "~/.config/SerStacker/data_annotation_labels.conf";
  return config_filename;
}


void create_default_data_annotation_labels(c_data_annotation_labels * annotations)
{
  using Label = c_data_annotation_labels::Label;
  using ColorMap = c_data_annotation_labels::ColorMap;

  annotations->colormaps().clear();

  annotations->colormaps().emplace_back(ColorMap::uptr(
      new ColorMap("Object Types")));

  const ColorMap::uptr &object_types =
      annotations->colormaps().back();

  const c_data_annotation_enum_member * object_types_enum =
      default_data_annotation_object_types();

  for (; !object_types_enum->name.empty(); ++object_types_enum) {
    object_types->emplace(object_types_enum->value,
        (Label ) { object_types_enum->name,
                object_types_enum->color });
  }


  annotations->colormaps().emplace_back(ColorMap::uptr(
      new ColorMap("Point Types")));

  const ColorMap::uptr &point_types =
      annotations->colormaps().back();

  const c_data_annotation_enum_member * point_types_enum =
      default_data_annotation_point_types();

  for (; !point_types_enum->name.empty(); ++point_types_enum) {
    point_types->emplace(point_types_enum->value,
        (Label ) { point_types_enum->name,
                point_types_enum->color});
  }

  annotations->emit_change_handlers();
}

bool save_data_annotation_labels(const std::string &filename, const c_data_annotation_labels &annotations)
{
  using Label = c_data_annotation_labels::Label;
  using ColorMap = c_data_annotation_labels::ColorMap;

  c_config config;
  c_config_setting settings;

  if (!save_settings(config.root(), "object_class", std::string("c_data_annotation_labels"))) {
    CF_FATAL("save_settings('object_class') fails");
    return false;
  }

  static const auto save_colormap =
      [](c_config_setting settings, const ColorMap::uptr & colormap) {

        save_settings(settings, "name",
            colormap->name());

        settings =
            settings.add_list("Items");

        for ( auto ii= colormap->begin(); ii != colormap->end(); ++ii ) {

          const auto & value = ii->first;
          const auto & label = ii->second;

          c_config_setting item = settings.add_group();

          save_settings(item, "name", label.name);
          save_settings(item, "label", (int)value);
          save_settings(item, "color", label.color);
        }
      };



  settings =
      config.root().add_list("colormaps");

  for (int i = 0, n = annotations.num_colormaps(); i < n; ++i) {
    save_colormap(settings.add_group(),
        annotations.colormap(i));
  }

  if (!config.write(filename)) {
    CF_FATAL("config.write(filename='%s') fails",
        filename.c_str());
    return false;
  }

  return true;
}

bool load_data_annotation_labels(const std::string & filename, c_data_annotation_labels * annotations)
{
  using Label = c_data_annotation_labels::Label;
  using ColorMap = c_data_annotation_labels::ColorMap;

  c_config config;
  c_config_setting settings;

  std::string object_class;

  if (!config.read(filename)) {
    CF_ERROR("config.read(filename='%s') fails",
        filename.c_str());
    return false;
  }


  if (!load_settings(config.root(), "object_class", &object_class)) {
    CF_ERROR("load_settings('object_class') fails");
    return false;
  }

  if ( object_class != "c_data_annotation_labels" ) {
    CF_ERROR("Not 'c_data_annotation' condig file");
    return false;
  }

  static const auto read_colormap =
      [](c_config_setting settings, c_data_annotation_labels *annotations) -> bool {

        c_config_setting items;
        std::string name;

        if ( !load_settings(settings, "name", &name) || name.empty() ) {
          CF_ERROR("No colormap name specified, ignored");
          return false;
        }

        if ( !(items = settings["Items"]).isList() ) {
          CF_ERROR("No colormap 'Items' list found, colormap '%s' ignored", name.c_str());
          return false;
        }

        annotations->colormaps().emplace_back(ColorMap::uptr(
                new ColorMap(name)));

        const ColorMap::uptr & colormap =
            annotations->colormaps().back();

        std::string label_name;
        //cv::Vec4b label_color = cv::Vec4b::all(255U);
        cv::Vec3b label_color_bgr;
        cv::Vec4b label_color_bgra;
        int label_value;

        for ( int i = 0, n = items.length(); i < n; ++i ) {

          c_config_setting item =
              items[i];

          if ( !load_settings(item, "name", &label_name)) {
            continue;
          }

          if ( !load_settings(item, "label", &label_value)) {
            continue;
          }

          if ( !load_settings(item, "color", &label_color_bgra)) {
            if ( !load_settings(item, "color", &label_color_bgr)) {
              continue;
            }
            label_color_bgra[0] = label_color_bgr(0);
            label_color_bgra[1] = label_color_bgr(1);
            label_color_bgra[2] = label_color_bgr(2);
            label_color_bgra[3] = 255;
          }

          colormap->emplace((uint8_t)label_value,
              (c_data_annotation_labels::Label) {
                label_name,
                label_color_bgra});

        }

        if (colormap->find(0) == colormap->end() ) {
          colormap->emplace(0, (Label){"CLEAR", cv::Vec4b(0, 0, 0, 255)});
        }

        return true;
      };


  annotations->colormaps().clear();

  if (!(settings = config.root()["colormaps"]).isList()) {
    CF_ERROR("No 'colormaps' list found in config file '%s'", filename.c_str());
    return false;
  }

  for (int i = 0, n = settings.length(); i < n; ++i) {
    read_colormap(settings[i],
        annotations);
  }

  annotations->emit_change_handlers();

  return true;
}

