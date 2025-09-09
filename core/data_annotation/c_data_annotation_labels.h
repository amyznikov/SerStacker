/*
 * c_data_annotation_labels.h
 *
 *  Created on: Oct 24, 2024
 *      Author: gandriim
 */

#ifndef __c_data_annotation_labels_h__
#define __c_data_annotation_labels_h__

#include <opencv2/opencv.hpp>
#include <limits>
#include <map>


class c_data_annotation_labels
{
public:
  typedef c_data_annotation_labels this_class;

  struct Label
  {
    std::string name;
    cv::Vec4b color;
  };

  class ColorMap :
      public std::map<uint8_t, Label, std::less<uint8_t>>
  {
  public:
    typedef ColorMap this_class;
    typedef std::map<uint8_t, Label, std::less<uint8_t>> base;
    typedef std::unique_ptr<this_class> uptr;

    ColorMap(const std::string & name, bool visible = true) :
      _name(name),
      _visible(visible)
    {
    }

    void set_name(const std::string & name)
    {
      _name = name;
    }

    const std::string & name() const
    {
      return _name;
    }

    void set_visible(bool v)
    {
      _visible = v;
    }

    bool visible() const
    {
      return _visible;
    }

    bool color_for_label(uint8_t lb, cv::Vec4b *color) const
    {
      const auto pos =
          find(lb);

      if (pos != end()) {
        *color = pos->second.color;
        return true;
      }

      return false;
    }

  protected:
    std::string _name;
    bool _visible = true;
  };

  std::vector<ColorMap::uptr>& colormaps()
  {
    return _colormaps;
  }

  const std::vector<ColorMap::uptr>& colormaps() const
  {
    return _colormaps;
  }

  const ColorMap::uptr & colormap(int index)
  {
    return _colormaps[index];
  }

  const ColorMap::uptr & colormap(int index) const
  {
    return _colormaps[index];
  }

  size_t num_colormaps() const
  {
    return _colormaps.size();
  }

  int add_change_handler(const std::function<void()> &f)
  {
    const int id = gen_change_handler_id();
    _change_handlers.emplace(id, f);
    return id;
  }

  int add_change_handler(const std::function<void()> &&f)
  {
    const int id = gen_change_handler_id();
    _change_handlers.emplace(id, f);
    return id;
  }

  void remove_change_handler(int id)
  {
    _change_handlers.erase(id);
  }

  void emit_change_handlers()
  {
    for ( auto ii = _change_handlers.begin(); ii != _change_handlers.end(); ++ii ) {
      if ( ii->second ) {
        ii->second();
      }
    }
  }

  static c_data_annotation_labels & default_instance()
  {
    static c_data_annotation_labels _default_instance;
    return _default_instance;
  }

protected:
  static int gen_change_handler_id()
  {
    static int _change_handler_id = 0;
    return ++_change_handler_id;
  }

protected:
  std::vector<ColorMap::uptr> _colormaps;
  std::map<int, std::function<void()>, std::less<int>> _change_handlers;
};


const std::string & default_data_annotation_config_filename();

void create_default_data_annotation_labels(c_data_annotation_labels * annotations);

bool save_data_annotation_labels(const std::string & filename,
    const c_data_annotation_labels & annotations);

bool load_data_annotation_labels(const std::string & filename,
    c_data_annotation_labels * annotations);

enum DEFAULT_DATA_ANNOTATION_OBJECT_TYPES {
  DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_CLEAR = 0,
  DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_GROUND = 1,
  DEFAULT_DATA_ANNOTATION_OBJECT_TYPE_OBSTACLE,
};

enum DEFAULT_DATA_ANNOTATION_POINT_TYPES {
  DEFAULT_DATA_ANNOTATION_POINT_TYPE_CLEAR = 0,
  DEFAULT_DATA_ANNOTATION_POINT_TYPE_NOISE,
};

#endif /* __c_data_annotation_labels_h__ */
