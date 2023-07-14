/*
 * c_stereo_input.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */
#include "c_stereo_input.h"
#include <core/io/load_image.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<stereo_input_frame_layout_type>()
{
  static constexpr c_enum_member members[] = {
      { stereo_frame_layout_horizontal, "horizontal", "" },
      { stereo_frame_layout_vertical, "vertical", "" },
      { stereo_frame_layout_separate_sources, "separate_sources", "" },
      { stereo_frame_layout_horizontal }
  };

  return members;
}

static bool read_input_frame(const c_input_source::sptr & source,
    bool enable_color_maxtrix,
    cv::Mat & output_image,
    cv::Mat & output_mask)
{
  INSTRUMENT_REGION("");

  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;


  if ( !source->read(output_image, &colorid, &bpp) ) {
    CF_FATAL("source->read() fails\n");
    return false;
  }

  if ( is_bayer_pattern(colorid) ) {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, colorid, algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, colorid) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << bpp)));

        if ( !nninterpolation(output_image, output_image, colorid) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        if( output_image.depth() != CV_8U ) {
          output_image.convertTo(output_image, CV_8U,
              255. / ((1 << bpp)));
        }
        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toString(algo));
        return false;
    }
  }
  else if ( colorid == COLORID_OPTFLOW || (output_image.channels() != 4 && output_image.channels() != 2) ) {
    output_mask.release();
  }
  else if( !splitbgra(output_image, output_image, &output_mask) ) {
    output_mask.release();
    return false;
  }

  if( enable_color_maxtrix && source->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        source->color_matrix());
  }

  return true;
}

bool open_stereo_source(c_stereo_input_source & source, stereo_input_frame_layout_type layout_type)
{
  if ( !source.inputs[0]->open() ) {
    CF_ERROR("ERROR: can not open input source '%s'", source.inputs[0]->cfilename());
    return false;
  }

  if ( layout_type == stereo_frame_layout_separate_sources ) {
    if ( !source.inputs[1]->open() ) {
      CF_ERROR("ERROR: can not open input source '%s'", source.inputs[1]->cfilename());
      return false;
    }

    if( source.inputs[0]->size() != source.inputs[1]->size() ) {
      CF_ERROR("ERROR: input sources sizes not match: left size=%d right size=%d ",
          source.inputs[0]->size(), source.inputs[1]->size());
      return false;
    }
  }

  return true;
}

void close_stereo_source(c_stereo_input_source & source)
{
  for( int i = 0; i < 2; ++i ) {
    if( source.inputs[i] ) {
      source.inputs[i]->close();
    }
  }
}

bool seek_stereo_source(c_stereo_input_source & source, int pos)
{
  if( !source.inputs[0]->seek(pos) ) {
    CF_ERROR("ERROR: source.inputs[0]->seek(pos=%d) fails", pos);
    return false;
  }

  if( source.inputs[1] && source.inputs[1]->is_open() ) {
    if( !source.inputs[1]->seek(pos) ) {
      CF_ERROR("ERROR: source.inputs[1]->seek(pos=%d) fails", pos);
      return false;
    }
  }

  return true;
}


bool read_stereo_source(c_stereo_input_source & source,
    stereo_input_frame_layout_type layout_type,
    bool swap_cameras,
    bool enable_color_maxtrix,
    cv::Mat output_frames[2],
    cv::Mat output_masks[2])
{
  switch (layout_type) {
    case stereo_frame_layout_separate_sources:

      for( int i = 0; i < 2; ++i ) {

        cv::Mat & frame =
            output_frames[i];

        cv::Mat & mask =
            output_masks[i];

        if( !read_input_frame(source.inputs[i], enable_color_maxtrix, frame, mask) ) {
          CF_ERROR("ERROR: read_input_frame() fails for source %d", i);
          return false;
        }
      }
      break;

    case stereo_frame_layout_horizontal: {

      cv::Mat image, mask;

      if( !read_input_frame(source.inputs[0], enable_color_maxtrix, image, mask) ) {
        CF_ERROR("ERROR: read_input_frame() fails");
        return false;
      }

      const cv::Rect roi[2] = {
          cv::Rect(0, 0, image.cols / 2, image.rows),
          cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows)
      };

      if( !swap_cameras ) {

        output_frames[0] = image(roi[0]);
        output_frames[1] = image(roi[1]);

        if( !mask.empty() ) {
          output_masks[0] = mask(roi[0]);
          output_masks[0] = mask(roi[1]);
        }
      }
      else {
        output_frames[0] = image(roi[1]);
        output_frames[1] = image(roi[0]);

        if( !mask.empty() ) {
          output_masks[0] = mask(roi[1]);
          output_masks[1] = mask(roi[0]);
        }
      }

      break;
    }

    case stereo_frame_layout_vertical: {

      cv::Mat image, mask;

      if( !read_input_frame(source.inputs[0], enable_color_maxtrix, image, mask) ) {
        CF_ERROR("ERROR: read_input_frame() fails");
        return false;
      }

      const cv::Rect roi[2] = {
          cv::Rect(0, 0, image.cols, image.rows / 2),
          cv::Rect(0, image.rows / 2, image.cols, image.rows / 2)
      };

      if( !swap_cameras ) {

        output_frames[0] = image(roi[0]);
        output_frames[1] = image(roi[1]);

        if( !mask.empty() ) {
          output_masks[0] = mask(roi[0]);
          output_masks[1] = mask(roi[1]);
        }
      }
      else {
        output_frames[0] = image(roi[1]);
        output_frames[1] = image(roi[0]);

        if( !mask.empty() ) {
          output_masks[0] = mask(roi[1]);
          output_masks[1] = mask(roi[0]);
        }
      }

      break;
    }
  }

  const cv::Mat & left_frame =
      output_frames[0];

  const cv::Mat & right_frame =
      output_frames[1];

  if( left_frame.size() != right_frame.size() ) {
    CF_ERROR("INPUT ERROR: Left (%dx%d) and right (%dx%d) image sizes not equal.\n"
        "Different image sizes are not yet supported",
        left_frame.cols, left_frame.rows,
        right_frame.cols, right_frame.rows);
    return false;
  }

  if( left_frame.channels() != right_frame.channels() ) {
    CF_ERROR("INPUT ERROR: Left (%d) and right (%d) number of image channels not equal.\n"
        "Different image types are not yet supported",
        left_frame.channels(),
        right_frame.channels());
    return false;
  }

  return true;
}
