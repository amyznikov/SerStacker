/*
 * c_output_frame_writer.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "c_output_frame_writer.h"
#include "c_input_source.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

bool get_data_range_for_pixel_depth(int ddepth, double * minval, double * maxval)
{
  switch (CV_MAT_DEPTH(ddepth)) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}

/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool get_scale_offset(int src_depth, int dst_depth, double * scale, double * offset)
{
  double srcmin, srcmax;
  double dstmin, dstmax;

  get_data_range_for_pixel_depth(src_depth, &srcmin, &srcmax);
  get_data_range_for_pixel_depth(dst_depth, &dstmin, &dstmax);

  *scale = (dstmax - dstmin) / (srcmax - srcmin);
  *offset = dstmin - *scale * srcmin;

  return true;
}

} // namespace
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


c_output_frame_writer::c_output_frame_writer()
{
}


c_output_frame_writer::~c_output_frame_writer()
{
  close();
}

bool c_output_frame_writer::is_open() const
{
  switch (output_type) {
    case output_type_images:
      return !output_file_name.empty();
    case output_type_ser:
      return ser.is_open();
    case output_type_video:
      return ffmpeg.is_open();
  }
  return false;
}

bool c_output_frame_writer::open(const std::string & filename, const cv::Size & frameSize, bool color,
    bool write_frame_mapping)
{
  output_file_name = filename;
  output_type = output_type_unknown;
  current_frame_index = 0;
  pts = 0;

  if ( filename.empty() ) {
    CF_ERROR("c_output_frame_writer: No output file name specified");
    return false;
  }

  if( !create_path(get_parent_directory(filename)) ) {
    CF_ERROR("c_output_frame_writer: create_path('%s') fails: %s",
        filename.c_str(),
        strerror(errno));
    return false;
  }

  switch (c_input_source::suggest_source_type(filename)) {
    case c_input_source::SER: {

      ser.create(filename, frameSize.width, frameSize.height,
          color ? COLORID_BGR : COLORID_MONO,
          16);

      if( !ser.is_open() ) {
        CF_ERROR("Can not write SER file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_ser;
      break;
    }

    case c_input_source::MOVIE: {

      ffmpeg.open(filename, frameSize,
          color, "-c huffyuv -r 10"); // ffv1

      if( !ffmpeg.is_open() ) {
        CF_ERROR("Can not write aligned video file '%s'", filename.c_str());
        return false;
      }

      output_type = output_type_video;
      break;
    }

    case c_input_source::REGULAR_IMAGE: {

      output_type = output_type_images;
      break;
    }

    default: {
      CF_ERROR("NOOT SUPPORTED output format requested for file '%s'", filename.c_str());
      return false;
    }
  }

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  if( write_frame_mapping ) {

    std::string mapfilename =
        ssprintf("%s.map.txt", filename.c_str());

    if( !(frame_mapping_fp = fopen(mapfilename.c_str(), "w")) ) {
      CF_ERROR("fopen('%s') fails : %s", mapfilename.c_str(), strerror(errno));
    }
    else {
      fprintf(frame_mapping_fp, "seqidx\tfrmidx\n");
    }
  }

  return true;
}

bool c_output_frame_writer::write(cv::InputArray currenFrame, cv::InputArray currentMask, bool with_alpha_mask, int seqindex)
{
  static const auto convert =
      [](cv::InputArray src, cv::OutputArray dst, int dst_depth) {
        double scale = 1;
        double offset = 0;
        get_scale_offset(src.depth(), dst_depth, &scale, &offset);
        src.getMat().convertTo(dst, dst_depth, scale, offset);
      };


  switch (output_type) {
    case output_type_video:
      if( ffmpeg.is_open() ) {
        if( currenFrame.depth() == CV_8U ) {
          if ( !ffmpeg.write(currenFrame.getMat(), pts++) ) {
            CF_ERROR("ffmpeg.write() fails");
            return false;
          }
        }
        else {
          convert(currenFrame, tmp, CV_8U);

          if( currentMask.type() == CV_8UC1 && currentMask.size() == tmp.size() ) {
            tmp.setTo(0, ~currentMask.getMat());
          }

          if ( !ffmpeg.write(tmp, pts++) ) {
            CF_ERROR("ffmpeg.write() fails");
            return false;
          }
        }
      }
      break;

    case output_type_ser:
      if( ser.is_open() ) {
        if ( currenFrame.depth() == CV_16U ) {
          if ( !ser.write(currenFrame) ) {
            CF_ERROR("ser.write() fails");
            return false;
          }
        }
        else {
          convert(currenFrame, tmp, CV_16U);
          if ( !ser.write(tmp) ) {
            CF_ERROR("ser.write() fails");
            return false;
          }
        }
      }
      break;

    case output_type_images: {

      std::string fname =
          output_file_name;

      const std::string suffix =
          get_file_suffix(fname);

      set_file_suffix(fname, ssprintf("-%06d%s",
          current_frame_index,
          suffix.c_str()));

      if( with_alpha_mask ) {
        if( !save_image(currenFrame, currentMask, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }
      else {
        if( !save_image(currenFrame, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }

      break;
    }

    default:
      CF_ERROR("ERROR: Output video file is not open");
      return false;
  }

  if( frame_mapping_fp ) {
    fprintf(frame_mapping_fp, "%5d\t%d\n",
        seqindex, current_frame_index);
  }

  current_input_sequence_index = seqindex;
  ++current_frame_index;
  return true;
}

void c_output_frame_writer::close()
{
  ffmpeg.close();
  ser.close();
  tmp.release();

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  output_type = output_type_unknown;
}
