/*
 * c_raw_file.cc
 *
 *  Created on: Dec 5, 2020
 *      Author: amyznikov
 */

#include "c_raw_file.h"
#include <tbb/tbb.h>
#include <core/debug.h>

static enum COLORID raw2colorid(LibRaw & raw)
{
  enum COLORID colorid = COLORID_UNKNOWN;

  if ( raw.imgdata.idata.filters ) {

    char filters_pattern[17] = "";
    int n = 0;

    if ( !raw.imgdata.idata.cdesc[3] ) {
      raw.imgdata.idata.cdesc[3] = 'G';
    }

    for ( int i = 0; i < 16; ++i, ++n ) {
      filters_pattern[n] = raw.imgdata.idata.cdesc[raw.fcol(i >> 1, i & 1)];
    }

    // CF_DEBUG("filters_pattern: %s", filters_pattern);

    constexpr static struct c_known_pattern {
      const char * pattern;
      enum COLORID colorid;
    } known_patterns[] = {
        { "RGGBRGGBRGGBRGGB", COLORID_BAYER_RGGB },
        { "GRBGGRBGGRBGGRBG", COLORID_BAYER_GRBG },
        { "GBRGGBRGGBRGGBRG", COLORID_BAYER_GBRG },
        { "BGGRBGGRBGGRBGGR", COLORID_BAYER_BGGR },
        { "CYYMCYYMCYYMCYYM", COLORID_BAYER_CYYM },
        { "YCMYYCMYYCMYYCMY", COLORID_BAYER_YCMY },
        { "YMCYYMCYYMCYYMCY", COLORID_BAYER_YMCY },
        { "MYYCMYYCMYYCMYYC", COLORID_BAYER_MYYC },
    };

    for ( uint i = 0; i < sizeof(known_patterns) / sizeof(known_patterns[0]); ++i ) {
      if ( strcmp(filters_pattern, known_patterns[i].pattern) == 0 ) {
        colorid = known_patterns[i].colorid;
        break;
      }
    }

  }
  else if ( raw.imgdata.idata.colors == 1 ) {
    colorid = COLORID_MONO;
  }
  else if ( raw.imgdata.idata.colors == 3 ) {
    colorid = COLORID_RGB;
  }

  return colorid;
}


int c_raw_file_reader::raw2mat(cv::Mat & output_image)
{
  const libraw_image_sizes_t & S = raw.imgdata.rawdata.sizes;

  const int output_width = S.iwidth;
  const int output_height = S.iheight;
  const uint black = raw.imgdata.color.black;
  const uint maximum = raw.imgdata.color.maximum;

  float scale_mul[4];

  int status = LIBRAW_SUCCESS;



  colorid_ = COLORID_UNKNOWN;
  bpc_ = 16; // fixme: raw.imgdata.color.raw_bps;
  black_level_ = raw.imgdata.color.black;

  if ( !output_image.empty() && !output_image.isContinuous() ) {
    output_image.release();
  }

  const float dmax = *std::min_element(raw.imgdata.color.cam_mul, raw.imgdata.color.cam_mul + 4);
  if ( dmax > 0 ) {
    const double scale = 65535. / (dmax * (maximum - black));
    for ( int c = 0; c < 4; ++c ) {
      scale_mul[c] = raw.imgdata.color.cam_mul[c] * scale;
    }
    bpc_ = 16;
  }

  if ( raw.imgdata.rawdata.raw_image && (raw.imgdata.idata.filters || raw.imgdata.idata.colors == 1) ) {
    // BAYER OR MONO

    const bool is_bayer = raw.imgdata.idata.filters;

    colorid_ = raw2colorid(raw);

    output_image.create(output_height, output_width, CV_16UC1);
    uint16_t * dstp = (uint16_t *) output_image.data;

    const int maxHeight = std::min(S.height, (ushort) (S.raw_height - S.top_margin));
    const int src_stride = S.raw_pitch / 2;
    const int src_offset = S.top_margin * S.raw_pitch / 2 + S.left_margin;
    const uint16_t * srcp = raw.imgdata.rawdata.raw_image;

    if ( !is_bayer || dmax <= 0 ) {

      for ( uint row = 0; row < maxHeight; ++row ) {
        for ( uint col = 0; col < S.width && col + S.left_margin < S.raw_width; ++col ) {
          const uint src_pos = row * src_stride + col + src_offset;
          const uint dst_pos = row * output_width + col;
          const uint16_t v = srcp[src_pos];
          if ( v <= black ) {
            dstp[dst_pos] = 0;
          }
          else {
            dstp[dst_pos] = cv::saturate_cast<uint16_t>(v - black);
          }
        }
      }
    }
    else {

      using range = tbb::blocked_range<int>;

      tbb::parallel_for(range(0, maxHeight, 256),
          [&scale_mul, &S, srcp, dstp, black, src_stride, src_offset, output_width](const range & rrange) {
            for ( int row = rrange.begin(), nrow = rrange.end(); row < nrow; ++row ) {
              for ( uint col = 0; col < S.width && col + S.left_margin < S.raw_width; ++col ) {

                const uint src_pos = row * src_stride + col + src_offset;
                const uint dst_pos = row * output_width + col;

#if 0
                dstp[dst_pos] = srcp[src_pos];
#else


                const uint16_t v = srcp[src_pos];

                if ( v <= black ) {
                  dstp[dst_pos] = 0;
                }
                else {
                  static constexpr uint cpos[2][2] = {
                    0, 1,
                    3, 2
                  };
                  dstp[dst_pos] = cv::saturate_cast<uint16_t>(
                      (v - black) * scale_mul[cpos[row & 0x1][col & 0x1]]);
                }
#endif
              }
            }
          });
    }
  }

  else if ( raw.imgdata.rawdata.color3_image ) {
    // RGB

    colorid_ = COLORID_BGR;

    output_image.create(output_height, output_width, CV_16UC3);
    cv::Vec3w * image = (cv::Vec3w * ) output_image.data;

    const uint8_t * c3image = (uint8_t *) raw.imgdata.rawdata.color3_image;

    for ( int row = 0; row < S.height && row + S.top_margin < S.raw_height; row++ ) {

      const ushort (*srcrow)[3] = (ushort (*)[3]) &c3image[(row + S.top_margin) * S.raw_pitch];

      for ( int col = 0; col < S.width && col + S.left_margin < S.raw_width; ++col ) {
        cv::Vec3w & dest = image[row * output_width + col];
        dest[2] = srcrow[S.left_margin + col][0];
        dest[1] = srcrow[S.left_margin + col][1];
        dest[0] = srcrow[S.left_margin + col][2];
      }
    }
  }

//  else if ( raw.imgdata.rawdata.color4_image) {
//      if ( S.raw_pitch != S.width * 8 ) {
//        for ( int row = 0; row < S.height && row + S.top_margin < S.raw_height;
//            row++ )
//          memmove(&imgdata.image[row * S.width],
//              &imgdata.rawdata
//                  .color4_image[(row + S.top_margin) * S.raw_pitch / 8 +
//                  S.left_margin],
//              MIN(S.width, S.raw_width - S.left_margin) *
//                  sizeof(*imgdata.image));
//      }
//      else
//      {
//        // legacy is always 4channel and not shrinked!
//        memmove(imgdata.image, imgdata.rawdata.color4_image,
//        MIN(S.raw_width - S.left_margin, S.width) *
//            MIN(S.raw_height - S.top_margin, S.height) *
//            sizeof(*imgdata.image));
//      }
//    }
    else{
      CF_DEBUG("UNKNOWN PATTERN");
      status = LIBRAW_FILE_UNSUPPORTED;
    }

  return status;
}




bool c_raw_file_reader::read(const std::string & filename, cv::Mat & output_image,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  int status;

  has_color_maxtrix_ = false;
  has_channel_multipliers_ = false;

  if ( (status = raw.open_file(filename.c_str())) != LIBRAW_SUCCESS ) {
    CF_FATAL("raw.open_file('%s') fails. status=%d",
        filename.c_str(),
        status);
  }
  else if ( (status = raw.unpack()) != LIBRAW_SUCCESS ) {
    CF_ERROR("'%s' : raw.unpack() fails. status=%d",
        filename.c_str(),
        status);
  }
  else if ( (status = raw2mat(output_image)) != LIBRAW_SUCCESS ) {
    CF_ERROR("'%s': raw2mat() fails. status=%d",
        filename.c_str(),
        status);
  }
  else if ( raw.imgdata.rawdata.color.rgb_cam[0][0] > 0.0001 ) {  // C.rgb_cam[0][0] > 0.0001

    const libraw_colordata_t & C = raw.imgdata.rawdata.color;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) {
        this->color_matrix_(i, j) = C.rgb_cam[i][j];
      }
    }

    has_color_maxtrix_ = true;
  }

  if ( output_colorid ) {
    * output_colorid = colorid_;
  }

  if ( output_bpc ){
    * output_bpc = bpc_;
  }

  return status == LIBRAW_SUCCESS;
}


void c_raw_file_reader::recycle()
{
  raw.recycle();
}

