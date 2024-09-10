/*
 * c_sply_video_file.cc
 *
 *  Created on: Sep 10, 2024
 *      Author: amyznikov
 */

#include "c_sply_video_file.h"
#include <core/debug.h>

c_sply_video_writer::c_sply_video_writer(const std::string & filename) :
  _dtcs(filename)
{
  set_file_tag(default_file_tag());
}

const std::string& c_sply_video_writer::filename() const
{
  return _dtcs.filename();
}

const char* c_sply_video_writer::cfilename() const
{
  return _dtcs.cfilename();
}

void c_sply_video_writer::set_file_tag(uint64_t v)
{
  _dtcs.set_file_tag(v);
}

uint64_t c_sply_video_writer::file_tag() const
{
  return _dtcs.file_tag();
}

bool c_sply_video_writer::create(const std::string & filename)
{
  if( _dtcs.create(filename) ) {
    _dtcs.add_stream("video");
    return true;
  }
  return false;
}

void c_sply_video_writer::close()
{
  return _dtcs.close();
}

bool c_sply_video_writer::is_open() const
{
  return _dtcs.is_open();
}

bool c_sply_video_writer::write(const c_sply_video_frame & frame)
{
  bool fOk =
      _dtcs.writex(0,
          [&](c_file_handle & fd) -> bool {

            if ( !write_gps(fd, frame) ) {
              CF_ERROR("write_gps() fails: %s", strerror(errno));
              return false;
            }

            if ( !write_image(fd, frame.image) ) {
              CF_ERROR("write_image(frame.image) fails: %s", strerror(errno));
              return false;
            }

            if ( !write_mask(fd, frame.mask) ) {
              CF_ERROR("write_mask(frame.mask) fails: %s", strerror(errno));
              return false;
            }

            if ( !write_mat(fd, frame.cloud3d) ) {
              CF_ERROR("write_mat(frame.cloud3d) fails: %s", strerror(errno));
              return false;
            }

            if ( !write_mat(fd, frame.colors3d) ) {
              CF_ERROR("write_mat(frame.colors3d) fails: %s", strerror(errno));
              return false;
            }

            return true;

          });

  return fOk;
}

bool c_sply_video_writer::write_gps(c_file_handle & fd, const c_sply_video_frame & frame)
{
  const gps_storage gps = {
    .ts = frame.ts,
    .lat = frame.lat,
    .lon = frame.lon,
    .elev = frame.elev
  };

  return fd.write(gps);
}

bool c_sply_video_writer::write_mat(c_file_handle & fd, const cv::Mat & image, CompressionType compression)
{
  if( image.empty() || compression == CompressionNone ) {

    const cv_mat_header_storage mh = {
        .rows = image.rows,
        .cols = image.cols,
        .type = image.type(),
        .datasize = 0
    };

    if( !fd.write(mh) ) {
      return false;
    }

    if( !image.empty() ) {
      if( image.isContinuous() ) {

        const ssize_t cb =
            image.total() * image.elemSize();

        if( fd.write(image.data, cb) != cb ) {
          return false;
        }

      }
      else {

        const ssize_t row_size =
            image.cols * image.elemSize();

        for( int y = 0, ny = image.rows; y < ny; ++y ) {
          if( fd.write(image.ptr(y), row_size) != row_size ) {
            return false;
          }
        }
      }
    }

  }
  else {

    std::string codec;

    switch (compression) {
      case CompressionJpeg:
        codec = ".jpg";
        break;
      case CompressionPng:
        codec = ".png";
        break;
    }

    std::vector<uint8_t> pkt;

    static const std::vector<int> params = {
        cv::IMWRITE_JPEG_QUALITY, 99,
        cv::IMWRITE_JPEG_PROGRESSIVE, 1,
        cv::IMWRITE_JPEG_OPTIMIZE, 1,
        cv::IMWRITE_PNG_STRATEGY, cv::IMWRITE_PNG_STRATEGY_DEFAULT
    };

    if( !cv::imencode(codec, image, pkt, params) ) {
      CF_ERROR("cv::imencode('%s') fails", codec.c_str());
      return false;
    }

    const cv_mat_header_storage mh = {
        .rows = image.rows,
        .cols = image.cols,
        .type = image.type(),
        .datasize = (int) pkt.size()
    };

    if( !fd.write(mh) || fd.write(pkt.data(), pkt.size()) != pkt.size() ) {
      return false;
    }
  }

  return true;
}

bool c_sply_video_writer::write_image(c_file_handle & fd, const cv::Mat & image)
{
  return write_mat(fd, image, CompressionJpeg);
}

bool c_sply_video_writer::write_mask(c_file_handle & fd, const cv::Mat & mask)
{
  return write_mat(fd, mask, CompressionPng);
}



c_sply_video_reader::c_sply_video_reader(const std::string & filename) :
    _dtcs(filename)
{
  set_file_tag(default_file_tag());
}

const std::string& c_sply_video_reader::filename() const
{
  return _dtcs.filename();
}

const char* c_sply_video_reader::cfilename() const
{
  return _dtcs.cfilename();
}

void c_sply_video_reader::set_file_tag(uint64_t v)
{
  _dtcs.set_file_tag(v);
}

uint64_t c_sply_video_reader::file_tag() const
{
  return _dtcs.file_tag();
}

bool c_sply_video_reader::open(const std::string & filename)
{
  if( _dtcs.open(filename) ) {
    if( _dtcs.select_stream("video") ) {
      return true;
    }
    _dtcs.close();
  }
  return false;
}

void c_sply_video_reader::close()
{
  return _dtcs.close();
}

bool c_sply_video_reader::is_open() const
{
  return _dtcs.is_open();
}

bool c_sply_video_reader::read(c_sply_video_frame & frame)
{
  bool fOk =
      _dtcs.readx([&](c_file_handle & fd) -> bool {

        if ( !read_gps(fd, frame) ) {
          CF_ERROR("read_gps() fails: %s", strerror(errno));
          return false;
        }

        if ( !read_mat(fd, frame.image) ) {
          CF_ERROR("read_mat(frame.image) fails: %s", strerror(errno));
          return false;
        }

        if ( !read_mat(fd, frame.mask) ) {
          CF_ERROR("read_mat(frame.mask) fails: %s", strerror(errno));
          return false;
        }

        if ( !read_mat(fd, frame.cloud3d) ) {
          CF_ERROR("read_mat(frame.cloud3d) fails: %s", strerror(errno));
          return false;
        }

        if ( !read_mat(fd, frame.colors3d) ) {
          CF_ERROR("read_mat(frame.colors3d) fails: %s", strerror(errno));
          return false;
        }

        return true;

      });

  return fOk;
}

bool c_sply_video_reader::read_gps(c_file_handle & fd, c_sply_video_frame & frame)
{
  gps_storage gps;

  if( fd.read(&gps) ) {

    frame.ts = gps.ts;
    frame.lat = gps.lat;
    frame.lon = gps.lon;
    frame.elev = gps.elev;

    return true;
  }

  return false;
}


bool c_sply_video_reader::read_mat(c_file_handle & fd, cv::Mat & image)
{
  cv_mat_header_storage mh;

  if( !fd.read(&mh) ) {
    return false;
  }

  const int depth =
      CV_MAT_DEPTH(mh.type);

  const int cn =
      CV_MAT_CN(mh.type);

  if( mh.rows < 0 || mh.cols < 0 || depth < CV_8U || depth >= CV_16F || cn < 1 || cn > CV_CN_MAX ) {
    errno = ENOMSG;
    return true;
  }

  if( mh.rows < 1 || mh.cols < 1 ) {
    return true;
  }

  if( !image.isContinuous() || image.rows != mh.rows || image.cols != mh.cols || image.type() != mh.type ) {
    image.release();
  }

  const uint32_t total_bytes =
      mh.rows * mh.cols * CV_ELEM_SIZE(mh.type);

  if( mh.datasize <= 0 ) {

    image.create(mh.rows, mh.cols, mh.type);

    if( fd.read(image.data, total_bytes) != total_bytes ) {
      return false;
    }
  }
  else {

    std::vector<uint8_t> pkt(mh.datasize);

    if ( fd.read(pkt.data(), mh.datasize) != mh.datasize ) {
      return false;
    }

    if ( cv::imdecode(pkt, cv::IMREAD_UNCHANGED, &image).empty() ) {
      CF_ERROR("imdecode() fails");
      return false;
    }
  }

  return true;
}

