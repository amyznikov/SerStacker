/*
 * c_sply_file.cc
 *
 *  Created on: Jul 22, 2024
 *      Author: amyznikov
 */

#include "c_sply_file.h"
#include <core/debug.h>

c_sply_writer::c_sply_writer(const std::string & filename) :
  _datafile(filename)
{
  _datafile.set_file_tag(default_sply_file_tag());
}

c_sply_writer::~c_sply_writer()
{
  close();
}

bool c_sply_writer::create(const std::string & filename)
{
  return _datafile.create(filename);
}

void c_sply_writer::close()
{
  _datafile.close();
}

bool c_sply_writer::is_open() const
{
  return _datafile.is_open();
}

int c_sply_writer::add_stream(const std::string & stream_name)
{
  return _datafile.add_stream(stream_name);
}

bool c_sply_writer::write(uint32_t stream_index, cv::InputArray points,
    cv::InputArray colors, cv::InputArray timestamps )
{
  if( stream_index >= _datafile.num_streams() ) {
    CF_ERROR("Invalid stream index %u specified. _datafile.num_streams()=%u", stream_index, _datafile.num_streams());
    errno = EINVAL;
    return false;
  }

  cv::Mat _points, _colors, _timestamps;

  if( !points.empty() ) {
    if( points.isContinuous() ) {
      _points = points.getMat();
    }
    else {
      points.copyTo(_points);
    }
  }

  if( !colors.empty() ) {
    if( colors.isContinuous() ) {
      _colors = colors.getMat();
    }
    else {
      colors.copyTo(_colors);
    }
  }

  if( !timestamps.empty() ) {
    if( timestamps.isContinuous() ) {
      _timestamps = timestamps.getMat();
    }
    else {
      timestamps.copyTo(_timestamps);
    }
  }

  _datafile.writex(stream_index,
      [&](c_file_handle & fd) -> bool {

        c_sply_frame_header frame_header;

        frame_header.frame_timestamp = 0;

        // CF_DEBUG("points: rows=%d cols=%d", _points.rows, _points.cols);

        frame_header.points_rows = _points.rows;
        frame_header.points_cols = _points.cols;
        frame_header.points_type = _points.type();

        frame_header.colors_rows = _colors.rows;
        frame_header.colors_cols = _colors.cols;
        frame_header.colors_type = _colors.type();

        frame_header.timestamps_rows = _timestamps.rows;
        frame_header.timestamps_cols = _timestamps.cols;
        frame_header.timestamps_type = _timestamps.type();

        if ( !fd.write(frame_header) ) {
          CF_ERROR("fd.write(frame_header) fails: %s", strerror(errno));
          return false;
        }

        if ( !_points.empty() ) {
          const ssize_t cb = _points.total() * _points.elemSize();
          if ( fd.write(_points.data, cb) != cb ) {
            CF_ERROR("fd.write(_points.data) fails: %s", strerror(errno));
            return false;
          }
        }

        if ( !_colors.empty() ) {
          const ssize_t cb = _colors.total() * _colors.elemSize();
          if ( fd.write(_colors.data, cb) != cb ) {
            CF_ERROR("fd.write(_colors.data) fails: %s", strerror(errno));
            return false;
          }
        }

        if ( !_timestamps.empty() ) {
          const ssize_t cb = _timestamps.total() * _timestamps.elemSize();
          if ( fd.write(_timestamps.data, cb) != cb ) {
            CF_ERROR("fd.write(_timestamps.data) fails: %s", strerror(errno));
            return false;
          }
        }

        return true;
      });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_sply_reader::c_sply_reader(const std::string & filename) :
   _datafile(filename)
{
  _datafile.set_file_tag(default_sply_file_tag());
}

c_sply_reader::~c_sply_reader()
{
  close();
}

bool c_sply_reader::is_open() const
{
  return _datafile.is_open();
}


bool c_sply_reader::open(const std::string & filename )
{
  return _datafile.open(filename);
}

void c_sply_reader::close()
{
  _datafile.close();
}

int c_sply_reader::find_stream(const std::string & stream_name)
{
  return _datafile.find_stream(stream_name);
}

bool c_sply_reader::select_stream(uint32_t index)
{
  return _datafile.select_stream(index);
}

uint32_t c_sply_reader::num_frames(uint32_t stream_index) const
{
  return _datafile.num_frames(stream_index);
}

uint32_t c_sply_reader::num_frames() const
{
  return _datafile.num_frames();
}

bool c_sply_reader::seek(uint32_t stream_index, uint32_t pos)
{
  return _datafile.seek(stream_index, pos);
}

bool c_sply_reader::seek(uint32_t pos)
{
  return _datafile.seek(pos);
}

uint32_t c_sply_reader::curpos(uint32_t stream_index) const
{
  return _datafile.curpos(stream_index);
}

uint32_t c_sply_reader::curpos() const
{
  return _datafile.curpos();
}

static bool readmat(c_file_handle & fd, int rows, int cols, int type, cv::OutputArray m)
{
  const int depth =
      CV_MAT_DEPTH(type);

  const int cn =
      CV_MAT_CN(type);

  //CF_DEBUG("rows=%d cols=%d depth=%d cn=%d", rows, cols, depth, cn);

  if( rows < 0 || cols < 0 ) {
    CF_ERROR("Invalid mat size %dx%d", rows, cols);
    return false;
  }

  if( rows == 0 || cols == 0 ) {
    return true;
  }


  if( depth < CV_8U || depth >= CV_16F ) {
    CF_ERROR("Invalid mat depth %d specified", depth);
    return false;
  }


  if( cn < 1 || cn > CV_CN_MAX ) {
    CF_ERROR("Invalid mat channels %d specified", cn);
    return false;
  }

  const uint32_t total_bytes =
      rows * cols * CV_ELEM_SIZE(type);

  if( !m.needed() ) {
    m.release();
    fd.seek(total_bytes, SEEK_CUR);
  }
  else if( !m.fixedType() ) {

    if( !m.empty() && !m.isContinuous() ) {
      m.release();
    }

    m.create(rows, cols, type);

    if( fd.read(m.getMatRef().data, total_bytes) != total_bytes ) {
      CF_ERROR("fd.read(mat.data) fails: %s", strerror(errno));
      return false;
    }

  }
  else {

    cv::Mat mat(rows, cols, type);

    if( fd.read(mat.data, total_bytes) != total_bytes ) {
      CF_ERROR("fd.read(mat.data) fails: %s", strerror(errno));
      return false;
    }

    if( m.channels() == cn ) {
      mat.convertTo(m, m.depth());
    }
    else if( cn == 1 && m.channels() > 1 ) {

      std::vector<cv::Mat> channels(m.channels());
      for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
        channels[c] = mat;
      }

      if( m.depth() == depth ) {
        cv::merge(channels, m);
      }
      else {
        cv::merge(channels, mat);
        mat.convertTo(m, m.depth());
      }
    }
    else {
      CF_ERROR("Can not convert matrix of type %d into destination type %d", mat.type(), m.type());
      return false;
    }
  }

  return true;
}

bool c_sply_reader::read(cv::OutputArray points, cv::OutputArray colors, cv::OutputArray timestamps)
{
  return read(_datafile.selected_stream(), points, colors, timestamps);
}

bool c_sply_reader::read(uint32_t stream_index, cv::OutputArray points, cv::OutputArray colors, cv::OutputArray timestamps)
{
  if( stream_index >= _datafile.num_streams() ) {
    CF_ERROR("Invalid stream index %u specified. _datafile.num_streams()=%u",
        stream_index, _datafile.num_streams());
    errno = EINVAL;
    return false;
  }

  bool fOk =
      _datafile.readx(stream_index,
          [&](c_file_handle & fd) {

            c_sply_frame_header fh;

            if ( !fd.read(&fh) ) {
              CF_ERROR("fd.read(frame_header) fails: %s", strerror(errno));
              return false;
            }

            if ( !readmat(fd, fh.points_rows, fh.points_cols, fh.points_type, points) ) {
              CF_ERROR("readmat(points) fails: %s", strerror(errno));
              return false;
            }

            if ( !readmat(fd, fh.colors_rows, fh.colors_cols, fh.colors_type, colors) ) {
              CF_ERROR("readmat(colors) fails: %s", strerror(errno));
              return false;
            }

            if ( !readmat(fd, fh.timestamps_rows, fh.timestamps_cols, fh.timestamps_type, timestamps) ) {
              CF_ERROR("readmat(timestamps) fails: %s", strerror(errno));
              return false;
            }

            return true;
          });

  return fOk;
}
