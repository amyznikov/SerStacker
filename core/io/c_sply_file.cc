/*
 * c_sply_file.cc
 *
 *  Created on: Jul 22, 2024
 *      Author: amyznikov
 */

#include "c_sply_file.h"
#include <core/ssprintf.h>
#include <core/debug.h>

//    enum KindFlag {

static int get_array_items_count(cv::InputArrayOfArrays a)
{
  switch (a.kind()) {
    case cv::_InputArray::MAT:
    case cv::_InputArray::MATX:
    case cv::_InputArray::UMAT:
    case cv::_InputArray::CUDA_GPU_MAT:
    case cv::_InputArray::STD_VECTOR:
    case cv::_InputArray::STD_BOOL_VECTOR:
#if OPENCV_ABI_COMPATIBILITY < 500
    case cv::_InputArray::STD_ARRAY:
#endif
      return 1;

    case cv::_InputArray::STD_VECTOR_VECTOR:
    case cv::_InputArray::STD_VECTOR_MAT:
    case cv::_InputArray::STD_VECTOR_UMAT:
    case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
    case cv::_InputArray::STD_ARRAY_MAT:
      return a.total();
  }

  return 0;
}

static cv::Mat getMat(cv::InputArrayOfArrays a, int i)
{
  if ( !a.empty() ) {

    switch (a.kind()) {
      case cv::_InputArray::MAT:
      case cv::_InputArray::MATX:
      case cv::_InputArray::UMAT:
      case cv::_InputArray::CUDA_GPU_MAT:
      case cv::_InputArray::STD_VECTOR:
      case cv::_InputArray::STD_ARRAY:
      case cv::_InputArray::STD_BOOL_VECTOR:
        return a.getMat(-1);

      case cv::_InputArray::STD_VECTOR_VECTOR:
      case cv::_InputArray::STD_VECTOR_MAT:
      case cv::_InputArray::STD_VECTOR_UMAT:
      case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
      case cv::_InputArray::STD_ARRAY_MAT:
        return a.getMat(i);
    }
  }

  return cv::Mat();
}


//static bool setMat(const cv::Mat & cloud, cv::OutputArrayOfArrays a)
//{
//  if ( !a.fixedType() ) {
//
//  }
//
//
//  const int cn =
//      cloud.channels();
//
//  if( a.channels() == cn ) {
//    m.convertTo(m, m.depth());
//  }
//  else if( cn == 1 && m.channels() > 1 ) {
//
//    std::vector<cv::Mat> channels(m.channels());
//    for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
//      channels[c] = mat;
//    }
//
//    if( m.depth() == depth ) {
//      cv::merge(channels, m);
//    }
//    else {
//      cv::merge(channels, mat);
//      mat.convertTo(m, m.depth());
//    }
//  }
//  else {
//    CF_ERROR("Can not convert matrix of type %d into destination type %d", mat.type(), m.type());
//    return false;
//  }
//
//}

static bool setClouds(const char * output_name,  cv::OutputArrayOfArrays a, const std::vector<cv::Mat> & clouds)
{
//    CF_DEBUG("%s: needed=%d fixedType=%d fixedSize=%d kind=%d (%s)",
//        output_name,
//        a.needed(),
//        a.fixedType(),
//        a.fixedSize(),
//        a.kind(),
//        toCString(a.kind()));

  if( a.needed() ) {

    if( clouds.empty() ) {
      a.release();
    }
    else {

      switch (a.kind()) {
        case cv::_InputArray::MAT:

          if ( !a.fixedType() ) {
            a.assign(clouds[0]);
          }
          else if ( clouds[0].channels() == a.channels() ) {
            clouds[0].convertTo(a, a.type());
          }
          else if ( clouds[0].channels() == 1  ) {
            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = clouds[0];
            }

            cv::Mat tmp;
            cv::merge(channels, tmp);

            if ( a.depth() != tmp.depth() ) {
              a.assign(tmp);
            }
            else {
              tmp.convertTo(a, a.depth());
            }
          }
          else {
            CF_ERROR("Not supported output array channels for '%s': fixedType=%d fixedSize=%d kind=%d (%s) src.channels=%d dst.channels=%d",
                output_name,
                a.fixedType(),
                a.fixedSize(),
                a.kind(),
                toCString(a.kind()),
                clouds[0].channels(),
                a.channels());

            return false;
          }

          break;

        case cv::_InputArray::STD_VECTOR_MAT:
          a.createSameSize(clouds, clouds[0].type());
          a.assign(clouds);
          break;

        case cv::_InputArray::STD_VECTOR:
          if ( clouds[0].channels() == a.channels() ) {
            clouds[0].convertTo(a, a.type());
          }
          else if ( clouds[0].channels() == 1  ) {

            std::vector<cv::Mat> channels(a.channels());
            for( uint32_t c = 0, cc = channels.size(); c < cc; ++c ) {
              channels[c] = clouds[0];
            }

            cv::Mat tmp;
            cv::merge(channels, tmp);
            tmp.convertTo(a, a.type());
          }
          else {
            CF_ERROR("Not supported output array channels for '%s': fixedType=%d fixedSize=%d kind=%d (%s) src.channels=%d dst.channels=%d",
                output_name,
                a.fixedType(),
                a.fixedSize(),
                a.kind(),
                toCString(a.kind()),
                clouds[0].channels(),
                a.channels());

            return false;
          }

          break;

        default:
          CF_ERROR("Not supported output array kind for '%s': fixedType=%d fixedSize=%d kind=%d (%s)",
              output_name,
              a.fixedType(),
              a.fixedSize(),
              a.kind(),
              toCString(a.kind()));

          return false;
      }

    }

  }


  return true;
}


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

bool c_sply_writer::write(uint32_t stream_index, cv::InputArrayOfArrays points,
    cv::InputArrayOfArrays colors, cv::InputArrayOfArrays timestamps )
{
  if( stream_index >= _datafile.num_streams() ) {
    CF_ERROR("Invalid stream index %u specified. _datafile.num_streams()=%u", stream_index, _datafile.num_streams());
    errno = EINVAL;
    return false;
  }


  _datafile.writex(stream_index,
      [&](c_file_handle & fd) -> bool {

        c_sply_frame_header2 fh;
        c_sply_cloud_header ch;


        const int num_clouds =
            get_array_items_count(points);

        //CF_DEBUG("num_clouds=%d", num_clouds);

        if ( num_clouds < 1 ) {
          CF_ERROR("get_array_items_count(points) fails: %d", num_clouds);
          return false;
        }


        fh.num_clouds = num_clouds;
        if ( !fd.write(fh) ) {
          CF_ERROR("fd.write(c_sply_frame_header2) fails: %s", strerror(errno));
          return false;
        }


        for ( int i = 0; i < num_clouds; ++i ) {

          cv::Mat _points = getMat(points, i);
          cv::Mat _colors = getMat(colors, i);
          cv::Mat _timestamps = getMat(timestamps, i);

          //          CF_DEBUG("[%d] points: %dx%d colors: %dx%d timestamps: %dx%d", i,
          //              _points.rows, _points.cols,
          //              _colors.rows, _colors.cols,
          //              _timestamps.rows, _timestamps.cols);

          if( !_points.empty() && !_points.isContinuous() ) {
            cv::Mat tmp; _points.copyTo(tmp); _points = tmp;
          }

          if( !_colors.empty() && !_colors.isContinuous() ) {
            cv::Mat tmp; _colors.copyTo(tmp); _colors = tmp;
          }

          if( !_timestamps.empty() && !_timestamps.isContinuous() ) {
            cv::Mat tmp; _timestamps.copyTo(tmp); _timestamps = tmp;
          }

          ch.cloud_timestamp = 0;
          ch.points.rows = _points.rows;
          ch.points.cols = _points.cols;
          ch.points.type = _points.type();

          ch.colors.rows = _colors.rows;
          ch.colors.cols = _colors.cols;
          ch.colors.type = _colors.type();

          ch.timestamps.rows = _timestamps.rows;
          ch.timestamps.cols = _timestamps.cols;
          ch.timestamps.type = _timestamps.type();

          if ( !fd.write(ch) ) {
            CF_ERROR("fd.write(c_sply_cloud_header) fails: %s", strerror(errno));
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

bool c_sply_reader::read(uint32_t stream_index, cv::OutputArrayOfArrays points, cv::OutputArrayOfArrays colors,
    cv::OutputArrayOfArrays timestamps)
{
  if( stream_index >= _datafile.num_streams() ) {
    CF_ERROR("Invalid stream index %u specified. _datafile.num_streams()=%u",
        stream_index, _datafile.num_streams());
    errno = EINVAL;
    return false;
  }

//  CF_DEBUG("points: fixedType=%d fixedSize=%d kind=%d (%s)",
//      points.fixedType(),
//      points.fixedSize(),
//      points.kind(),
//      toCString(points.kind()));

  bool fOk =
      _datafile.readx(stream_index,
          [&](c_file_handle & fd) {

            c_sply_frame_header2 fh;
            c_sply_cloud_header ch;

            if ( !fd.read(&fh) ) {
              CF_ERROR("fd.read(frame_header) fails: %s", strerror(errno));
              return false;
            }

//            CF_DEBUG("fh.num_clouds=%d", fh.num_clouds);

            std::vector<cv::Mat> _points(fh.num_clouds);
            std::vector<cv::Mat> _colors(fh.num_clouds);
            std::vector<cv::Mat> _timestamps(fh.num_clouds);

            for ( int i = 0; i < fh.num_clouds; ++i ) {

              if ( !fd.read(&ch) ) {
                CF_ERROR("fd.read(cloud_header[%d]) fails: %s", i, strerror(errno));
                return false;
              }

//              CF_DEBUG("\n"
//                  "cloud[%d]: \n"
//                  "  timestamp=%g \n"
//                  "  points: %dx%d depth=%d channels=%d\n"
//                  "  colors: %dx%d depth=%d channels=%d\n"
//                  "  timestamps: %dx%d depth=%d channels=%d\n"
//                  ,
//                  i,
//                  ch.cloud_timestamp,
//                  ch.points.rows, ch.points.cols, CV_MAT_DEPTH(ch.points.type), CV_MAT_CN(ch.points.type),
//                  ch.colors.rows, ch.colors.cols, CV_MAT_DEPTH(ch.colors.type), CV_MAT_CN(ch.colors.type),
//                  ch.timestamps.rows, ch.timestamps.cols, CV_MAT_DEPTH(ch.timestamps.type), CV_MAT_CN(ch.timestamps.type));

              if ( !readmat(fd, ch.points.rows, ch.points.cols, ch.points.type, _points[i]) ) {
                CF_ERROR("readmat(_points[%d]) fails: %s", i, strerror(errno));
                return false;
              }

              if ( !readmat(fd, ch.colors.rows, ch.colors.cols, ch.colors.type, _colors[i]) ) {
                CF_ERROR("readmat(_colors[%d) fails: %s", i, strerror(errno));
                return false;
              }

              if ( !readmat(fd, ch.timestamps.rows, ch.timestamps.cols, ch.timestamps.type, _timestamps[i]) ) {
                CF_ERROR("readmat(_timestamps[%d]) fails: %s", i, strerror(errno));
                return false;
              }
            }


            setClouds("points", points, _points);
            setClouds("colors", colors, _colors);
            setClouds("timestamps", timestamps, _timestamps);

            return true;
          });

  return fOk;
}
