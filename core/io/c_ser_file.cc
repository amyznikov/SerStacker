/*
 * c_ser_file.cc
 *
 *  Created on: Mar 2, 2020
 *      Author: amyznikov
 */

#include "c_ser_file.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <core/proc/bswap.h>
#include <tbb/tbb.h>
#include <unistd.h>
#include <fcntl.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

static inline bool is_current_machine_little_endian()
{
  static const int num = 1;
  return (*(const char *) &num == 1);
}

template<typename T>
static inline void swap_endianess(T data[], size_t count)
{
  if ( sizeof(T) == 2 ) {
    union U
    {
      T x;
      uint16_t y;
    }* u;
    for ( u = reinterpret_cast<U*>(data); count--; ++u ) {
      u->x = bswap_16(u->x);
    }
  }
  else if ( sizeof(T) == 4 ) {
    union U
    {
      T x;
      uint32_t y;
    }* u;
    for ( u = reinterpret_cast<U*>(data); count--; ++u ) {
      u->x = bswap_16(u->x);
    }
  }
  else if ( sizeof(T) == 8 ) {
    union U
    {
      T x;
      uint64_t y;
    }* u;
    for ( u = reinterpret_cast<U*>(data); count--; ++u ) {
      u->x = bswap_16(u->x);
    }
  }
}

//static ssize_t file_size(int fd)
//{
//  ssize_t old_position, end_position;
//
//  if ( (old_position = lseek64(fd, 0, SEEK_CUR)) < 0 ) {
//    return -1;
//  }
//
//  if ( (end_position = lseek64(fd, 0, SEEK_END)) < 0 ) {
//    return -1;
//  }
//
//  lseek64(fd, old_position, SEEK_SET);
//
//  return end_position;
//}

c_ser_file::c_ser_file()
{
}

const c_ser_file::file_header & c_ser_file::header() const
{
  return _header;
}

int c_ser_file::image_width() const
{
  return _header.image_width;
}

int c_ser_file::image_height() const
{
  return _header.image_height;
}

int c_ser_file::bits_per_plane() const
{
  return _header.bits_per_plane;
}

int c_ser_file::bits_per_plane(int cvdepth)
{
  switch (cvdepth) {
    case CV_8U:
      return 8;
    case CV_8S:
      return 8;
    case CV_16U:
      return 16;
    case CV_16S:
      return 16;
    case CV_32S:
      return 32;
    case CV_32F:
      return -32;
    case CV_64F:
      return -64;
  }

  return -1;
}


int c_ser_file::bytes_per_plane(int bits_per_plane)
{
  if( bits_per_plane > 0 && bits_per_plane <= 8 ) {
    return 1; // CV_8U
  }

  if( bits_per_plane > 8 && bits_per_plane <= 16 ) {
    return 2; // CV_16U
  }

  if( bits_per_plane > 16 && bits_per_plane <= 32 ) {
    return 4; // CV_3S
  }

  if( bits_per_plane == -32 ) {
    return 4; // CV_32F
  }

  if( bits_per_plane == -64 ) {
    return 8; // CV_64F
  }

  return -1; // not supported
}

int c_ser_file::bytes_per_plane() const
{
  return bytes_per_plane(_header.bits_per_plane);
}

int c_ser_file::bytes_per_pixel() const
{
  return channels() * bytes_per_plane();
}

int c_ser_file::cvdepth(int bits_per_plane)
{
  if( bits_per_plane > 0 && bits_per_plane <= 8 ) {
    return CV_8U;
  }

  if( bits_per_plane > 8 && bits_per_plane <= 16 ) {
    return CV_16U;
  }

  if( bits_per_plane > 16 && bits_per_plane <= 32 ) {
    return CV_32S;
  }

  if( bits_per_plane == -32 ) {
    return CV_32F;
  }

  if( bits_per_plane == -64 ) {
    return CV_64F;
  }

  return -1;
}

int c_ser_file::cvdepth() const
{
  return cvdepth(_header.bits_per_plane);
}

int c_ser_file::cvtype() const
{
  const int depth = cvdepth();
  return depth < 0 ? -1 : CV_MAKETYPE(depth, channels());
}

enum COLORID c_ser_file::color_id() const
{
  return _header.color_id;
}

int c_ser_file::channels() const
{
  return (_header.color_id == COLORID_RGB || _header.color_id == COLORID_BGR) ? 3 : 1;
}


const char * c_ser_file::observer() const
{
  return _header.observer;
}

const char * c_ser_file::instrument() const
{
  return _header.instrument;
}

const char * c_ser_file::telescope() const
{
  return _header.telescope;
}

int c_ser_file::frame_size() const
{
  return _header.image_width * _header.image_height * bytes_per_pixel();
}

int c_ser_file::num_frames() const
{
  return _header.frames_count;
}

const std::vector<uint64_t> & c_ser_file::timestamps() const
{
  return _timestamps;
}

uint64_t c_ser_file::timestamps(int index) const
{
  return index >= 0 && index < (int) _timestamps.size() ? _timestamps[index] : 0;
}

///////////////////////////////////////////////////////////////////////////////

c_ser_reader::c_ser_reader(const std::string & filename)
{
  open(filename);
}

c_ser_reader::~c_ser_reader()
{
  close();
}

int32_t c_ser_reader::curpos() const
{
  return _curpos;
}

void c_ser_reader::close()
{
  _fd.close();
  _curpos = -1;
}

bool c_ser_reader::is_open() const
{
  return _fd.is_open();
}


bool c_ser_reader::open(const std::string & filename)
{
  close();

  if ( !_fd.open(filename.c_str(), O_RDONLY) ) {
    CF_ERROR("fd_.open('%s') fails: %s", filename.c_str(),
        strerror(errno));
    return false;
  }

  const ssize_t current_file_size =
      _fd.size();

  if ( current_file_size < sizeof(file_header) ) {
    CF_ERROR("Too small file size: %zd < sizeof(HEADER)", current_file_size);
    close();
    errno = ENODATA;
    return false;
  }

  if ( _fd.read(&_header, sizeof(_header)) != sizeof(_header) ) {
    CF_ERROR("read(SER header) fails: %s", strerror(errno));
    close();
    return false;
  }

  if ( strncmp(_header.file_id, "LUCAM-RECORDER", 14) != 0 ) {
    CF_ERROR("NOT A SER file: %s", filename.c_str());
    close();
    errno = ENODATA;
    return false;
  }

  /// There is well-known bug with endiannes in ser file format
  _header.is_little_endian = _header.is_little_endian == 0;  // force from incorrect endian

  if ( !is_current_machine_little_endian() ) {
    swap_endianess(&_header.luid, 1);
    swap_endianess((int32_t*) &_header.color_id, 1);
    swap_endianess(&_header.image_width, 1);
    swap_endianess(&_header.image_height, 1);
    swap_endianess(&_header.bits_per_plane, 1);
    swap_endianess(&_header.frames_count, 1);
    swap_endianess(&_header.date_time, 1);
    swap_endianess(&_header.date_time_utc, 1);
  }

  if ( _header.image_width < 1 || _header.image_height < 1 ) {
    CF_ERROR("Unsupported image size in %s : ImageWidth=%d ImageHeight=%d PixelDepthPerPlane=%d",
        filename.c_str(), _header.image_width, _header.image_height, _header.bits_per_plane);
    close();
    errno = ENODATA;
    return false;
  }

  if ( cvdepth(_header.bits_per_plane) < 0 ) {
    CF_ERROR("Unsupported pixel format in %s : ImageWidth=%d ImageHeight=%d PixelDepthPerPlane=%d",
        filename.c_str(), _header.image_width, _header.image_height, _header.bits_per_plane);
    close();
    errno = ENODATA;
    return false;
  }


  switch ( _header.color_id ) {
  case COLORID_MONO :
    case COLORID_BAYER_RGGB :
    case COLORID_BAYER_GRBG :
    case COLORID_BAYER_GBRG :
    case COLORID_BAYER_BGGR :
    case COLORID_BAYER_CYYM :
    case COLORID_BAYER_YCMY :
    case COLORID_BAYER_YMCY :
    case COLORID_BAYER_MYYC :
    case COLORID_RGB :
    case COLORID_BGR :
    break;
  default :
    CF_FATAL("Unsupported ColorId=%d in %s", _header.color_id, filename.c_str());
    close();
    errno = ENODATA;
    return false;
  }

  _curpos = 0;

  // Check file is large enough to have time stamps
  const ssize_t timestamps_array_offset =
      sizeof(file_header) + _header.frames_count * frame_size();


  const ssize_t timestamps_array_size_required =
      _header.frames_count * sizeof(_timestamps[0]);

//  CF_DEBUG("timestamps_array_offset=%zd timestamps_array_size_required=%zd sum=%zd current_file_size=%zd",
//      timestamps_array_offset,
//      timestamps_array_size_required,
//      timestamps_array_offset + timestamps_array_size_required,
//      current_file_size);

  if ( !(current_file_size >= timestamps_array_offset + timestamps_array_size_required) ) {
    // CF_DEBUG("No valid timestamps found");
  }
  else {

    // CF_DEBUG("timestamps found");

    const ssize_t backup_pos =
        _fd.whence();

    if ( _fd.seek(timestamps_array_offset, SEEK_SET) != timestamps_array_offset ) {
      CF_ERROR("fd_.seek(timestamps_array_offset) fails");
    }
    else {

      _timestamps.resize(_header.frames_count, 0);

      const ssize_t bytest_to_read =
          sizeof(_timestamps[0]) * _timestamps.size();

      if ( _fd.read(_timestamps.data(), bytest_to_read) != bytest_to_read ) {
        CF_ERROR("::read(timestamps_) fails : %s", strerror(errno));
        return false;
      }

      _fd.seek(backup_pos, SEEK_SET);

      if ( _header.is_little_endian != is_current_machine_little_endian() ) {

        swap_endianess(_timestamps.data(),
            _timestamps.size());
      }

    }
  }

  //  CF_DEBUG("timestamps_.size=%zu", timestamps_.size());

  return true;
}

bool c_ser_reader::seek(int frame_index)
{
  if ( !is_open() ) {
    CF_ERROR("File is not opened");
    errno = ESPIPE;
    return false;
  }

  if ( frame_index < 0 ) {
    frame_index = 0;
  }

  if ( frame_index != _curpos ) {

    if ( frame_index >= _header.frames_count ) {
      CF_ERROR("Invalid seek %d >= FrameCount=%d",
          frame_index,
          _header.frames_count);

      errno = ESPIPE;
      return false;
    }

    const ssize_t seekpos =
        sizeof(_header) + (ssize_t) frame_index * frame_size();

    if ( _fd.seek(seekpos, SEEK_SET) != seekpos ) {

      CF_ERROR("lseek64(seekpos=%zd, num_frames=%d) fails: %s",
          seekpos,
          _header.frames_count,
          strerror(errno));

      return false;
    }

    _curpos = frame_index;
  }

  return true;
}

bool c_ser_reader::read(cv::OutputArray output_image)
{
  if ( !is_open() ) {
    CF_ERROR("File is not opened");
    errno = ESPIPE;
    return false;
  }

  if ( _curpos >= _header.frames_count ) {
    errno = EPIPE;
    return false;
  }

  errno = 0;

  const int required_image_type =
      this->cvtype();


  if( output_image.fixedType() && output_image.type() != required_image_type ) {
    CF_ERROR("Requested output image fixed type (depth=%d channels=%d) not match to \n"
        "image type in SER file (depth=%d channels=%d)",
        output_image.depth(), output_image.channels(),
        CV_MAT_DEPTH(required_image_type), this->channels());
    return false;
  }

  const cv::Size required_image_size(_header.image_width, _header.image_height);

  if( output_image.fixedSize() && output_image.size() != required_image_size ) {
    CF_ERROR("Requested output image fixed size %dx%d not match to \n"
        "image size in SER file %dx%d",
        output_image.cols(), output_image.rows(),
        required_image_size.width, required_image_size.height);
    return false;
  }

  if ( !output_image.empty() && !output_image.isContinuous() ) {
    output_image.release();
  }

  output_image.create(required_image_size, required_image_type);

  cv::Mat & image = output_image.getMatRef();

  const ssize_t savedpos = _fd.whence();
  const ssize_t bytes_to_read = image.total() * image.elemSize();
  const ssize_t bytes_read = _fd.read(image.data, bytes_to_read);

  if ( bytes_read != bytes_to_read ) {

    CF_ERROR("read() fails: %s. bytes_to_read=%zd bytes_read=%zd curpos=%zd",
        strerror(errno),
        bytes_to_read,
        bytes_read,
        savedpos);

    _fd.seek(savedpos, SEEK_SET);

    return false;
  }

  if( _header.is_little_endian != is_current_machine_little_endian() ) {

    switch (bytes_per_pixel()) {
      case 2:
        swap_endianess(reinterpret_cast<uint16_t*>(image.data), image.total());
        break;
      case 4:
        swap_endianess(reinterpret_cast<uint32_t*>(image.data), image.total());
        break;
    }
  }

  ++_curpos;

  return true;
}

c_ser_writer::~c_ser_writer()
{
  close();
}

bool c_ser_writer::is_open() const
{
  return _fd.is_open();
}

bool c_ser_writer::create(const std::string & filename, int image_width, int image_height,
    enum COLORID color_id, int bits_per_plane)
{
  close();

  if ( image_width < 1 || image_height < 1 || cvdepth(bits_per_plane) < 0 ) {

    CF_FATAL("Unsupported image size specified: "
        "width=%d height=%d PixelDepthPerPlane=%d",
        image_width, image_height,
        bits_per_plane);

    errno = EINVAL;
    return false;
  }

  switch ( color_id ) {
  case COLORID_MONO :
    case COLORID_BAYER_RGGB :
    case COLORID_BAYER_GRBG :
    case COLORID_BAYER_GBRG :
    case COLORID_BAYER_BGGR :
    case COLORID_BAYER_CYYM :
    case COLORID_BAYER_YCMY :
    case COLORID_BAYER_YMCY :
    case COLORID_BAYER_MYYC :
    case COLORID_RGB :
    case COLORID_BGR :
    break;
  default :
    CF_FATAL("Invalid ColorId specified: %d", color_id);
    errno = EINVAL;
    return false;
  }

  _header.color_id = color_id;
  _header.is_little_endian = !is_current_machine_little_endian();  // force incorrect endian
  _header.image_width = image_width;
  _header.image_height = image_height;
  _header.bits_per_plane = bits_per_plane;
  _header.frames_count = 0;
  _header.date_time = 0;
  _header.date_time_utc = 0;

//  const int openflags =
//      O_CREAT | O_TRUNC | O_WRONLY;

  if ( !_fd.create(filename) ) {
    CF_FATAL("open('%s') fails: %s", filename.c_str(), strerror(errno));
    return false;
  }

  if ( _fd.write(&_header, sizeof(_header)) != sizeof(_header) ) {
    CF_FATAL("write(SER HEADER) fails: %s", strerror(errno));
    _fd.close();
    return false;
  }

  flush();

  return true;
}

bool c_ser_writer::close()
{
  bool fok = true;

  if( _fd.is_open() ) {

    _fd.seek(0, SEEK_SET);

    if( _fd.write(&_header, sizeof(_header)) != sizeof(_header) ) {
      CF_FATAL("write(SER HEADER) fails: %s", strerror(errno));
      fok = false;
    }

    if ( !_timestamps.empty() ) {

      // time stamps must be in little endian format
      if( !is_current_machine_little_endian() ) {
        swap_endianess(_timestamps.data(), _timestamps.size());
      }

      const size_t cb =
          _timestamps.size() * sizeof(_timestamps[0]);

      _fd.seek(0, SEEK_END);

      if( _fd.write(_timestamps.data(), cb) != cb ) {
        CF_ERROR("write(timestamps) fails: %s", strerror(errno));
      }

    }

    _fd.close();
  }

  _timestamps.clear();

  return fok;
}

bool c_ser_writer::flush()
{
  return _fd.flush();
}

bool c_ser_writer::write(cv::InputArray _image, uint64_t ts)
{
  if ( !is_open() ) {
    CF_ERROR("File is not opened");
    errno = EBADF;
    return false;
  }

  const cv::Mat image =
      _image.getMat();

  if ( image.cols != _header.image_width || image.rows != _header.image_height ) {
    CF_ERROR("invalid image specified: %dx%dx%d depth:%d expected: %dx%dx%d depth:%d",
        image.cols, image.rows, image.channels(), image.depth(),
        _header.image_width, _header.image_height, channels(), cvdepth());
    errno = EINVAL;
    return false;
  }

  if ( image.channels() != this->channels() ) {
    CF_ERROR("Invalid number of channels in input image: %d. Expected %d channels",
        image.channels(), this->channels());
    errno = EINVAL;
    return false;
  }

  if ( image.depth() != this->cvdepth() ) {
    CF_ERROR("Invalid input image depth: %d. Expected depth=%d",
        image.depth(), this->cvdepth());
    errno = EINVAL;
    return false;
  }

  errno = 0;

  const int64_t savedpos =
      _fd.whence();

  const size_t bytes_to_write =
      image.total() * image.elemSize();

  const size_t bytes_written =
      _fd.write(image.data, bytes_to_write);


  if ( bytes_written != bytes_to_write ) {

    CF_ERROR("write() fails: %s. "
        "bytes_to_write=%zu bytes_written=%zu",
        strerror(errno), bytes_to_write, bytes_written);

    _fd.seek(savedpos);

    return false;
  }

  if( _header.frames_count < 1 ) {
    _header.date_time =
        _header.date_time_utc =
            ts;
  }

  _header.frames_count++;

  _timestamps.emplace_back(ts);




  return true;
}

///////////////////////////////////////////////////////////////////////////////

