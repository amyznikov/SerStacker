/*
 * c_ser_file.cc
 *
 *  Created on: Mar 2, 2020
 *      Author: amyznikov
 */

#include "c_ser_file.h"
#include <byteswap.h>
#include <endian.h>
#include <tbb/tbb.h>
#include <unistd.h>
#include <fcntl.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

static inline constexpr bool is_current_machine_little_endian()
{
  constexpr int num = 1;
  return (*(char *) &num == 1);
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

static ssize_t file_size(int fd)
{
  ssize_t old_position, end_position;

  if ( (old_position = lseek64(fd, 0, SEEK_CUR)) < 0 ) {
    return -1;
  }

  if ( (end_position = lseek64(fd, 0, SEEK_END)) < 0 ) {
    return -1;
  }

  lseek64(fd, old_position, SEEK_SET);

  return end_position;
}

c_ser_file::c_ser_file()
{
}

const c_ser_file::file_header & c_ser_file::header() const
{
  return header_;
}

int c_ser_file::image_width() const
{
  return header_.image_width;
}

int c_ser_file::image_height() const
{
  return header_.image_height;
}

int c_ser_file::bits_per_plane() const
{
  return header_.bits_per_plane;
}

int c_ser_file::bytes_per_plane() const
{
  return bits_per_plane() <= 8 ? 1 : 2;
}

enum COLORID c_ser_file::color_id() const
{
  return header_.color_id;
}

int c_ser_file::channels() const
{
  return (header_.color_id == COLORID_RGB || header_.color_id == COLORID_BGR) ? 3 : 1;
}

int c_ser_file::bytes_per_pixel() const
{
  return channels() * bytes_per_plane();
}

int c_ser_file::cvdepth() const
{
  return (header_.bits_per_plane <= 8 ? CV_8U : CV_16U);
}

int c_ser_file::cvtype() const
{
  return CV_MAKETYPE(cvdepth(), channels());
}

const char * c_ser_file::observer() const
{
  return header_.observer;
}

const char * c_ser_file::instrument() const
{
  return header_.instrument;
}

const char * c_ser_file::telescope() const
{
  return header_.telescope;
}

int c_ser_file::frame_size() const
{
  return header_.image_width * header_.image_height * bytes_per_pixel();
}

int c_ser_file::num_frames() const
{
  return header_.frames_count;
}

const std::vector<uint64_t> & c_ser_file::timestamps() const
{
  return timestamps_;
}

uint64_t c_ser_file::timestamps(int index) const
{
  return index >= 0 && index < (int) timestamps_.size() ? timestamps_[index] : 0;
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
  return curpos_;
}

bool c_ser_reader::close()
{
  if ( fd >= 0 ) {
    ::close(fd), fd = -1;
  }

  curpos_ = -1;
  return true;
}

bool c_ser_reader::is_open() const
{
  return fd >= 0;
}


bool c_ser_reader::open(const std::string & filename)
{
  close();

  if ( (fd = ::open(filename.c_str(), O_RDONLY)) < 0 ) {
    CF_ERROR("fopen('%s') fails: %s", filename.c_str(), strerror(errno));
    return false;
  }

  const ssize_t current_file_size = file_size(fd);
  if ( current_file_size < sizeof(file_header) ) {
    CF_ERROR("Too small file size: %zd < sizeof(HEADER)", current_file_size);
    close();
    errno = ENODATA;
    return false;
  }

  if ( ::read(fd, &header_, sizeof(header_)) != sizeof(header_) ) {
    CF_ERROR("fread(SER header) fails: %s", strerror(errno));
    close();
    return false;
  }

  if ( strncmp(header_.file_id, "LUCAM-RECORDER", 14) != 0 ) {
    CF_ERROR("NOT A SER file: %s", filename.c_str());
    close();
    errno = ENODATA;
    return false;
  }

  /// There is well-known bug with endiannes in ser file format
  header_.is_little_endian = header_.is_little_endian == 0;  // force from incorrect endian
  if ( !is_current_machine_little_endian() ) {
    swap_endianess(&header_.luid, 1);
    swap_endianess((int32_t*) &header_.color_id, 1);
    swap_endianess(&header_.image_width, 1);
    swap_endianess(&header_.image_height, 1);
    swap_endianess(&header_.bits_per_plane, 1);
    swap_endianess(&header_.frames_count, 1);
    swap_endianess(&header_.date_time, 1);
    swap_endianess(&header_.date_time_utc, 1);
  }

  if ( header_.image_width < 1 || header_.image_height < 1 || header_.bits_per_plane < 1
      || header_.bits_per_plane > 16 ) {
    CF_ERROR("Unsupported image size in %s : ImageWidth=%d ImageHeight=%d PixelDepthPerPlane=%d",
        filename.c_str(), header_.image_width, header_.image_height, header_.bits_per_plane);
    close();
    errno = ENODATA;
    return false;
  }

  switch ( header_.color_id ) {
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
    CF_FATAL("Unsupported ColorId=%d in %s", header_.color_id, filename.c_str());
    close();
    errno = ENODATA;
    return false;
  }

  curpos_ = 0;

  // Check file is large enough to have time stamps
  const ssize_t timestamps_array_offset =
      sizeof(file_header) + header_.frames_count * frame_size();


  const ssize_t timestamps_array_size_required =
      header_.frames_count * sizeof(uint64_t);

  if ( current_file_size >= timestamps_array_offset + timestamps_array_size_required ) {

    const ssize_t backup_pos =
        lseek64(fd, 0, SEEK_CUR);

    if ( lseek64(fd, timestamps_array_offset, SEEK_SET) == timestamps_array_offset ) {

      timestamps_.resize(header_.frames_count, 0);

      ::read(fd, timestamps_.data(),
          sizeof(timestamps_[0]) * timestamps_.size());

      lseek64(fd, backup_pos, SEEK_SET);

      if ( header_.is_little_endian != is_current_machine_little_endian() ) {

        swap_endianess(timestamps_.data(),
            timestamps_.size());
      }

    }
  }

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

  if ( frame_index != curpos_ ) {

    if ( frame_index >= header_.frames_count ) {
      CF_ERROR("Invalid seek %d >= FrameCount=%d",
          frame_index,
          header_.frames_count);

      errno = ESPIPE;
      return false;
    }

    const ssize_t seekpos =
        sizeof(header_) + (ssize_t) frame_index * frame_size();

    if ( lseek64(fd, seekpos, SEEK_SET) != seekpos ) {

      CF_ERROR("lseek64(seekpos=%zd, num_frames=%d) fails: %s",
          seekpos,
          header_.frames_count,
          strerror(errno));

      return false;
    }

    curpos_ = frame_index;
  }

  return true;
}

bool c_ser_reader::read(cv::Mat & image)
{
  if ( !is_open() ) {
    CF_ERROR("File is not opened");
    errno = ESPIPE;
    return false;
  }

  if ( curpos_ >= header_.frames_count ) {
    errno = EPIPE;
    return false;
  }

  errno = 0;

  if ( !image.empty() && !image.isContinuous() ) {
    image.release();
  }

  image.create(header_.image_height, header_.image_width, cvtype());

  const ssize_t savedpos =
      lseek64(fd, 0, SEEK_CUR);

  const ssize_t bytes_to_read =
      image.total() * image.elemSize();

  const ssize_t bytes_read =
      ::read(fd, image.data, bytes_to_read);

  if ( bytes_read != bytes_to_read ) {

    CF_ERROR("fread() fails: %s. bytes_to_read=%zd bytes_read=%zd curpos=%zd",
        strerror(errno),
        bytes_to_read,
        bytes_read,
        savedpos);

    lseek64(fd, savedpos, SEEK_SET);

    return false;
  }

  if ( bytes_per_pixel() == 2 && header_.is_little_endian != is_current_machine_little_endian() ) {

    swap_endianess(reinterpret_cast<uint16_t*>(image.data),
        image.total());

  }

  ++curpos_;

  return true;
}

c_ser_writer::~c_ser_writer()
{
  close();
}

bool c_ser_writer::is_open() const
{
  return fd >= 0;
}

bool c_ser_writer::create(const std::string & filename, int image_width, int image_height,
    enum COLORID color_id, int bits_per_plane)
{
  close();

  if ( image_width < 1 || image_height < 1 || bits_per_plane < 1 || bits_per_plane > 16 ) {

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
    CF_FATAL("Invalid ColorId specified: %d",
        color_id);
    errno = EINVAL;
    return false;
  }

  header_.color_id = color_id;
  header_.is_little_endian = !is_current_machine_little_endian();  // force incorrect endian
  header_.image_width = image_width;
  header_.image_height = image_height;
  header_.bits_per_plane = bits_per_plane;
  header_.frames_count = 0;
  header_.date_time = 0;
  header_.date_time_utc = 0;

  if ( (fd = ::open(filename.c_str(), O_CREAT | O_TRUNC | O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH)) == -1 ) {
    CF_FATAL("open('%s') fails: %s", filename.c_str(), strerror(errno));
    return false;
  }

  if ( ::write(fd, &header_, sizeof(header_)) != sizeof(header_) ) {
    CF_FATAL("write(SER HEADER) fails: %s", strerror(errno));
    ::close(fd);
    fd = -1;
    return false;
  }

  flush();

  return true;
}

bool c_ser_writer::close()
{
  bool fok = true;

  if ( fd >= 0 ) {
    lseek64(fd, 0, SEEK_SET);
    if ( ::write(fd, &header_, sizeof(header_)) != sizeof(header_) ) {
      CF_FATAL("write(SER HEADER) fails: %s", strerror(errno));
      fok = false;
    }
    ::close(fd);
    fd = -1;
  }
  return fok;
}

bool c_ser_writer::flush()
{
  if ( fd >= 0 ) {
    fdatasync(fd);
    return true;
  }
  return false;
}

bool c_ser_writer::write(cv::InputArray _image, uint64_t ts)
{
  if ( !is_open() ) {
    CF_ERROR("File is not opened");
    errno = EBADF;
    return false;
  }

  const cv::Mat image = _image.getMat();

  if ( image.cols != header_.image_width || image.rows != header_.image_height ) {
    CF_ERROR("invalid image specified: %dx%dx%d depth:5d expected: %dx%d%d depth:%d",
        image.cols, image.rows, image.channels(), image.depth(),
        header_.image_width, header_.image_height, channels(), cvdepth());
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

  const int64_t savedpos =
      lseek64(fd, 0, SEEK_CUR);

  const size_t bytes_to_write =
      image.total() * image.elemSize();

  const size_t bytes_written =
      ::write(fd, image.data, bytes_to_write);


  if ( bytes_written != bytes_to_write ) {

    CF_ERROR("write() fails: %s. "
        "bytes_to_write=%zu bytes_written=%zu",
        strerror(errno), bytes_to_write, bytes_written);

    lseek64(fd, savedpos, SEEK_SET);

    return false;
  }

  header_.frames_count++;

  // FIXME: update time stamps
  //frames_timestamps

  return true;
}

///////////////////////////////////////////////////////////////////////////////

