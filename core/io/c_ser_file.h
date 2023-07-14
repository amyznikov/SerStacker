/*
 * c_ser_file.h
 *
 *  Created on: December 1, 2020
 *      Author: amyznikov
 *
 * SER file read / write utility class.
 *
 * Based on SER_Doc_V3b.pdf
 *   "SER format description version 3",
 *     by Heiko Wilkens and Grischa Hahn,
 *      2014 Feb 06
 */

#ifndef __c_ser_file_h__
#define __c_ser_file_h__

#include "debayer.h" // for COLORID

/// @brief c_ser_file
/// Base class for SER reader and writer
class c_ser_file
{
public:

  /// @brief file_header
  /// SER file header as described in "SER format description version 3",
  /// by Heiko Wilkens and Grischa Hahn
  /// at 2014 Feb 06
  #pragma pack(push,1)
  struct file_header {
      char file_id[14] = {'L', 'U', 'C', 'A', 'M', '-', 'R','E','C','O','R','D','E','R'};
      int32_t luid = 0;
      enum COLORID color_id = COLORID_MONO;
      int32_t is_little_endian = 0; // long story...
      int32_t image_width = 0;
      int32_t image_height = 0;
      int32_t bits_per_plane = 0;
      int32_t frames_count = 0;
      char observer[40] = {};
      char instrument[40] = {};
      char telescope[40] = {};
      uint64_t date_time = 0;
      uint64_t date_time_utc = 0;
  };
  #pragma pack(pop)

  /// @brief Read-Only access to file header directly
  const file_header & header() const;

  /// @brief Read-Only access to file header.image_width field
  int image_width() const;

  /// @brief Read-Only access to file header.image_height field
  int image_height() const;

  /// @brief Read-Only access to file header.bits_per_plane field
  int bits_per_plane() const;

  /// @brief Get number of bytes required to store single pixel plane
  /// Returns 1 for bits_per_plane <= 8, else 2
  int bytes_per_plane() const;

  /// @brief Read-Only access to file header.color_id field
  enum COLORID color_id() const;

  /// @brief Get number of color channels for images basing on file_header.color_id field.
  /// Returns 3 for COLORID_RGB and COLORID_BGR, and 1 for others
  int channels() const;

  /// @brief Return bytes per pixel basing on file_header color_id and pixel_depth_per_plane.
  /// Computed as channels() * bytes_per_plane()
  int bytes_per_pixel() const;

  /// @brief Return CV_DEPTH (CV_8U or CV_16U)
  /// appropriate for given bytes_per_plane
  int cvdepth() const;

  /// @brief Return CV_MAKETYPE(cvdepth(), channels())
  //  appropriate for given bytes_per_plane() and channels()
  int cvtype() const;

  /// @brief Read-Only access to file header.observer field
  const char * observer() const;

  /// @brief Read-Only access to file header.instrument field
  const char * instrument() const;

  /// @brief Read-Only access to file header.telescope field
  const char * telescope() const;

  /// @brief get frame size in bytes based on image width x height x bytes_per_pixel
  int frame_size() const;

  /// @brief Read-Only access to file header.frames_count field
  int num_frames() const;

  /// @brief Read-Only access to frame timestamps array
  const std::vector<uint64_t> & timestamps() const;

  /// @brief Read-Only access to frame timestamps array item by index
  uint64_t timestamps(int index) const;


protected:
  /// @brief use one from derived classes to instantiate SER reader or writer
  c_ser_file();

  c_ser_file::file_header header_;
  std::vector<uint64_t> timestamps_;

protected:
  static_assert(sizeof(enum COLORID) == sizeof(int32_t),
      "enum COLORID must have size 32 bits");
  static_assert(sizeof(header_) == 178,
      "APP BUG: Wrong SER Header size");
};


/// @brief c_ser_reader
/// SER file reader into OpenCV cv::Mat image.
/// No debayer/rgb swap is made, to get bayer pattern use c_ser_file::is_bayer_pattern() and c_ser_file::color_id(),
/// and use OpenCV or external routine calls for debayer
class c_ser_reader
    : public c_ser_file
{
public:
  c_ser_reader() = default;
  c_ser_reader(const std::string & filename);
  ~c_ser_reader();

  /// @brief open()
  /// Open existing SER file read-only,
  /// read timestamps if available,
  //  set current position to 0
  bool open(const std::string & filename);

  /// @brief Return true if file is opened
  bool is_open() const;

  /// @brief read()
  /// Read next frame from current read position,
  /// advance current position to next frame
  bool read(cv::Mat & image);

  /// @brief curpos()
  //  Return current frame read position
  int32_t curpos() const;

  /// @brief seek()
  /// Try to set current frame read position to specified value
  bool seek(int32_t frame_index);

  /// @brief close()
  /// Close SER reader,
  /// release resources
  bool close();

protected:
  int fd = -1;  // file descriptopr
  int32_t curpos_ = -1; // current frame read position counter
};



/// @brief c_ser_writer
/// SER file writer for OpenCV cv::Mat images.
/// No automatic bayer/debayer/rgb swap is made,
/// the input image format must match to color_id, dimensions and format
//  specified for c_ser_writer::create()
class c_ser_writer :
    public c_ser_file
{
public:
  ~c_ser_writer();

  /// @brief create()
  /// Create new empty SER file with specified frame dimension and pixel format
  bool create(const std::string & filename, int image_width, int image_height,
      enum COLORID color_id, int bits_per_plane);

  /// @brief is_open()
  /// Return true if file still open and next frame can be written into
  bool is_open() const;

  /// @brief write()
  /// Write OpenCV frame into SER file.
  /// Only basic checks are make: image dimension and byte size.
  bool write(cv::InputArray image, uint64_t ts = UINT64_MAX);

  /// @brief flush()
  /// Flush IO buffers immediatelly
  bool flush();

  /// @brief close()
  /// Close SER file
  bool close();

protected:
  int fd = -1;  // file descriptopr
};

#endif /* __c_ser_file_h__ */
