/*
 * c_ifhd_file.h
 *
 *  Created on: Oct 25, 2023
 *      Author: amyznikov
 *
 *  File Header struct is cloned from <https://github.com/cariad-tech/adtf_file>
 *  <indexedfile_types_v201_v301.h>
 *
 */

#pragma once
#ifndef __c_ifhd_file_h__
#define __c_ifhd_file_h__

#include <cstdint>
#include <string>
#include <vector>

namespace ifhd {

enum Endianess : uint8_t
{
  platform_not_supported = 0x00,
  platform_little_endian = 0x01,
  platform_big_endian = 0x02
};

#define MAX_FILEEXTENSIONIDENTIFIER_LENGTH 384
#define MAX_STREAMNAME_LENGTH 228

#pragma pack(push, 1)

struct FileHeader
{
  /// identifier for every dat-File. intel cpu implementations uses
  /// "IFHD" motorola uses "DHFI" see also \c headerByteOrder
  uint32_t file_id;
  /// format version of dat files, current version is
  /// IndexedFile::m_nVersionId
  /// or IndexedFile::m_nVersionIdWithHistory
  uint32_t version_id;
  /// flags ... not in use yet
  uint32_t flags;
  /// amount of extension blocks
  uint32_t extension_count;
  /// file-offset to the begin of extension table block (absolute)
  uint64_t extension_offset;
  /// file-offset to the begin of data block (absolute)
  uint64_t data_offset;
  /// size of the data area block (in bytes)
  uint64_t data_size;
  /// amount of chunks
  uint64_t chunk_count;
  /// greatest user data size of chunk
  uint64_t max_chunk_size;
  /// timestamp of the last chunk
  uint64_t duration;
  /// creation time of file
  uint64_t file_time;
  /// endianess of management structures (little or big endian).
  /// every single value within the structures
  /// (FileHeader, FileExtension, ChunkHeader ... ) are stored in
  /// this corresponding byteorder!
  uint8_t header_byte_order;
  /// time offset for every time within the file is referred to
  /// (timestamp zero)
  uint64_t time_offset;
  /// patch number (not in use yet)
  uint8_t patch_number;
  /// the file offset of the first chunk
  /// (only needed for IndexedFile::m_nVersionIdWithHistory)
  uint64_t first_chunk_offset;
  /// the offset of the first chunk of the continuous section of the file
  /// (only needed for IndexedFile::m_nVersionIdWithHistory)
  uint64_t continuous_offset;
  /// the end position withing the ring buffer section
  /// (only needed for IndexedFile::m_nVersionIdWithHistory)
  uint64_t ring_buffer_end_offset;
  /// reserved bytes. currently not in use
  int8_t reserved[30];
  /// common string description.
  /// This value is separated into a short and detailed description
  /// separated by '\n'.
  int8_t description[1912];
};
// size is 2048 Bytes

/**
 * \struct FileExtension
 * @brief Header for a file extensions.
 *
 */
struct FileExtension
{
  /// Identifier
  int8_t identifier[MAX_FILEEXTENSIONIDENTIFIER_LENGTH];
  /// related Stream identifier. 0 for every stream 1> id >= Max streams)
  uint16_t stream_id;
  /// reserved. currently not in use
  uint8_t reserved1[2];
  /// optional user id
  uint32_t user_id;
  /// optional type id
  uint32_t type_id;
  /// optional version id
  uint32_t version_id;
  /// file offset of the extension data (absolute),
  /// will be changed to int64_t in a future version
  uint64_t data_pos;
  /// size of the extension-data in bytes
  uint64_t data_size;
  /// reserved. currently not in use
  uint8_t reserved[96];
};
// size is 512 Bytes

/**
 * \struct ChunkHeader
 * @brief header for chunks
 * Each Chunk header 16 Byte aligned within the file.
 */
struct ChunkHeader
{
  /// timestamp of the chunk @see tTimestamp
  uint64_t time_stamp;
  /// referring to the master index table
  uint32_t ref_master_table_index;
  /// relative byte offset to the previous chunk header (in bytes)
  /// @remark this is NOT necessarily the size of the previous chunk.
  /// Chunk headers are 16 Byte aligned!
  uint32_t offset_to_last;
  /// size of the chunks (in bytes)
  /// @remark This value includes the size of the chunk data AND the
  /// chunk header size!
  uint32_t size;
  /// stream identifier the chunk belongs to
  uint16_t stream_id;
  /// key data / flags
  /// see also IndexedFile::tChunkType
  uint16_t flags;
  /// number of the chunk within stream it belongs to
  uint64_t stream_index;
};
// size is 32 Bytes

/**
 * \struct chunkRef
 * @brief header for a chunk reference
 */
struct ChunkRef
{
  /// timestamp of the chunk it refers to @see tTimestamp
  uint64_t time_stamp;
  /// size of the chunk it refers to
  /// @remark This value includes the size of the Chunk Data size
  /// \b AND the Chunk Header size!
  uint32_t size;
  /// stream identifier of the chunk it refers to
  uint16_t stream_id;
  /// key data / flags of the chunk it refers to
  /// see also IIndexFile::tChunkType
  uint16_t flags;
  /// file offset position of the chunk it refers to (in byte)
  uint64_t chunk_offset;
  /// number of chunk
  uint64_t chunk_index;
  /// number of chunk within the stream it belongs to
  uint64_t stream_index;
  /**
   * number of stream index table entry this master index entry belongs to
   */
  uint32_t ref_stream_table_index;
};
// size is 44 Bytes

/**
 * \struct StreamRef
 * header for a stream reference elements
 */
struct StreamRef
{
  /**
   * number of master index entry it belongs to
   */
  uint32_t ref_master_table_index;
};
// size is 4 Bytes

/**
 * \struct StreamInfoHeader
 *  Stream info header
 */
struct StreamInfoHeader
{
  /// Amount of stream indexes
  uint64_t stream_index_count;
  /// First timestamp of stream
  uint64_t stream_first_time;
  /// Last timestamp of stream
  uint64_t stream_last_time;
  /// Info data size
  uint32_t info_data_size;
  /// Stream name
  int8_t stream_name[MAX_STREAMNAME_LENGTH]; //use Ascii 7
};
// size is 256 Byte

/// Additional index table information
struct AdditionalIndexInfo
{
  /// stream index count offset (the offset is > 0 if data was dropped
  /// within the ring buffer while recording)
  uint64_t stream_index_offset;
  /// Offset of the Index table entry position within master table
  /// (dropped indextable entries while recording with history)
  uint32_t stream_table_index_offset;
  /// for later use
  uint8_t reserved[20];
};

#pragma pack(pop)

} // namespace ifhd

class c_ifhd_file
{
public:
  typedef c_ifhd_file this_class;

  c_ifhd_file();
  c_ifhd_file(const std::string & filename);
  const std::string& filename() const;

protected:
  std::string filename_;
};

class c_ifhd_reader :
    public c_ifhd_file
{
public:
  typedef c_ifhd_reader this_class;
  typedef c_ifhd_file base;
  using FileHeader = ifhd::FileHeader;
  using FileExtension = ifhd::FileExtension;
  using StreamInfoHeader = ifhd::StreamInfoHeader;

  c_ifhd_reader();
  c_ifhd_reader(const std::string & filename);
  ~c_ifhd_reader();

  bool open(const std::string & filename = "");
  bool is_open() const;
  void close();

  bool set_stream(const std::string& stream_name);
  bool seek(int32_t frame_index);
  int32_t curpos() const;
  ssize_t current_chunk_size();
  bool read_current_chunk(void * data);

  /// @brief get number of frames in this stream
  ssize_t num_frames() const;

protected:
  int fd_ = -1;
  FileHeader file_header_;
  std::vector<FileExtension> file_exstensions_;
  int32_t current_stream_id_ = -1;
  int64_t current_chunk_offset_ = 1;
  int32_t curpos_ = -1;
};

#endif /* __c_ifhd_file_h__ */
