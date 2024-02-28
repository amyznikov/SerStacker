/*
 * c_hdl_packet_parser.h
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#ifndef __c_hdl_packet_parser_h__
#define __c_hdl_packet_parser_h__

#include "c_hdl_frame.h"
#include "c_hdl_specification.h"

#if HAVE_PCAP

#include <inttypes.h>
#include <string>
#include <vector>
#include <memory>

constexpr int HDL_DATA_BLOCKS_PER_PKT = 12;
constexpr int HDL_LASERS_PER_DATA_BLOCK = 32;



/* in host byte order */
enum HDLLaserBlockID
{
  HDL_BLOCK_00_31 = 0xEEFF,
  HDL_BLOCK_32_63 = 0xDDFF,
  HDL_BLOCK_64_95 = 0xCCFF,
  HDL_BLOCK_96_127 = 0xBBFF,
};


//
//enum HDL_ExtDataPacketMode
//{
//  HDL_EXT_MODE_NONE = -1,
//  HDL_EXT_MODE_TRIPLE_RETURN = 0,
//  HDL_EXT_MODE_CONFIDENCE = 1,
//};
//
//enum HDL_PowerMode
//{
//  HDL_CorrectionOff = 0,
//  HDL_NO_INTERNAL_CORRECTION_0 = 0xa0,
//  HDL_NO_INTERNAL_CORRECTION_1 = 0xa1,
//  HDL_NO_INTERNAL_CORRECTION_2 = 0xa2,
//  HDL_NO_INTERNAL_CORRECTION_3 = 0xa3,
//  HDL_NO_INTERNAL_CORRECTION_4 = 0xa4,
//  HDL_NO_INTERNAL_CORRECTION_5 = 0xa5,
//  HDL_NO_INTERNAL_CORRECTION_6 = 0xa6,
//  HDL_NO_INTERNAL_CORRECTION_7 = 0xa7,
//  HDL_CorrectionOn = 0xa8,
//};
//
//enum HDL_DualFlag
//{
//  HDL_DUAL_DISTANCE_NEAR = 0x1,  // point with lesser distance
//  HDL_DUAL_DISTANCE_FAR = 0x2,   // point with greater distance
//  HDL_DUAL_INTENSITY_HIGH = 0x4, // point with lesser intensity
//  HDL_DUAL_INTENSITY_LOW = 0x8,  // point with greater intensity
//  HDL_DUAL_DOUBLED = 0xf,        // point is single return
//  HDL_DUAL_DISTANCE_MASK = 0x3,
//  HDL_DUAL_INTENSITY_MASK = 0xc,
//};

#pragma pack(push, 1)

struct HDLLaserReturn
{
  uint16_t distance;
  uint8_t intensity;
};

struct HDLDataBlock
{
  uint16_t blockId;
  uint16_t azimuth;
  HDLLaserReturn laserReturns[HDL_LASERS_PER_DATA_BLOCK];
};

struct HDLDataPacket
{
  HDLDataBlock dataBlocks[HDL_DATA_BLOCKS_PER_PKT];
  uint32_t TohTimestamp;
  uint8_t factoryBytes[2];
};

#pragma pack(pop)


class c_hdl_packet_parser
{
public:

  typedef std::function<void(const c_hdl_packet_parser &, const c_hdl_frame::sptr & frame)>
    on_frame_callback;

  struct State
  {
    HDLReturnMode return_mode_ = HDLReturnMode_unknown;
    HDLFramingMode hdl_framing_mode_ = HDLFraming_Rotation;
    int start_block = 0;
    int last_known_azimuth_ = 0;
    int pktcounter_ = 0;
    double hdl_frame_seam_azimuth_ = 0;
    bool sensor_changed_ = false;
    uint32_t previousTohTimestamp_ = 0;
    uint32_t hours_counter_ = 0;
    size_t frame_counter_ = 0;
  };


  bool parse(const uint8_t * data, uint size, int start_block = 0);
  void reset();
  void clear(const struct State * state = nullptr);

  const State & state() const;

  void set_frame_created_callback(const on_frame_callback & cb)
  {
    frame_created_callback_ = cb;
  }

  void set_frame_created_callback(on_frame_callback && cb)
  {
    frame_created_callback_ = std::move(cb);
  }

  const on_frame_callback & frame_created_callback() const
  {
    return frame_created_callback_;
  }

  void set_frame_populated_callback(const on_frame_callback & cb)
  {
    frame_populated_callback_ = cb;
  }

  void set_frame_populated_callback(on_frame_callback && cb)
  {
    frame_populated_callback_ = std::move(cb);
  }

  const on_frame_callback & frame_populated_callback() const
  {
    return frame_populated_callback_;
  }

  bool sensor_changed() const;


  void set_hdl_framing_mode(enum HDLFramingMode v);
  enum HDLFramingMode hdl_framing_mode() const;

  void set_hdl_frame_seam_azimuth(double azimuth_in_degrees);
  double hdl_frame_seam_azimuth() const;

  void set_lidar_config_xml(const std::string & v);
  const std::string & lidar_config_xml() const;

  void set_only_extract_frame_seams(bool v);
  bool only_extract_frame_seams() const;

  HDLSensorType sensor_type() const;
  HDLReturnMode return_mode() const;
  const c_hdl_specification * lidar_specification() const;


  int last_known_azimuth() const;
  void set_last_known_azimuth(int );

  int pktcounter() const;
  void set_pktcounter(int );

  uint32_t previousTohTimestamp() const;
  void set_previousTohTimestamp(uint32_t );

  uint32_t hours_counter() const;
  void set_hours_counter(uint32_t );

  size_t frame_counter() const;
  void set_frame_counter(size_t );

  const c_hdl_frame::sptr & current_frame() const;

  // std::vector<c_hdl_frame::sptr> frames;

protected:
  bool setup(HDLSensorType sensor_type, HDLReturnMode return_mode);
  bool precompute_correction_tables();
  bool parse_vlp16(const HDLDataPacket *dataPacket, int start_block = 0);
  bool parse_vlp32(const HDLDataPacket *dataPacket, int start_block = 0);
  bool parse_hdl32(const HDLDataPacket *dataPacket, int start_block = 0);
  bool parse_hdl64(const HDLDataPacket *dataPacket, int start_block = 0);
  bool parse_vls128(const HDLDataPacket *dataPacket, int start_block = 0);
  bool is_hdl_frame_seam(int current_packet_azimuth, int previous_packet_azimuth) const;
  void set_sensor_changed(bool v);
  //c_hdl_frame * new_hdl_frame(int block);
  void new_current_frame(int block);
  void on_frame_creaated(const c_hdl_frame::sptr & f);
  void on_frame_populated(const c_hdl_frame::sptr & f);

protected:
  struct State state_;
  bool only_extract_frame_seams_ = false;

  //bool unexpected_sensor_change_reported_ = false;

  c_hdl_frame::sptr current_frame_;

  on_frame_callback frame_created_callback_;
  on_frame_callback frame_populated_callback_;

  // current lidar specification table
  std::string lidar_config_xml_;
  c_hdl_specification lidar_specification_;

  struct c_lasers_corrections_table {
    double sin_rot_correction;
    double cos_rot_correction;
    double sin_vert_correction;
    double cos_vert_correction;
  };

  // precomputed coordinate corrections table
  std::vector<c_lasers_corrections_table> precomuted_corrections_table_;

  // precomputed timing offset lookup table
  std::vector< std::vector<float> > precomputed_timing_offsets_;

  // Caches the azimuth percent offset for the VLS-128 laser firings
  double vls_128_laser_azimuth_cache[16];
};



/** Data-Packet Specifications says that laser-packets are 1206 byte long.
 *  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
 *                #lasers^   ^#firingPerPkt
 **/
inline constexpr uint hdl_lidar_packet_size()
{
  return 1206;
}

/**
 * get_sensor_type()
 *
 *  For HDL64 check the sequence of block identifiers, see HDL-64E_S3.pdf Appendix E: Data Packet Format.
 *  For the rest use the factoryField2, see VLP32CManual.pdf Table 9-1 Factory Byte Values.
 */
inline HDLSensorType get_sensor_type(const HDLDataPacket & packet)
{
  const int blockid1 = packet.dataBlocks[1].blockId;
  const int blockid2 = packet.dataBlocks[2].blockId;
  if( blockid1 == HDL_BLOCK_32_63 && blockid2 == HDL_BLOCK_00_31 ) {
    return HDLSensor_HDL64;
  }
  return static_cast<HDLSensorType>(packet.factoryBytes[1]);
}

/**
 * get_return_mode()
 *
 *  For HDL64 check the sequence of block identifiers, see HDL-64E_S3.pdf Appendix E: Data Packet Format.
 *  For the rest use the factoryField1, see VLP32CManual.pdf Table 9-1 Factory Byte Values.
 */
inline HDLReturnMode get_return_mode(const HDLDataPacket & packet)
{
  const HDLSensorType sensor_type =
      get_sensor_type(packet);

  if( sensor_type == HDLSensor_HDL64 ) {
    return (packet.dataBlocks[2].azimuth == packet.dataBlocks[0].azimuth) ?
        HDL_DUAL_RETURN : HDL_STRONGEST_RETURN;
  }

  return static_cast<HDLReturnMode>(packet.factoryBytes[0]);
}


/**
 * is_single_return_mode()
 */
inline bool is_single_return_mode(HDLReturnMode return_mode)
{
  switch (return_mode) {
  case HDL_STRONGEST_RETURN:
    case HDL_LAST_RETURN:
    return true;
  }
  return false;
}

#endif // HAVE_PCAP


#endif /* __c_hdl_packet_parser_h__ */
