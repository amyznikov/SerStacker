/*
 * c_vlo_file.h
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_file_h__
#define __c_vlo_file_h__

#include <unistd.h>
#include <inttypes.h>
#include <string>
#include <memory>

#define HAVE_VLO_FILE 0

constexpr int VLO_SCAN_LAYERS = 360;
constexpr int VLO_SCAN_ECHOS = 3;
constexpr int VLO_SCAN_SLOTS = 100 * 120 / 20;
constexpr int VLO_SCAN_POINTS = VLO_SCAN_SLOTS * VLO_SCAN_LAYERS * VLO_SCAN_ECHOS;
constexpr int VLO_SCAN_REFERENCE_SHOTS = 2;
constexpr int VLO_SCAN_FACE_VERSION = 5;

enum VLO_VERSION {
  VLO_VERSION_UNKNOWN = -1,
  VLO_VERSION_1 = 1,
  VLO_VERSION_3 = 3,
  VLO_VERSION_5 = 5,
  VLO_VERSION_18 = 18,
};

#pragma pack(push, 1)
struct c_vlo_timestamp
{
  uint32_t nanoseconds;
  uint32_t seconds;
  uint32_t secondsHi;
};
#pragma pack(pop)


#pragma pack(push, 1)
struct c_vlo_echo
{
  uint16_t distCm;
  uint16_t area;
  uint8_t peak;
  uint8_t width;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct c_vlo_slot
{
  uint16_t slotIdx;
  uint16_t padding[3];
  int32_t angleTicks;
  c_vlo_echo echo[VLO_SCAN_LAYERS][VLO_SCAN_ECHOS];
  uint16_t ambient[VLO_SCAN_LAYERS];
  uint8_t numTotalEchoes[VLO_SCAN_LAYERS];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct c_vlo_scan_config
{
  uint16_t numSlots;
  uint16_t numLayersPerSlot;
  uint8_t numEchos;
  uint8_t echoStructSizeInBit;
  uint8_t echoOrdering;
  uint8_t numLasershotsPerSlot;
  uint16_t macroPixelWidth;
  uint16_t macroPixelHeight;
  uint16_t LUT_FS_X[16];
  uint16_t LUT_FS_Y[16];
  uint8_t LUT_ACCU_X[32];
  uint8_t LUT_ACCU_Y[32];
  uint16_t Rx_G_THACCU_0[8];
  uint16_t laserVoltageSetValue;
  uint16_t shotTiming_x;
  uint16_t shotTiming_y;
  uint16_t shotTiming_z;
  uint32_t shotTiming_frequency;
  uint16_t shotTiming_sh;
  uint8_t shotTiming_numLoadingPulses;
  uint8_t shotTiming_dutyCycles;
  uint8_t R_DISTANCE_SWITCH;
  uint8_t R_FAR_REGION_RULE;
  uint8_t R_PRIO_N;
  uint8_t R_BIN_R;
  uint16_t Rx_BIN_A_0[8];
  uint16_t Rx_BIN_B_0[8];
  uint8_t binWidthInCm;
  int8_t temperatureReceiverChip;
  int8_t temperatureLaser;
  uint8_t imagerType;
};
#pragma pack(pop)


#pragma pack(push, 1)
struct c_vlo_scan
{
  uint16_t interfaceVersion;
  uint16_t scanNumber;
  c_vlo_timestamp timeStamp;
  uint32_t scanDuration_ns;
  uint16_t mirrorSide;
  uint16_t paddingBytes;
  uint16_t startSlotIdxLowRes;
  uint16_t endSlotIdxLowRes;
  uint16_t startSlotIdxHighRes;
  uint16_t endSlotIdxHighRes;
  c_vlo_scan_config config;
  uint8_t reserveBeforeSlotData[20];
  int32_t verticalAngles[VLO_SCAN_LAYERS];
  c_vlo_slot slot[VLO_SCAN_SLOTS];
  c_vlo_slot referenceShot[VLO_SCAN_REFERENCE_SHOTS];
  uint16_t errorState;
  uint16_t laserVoltageActualMin;
  uint16_t laserVoltageActualMax;
  uint16_t laserVoltageActualAvg;
  uint8_t reserveAfterSlotData[20];
  uint32_t crc;
};
#pragma pack(pop)

////////////

struct c_vlo_scan3
{
    static const uint16_t INTERFACE_VERSION = 3U;
    static const uint16_t NUM_SLOTS = 600U;
    static const uint16_t NUM_LAYERS = 360U;
    static const uint16_t NUM_ECHOS = 3U;
    static const uint32_t NUM_POINTS  = (NUM_SLOTS*NUM_LAYERS)*NUM_ECHOS;

    struct Config
    {
        uint16_t numSlots;
        uint16_t numLayersPerSlot;
        uint8_t numEchos;
        uint8_t paddingConfig;
        uint16_t macroPixelWidth;
        uint16_t macroPixelHeight;
        uint8_t numLasershotsPerSlot;
        uint8_t echoOrdering;
        uint16_t LUT_FS_X[16U];
        uint16_t LUT_FS_Y[16U];
        uint8_t LUT_ACCU_X[32U];
        uint8_t LUT_ACCU_Y[32U];
        uint16_t Rx_G_THACCU_0[8U];
        uint16_t laserVoltageSetValue;
        uint16_t shotTiming_x;
        uint16_t shotTiming_y;
        uint16_t shotTiming_z;
        uint32_t shotTiming_frequency;
        uint8_t shotTiming_numLoadingPulses;
        uint8_t shotTiming_dutyCycles;
        uint16_t shotTiming_sh;
    };

    struct Echo
    {
        uint16_t radDistCm;
        uint16_t width;
        uint16_t area;
        uint16_t peak;
    };

    struct Slot
    {
        uint16_t slotIdx;
        uint16_t paddingBytes;
        int32_t angleTicks;
        Echo echo[NUM_LAYERS][NUM_ECHOS];
        uint16_t ambientLight[NUM_LAYERS];
    };

    uint16_t interfaceVersion;
    uint16_t scanNumber;
    uint32_t timeStampNanoSeconds;
    uint32_t timeStampSeconds;
    uint32_t timeStampSecondsHi;
    uint16_t mirrorSide;
    uint16_t paddingBytes;
    uint16_t startSlotIdxLowRes;
    uint16_t endSlotIdxLowRes;
    uint16_t startSlotIdxHighRes;
    uint16_t endSlotIdxHighRes;
    Config config;
    int32_t verticalAngles[NUM_LAYERS];
    uint8_t reserveBeforeSlotData[112];
    Slot slot[NUM_SLOTS];
    uint16_t errorState;
    uint16_t laserVoltageActualMin;
    uint16_t laserVoltageActualMax;
    uint16_t laserVoltageActualAvg;
    int32_t startAngleTicksLowRes;
    int32_t endAngleTicksLowRes;
    int32_t startAngleTicksHighRes;
    int32_t endAngleTicksHighRes;
    uint8_t reserveAfterSlotData[124];
    uint32_t crc;
};
/////////////

class c_vlo_file
{
public:
  typedef c_vlo_file this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_file()
  {
  }

  c_vlo_file(const std::string & filename) :
    filename_(filename)
  {
  }

  const std::string & filename() const
  {
    return filename_;
  }

  VLO_VERSION version() const
  {
    return version_;
  }

protected:
  std::string filename_;
  VLO_VERSION version_ = VLO_VERSION_UNKNOWN;
};

class c_vlo_reader :
    public c_vlo_file
{
public:
  typedef c_vlo_reader this_class;
  typedef c_vlo_file base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_reader();
  c_vlo_reader(const std::string & filename);

  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;
  bool seek(int32_t frame_index);
  int32_t curpos() const;
  bool read(c_vlo_scan * scan);

  /// @brief get total file size in bytes
  ssize_t file_size() const;

  /// @brief get frame size in bytes
  ssize_t frame_size() const;

  /// @brief get number of framesin this file
  ssize_t num_frames() const;


protected:
  ssize_t file_size_ = -1;
  ssize_t num_frames_ = -1;
  int fd_ = -1;

};


#endif /* __c_vlo_file_h__ */
