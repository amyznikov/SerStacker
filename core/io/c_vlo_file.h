/*
 * c_vlo_file.h
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_file_h__
#define __c_vlo_file_h__

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <inttypes.h>
#include <string>
#include <memory>

#define HAVE_VLO_FILE 1

enum VLO_VERSION
{
  VLO_VERSION_UNKNOWN = -1,
  VLO_VERSION_1 = 1,
  VLO_VERSION_3 = 3,
  VLO_VERSION_5 = 5,
  VLO_VERSION_18 = 18,
};

enum VLO_IMAGER_TYPE
{
  VLO_IMAGER_IMX449 = 0,
  VLO_IMAGER_IMX479 = 1,
  VLO_IMAGER_SNA = 255
};


#pragma pack(push, 1)
struct c_vlo_scan1
{
  static constexpr uint16_t INTERFACE_VERSION = 1U;
  static constexpr uint16_t NUM_SLOTS = 380U;
  static constexpr uint16_t NUM_LAYERS = 360U;
  static constexpr uint16_t NUM_ECHOS = 3U;
  static constexpr uint32_t NUM_POINTS = (NUM_SLOTS * NUM_LAYERS) * NUM_ECHOS;

  struct c_vlo_scan_config
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

  struct c_vlo_echo
  {
    uint16_t distCm;
    uint16_t width;
    uint16_t area;
    uint16_t peak;
  };

  struct c_vlo_slot
  {
    uint16_t slotIdx;
    uint16_t paddingBytes;
    int32_t angleTicks;
    c_vlo_echo echo[NUM_LAYERS][NUM_ECHOS];
    uint16_t ambient[NUM_LAYERS];
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
  c_vlo_scan_config config;
  int32_t verticalAngles[NUM_LAYERS];
  uint8_t reserveBeforeSlotData[112];
  c_vlo_slot slot[NUM_SLOTS];
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
#pragma pack(pop)

#pragma pack(push, 1)
struct c_vlo_scan3
{
  static constexpr uint16_t INTERFACE_VERSION = 3U;
  static constexpr uint16_t NUM_SLOTS = 600U;
  static constexpr uint16_t NUM_LAYERS = 360U;
  static constexpr uint16_t NUM_ECHOS = 3U;
  static constexpr uint32_t NUM_POINTS = (NUM_SLOTS * NUM_LAYERS) * NUM_ECHOS;

  struct c_vlo_scan_config
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

  struct c_vlo_echo
  {
    uint16_t distCm;
    uint16_t width;
    uint16_t area;
    uint16_t peak;
  };

  struct c_vlo_slot
  {
    uint16_t slotIdx;
    uint16_t paddingBytes;
    int32_t angleTicks;
    c_vlo_echo echo[NUM_LAYERS][NUM_ECHOS];
    uint16_t ambient[NUM_LAYERS];
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
  c_vlo_scan_config config;
  int32_t verticalAngles[NUM_LAYERS];
  uint8_t reserveBeforeSlotData[112];
  c_vlo_slot slot[NUM_SLOTS];
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
#pragma pack(pop)

#pragma pack(push, 1)
struct c_vlo_scan
{
  static constexpr int INTERFACE_VERSION = 5;
  static constexpr int NUM_LAYERS = 360;
  static constexpr int NUM_SLOTS = 100 * 120 / 20;
  static constexpr int NUM_ECHOS = 3;
  static constexpr int NUM_POINTS = NUM_SLOTS * NUM_LAYERS * NUM_ECHOS;
  static constexpr int NUM_REFERENCE_SHOTS = 2;

  struct c_vlo_timestamp
  {
    uint32_t nanoseconds;
    uint32_t seconds;
    uint32_t secondsHi;
  };

  struct c_vlo_echo
  {
    uint16_t distCm;
    uint16_t area;
    uint8_t peak;
    uint8_t width;
  };

  struct c_vlo_slot
  {
    uint16_t slotIdx;
    uint16_t padding[3];
    int32_t angleTicks;
    c_vlo_echo echo[NUM_LAYERS][NUM_ECHOS];
    uint16_t ambient[NUM_LAYERS];
    uint8_t numTotalEchoes[NUM_LAYERS];
  };

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
  int32_t verticalAngles[NUM_LAYERS];
  c_vlo_slot slot[NUM_SLOTS];
  c_vlo_slot referenceShot[NUM_REFERENCE_SHOTS];
  uint16_t errorState;
  uint16_t laserVoltageActualMin;
  uint16_t laserVoltageActualMax;
  uint16_t laserVoltageActualAvg;
  uint8_t reserveAfterSlotData[20];
  uint32_t crc;
};
#pragma pack(pop)

static_assert(sizeof(c_vlo_scan) == (36480640 / 8));

////////////


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
  bool read(c_vlo_scan1 * scan);
  bool read(c_vlo_scan3 * scan);
  bool read_ambient(cv::Mat * image);

  /// @brief get total file size in bytes
  ssize_t file_size() const;

  /// @brief get frame size in bytes
  ssize_t frame_size() const;

  /// @brief get number of framesin this file
  ssize_t num_frames() const;


  static cv::Mat1w get_ambient_image(const c_vlo_scan1 & scan);
  static cv::Mat1w get_ambient_image(const c_vlo_scan3 & scan);
  static cv::Mat1w get_ambient_image(const c_vlo_scan & scan);

  static cv::Mat get_thumbnail_image(const std::string & filename);

protected:
  ssize_t file_size_ = -1;
  ssize_t num_frames_ = -1;
  ssize_t frame_size_ = -1;
  int fd_ = -1;
};



#endif /* __c_vlo_file_h__ */
