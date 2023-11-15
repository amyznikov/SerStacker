/*
 * c_vlo_file.h
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_file_h__
#define __c_vlo_file_h__

#include "c_ifhd_file.h"
#include <opencv2/opencv.hpp>
#include <type_traits>
#include <memory>

#define HAVE_VLO_FILE 1

enum VLO_VERSION
{
  VLO_VERSION_UNKNOWN = -1,
  VLO_VERSION_1 = 1,
  VLO_VERSION_3 = 3,
  VLO_VERSION_5 = 5,

  VLO_VERSION_6_IMX449 = 600,
  VLO_VERSION_6_IMX479 = 601,
  VLO_VERSION_6_SLM = 602,
};

enum VLO_IMAGER_TYPE
{
  VLO_IMAGER_IMX449 = 0,
  VLO_IMAGER_IMX479 = 1,
  VLO_IMAGER_SNA = 255,
  VLO_IMAGER_SMART = VLO_IMAGER_IMX449,
  VLO_IMAGER_SATELLITE = VLO_IMAGER_IMX479,
};

enum VLO_ECHO_ORDER
{
  VLO_ECHO_ORDER_UNKNOWN = 0,
  VLO_ECHO_ORDER_INTENSITY_HIGH_TO_LOW = 1,
  VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR = 2,
  VLO_ECHO_ORDER_DISTANCE_FAR_TO_NEAR_IGNORE_CLOSEST = 3,
  VLO_ECHO_ORDER_DISTANCE_FAR_TO_NEAR_IGNORE_2ND_CLOSEST = 4,
  VLO_ECHO_ORDER_DISTANCE_FAR_TO_NEAR_IGNORE_3RD_CLOSEST = 5,
  VLO_ECHO_ORDER_DISTANCE_FAR_TO_NEAR_IGNORE_FARTHEST = 6
};

#pragma pack(push, 1)
struct c_vlo_scan1
{
  static constexpr uint16_t INTERFACE_VERSION = 1U;
  static constexpr uint16_t NUM_SLOTS = 380U;
  static constexpr uint16_t NUM_LAYERS = 360U;
  static constexpr uint16_t NUM_ECHOS = 3U;
  static constexpr uint32_t NUM_POINTS = (NUM_SLOTS * NUM_LAYERS) * NUM_ECHOS;

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
    uint16_t dist;
    uint16_t width;
    uint16_t area;
    uint16_t peak;
  };

  struct Slot
  {
    uint16_t slotIdx;
    uint16_t paddingBytes;
    int32_t angleTicks;
    struct Echo echo[NUM_LAYERS][NUM_ECHOS];
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
  struct Config config;
  int32_t verticalAngles[NUM_LAYERS];
  uint8_t reserveBeforeSlotData[112];
  struct Slot slot[NUM_SLOTS];
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
  static constexpr uint16_t VERSION = 3U;
  static constexpr uint16_t NUM_SLOTS = 600U;
  static constexpr uint16_t NUM_LAYERS = 360U;
  static constexpr uint16_t NUM_ECHOS = 3U;
  static constexpr uint32_t NUM_POINTS = (NUM_SLOTS * NUM_LAYERS) * NUM_ECHOS;

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
    uint16_t dist;
    uint16_t width;
    uint16_t area;
    uint16_t peak;
  };

  struct Slot
  {
    uint16_t slotIdx;
    uint16_t paddingBytes;
    int32_t angleTicks;
    struct Echo echo[NUM_LAYERS][NUM_ECHOS];
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
  struct Config config;
  int32_t verticalAngles[NUM_LAYERS];
  uint8_t reserveBeforeSlotData[112];
  struct Slot slot[NUM_SLOTS];
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
struct c_vlo_scan5
{
  static constexpr int VERSION = 5;
  static constexpr int NUM_LAYERS = 360;
  static constexpr int NUM_SLOTS = 100 * 120 / 20;
  static constexpr int NUM_ECHOS = 3;
  static constexpr int NUM_POINTS = NUM_SLOTS * NUM_LAYERS * NUM_ECHOS;
  static constexpr int NUM_REFERENCE_SHOTS = 2;

  struct timestamp
  {
    uint32_t nanoseconds;
    uint32_t seconds;
    uint32_t secondsHi;
  };

  struct Config
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

  struct Echo
  {
    uint16_t dist;
    uint16_t area;
    uint8_t peak;
    uint8_t width;
  };

  struct Slot
  {
    uint16_t slotIdx;
    uint16_t padding[3];
    int32_t angleTicks;
    struct Echo echo[NUM_LAYERS][NUM_ECHOS];
    uint16_t ambient[NUM_LAYERS];
    uint8_t numTotalEchoes[NUM_LAYERS];
  };

  uint16_t interfaceVersion;
  uint16_t scanNumber;
  struct timestamp timeStamp;
  uint32_t scanDuration_ns;
  uint16_t mirrorSide;
  uint16_t paddingBytes;
  uint16_t startSlotIdxLowRes;
  uint16_t endSlotIdxLowRes;
  uint16_t startSlotIdxHighRes;
  uint16_t endSlotIdxHighRes;
  struct Config config;
  uint8_t reserveBeforeSlotData[20];
  int32_t verticalAngles[NUM_LAYERS];
  struct Slot slot[NUM_SLOTS];
  struct Slot referenceShot[NUM_REFERENCE_SHOTS];
  uint16_t errorState;
  uint16_t laserVoltageActualMin;
  uint16_t laserVoltageActualMax;
  uint16_t laserVoltageActualAvg;
  uint8_t reserveAfterSlotData[20];
  uint32_t crc;
};
#pragma pack(pop)

static_assert(sizeof(c_vlo_scan5) == (36480640 / 8));

////////////

#pragma pack(push, 1)
struct c_vlo_metadata_imx449
{
  uint16_t laserVoltageActualMin;
  uint16_t laserVoltageActualMax;
  uint16_t laserVoltageActualAvg;
  uint8_t reserveAfterVoltage[20];
  int8_t temperatureReceiverChip;
  int8_t temperatureLaser;
  uint8_t reserveAfterTemperature[12];
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
};
#pragma pack(pop)

#pragma pack(push, 1)
struct c_vlo_metadata_imx479
{
  uint8_t SM_ESTS_7_0;
  uint8_t SM_ESTS_15_8;
  uint8_t SM_ESTS_23_16;
  uint8_t SM_ESTS_31_24;
  uint8_t SM_ESTS_39_32;
  uint16_t input_slot_number;
  uint16_t processed_slot_number;
  uint8_t context_cat1;
  uint8_t context_cat2;
  uint8_t context_cat3;
  uint8_t context_cat4;
  uint8_t context_cat5;
  uint8_t context_cat6;
  uint8_t reserved_1[16];
  uint16_t num_out_lines;
  uint16_t num_out_data;
  uint8_t reserved_2[5];
  uint16_t crc_slotdata;
  uint32_t chip_id_0;
  uint32_t chip_id_1;
  uint32_t chip_id_2;
  uint8_t frame_number;
  uint8_t reserved_3;
  uint16_t num_ebd_output_lines;
  uint16_t num_ebd_output_data;
  uint16_t num_amb_output_lines;
  uint16_t num_amb_output_data;
  uint16_t num_stats_output_lines;
  uint16_t num_stats_output_data;
  uint16_t num_raw_output_lines;
  uint16_t num_raw_output_data;
  uint16_t num_misalign_output_lines;
  uint16_t num_misalign_output_data;
  uint16_t imx_temp_N;
  uint16_t imx_temp_S;
  uint16_t imx_VT0_N;
  uint16_t imx_VT0_S;
  uint16_t imx_VT1_N;
  uint16_t imx_VT1_S;
  uint16_t imx_VT2_N;
  uint16_t imx_VT2_S;
  uint16_t imx_VT3_N;
  uint16_t imx_VT3_S;
  uint16_t imx_VT4_N;
  uint16_t imx_VT4_S;
  uint16_t imx_VT5_N;
  uint16_t imx_VT5_S;
  uint16_t imx_VT6_N;
  uint16_t imx_VT6_S;
  uint16_t imx_VT7_N;
  uint16_t imx_VT7_S;
  uint8_t reserved_4;
  uint8_t fov_offset_in_degrees;
  uint8_t IMX_pins;
  uint8_t FPGA_mipi_control_reg;
  uint8_t resolution_reduction;
  uint8_t sensor_head_firmware_HB;
  uint8_t sensor_head_firmware_LB;
  uint8_t reserved_5[2];
  uint8_t last_mirror_side;
  uint8_t temperature_FPGA_HB;
  uint8_t temperature_FPGA_LB;
  uint8_t temperature_Laser_zone_HB;
  uint8_t temperature_Laser_zone_LB;
  uint8_t temperature_IMU_zone_HB;
  uint8_t temperature_IMU_zone_LB;
  uint8_t temperature_HUM_module_HB;
  uint8_t temperature_HUM_module_LB;
  uint8_t reserved_6[14];
  uint8_t out_data_mode;
  uint16_t FS_AST_width;
  uint8_t PTRG_mode;
  uint16_t crc_framedata;

  //
  uint8_t _padding1[2];
  uint8_t statsMetaData[153];
  uint8_t _padding2[3];
  uint8_t reservedMetaData[12];
  uint8_t misalignmentMetaData[351];
  uint8_t _padding3[1];
  uint8_t ambientLightMetaData[1560];
};
#pragma pack(pop)




#pragma pack(push, 1)
struct c_vlo_scan6_base
{
  //! Interface version of the calibrated raw point cloud interface
  static constexpr  uint16_t VERSION = 6U;
  static constexpr  uint16_t NUM_SLOTS = 600;
  static constexpr  uint16_t NUM_LAYERS = 360;
  static constexpr  uint16_t NUM_ECHOS = 3;
  static constexpr  uint32_t NUM_POINTS = NUM_SLOTS * NUM_LAYERS * NUM_ECHOS;
  static constexpr  uint16_t NUM_REFERENCE_SHOTS = 2;
  static constexpr  uint16_t NUM_ZONES = 3;

  struct timestamp
  {
    uint32_t nanoseconds;
    uint32_t seconds;
    uint32_t secondsHi;
  };

  struct Config
  {
    uint32_t numPoints_max;
    uint16_t numEchos_max;
    uint16_t numLayers_max;
    uint16_t numSlots_max;
    uint8_t echoOrdering;
    uint8_t imagerType;
    uint8_t numHorizontalZones;
    uint8_t numEchos[NUM_ZONES];
    uint16_t numSlots[NUM_ZONES];
    uint16_t numLayers;
    uint8_t _reserved1[16];
    float horzAngleIncrement[NUM_ZONES];
    float verticalAngleIncrement;
    uint8_t _reserved2[32];
    uint8_t echoStructSize[NUM_ZONES];
    uint8_t sensorType;
    uint8_t numberOfMirrors;
    uint8_t binWidthInCm;
    uint16_t macroPixelWidth;
    uint16_t macroPixelHeight;
    uint8_t numLasershotsPerSlot;
    uint8_t _padding;
  };

  struct Echo
  {
    uint16_t dist;
    uint16_t area;
    uint8_t peak;
    uint8_t width;
  };


  uint16_t interfaceVersion;
  uint16_t scanNumber;
  struct timestamp timeStamp;
  uint16_t mirrorSide;
  uint16_t _padding1;
  uint32_t scanDuration_ns;
  struct Config config;
  uint8_t reserveBeforeSlotData[20];
  struct Echo echo[NUM_SLOTS][NUM_LAYERS][NUM_ECHOS];
  struct Echo referenceEcho[NUM_REFERENCE_SHOTS][NUM_LAYERS][NUM_ECHOS];
  uint16_t ambient[NUM_SLOTS][NUM_LAYERS];
  uint16_t referenceAmbiLight[NUM_REFERENCE_SHOTS][NUM_LAYERS];
  uint8_t numTotalEchoes[NUM_SLOTS][NUM_LAYERS];
  float horizontalAngles[NUM_SLOTS];
  float verticalAngles[NUM_LAYERS];
  uint16_t errorState;
  uint16_t _padding2;
};

struct c_vlo_scan6_imx449:
    c_vlo_scan6_base
{
  c_vlo_metadata_imx449 metaData;
  uint32_t crc;
};

struct c_vlo_scan6_imx479:
    c_vlo_scan6_base
{
  c_vlo_metadata_imx479 metaData;
  uint32_t crc;
};

#pragma pack(pop)

static_assert(sizeof(c_vlo_scan6_imx449) == (4554628));
static_assert(sizeof(c_vlo_scan6_imx479) == (4556624));


#pragma pack(push, 1)
struct c_vlo_scan6_slm
{
  static constexpr  uint16_t VERSION = 6U;
  static constexpr  uint32_t NUM_SLOTS = 641;
  static constexpr  uint32_t NUM_LAYERS = 460;
  static constexpr  uint32_t NUM_ECHOS = 3;
  static constexpr  uint8_t NUM_ZONES = 3;

  static constexpr  uint32_t BAD_LAYERS = 292;
  static constexpr  uint16_t MIN_DISTANCE = 100;
  static constexpr  uint16_t MAX_DISTANCE = 30000;

  struct Echo
  {
    uint16_t dist;
    uint16_t area;
  };

  struct Config
  {
    uint32_t numTotalPoints;
    uint16_t maxNumEchos;
    uint16_t maxNumLayers;
    uint16_t maxNumSlots;
    uint8_t echoOrdering;
    uint8_t imagerType;
    uint8_t numHorizontalZones;
    uint8_t numEchos[NUM_ZONES];
    uint16_t numSlots[NUM_ZONES];
    uint16_t numLayers[NUM_ZONES];
    uint16_t reserved[6];
    uint32_t horizontalAngleIncrement [NUM_ZONES];
    uint32_t verticalAngleIncrementHighResolution;
    uint32_t verticalAngleIncrementLowResolution;
    uint16_t reserved2[14];
    uint16_t reserved3;
    uint8_t interfaceStuct;
    uint8_t sensorType;
    uint8_t numberOfMirrors;
    uint8_t binWidthInCm;
    uint16_t macroPixelWidth;
    uint16_t macroPixelHeight;
    uint8_t numLasershotsPerSlot;
    uint8_t padding;
  };

  struct Meta
  {
    uint8_t SMESTS7_0;
    uint8_t SMESTS15_8;
    uint8_t SMESTS23_16;
    uint8_t SMESTS31_24;
    uint8_t SMESTS39_32;
    uint16_t numberOfInputSlots;
    uint16_t numberOfProcessedSlots;
    uint8_t contextSwStatusC1;
    uint8_t contextSwStatusC2;
    uint8_t contextSwStatusC3;
    uint8_t contextSwStatusC4;
    uint8_t contextSwStatusC5;
    uint8_t contextSwStatusC6;
    uint16_t reserved[8];
    uint16_t activeAreaMacroPixelData;
    uint16_t numberOfActiveAreaOutputData;
    uint8_t reserved2[5];

    uint16_t CheckSumOfSlotUpdateData;
    uint32_t chipId0;
    uint32_t chipId1;
    uint32_t chipId2;
    uint8_t frameNumber;
    uint8_t reserved3;

    uint16_t numberOfEmbeddedDataOutputLines;
    uint16_t numberOfEmbeddedDataOutputData;
    uint16_t ambientDataOutputLines;
    uint16_t ambientDataOutputDataCount;
    uint16_t statisticsDataOutputLine;
    uint16_t statisticsDataOutputData;
    uint16_t numberOfRawOutputLines;
    uint16_t numberOfRawOutputData;
    uint16_t numberOfMisalignmentOutputLines;
    uint16_t numberOfMisalignmentRawOutputSata;
    int16_t imxMeasuredTemperatureN;
    int16_t imxMeasuredTemperatureS;

    uint16_t imxVT0VBGRMeasuredVoltageN;
    uint16_t imxVT0VBGRMeasuredVoltageS;
    uint16_t imxVT1VDDLSCMeasuredVoltageN;
    uint16_t imxVT1VDDLSCMeasuredVoltageS;

    uint16_t imxVT2VDDMIOMeasuredVoltageN;
    uint16_t imxVT2VDDMIOMeasuredVoltageS;

    uint16_t imxVT3IBIAS2MeasuredVoltageN;
    uint16_t imxVT3IBIAS2MeasuredVoltageS;

    uint16_t imxVT4VRLDMeasuredVoltageN;
    uint16_t imxVT4VRLDMeasuredVoltageS;

    uint16_t imxVT5VDDHPFMeasuredVoltageN;
    uint16_t imxVT5VDDHPFMeasuredVoltageS;

    uint16_t imxVT6VDDHPFMeasuredVoltageN;
    uint16_t imxVT6VDDHPFMeasuredVoltageS;

    uint16_t imxVT7VDDHANMeasuredVoltageN;
    uint16_t imxVT7VDDHANMeasuredVoltageS;

    uint8_t reserved4;
    uint8_t FoVOffsetdegrees;
    uint8_t imxPins;
    uint8_t fpgaMipiControlRegister;
    uint8_t resolutionReduction;
    uint8_t sensorHeadFirmwareHB;
    uint8_t sensorHeadFirmwareLB;
    uint8_t reserved5 [2];

    uint8_t lastMirrorSide;
    int16_t temperatureFPGA;
    int16_t temperatureLaser;
    int16_t temperatureIMU;
    uint16_t temperatureHUMModule;
    uint8_t reserved6 [14];

    uint8_t dataOutputMode;
    uint16_t fSyncSetActivePeriodSlotUnits;
    uint8_t lightEmissionSettingOfPstTrg;
    uint16_t checkSumOfFrameUpdateData;
    uint16_t reserved7;
  };

  uint16_t interfaceVersion;
  uint16_t scanNumber;
  uint32_t nanoseconds;
  uint32_t seconds ;
  uint32_t secondsHi;

  uint16_t mirrorSide;
  uint16_t paddingBytes;
  uint32_t scanDuration_ns;
  struct Config config;
  uint8_t reserveBeforeSlotData[20];
  Echo echo[NUM_SLOTS][NUM_LAYERS][NUM_ECHOS];
  float horizontalAngles[NUM_SLOTS];
  float verticalAngles[NUM_LAYERS];
  uint16_t reserve[2];
  Meta metaData;
  uint32_t crc;
};
#pragma pack(pop)

////////////

struct c_vlo_scan
{
  VLO_VERSION version;
  union
  {
    c_vlo_scan1 scan1;
    c_vlo_scan3 scan3;
    c_vlo_scan5 scan5;
    c_vlo_scan6_imx449 scan6_imx449;
    c_vlo_scan6_imx479 scan6_imx479;
    c_vlo_scan6_slm scan6_slm;
  };
};

template<class ScanType>
struct c_vlo_scan_type_traits;

template<> struct c_vlo_scan_type_traits<c_vlo_scan1> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_1;
};
template<> struct c_vlo_scan_type_traits<c_vlo_scan3> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_3;
};
template<> struct c_vlo_scan_type_traits<c_vlo_scan5> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_5;
};
template<> struct c_vlo_scan_type_traits<c_vlo_scan6_slm> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_6_SLM;
};
template<> struct c_vlo_scan_type_traits<c_vlo_scan6_imx449> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_6_IMX449;
};
template<> struct c_vlo_scan_type_traits<c_vlo_scan6_imx479> {
  static constexpr VLO_VERSION VERSION = VLO_VERSION_6_IMX479;
};



class c_vlo_file
{
public:
  typedef c_vlo_file this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  enum DATA_CHANNEL {
    DATA_CHANNEL_AMBIENT,
    DATA_CHANNEL_DISTANCES,
    DATA_CHANNEL_ECHO_AREA,
    DATA_CHANNEL_ECHO_PEAK,
    DATA_CHANNEL_ECHO_WIDTH,
    DATA_CHANNEL_ECHO_AREA_DIV_WIDTH,
    DATA_CHANNEL_ECHO_PEAK_DIV_WIDTH,

    DATA_CHANNEL_ECHO_AREA_MUL_DIST,
    DATA_CHANNEL_ECHO_PEAK_MUL_DIST,

    DATA_CHANNEL_ECHO_AREA_MUL_DIST2,
    DATA_CHANNEL_ECHO_PEAK_MUL_DIST2,

    DATA_CHANNEL_ECHO_AREA_MUL_SQRT_DIST,
    DATA_CHANNEL_ECHO_PEAK_MUL_SQRT_DIST,

    DATA_CHANNEL_DOUBLED_ECHO_PEAKS,

    DATA_CHANNEL_DIST_TO_MAX_PEAK,

  };

  c_vlo_file();
  c_vlo_file(const std::string & filename);

  const std::string & filename() const;
  VLO_VERSION version() const;

  static bool sort_echos_by_distance(c_vlo_scan & scan);

  static cv::Mat get_image(const c_vlo_scan & scan, DATA_CHANNEL channel,
      cv::InputArray exclude_mask = cv::noArray());

  static bool get_cloud3d(const c_vlo_scan & scan, DATA_CHANNEL intensity_channel,
      cv::OutputArray points, cv::OutputArray colors);

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
  ~c_vlo_reader();

  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;
  bool seek(int32_t frame_index);
  int32_t curpos() const;

  /// @brief get frame size in bytes
  ssize_t frame_size() const;

  /// @brief get number of frames in this file
  ssize_t num_frames() const;

  bool read(c_vlo_scan * scan);
  bool read(cv::Mat * image, c_vlo_file::DATA_CHANNEL channel);
  bool read_cloud3d(cv::OutputArray points, cv::OutputArray colors, c_vlo_file::DATA_CHANNEL colors_channel);

protected:
  template<class ScanType> std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION > 0),
    bool> read(ScanType * scan);

protected:
  c_ifhd_reader ifhd_;
  ssize_t num_frames_ = -1;
  ssize_t frame_size_ = -1;
  int fd_ = -1;
};



#endif /* __c_vlo_file_h__ */
