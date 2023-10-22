/*
 * c_vlo_file.cc
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#include "c_vlo_file.h"
#include <fcntl.h>
#include <string.h>
// #include <core/proc/bswap.h>
#include <core/proc/autoclip.h>
#include <core/debug.h>

//@brief get current file position
static inline ssize_t whence(int fd)
{
  return ::lseek(fd, 0, SEEK_CUR);
}

static inline bool readfrom(int fd, ssize_t offset, uint16_t * data)
{
  if( ::lseek(fd, offset, SEEK_SET) != offset ) {
    return false;
  }

  if( ::read(fd, data, sizeof(*data)) != sizeof(*data) ) {
    return false;
  }

  return true;
}

template<class T>
static inline void swap( T & a, T & b)
{
  const T c(a);
  a = b;
  b = c;
}


static void init(c_vlo_scan & scan)
{
  scan.interfaceVersion = c_vlo_scan::INTERFACE_VERSION;
  scan.scanNumber = 0U;
  scan.timeStamp.nanoseconds = UINT32_MAX;
  scan.timeStamp.seconds = UINT32_MAX;
  scan.timeStamp.secondsHi = UINT32_MAX;
  scan.scanDuration_ns = UINT32_MAX;
  scan.mirrorSide = UINT16_MAX;
  scan.paddingBytes = UINT16_MAX;
  scan.startSlotIdxLowRes = UINT16_MAX;
  scan.endSlotIdxLowRes = UINT16_MAX;
  scan.startSlotIdxHighRes = UINT16_MAX;
  scan.endSlotIdxHighRes = UINT16_MAX;

  scan.config.numSlots = 0U;
  scan.config.numLayersPerSlot = 0U;
  scan.config.numEchos = 0U;
  scan.config.echoStructSizeInBit = UINT8_MAX;
  scan.config.echoOrdering = UINT8_MAX;
  scan.config.numLasershotsPerSlot = 0U;
  scan.config.macroPixelWidth = 0U;
  scan.config.macroPixelHeight = 0U;
  memset(scan.config.LUT_FS_X, UINT8_MAX, sizeof(scan.config.LUT_FS_X));
  memset(scan.config.LUT_FS_Y, UINT8_MAX, sizeof(scan.config.LUT_FS_Y));
  memset(scan.config.LUT_ACCU_X, UINT8_MAX, sizeof(scan.config.LUT_ACCU_X));
  memset(scan.config.LUT_ACCU_Y, UINT8_MAX, sizeof(scan.config.LUT_ACCU_Y));
  memset(scan.config.Rx_G_THACCU_0, UINT8_MAX, sizeof(scan.config.Rx_G_THACCU_0));
  scan.config.laserVoltageSetValue = UINT16_MAX;
  scan.config.shotTiming_x = UINT16_MAX;
  scan.config.shotTiming_y = UINT16_MAX;
  scan.config.shotTiming_z = UINT16_MAX;
  scan.config.shotTiming_frequency = UINT32_MAX;
  scan.config.shotTiming_sh = UINT16_MAX;
  scan.config.shotTiming_numLoadingPulses = UINT8_MAX;
  scan.config.shotTiming_dutyCycles = UINT8_MAX;
  scan.config.R_DISTANCE_SWITCH = UINT8_MAX;
  scan.config.R_FAR_REGION_RULE = UINT8_MAX;
  scan.config.R_PRIO_N = UINT8_MAX;
  scan.config.R_BIN_R = UINT8_MAX;
  memset(scan.config.Rx_BIN_A_0, UINT8_MAX, sizeof(scan.config.Rx_BIN_A_0));
  memset(scan.config.Rx_BIN_B_0, UINT8_MAX, sizeof(scan.config.Rx_BIN_B_0));
  scan.config.binWidthInCm = UINT8_MAX;
  scan.config.temperatureReceiverChip = INT8_MAX;
  scan.config.temperatureLaser = INT8_MAX;
  scan.config.imagerType = UINT8_MAX;

  memset(scan.reserveBeforeSlotData, UINT8_MAX, sizeof(scan.reserveBeforeSlotData));

  for( uint16_t layerIdx = 0U; layerIdx < c_vlo_scan::NUM_LAYERS; ++layerIdx ) {
    scan.verticalAngles[layerIdx] = INT32_MAX;
  }

  scan.slot[0].slotIdx = UINT16_MAX;
  memset(scan.slot[0].padding, UINT8_MAX, sizeof(scan.slot[0].padding));
  scan.slot[0].angleTicks = INT32_MAX;
  for( uint16_t layerIdx = 0U; layerIdx < c_vlo_scan::NUM_LAYERS; ++layerIdx ) {
    for( uint16_t echoIdx = 0U; echoIdx < c_vlo_scan::NUM_ECHOS; ++echoIdx ) {
      scan.slot[0].echo[layerIdx][echoIdx].distCm = UINT16_MAX;
      scan.slot[0].echo[layerIdx][echoIdx].area = UINT16_MAX;
      scan.slot[0].echo[layerIdx][echoIdx].width = UINT8_MAX;
      scan.slot[0].echo[layerIdx][echoIdx].peak = UINT8_MAX;
    }
    scan.slot[0].ambient[layerIdx] = UINT16_MAX;
    scan.slot[0].numTotalEchoes[layerIdx] = UINT8_MAX;
  }
  for( uint16_t slotIdx = 1U; slotIdx < c_vlo_scan::NUM_SLOTS; ++slotIdx ) {
    memcpy(&scan.slot[slotIdx], &scan.slot[0], sizeof(scan.slot[slotIdx]));
  }

  // Reference Shot
  for( uint16_t refShotIdx = 0U; refShotIdx < c_vlo_scan::NUM_REFERENCE_SHOTS; ++refShotIdx ) {
    memcpy(&scan.referenceShot[refShotIdx], &scan.slot[0], sizeof(scan.referenceShot[refShotIdx]));
  }

  // Post Slot data:
  scan.errorState = UINT16_MAX;
  scan.laserVoltageActualMin = UINT16_MAX;
  scan.laserVoltageActualMax = UINT16_MAX;
  scan.laserVoltageActualAvg = UINT16_MAX;
  memset(scan.reserveAfterSlotData, UINT8_MAX, sizeof(scan.reserveAfterSlotData));
  scan.crc = UINT32_MAX;
}



static bool convert(const c_vlo_scan1 & scan1, c_vlo_scan & scan)
{
  if( (scan1.interfaceVersion == scan1.INTERFACE_VERSION) || (scan1.interfaceVersion == 18U) ) {

    init(scan);

    scan.scanNumber = scan1.scanNumber;

    scan.timeStamp.secondsHi = scan1.timeStampSecondsHi;
    scan.timeStamp.seconds = scan1.timeStampSeconds;
    scan.timeStamp.nanoseconds = scan1.timeStampNanoSeconds;

    scan.scanDuration_ns = 33333333U;
    scan.mirrorSide = scan1.mirrorSide;
    scan.paddingBytes = scan1.paddingBytes;
    scan.startSlotIdxLowRes = scan1.startSlotIdxLowRes;
    scan.endSlotIdxLowRes = scan1.endSlotIdxLowRes;
    scan.startSlotIdxHighRes = scan1.startSlotIdxHighRes;
    scan.endSlotIdxHighRes = scan1.endSlotIdxHighRes;

    scan.config.numSlots = scan.NUM_SLOTS;
    scan.config.numLayersPerSlot = scan.NUM_LAYERS;
    scan.config.numEchos = scan.NUM_ECHOS;
    scan.config.numSlots = std::min(scan1.config.numSlots, scan.config.numSlots);
    scan.config.numLayersPerSlot = std::min(scan1.config.numLayersPerSlot, scan.config.numLayersPerSlot);
    scan.config.numEchos = std::min<uint16_t>(scan1.config.numEchos, scan.config.numEchos);
    scan.config.echoStructSizeInBit = 48U;
    scan.config.echoOrdering = scan1.config.echoOrdering;
    scan.config.numLasershotsPerSlot = scan1.config.numLasershotsPerSlot;
    scan.config.macroPixelWidth = scan1.config.macroPixelWidth;
    scan.config.macroPixelHeight = scan1.config.macroPixelHeight;
    memcpy(scan.config.LUT_FS_X, scan1.config.LUT_FS_X, sizeof(scan.config.LUT_FS_X));
    memcpy(scan.config.LUT_FS_Y, scan1.config.LUT_FS_Y, sizeof(scan.config.LUT_FS_Y));
    memcpy(scan.config.LUT_ACCU_X, scan1.config.LUT_ACCU_X, sizeof(scan.config.LUT_ACCU_X));
    memcpy(scan.config.LUT_ACCU_Y, scan1.config.LUT_ACCU_Y, sizeof(scan.config.LUT_ACCU_Y));
    memcpy(scan.config.Rx_G_THACCU_0, scan1.config.Rx_G_THACCU_0, sizeof(scan.config.Rx_G_THACCU_0));
    scan.config.laserVoltageSetValue = scan1.config.laserVoltageSetValue;
    scan.config.shotTiming_x = scan1.config.shotTiming_x;
    scan.config.shotTiming_y = scan1.config.shotTiming_y;
    scan.config.shotTiming_z = scan1.config.shotTiming_z;
    scan.config.shotTiming_frequency = scan1.config.shotTiming_frequency;
    scan.config.shotTiming_sh = scan1.config.shotTiming_sh;
    scan.config.shotTiming_numLoadingPulses = scan1.config.shotTiming_numLoadingPulses;
    scan.config.shotTiming_dutyCycles = scan1.config.shotTiming_dutyCycles;
    scan.config.imagerType = VLO_IMAGER_IMX449;

    const uint16_t COMMON_NUM_LAYERS = scan.config.numLayersPerSlot;
    const uint16_t COMMON_NUM_SLOTS = scan.config.numSlots;
    const uint16_t COMMON_NUM_ECHOS = scan.config.numEchos;

    for( uint16_t layerIdx = 0U; layerIdx < COMMON_NUM_LAYERS; ++layerIdx ) {
      scan.verticalAngles[layerIdx] = scan1.verticalAngles[layerIdx];
    }

    for( uint16_t slotIdx = 0U; slotIdx < COMMON_NUM_SLOTS; ++slotIdx ) {

      const auto &slotIN = scan1.slot[slotIdx];
      auto &slotOUT = scan.slot[slotIdx];

      slotOUT.slotIdx = scan1.slot[slotIdx].slotIdx;
      slotOUT.angleTicks = slotIN.angleTicks;
      memcpy(slotOUT.ambient, slotIN.ambient, sizeof(slotOUT.ambient));
      memset(slotOUT.numTotalEchoes, 0U, sizeof(slotOUT.numTotalEchoes));

      for( uint16_t layerIdx = 0U; layerIdx < COMMON_NUM_LAYERS; ++layerIdx ) {
        for( uint16_t echoIdx = 0U; echoIdx < COMMON_NUM_ECHOS; ++echoIdx ) {

          const auto &echoIN = slotIN.echo[layerIdx][echoIdx];
          auto &echoOUT = slotOUT.echo[layerIdx][echoIdx];

          if( echoIN.area == 0 ) { // Invalid data
            // Do nothing, keep defaults
          }
          else if( echoIN.area == 65534U ) { // No laser shot
            echoOUT.distCm = 65534U;
            echoOUT.area = 65534U;
            echoOUT.peak = 254U;
            echoOUT.width = 254U;
          }
          else if( echoIN.area == 65535U ) { // No echo
            echoOUT.distCm = 65535U;
            echoOUT.area = 65535U;
            echoOUT.peak = 255U;
            echoOUT.width = 255U;
          }
          else {
            echoOUT.distCm = echoIN.distCm;
            echoOUT.area = echoIN.area;
            echoOUT.peak = static_cast<uint8_t>(echoIN.peak);
            echoOUT.width = static_cast<uint8_t>(echoIN.width);
            slotOUT.numTotalEchoes[layerIdx] += 1;
          }
        }

        for( uint8_t i = 0; i < scan.NUM_ECHOS; ++i ) {
          for( uint8_t j = i + 1; j < scan.NUM_ECHOS; ++j ) {
            if( (slotOUT.echo[layerIdx][j].distCm > slotOUT.echo[layerIdx][i].distCm)
                && (slotOUT.echo[layerIdx][j].distCm < 65534U) ) {
              swap(slotOUT.echo[layerIdx][i], slotOUT.echo[layerIdx][j]);
            }
          }
        }
      }
    }

    scan.errorState = scan1.errorState;
    scan.laserVoltageActualMin = scan1.laserVoltageActualMin;
    scan.laserVoltageActualMax = scan1.laserVoltageActualMax;
    scan.laserVoltageActualAvg = scan1.laserVoltageActualAvg;
    scan.crc = scan1.crc;

    return true;
  }

  return false;
}

static bool convert(const c_vlo_scan3 & scan3, c_vlo_scan & scan)
{
  if( (scan3.interfaceVersion == scan3.INTERFACE_VERSION) || (scan3.interfaceVersion == 18U) ) {

    init(scan);

    scan.scanNumber = scan3.scanNumber;
    scan.timeStamp.secondsHi = scan3.timeStampSecondsHi;
    scan.timeStamp.seconds = scan3.timeStampSeconds;
    scan.timeStamp.nanoseconds = scan3.timeStampNanoSeconds;
    scan.scanDuration_ns = 33333333U; // estimated 120deg/180deg * 50ms
    scan.mirrorSide = scan3.mirrorSide;
    scan.paddingBytes = scan3.paddingBytes;
    scan.startSlotIdxLowRes = scan3.startSlotIdxLowRes;
    scan.endSlotIdxLowRes = scan3.endSlotIdxLowRes;
    scan.startSlotIdxHighRes = scan3.startSlotIdxHighRes;
    scan.endSlotIdxHighRes = scan3.endSlotIdxHighRes;

    scan.config.numSlots = scan.NUM_SLOTS;
    scan.config.numLayersPerSlot = scan.NUM_LAYERS;
    scan.config.numEchos = scan.NUM_ECHOS;
    scan.config.numLayersPerSlot = std::min(scan3.config.numLayersPerSlot, scan.config.numLayersPerSlot);
    scan.config.numEchos = std::min<uint16_t>(scan3.config.numEchos, scan.config.numEchos);
    scan.config.echoStructSizeInBit = 48U;
    scan.config.echoOrdering = scan3.config.echoOrdering;
    scan.config.numLasershotsPerSlot = scan3.config.numLasershotsPerSlot;
    scan.config.macroPixelWidth = scan3.config.macroPixelWidth;
    scan.config.macroPixelHeight = scan3.config.macroPixelHeight;
    memcpy(scan.config.LUT_FS_X, scan3.config.LUT_FS_X, sizeof(scan.config.LUT_FS_X));
    memcpy(scan.config.LUT_FS_Y, scan3.config.LUT_FS_Y, sizeof(scan.config.LUT_FS_Y));
    memcpy(scan.config.LUT_ACCU_X, scan3.config.LUT_ACCU_X, sizeof(scan.config.LUT_ACCU_X));
    memcpy(scan.config.LUT_ACCU_Y, scan3.config.LUT_ACCU_Y, sizeof(scan.config.LUT_ACCU_Y));
    memcpy(scan.config.Rx_G_THACCU_0, scan3.config.Rx_G_THACCU_0, sizeof(scan.config.Rx_G_THACCU_0));
    scan.config.laserVoltageSetValue = scan3.config.laserVoltageSetValue;
    scan.config.shotTiming_x = scan3.config.shotTiming_x;
    scan.config.shotTiming_y = scan3.config.shotTiming_y;
    scan.config.shotTiming_z = scan3.config.shotTiming_z;
    scan.config.shotTiming_frequency = scan3.config.shotTiming_frequency;
    scan.config.shotTiming_sh = scan3.config.shotTiming_sh;
    scan.config.shotTiming_numLoadingPulses = scan3.config.shotTiming_numLoadingPulses;
    scan.config.shotTiming_dutyCycles = scan3.config.shotTiming_dutyCycles;
    scan.config.imagerType = VLO_IMAGER_IMX449;

    const uint16_t COMMON_NUM_LAYERS = scan.config.numLayersPerSlot;
    const uint16_t COMMON_NUM_SLOTS = scan.config.numSlots;
    const uint16_t COMMON_NUM_ECHOS = scan.config.numEchos;

    for( uint16_t layerIdx = 0U; layerIdx < COMMON_NUM_LAYERS; ++layerIdx ) {
      scan.verticalAngles[layerIdx] = scan3.verticalAngles[layerIdx];
    }

    // Slot:
    for( uint16_t slotIdx = 0U; slotIdx < COMMON_NUM_SLOTS; ++slotIdx ) {

      const auto & slotIN = scan3.slot[slotIdx];
      auto &slotOUT = scan.slot[slotIdx];

      slotOUT.slotIdx = scan3.slot[slotIdx].slotIdx;
      slotOUT.angleTicks = slotIN.angleTicks;
      memcpy(slotOUT.ambient, slotIN.ambient, sizeof(slotOUT.ambient));
      memset(slotOUT.numTotalEchoes, 0U, sizeof(slotOUT.numTotalEchoes));

      for( uint16_t layerIdx = 0U; layerIdx < COMMON_NUM_LAYERS; ++layerIdx ) {
        for( uint16_t echoIdx = 0U; echoIdx < COMMON_NUM_ECHOS; ++echoIdx ) {

          const auto & echoIN = slotIN.echo[layerIdx][echoIdx];
          auto & echoOUT = slotOUT.echo[layerIdx][echoIdx];

          if( echoIN.area == 0 ) { // Invalid data
            // Do nothing, keep defaults
          }
          else if( echoIN.area == 65534U ) { // No laser shot
            echoOUT.distCm = 65534U;
            echoOUT.area = 65534U;
            echoOUT.peak = 254U;
            echoOUT.width = 254U;
          }
          else if( echoIN.area == 65535U ) { // No echo
            echoOUT.distCm = 65535U;
            echoOUT.area = 65535U;
            echoOUT.peak = 255U;
            echoOUT.width = 255U;
          }
          else {
            echoOUT.distCm = echoIN.distCm;
            echoOUT.area = echoIN.area;
            echoOUT.peak = static_cast<uint8_t>(echoIN.peak);
            echoOUT.width = static_cast<uint8_t>(echoIN.width);
            slotOUT.numTotalEchoes[layerIdx] += 1;
          }
        }

        for( uint8_t i = 0; i < scan.NUM_ECHOS; ++i ) {
          for( uint8_t j = i + 1; j < scan.NUM_ECHOS; ++j ) {
            if( (slotOUT.echo[layerIdx][j].distCm > slotOUT.echo[layerIdx][i].distCm) &&  (slotOUT.echo[layerIdx][j].distCm < 65534U) ) {
              swap(slotOUT.echo[layerIdx][i], slotOUT.echo[layerIdx][j]);
            }
          }
        }
      }
    }

    scan.errorState = scan3.errorState;
    scan.laserVoltageActualMin = scan3.laserVoltageActualMin;
    scan.laserVoltageActualMax = scan3.laserVoltageActualMax;
    scan.laserVoltageActualAvg = scan3.laserVoltageActualAvg;
    scan.crc = scan3.crc;

    return true;
  }

  return false;
}

c_vlo_reader::c_vlo_reader()
{
}

c_vlo_reader::c_vlo_reader(const std::string & filename) :
    base(filename)
{
  if ( !filename_.empty() && !open() ) {
    CF_ERROR("open('%s') fails: %s", filename_.c_str(),
        strerror(errno));
  }
}

bool c_vlo_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_vlo_reader::open() : no filename specified");
    return false;
  }

  const char *fname = filename_.c_str();
  bool fOk = false;
  uint16_t format_version = 0;

  if( (fd_ = ::open(fname, O_RDONLY | O_NOATIME)) < 0 ) {
    CF_ERROR("open('%s') fails: %s", fname, strerror(errno));
    goto end;
  }

  if( ::read(fd_, &format_version, sizeof(format_version)) != sizeof(format_version) ) {
    CF_ERROR("read('%s', format_version) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  CF_DEBUG("vlo struct sizes: scan1=%zu scan3=%zu scan=%zu",
      sizeof(c_vlo_scan1),
      sizeof(c_vlo_scan3),
      sizeof(c_vlo_scan));

  CF_DEBUG("vlo format version %u in '%s'",
      format_version, fname);

  switch ((version_ = (VLO_VERSION)(format_version))) {
    case VLO_VERSION_1:
      frame_size_ = sizeof(c_vlo_scan1);
      break;
    case VLO_VERSION_3:
      frame_size_ = sizeof(c_vlo_scan3);
      break;
    case VLO_VERSION_5:
      frame_size_ = sizeof(c_vlo_scan);
      break;
    case VLO_VERSION_18: {

      uint16_t data11, data12, data31, data32;
      bool isv1, isv3;
      long offset;

      if ( !readfrom(fd_, offset = 1 * sizeof(c_vlo_scan1), &data11) ) {
        CF_ERROR("readfrom('%s', offset=%ld, data11) fails: %s", fname, offset,
            strerror(errno));
        goto end;
      }
      if ( !readfrom(fd_, offset = 2 * sizeof(c_vlo_scan1), &data12) ) {
        CF_ERROR("readfrom('%s', offset=%ld, data12) fails: %s", fname, offset,
            strerror(errno));
        goto end;
      }

      if ( !readfrom(fd_, offset = 1 * sizeof(c_vlo_scan3), &data31) ) {
        CF_ERROR("readfrom('%s', offset=%ld, data31) fails: %s", fname, offset,
            strerror(errno));
        goto end;
      }
      if ( !readfrom(fd_, offset = 2 * sizeof(c_vlo_scan3), &data32) ) {
        CF_ERROR("readfrom('%s', offset=%ld, data32) fails: %s", fname, offset,
            strerror(errno));
        goto end;
      }

      isv1 = data11 == 18 && data12 == 18;
      isv3 = data31 == 18 && data32 == 18;

      if ( isv1 && !isv3 ) {
        version_ = VLO_VERSION_1;
        frame_size_ = sizeof(c_vlo_scan1);
      }
      else if ( isv3 && !isv1 ) {
        version_ = VLO_VERSION_3;
        frame_size_ = sizeof(c_vlo_scan3);
      }
      else {
        CF_ERROR("Can not determine exact format version for buggy vlo file '%s' : '%u' ", fname, format_version);
        errno = ENOMSG;
        goto end;
      }

      CF_DEBUG("vlo version %u selected for '%s'",
          version_, fname);

      break;
    }
    default:
      CF_ERROR("Not supported format version in vlo file '%s' : '%u' ", fname,
          format_version);
      errno = ENOMSG;
      goto end;
  }

  if( ::lseek(fd_, 0, SEEK_SET) != 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if( (file_size_ = ::lseek(fd_, 0, SEEK_END)) < 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_END) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if( ::lseek(fd_, 0, SEEK_SET) != 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  num_frames_ = file_size_ / frame_size();

  if ( num_frames_ * frame_size() != file_size_ ) { // temporary ignore this error
    CF_ERROR("vlo file '%s': file size = %zd bytes not match to expected number of frames %zd in", fname,
        file_size_, num_frames_);
  }

  fOk = true;

end:
  if( !fOk && fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
  }

  return fOk;
}

void c_vlo_reader::close()
{
  if( fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool c_vlo_reader::is_open() const
{
  return fd_ >= 0;
}

/// @brief get total file size in bytes
ssize_t c_vlo_reader::file_size() const
{
  return file_size_;
}

/// @brief get frame size in bytes
ssize_t c_vlo_reader::frame_size() const
{
  return frame_size_;
}

/// @brief get number of framesin this file
ssize_t c_vlo_reader::num_frames() const
{
  return num_frames_;
}

bool c_vlo_reader::seek(int32_t frame_index)
{
  if( fd_ >= 0 ) {
    return ::lseek(fd_, frame_index * frame_size(), SEEK_CUR) >= 0;
  }

  errno = EBADF;
  return false;
}

int32_t c_vlo_reader::curpos() const
{
  if( fd_ >= 0 ) {
    return whence(fd_) / frame_size();
  }
  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(c_vlo_scan * scan)
{
  switch (version_) {
    case VLO_VERSION_1: {
      c_vlo_scan1 scan1;
      if( ::read(fd_, &scan1, sizeof(scan1)) == sizeof(scan1) ) {
        return convert(scan1, *scan);
      }
      break;
    }
    case VLO_VERSION_3: {
      c_vlo_scan3 scan3;
      if( ::read(fd_, &scan3, sizeof(scan3)) == sizeof(scan3) ) {
        return convert(scan3, *scan);
      }
      break;
    }
    case VLO_VERSION_5:
      return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
    default:
      errno = ENOMSG;
      break;
  }
  return false;
}

bool c_vlo_reader::read(c_vlo_scan1 * scan)
{
  switch (version_) {
    case VLO_VERSION_1:
      return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
    default:
      errno = ENOMSG;
      break;
  }
  return false;
}

bool c_vlo_reader::read(c_vlo_scan3 * scan)
{
  switch (version_) {
    case VLO_VERSION_3:
      return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
    default:
      errno = ENOMSG;
      break;
  }
  return false;
}

bool c_vlo_reader::read_ambient(cv::Mat * image)
{
  switch (version()) {
    case VLO_VERSION_1: {
      c_vlo_scan1 scan;
      if( read(&scan) ) {
        *image = get_ambient_image(scan);
        return true;
      }
      break;
    }
    case VLO_VERSION_3: {
      c_vlo_scan3 scan;
      if( read(&scan) ) {
        *image = get_ambient_image(scan);
        return true;
      }
      break;
    }
    case VLO_VERSION_5: {
      c_vlo_scan scan;
      if( read(&scan) ) {
        *image = get_ambient_image(scan);
        return true;
      }
      break;
    }
  }

  return false;
}

cv::Mat1w c_vlo_reader::get_ambient_image(const c_vlo_scan1 & scan)
{
  cv::Mat1w image(scan.NUM_LAYERS, scan.NUM_SLOTS);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      image[l][s] = scan.slot[s].ambient[l];
    }
  }

  return image;
}

cv::Mat1w c_vlo_reader::get_ambient_image(const c_vlo_scan3 & scan)
{
  cv::Mat1w image(scan.NUM_LAYERS, scan.NUM_SLOTS);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      image[l][s] = scan.slot[s].ambient[l];
    }
  }

  return image;
}

cv::Mat1w c_vlo_reader::get_ambient_image(const c_vlo_scan & scan)
{
  cv::Mat1w image(scan.NUM_LAYERS, scan.NUM_SLOTS);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      image[l][s] = scan.slot[s].ambient[l];
    }
  }

  return image;
}

cv::Mat c_vlo_reader::get_thumbnail_image(const std::string & filename)
{
  cv::Mat image;
  c_vlo_reader vlo(filename);

  if ( vlo.is_open() ) {

    switch (vlo.version()) {
      case VLO_VERSION_1:
        break;
      case VLO_VERSION_3: {

        c_vlo_scan3 scan;

        if( vlo.read(&scan) ) {

          autoclip(image = get_ambient_image(scan),
              cv::noArray(),
              0.5, 99.5,
              0, 255);

          image.convertTo(image, CV_8U);
        }

        break;
      }
      case VLO_VERSION_5:
        break;
      case VLO_VERSION_18:
        break;
    }
  }

  return image;
}
