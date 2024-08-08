/*
 * oxts_track.h
 *
 *  Created on: Sep 5, 2019
 *      Author: amyznikov
 *
 * The c_oxts_track class represents array of oxts data instances.
 *
 */

#ifndef __oxts_track_h__
#define __oxts_track_h__

#include "oxts.h"
#include <memory>

//! @addtogroup oxts
//! @{

/** @brief
The c_oxts_track class represents array of oxts instances.
Additionaly this class implemens some low-level utilites for coordinate system conversions and trajectory interpolation.
 */
class c_oxts_track
{
public:
  typedef c_oxts_track this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::shared_ptr<const this_class> const_sptr;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<const this_class> const_uptr;


  explicit c_oxts_track(const std::string & name = "");
  virtual ~c_oxts_track() = default;

  std::string name;

  size_t size() const;

  bool empty() const;


  std::vector<oxts_data> & oxts();
  const std::vector<oxts_data> & oxts() const;

  oxts_data & oxts(size_t index);
  const oxts_data & oxts(size_t index) const;


  oxts_data & front();
  const oxts_data & front() const;

  oxts_data & back();
  const oxts_data & back() const;

  oxts_data & middle();
  const oxts_data & middle() const;

  std::vector<oxts_data>::iterator begin();
  std::vector<oxts_data>::const_iterator begin() const;

  std::vector<oxts_data>::iterator end();
  std::vector<oxts_data>::const_iterator end() const;

  /**
   * @brief return 3x4 world to reference (oxts) matrix
   * @return 3x4 matrix of doubles
   */
  const cv::Matx34d & imu2ref() const;

  /**
   * @brief set world to reference (oxts) transformation matrix
   * @param imu2ref 3x4 transformation matrix of doubles
   * @param update_inverse true if matrix should be inversed, false otherwise
   */
  void set_imu2ref(const cv::Matx34d & imu2ref, bool update_inverse = true);

  /**
   * @brief set world to reference (oxts) transformation matrix from rotation matrix and translation vector
   * @param R 3x3 rotation matrix
   * @param T 3d translation vector
   * @param update_inverse true if matrix should be inversed, false otherwise
   */
  void set_imu2ref(const cv::Matx33d & R, const cv::Vec3f & T, bool update_inverse = true);

  /**
   * @brief get reference (oxts) to world transformation matrix
   * @return 3x4 matrix of doubles
   */
  const cv::Matx34d & ref2imu() const;

  /**
   * @brief set reference (oxts) to world transformation matrix
   * @param ref2imu 3x4 transformation matrix
   * @param update_inverse true if matrux should be inversed, false otherwise
   */
  void set_ref2imu(const cv::Matx34d & ref2imu, bool update_inverse = true);

  /**
   * @brief set reference (oxts) to world transformation matrix from rotation matrix and translation vector
   * @param R 3x3 rotation matrix
   * @param T 3d translation vector
   * @param update_inverse true if matrix should be inversed, false otherwise
   */
  void set_ref2imu(const cv::Matx33d & R, const cv::Vec3f & T, bool update_inverse = true);


  /** @brief Calls oxts_.clear(). No actual memory is freed though.
   */
  void clear();

  /** @brief Clear and free all memory.
   */
  void release();

  /** @brief Free unused memory.
   */
  void shrink_to_fit();

  /** @brief Sort oxts_[] atray by timestamp.
   */
  void sort_by_timestamp();

  /**
   * @brief Search oxts by time stamp assuming oxts[] is sorted in increasing order,
   * interpolate for specific ts, and return interpolated values into oxtsi.
   * @param ts timestamp to interpolate
   * @param out interpolated oxts
   * @return false if requested timestamp is not in the oxts range, true otherwise
   */
  bool interpolate_for_timestamp(double ts, oxts_data * out) const;

  /**
   * @brief apply correction for all oxts in array
   * @param roll oxts roll
   * @param pitch oxts pitch
   * @param yaw oxts yaw
   * @param ts oxts timestamp
   */
  void apply_oxts_corrections(double roll, double pitch, double yaw, double ts);


protected:
  /// @brief actual oxts array
  std::vector<oxts_data> oxts_;

  /// @brief forward and reverse transforms between Ref and IMU.
  ///        It is assumed that Ref and IMU system COULD be different
  cv::Matx34d imu2ref_ = cv::Matx34d::eye();
  cv::Matx34d ref2imu_ = cv::Matx34d::eye();
};

//! @} oxts

#endif /* __oxts_track_h__ */
