/*
 * c_hdl_mog2_filter.h
 *
 *  Created on: Dec 4, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_motion_filter_h__
#define __c_hdl_motion_filter_h__

#include <opencv2/opencv.hpp>

class c_hdl_motion_filter
{
public:
  typedef c_hdl_motion_filter this_class;

  // Max number of modes in mixture
  enum { MAX_MODES = 4 };

  // Mode
  struct GMM
  {
    float m, s, w;
  };

  // Mixture
  struct GM
  {
    int nmodes;
    GMM modes[MAX_MODES];
  };

  c_hdl_motion_filter();

  void set_enabled(bool );
  bool enabled() const;

  void set_max_distance(double v);
  double max_distance() const;

  void set_bg_history(int v);
  int bg_history() const;

  void set_var_threshold(double v);
  double var_threshold() const;

  void set_var_init(double v);
  double var_init() const;

  void set_var_min(double v);
  double var_min() const;

  void set_var_max(double v);
  double var_max() const;

  void set_motion_threshold(double v);
  double motion_threshold() const;

  void set_enable_noise_reduction(bool v);
  bool enable_noise_reduction() const;

  void set_denoise_filter_radius(const cv::Size & v);
  const cv::Size & denoise_filter_radius() const;

  void set_denoise_filter_depth(double v);
  double denoise_filter_depth() const;

  void set_denoise_filter_threshold(int v);
  int denoise_filter_threshold() const;


  bool apply(const cv::Mat1f & range_image,
      cv::OutputArray output_goreground_mask );

  int nb_frames_processed() const;

  const std::vector<GM> & current_histogram() const;
  const GM & current_histogram(int r, int c) const;
  const cv::Size & histogram_size() const;

  static void create_background_image(const std::vector<c_hdl_motion_filter::GM> & historam,
      const cv::Size & histogram_size,
      double motion_threshold,
      cv::OutputArray background_image);

  static void create_bgn_image(const std::vector<c_hdl_motion_filter::GM> & historam,
      const cv::Size & histogram_size,
      cv::OutputArray background_image);

  static void create_bgw_image(const std::vector<c_hdl_motion_filter::GM> & historam,
      const cv::Size & histogram_size,
      cv::OutputArray background_image);

  static void create_bgs_image(const std::vector<c_hdl_motion_filter::GM> & historam,
      const cv::Size & histogram_size,
      cv::OutputArray background_image);


  void clear();
  void reset();

protected:
  void initialize(const cv::Size & range_image_size);

  static inline void remove_mode(struct GM & gm, int erase_pos);
  static inline void insert_mode(struct GM & gm, int insert_pos);

  bool update(const cv::Mat1f & range_image, cv::OutputArray output_foreground_mask);

  void reduce_motion_noise(const cv::Mat1f & range_image,
      cv::OutputArray output_foreground_mask) const;


protected:
  // 2D array of mixtures
  std::vector<GM> histogram_;

  // number of processed input frames
  int nb_frames_processed_ = 0;

  // range image layout dimensions cols x rows
  cv::Size histogram_size_;

  // background history (learning rate)
  int bg_history_ = 200;

  // variance threshold
  double var_threshold_ = 5;

  // initial variance for new mode
  double var_init_ = 0.2;

  // minimal variance
  double var_min_ = 0.1;

  // maximal variance
  double var_max_ = 1.0;

  // motion threshold
  double motion_threshold_ = 0.2;

  double max_distance_ = 300;

  cv::Size denoise_filter_radius_ = cv::Size(1, 1);
  double denoise_filter_depth_ = 1;
  int denoise_filter_threshold_ = 4;

  bool enabeld_ = true;
  bool enable_noise_reduction_ = true;

};

typedef std::vector<c_hdl_motion_filter::GM>
  c_mog2_histogram;

#endif /* __c_hdl_motion_filter_h__ */
