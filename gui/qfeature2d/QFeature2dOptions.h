/*
 * QFeature2dOptions.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFeature2dOptions_h__
#define __QFeature2dOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/feature2d/feature2d.h>


class QSparseFeatureDetectorOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDetectorOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DETECTOR_TYPE> QSparseFeatureDetectorTypeCombo;

  QSparseFeatureDetectorOptions(QWidget * parent = nullptr);

  void set_feature_detector_options(c_sparse_feature_detector_options * options);
  c_sparse_feature_detector_options* feature_detector_options() const;

Q_SIGNALS:
 void detectorTypeChanged();

protected:
  void onupdatecontrols() override;
  void update_detector_specific_controls();

protected:
  c_sparse_feature_detector_options *options_ = nullptr;
  QSparseFeatureDetectorTypeCombo * detectorType_ctl = nullptr;
  QNumericBox * max_keypoints_ctl = nullptr;
  QWidgetList controls;
};

class QSparseFeatureDescriptorOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDescriptorOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DESCRIPTOR_TYPE> QSparseFeatureDecriptorTypeCombo;

  QSparseFeatureDescriptorOptions(QWidget * parent = nullptr);

  void set_feature_descriptor_options(c_sparse_feature_descriptor_options * options);
  c_sparse_feature_descriptor_options* feature_descriptor_options() const;

protected:
  void onupdatecontrols() override;
  void update_descriptor_specific_controls();

protected:
  c_sparse_feature_descriptor_options *options_ = nullptr;
  QSparseFeatureDecriptorTypeCombo * descriptorType_ctl = nullptr;
  // QCheckBox * useDetectorSettings_ctl = nullptr;
  QWidgetList controls;
};

class QHammingDistanceFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QHammingDistanceFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QHammingDistanceFeature2dMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options(c_hamming_distance_feature2d_matcher_options * options);
  c_hamming_distance_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_hamming_distance_feature2d_matcher_options * options_ = nullptr;
  QNumericBox * max_acceptable_distance_ctl = nullptr;
  QNumericBox * octavedif_ctl = nullptr;
};

class QFlannBasedFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFlannBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<FlannIndexType > QFlannIndexTypeCombo;
  typedef QEnumComboBox<cvflann::flann_distance_t> QFlannDistanceTypeCombo;

  QFlannBasedFeature2dMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options( c_flann_based_feature2d_matcher_options * options);
  c_flann_based_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;
  void update_matcher_specific_controls();

protected:
  c_flann_based_feature2d_matcher_options * options_ = nullptr;
  QFlannIndexTypeCombo * flannIndexType_ctl = nullptr;
  QFlannDistanceTypeCombo * flannDistanceType_ctl = nullptr;
  QNumericBox * lowe_ratio_ctl = nullptr;
  QWidgetList controls;
};

class QOptFlowPyrLKMatcherOptions  :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QOptFlowPyrLKMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QOptFlowPyrLKMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options(c_optflowpyrlk_feature2d_matcher_options * options);
  c_optflowpyrlk_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_optflowpyrlk_feature2d_matcher_options * options_ = nullptr;
  QNumericBox * maxLevel_ctl = nullptr;
  QNumericBox * winSize_ctl = nullptr;
  QNumericBox * maxIterations_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QNumericBox * flags_ctl = nullptr;
  QNumericBox * minEigThreshold_ctl = nullptr;
  QNumericBox * maxErr_ctl = nullptr;
};

class QTriangleMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QTriangleMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QTriangleMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options( c_triangle_matcher_options * options);
  c_triangle_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_triangle_matcher_options * options_ = nullptr;
  QNumericBox * eps_ctl = nullptr;
};

class QSnormBasedFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSnormBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QSnormBasedFeature2dMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options( c_snorm_based_feature2d_matcher_options * options);
  c_snorm_based_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_snorm_based_feature2d_matcher_options * options_ = nullptr;
  QNumericBox * max_acceptable_distance_ctl = nullptr;
  QNumericBox * lowe_ratio_ctl = nullptr;
};


class QSparseFeatureMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureMatcherOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<FEATURE2D_MATCHER_TYPE> QSparseFeatureMatcherTypeCombo;

  QSparseFeatureMatcherOptions(QWidget * parent = nullptr);

  void set_feature_matcher_options(c_feature2d_matcher_options * options);
  c_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;
  void show_current_matcher_controls();

protected:
  c_feature2d_matcher_options * options_ = nullptr;
  QSparseFeatureMatcherTypeCombo * sparseFeatureMatcherType_ctl = nullptr;
  QHammingDistanceFeature2dMatcherOptions * hammingDistanceFeature2dMatcherOptions_ctl = nullptr;
  QFlannBasedFeature2dMatcherOptions * flannBasedFeature2dMatcherOptions_ctl = nullptr;
  QOptFlowPyrLKMatcherOptions * optFlowPyrLKMatcherOptions_ctl = nullptr;
  QTriangleMatcherOptions * triangleMatcherOptions_ctl = nullptr;

  QSnormBasedFeature2dMatcherOptions * snormBasedFeature2dMatcherOptions_ctl = nullptr;
};


#endif /* __QFeature2DOptions_h__ */
