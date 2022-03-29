/*
 * QFeature2DSettings.h
 *
 *  Created on: Mar 29, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFeature2DSettings_h__
#define __QFeature2DSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/feature2d/feature2d.h>

class QSparseFeatureDetectorSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDetectorSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QSparseFeatureDetectorSettingsWidget(QWidget * parent = Q_NULLPTR);

  void set_feature_detector_options(c_sparse_feature_detector_options * opts);
  const c_sparse_feature_detector_options * feature_detector_options() const;

protected:
  void clearFeatureExtractorSpecificControls();
  void populateFeatureExtractorSpecificControls();
  void updateFeatureExtractorSpecificControls();
  void onupdatecontrols() override;


protected:
  c_sparse_feature_detector_options * options_ = Q_NULLPTR;
  QEnumComboBox<SPARSE_FEATURE_DETECTOR_TYPE> * featureExtractorType_ctl = Q_NULLPTR;
  QList<QWidget*> controls_;
};


class QSparseDescriptorExtractorSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseDescriptorExtractorSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QSparseDescriptorExtractorSettingsWidget(QWidget * parent = Q_NULLPTR);

  void set_feature_descriptor_options(c_sparse_feature_descriptor_options * opts);
  const c_sparse_feature_descriptor_options * feature_descriptor_options() const;

protected:
  void clearDescriptorExtractorSpecificControls();
  void populateDescriptorExtractorSpecificControls();
  void updateDescriptorExtractorSpecificControls();
  void onupdatecontrols() override;


protected:
  c_sparse_feature_descriptor_options * options_ = Q_NULLPTR;
  QCheckBox * useDetectorSettings_ctl = Q_NULLPTR;
  QEnumComboBox<SPARSE_FEATURE_DESCRIPTOR_TYPE> * descriptorExtractorType_ctl = Q_NULLPTR;
  QList<QWidget*> controls_;
};


class QSparseFeatureExtractorSettingsWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureExtractorSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QSparseFeatureExtractorSettingsWidget(QWidget * parent = Q_NULLPTR);

  void set_sparse_feature_extractor_options(c_sparse_feature_extractor_options * opts);
  const c_sparse_feature_extractor_options* sparse_feature_extractor_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_sparse_feature_extractor_options * options_ = Q_NULLPTR;
  QSparseFeatureDetectorSettingsWidget * detectorSettings_ctl = Q_NULLPTR;
  QSparseDescriptorExtractorSettingsWidget * descriptorSettings_ctl = Q_NULLPTR;
};

class QSparseFeature2DMatcherSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeature2DMatcherSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QSparseFeature2DMatcherSettingsWidget(QWidget * parent = Q_NULLPTR);

  void set_feature2d_matcher_options(c_feature2d_matcher_options * opts);
  const c_feature2d_matcher_options * feature2d_matcher_options() const;

protected:
  void clearMatcherSpecificControls();
  void populateMatcherSpecificControls();
  void updateMatcherSpecificControls();
  void onupdatecontrols() override;

protected:
  c_feature2d_matcher_options * options_ = Q_NULLPTR;
  QEnumComboBox<FEATURE2D_MATCHER_TYPE> * matcherType_ctl = Q_NULLPTR;
  QList<QWidget*> controls_;
};

#endif /* __QFeature2DSettings_h__ */
