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


template<class Feature2DDetectorType>
class QSparseFeature2DOptionsTemplate :
    public QSettingsWidgetTemplate<typename Feature2DDetectorType::options>
{
  // Q_OBJECT;
public:
  typedef QSparseFeature2DOptionsTemplate ThisClass;
  typedef QSettingsWidgetTemplate<typename Feature2DDetectorType::options> Base;

  QSparseFeature2DOptionsTemplate(QWidget * parent = nullptr) :
    Base(parent)
  {
    Base::setObjectName(QString::fromStdString(toString(feature2d_traits<Feature2DDetectorType>::type)));
    populate_feature2d_options(this);
  }
};


class QSparseFeatureDetectorOptions :
    public QSettingsWidgetTemplate<c_sparse_feature_detector_options>
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDetectorOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_sparse_feature_detector_options> Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DETECTOR_TYPE> QSparseFeatureDetectorCombo;

  QSparseFeatureDetectorOptions(QWidget * parent = nullptr);

Q_SIGNALS:
 void detectorTypeChanged();

protected:
 void updateDetectorSpecificControls();

 template<class F>
 void addStackWidget(F c_sparse_feature_detector_options::* mp);

protected:
  QSparseFeatureDetectorCombo * detectorType_ctl = nullptr;
  QNumericBox * max_keypoints_ctl = nullptr;
  QStackedWidget * _stack = nullptr;
};

class QSparseFeatureDescriptorOptions:
    public QSettingsWidgetTemplate<c_sparse_feature_descriptor_options>
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDescriptorOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_sparse_feature_descriptor_options> Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DESCRIPTOR_TYPE> QSparseFeatureDecriptorCombo;

  QSparseFeatureDescriptorOptions(QWidget * parent = nullptr);

protected:
  void updateDescriptorSpecificControls();

  template<class F>
  void addStackWidget(F c_sparse_feature_descriptor_options::* mp);

protected:
  QSparseFeatureDecriptorCombo * descriptorType_ctl = nullptr;
  QStackedWidget * _stack = nullptr;
};

class QHammingDistanceFeature2dMatcherOptions :
    public QSettingsWidgetTemplate<c_hamming_distance_feature2d_matcher_options>
{
  Q_OBJECT;
public:
  typedef QHammingDistanceFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_hamming_distance_feature2d_matcher_options> Base;

  QHammingDistanceFeature2dMatcherOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * max_acceptable_distance_ctl = nullptr;
  QNumericBox * octavedif_ctl = nullptr;
};



template<class flann_index_options>
class QFlannIndexOptions :
    public QSettingsWidgetTemplate<flann_index_options>
{
public:
  typedef QFlannIndexOptions ThisClass;
  typedef QSettingsWidgetTemplate<flann_index_options> Base;

  QFlannIndexOptions(QWidget * parent = nullptr) : Base(parent)
  {
    populate_feature2d_options(this);
  }
};

class QFlannBasedFeature2dMatcherOptions :
    public QSettingsWidgetTemplate<c_flann_based_feature2d_matcher_options>
{
  Q_OBJECT;
public:
  typedef QFlannBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_flann_based_feature2d_matcher_options> Base;
  typedef QEnumComboBox<FlannIndexType> QFlannIndexTypeCombo;
  typedef QEnumComboBox<cvflann::flann_distance_t> QFlannDistanceTypeCombo;

  QFlannBasedFeature2dMatcherOptions(QWidget * parent = nullptr);

  template<class F>
  QFlannIndexOptions<F> * addStackWidget(F c_flann_based_feature2d_matcher_options::* mp);

protected:
  void showMatcherSpecificControls();

protected:
  QFlannIndexTypeCombo * flannIndexType_ctl = nullptr;
  QFlannDistanceTypeCombo * flannDistanceType_ctl = nullptr;
  QNumericBox * lowe_ratio_ctl = nullptr;
  QStackedWidget * _stack = nullptr;
  QFlannIndexOptions<c_flann_linear_index_options> * _linear_index_options = nullptr;
  QFlannIndexOptions<c_flann_kdtree_index_options> * _kdtree_index_options = nullptr;
  QFlannIndexOptions<c_flann_kmeans_index_options> * _kmeans_index_options = nullptr;
  QFlannIndexOptions<c_flann_composite_index_options> * _composite_index_options = nullptr;
  QFlannIndexOptions<c_flann_hierarchical_index_options> * _hierarchical_index_options = nullptr;
  QFlannIndexOptions<c_flann_lsh_index_options> * _lsh_index_options = nullptr;
  QFlannIndexOptions<c_flann_autotuned_index_options> * _autotuned_index_options = nullptr;
};

class QOptFlowPyrLKMatcherOptions  :
    public QSettingsWidgetTemplate<c_optflowpyrlk_feature2d_matcher_options>
{
  Q_OBJECT;
public:
  typedef QOptFlowPyrLKMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_optflowpyrlk_feature2d_matcher_options> Base;

  QOptFlowPyrLKMatcherOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * maxLevel_ctl = nullptr;
  QNumericBox * winSize_ctl = nullptr;
  QNumericBox * maxIterations_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QNumericBox * flags_ctl = nullptr;
  QNumericBox * minEigThreshold_ctl = nullptr;
  QNumericBox * maxErr_ctl = nullptr;
};

class QTriangleMatcherOptions :
    public QSettingsWidgetTemplate<c_triangle_matcher_options>
{
  Q_OBJECT;
public:
  typedef QTriangleMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_triangle_matcher_options> Base;

  QTriangleMatcherOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * eps_ctl = nullptr;
};

class QSnormBasedFeature2dMatcherOptions :
    public QSettingsWidgetTemplate<c_snorm_based_feature2d_matcher_options>
{
  Q_OBJECT;
public:
  typedef QSnormBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_snorm_based_feature2d_matcher_options> Base;

  QSnormBasedFeature2dMatcherOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * max_acceptable_distance_ctl = nullptr;
  QNumericBox * lowe_ratio_ctl = nullptr;
};


class QSparseFeatureMatcherOptions :
    public QSettingsWidgetTemplate<c_feature2d_matcher_options>
{
  Q_OBJECT;
public:
  typedef QSparseFeatureMatcherOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_feature2d_matcher_options> Base;
  typedef QEnumComboBox<FEATURE2D_MATCHER_TYPE> QSparseFeatureMatcherTypeCombo;

  QSparseFeatureMatcherOptions(QWidget * parent = nullptr);

protected:
  void showMatcherSpecificControls();

protected:
  QSparseFeatureMatcherTypeCombo * sparseFeatureMatcherType_ctl = nullptr;
  QHammingDistanceFeature2dMatcherOptions * hammingDistanceFeature2dMatcherOptions_ctl = nullptr;
  QFlannBasedFeature2dMatcherOptions * flannBasedFeature2dMatcherOptions_ctl = nullptr;
  QOptFlowPyrLKMatcherOptions * optFlowPyrLKMatcherOptions_ctl = nullptr;
  QTriangleMatcherOptions * triangleMatcherOptions_ctl = nullptr;
  QSnormBasedFeature2dMatcherOptions * snormBasedFeature2dMatcherOptions_ctl = nullptr;
  QStackedWidget * _stack = nullptr;
};


#endif /* __QFeature2DOptions_h__ */
