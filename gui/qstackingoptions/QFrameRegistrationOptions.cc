/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"
//#include <gui/widgets/addctrl.h>
#include <core/debug.h>

#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
//}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QHammingDistanceFeature2dMatcherOptions::QHammingDistanceFeature2dMatcherOptions(QWidget * parent) :
    Base("QHammingDistanceFeature2dMatcherOptions", parent)
{
  max_acceptable_distance_ctl =
      add_numeric_box<int>("max_acceptable_distance",
          [this](int value) {
            if ( options_ && options_->max_acceptable_distance != value ) {
              options_->max_acceptable_distance = value;
              emit parameterChanged();
            }
          });

}

void QHammingDistanceFeature2dMatcherOptions::set_feature_matcher_options(c_hamming_distance_feature2d_matcher_options * options)
{
  options_ = options;
  updateControls();
}

c_hamming_distance_feature2d_matcher_options* QHammingDistanceFeature2dMatcherOptions::feature_matcher_options() const
{
  return options_;
}

void QHammingDistanceFeature2dMatcherOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    max_acceptable_distance_ctl->setValue(options_->max_acceptable_distance);
    setEnabled(true);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QFlannBasedFeature2dMatcherOptions::QFlannBasedFeature2dMatcherOptions(QWidget * parent) :
    Base("QFlannBasedFeature2dMatcherOptions", parent)
{
  flannIndexType_ctl =
      add_enum_combobox<FlannIndexType>("Flann Index Type:",
          [this](FlannIndexType value) {
            if ( options_ && options_->index.type != value ) {
              options_->index.type = value;
              update_matcher_specific_controls();
              emit parameterChanged();
            }
          });

  flannDistanceType_ctl =
      add_enum_combobox<cvflann::flann_distance_t>("Flann Distance Type:",
          [this](cvflann::flann_distance_t value) {
            if ( options_ && options_->distance_type != value ) {
              options_->distance_type = value;
              emit parameterChanged();
            }
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio",
          [this](double value) {
            if ( options_ && options_->lowe_ratio != value ) {
              options_->lowe_ratio = value;
              emit parameterChanged();
            }
          });

  updateControls();
}

void QFlannBasedFeature2dMatcherOptions::set_feature_matcher_options( c_flann_based_feature2d_matcher_options * options)
{
  options_ = options;
  updateControls();
}

c_flann_based_feature2d_matcher_options * QFlannBasedFeature2dMatcherOptions::feature_matcher_options() const
{
  return options_;
}

void QFlannBasedFeature2dMatcherOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    flannIndexType_ctl->setValue(options_->index.type);
    flannDistanceType_ctl->setValue(options_->distance_type);
    lowe_ratio_ctl->setValue(options_->lowe_ratio);
    update_matcher_specific_controls();
    setEnabled(true);
  }
}

void QFlannBasedFeature2dMatcherOptions::update_matcher_specific_controls()
{
  for ( QWidget * w : controls ) {
    form->removeRow(w);
  }
  controls.clear();

  if ( !options_ ) {
    flannIndexType_ctl->setEnabled(false);
    flannDistanceType_ctl->setEnabled(false);
    lowe_ratio_ctl->setEnabled(false);
  }
  else {

#define ADD_FLANN_INDEX_CTL(f, name) \
    controls.append(add_ctl<decltype(options_->index.f.name)>(#name, \
        [this](decltype(options_->index.f.name) v){ \
          if ( options_ ) { \
            options_->index.f.name = v; \
          }}, \
        [this](decltype(options_->index.f.name) * v) -> bool { \
          if ( options_ ) { \
            *v = options_->index.f.name; \
            return true; \
          } \
          return false; \
        } \
    ))


    switch (options_->index.type) {
    case FlannIndex_linear:
      break;

    case FlannIndex_kdtree:
      ADD_FLANN_INDEX_CTL(kdtree, trees);
      break;

    case FlannIndex_kmeans:
      ADD_FLANN_INDEX_CTL(kmeans, centers_init);
      ADD_FLANN_INDEX_CTL(kmeans, branching);
      ADD_FLANN_INDEX_CTL(kmeans, iterations);
      ADD_FLANN_INDEX_CTL(kmeans, cb_index);
      break;

    case FlannIndex_composite:
      ADD_FLANN_INDEX_CTL(composite, centers_init);
      ADD_FLANN_INDEX_CTL(composite, trees);
      ADD_FLANN_INDEX_CTL(composite, branching);
      ADD_FLANN_INDEX_CTL(composite, iterations);
      ADD_FLANN_INDEX_CTL(composite, cb_index);
      break;

    case FlannIndex_hierarchical:
      ADD_FLANN_INDEX_CTL(hierarchical, centers_init);
      ADD_FLANN_INDEX_CTL(hierarchical, branching);
      ADD_FLANN_INDEX_CTL(hierarchical, trees);
      ADD_FLANN_INDEX_CTL(hierarchical, leaf_size);
      break;

    case FlannIndex_lsh:
      ADD_FLANN_INDEX_CTL(lsh, table_number);
      ADD_FLANN_INDEX_CTL(lsh, key_size);
      ADD_FLANN_INDEX_CTL(lsh, multi_probe_level);
      break;

    case FlannIndex_autotuned:
      ADD_FLANN_INDEX_CTL(autotuned, target_precision);
      ADD_FLANN_INDEX_CTL(autotuned, build_weight);
      ADD_FLANN_INDEX_CTL(autotuned, memory_weight);
      ADD_FLANN_INDEX_CTL(autotuned, sample_fraction);
      break;
    }

    emit populatecontrols();

#undef ADD_FLANN_INDEX_CTL
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QTriangleMatcherOptions::QTriangleMatcherOptions(QWidget * parent) :
    Base("QTriangleMatcherOptions", parent)
{
  eps_ctl =
      add_numeric_box<double>("eps",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              emit parameterChanged();
            }
          });

}

void QTriangleMatcherOptions::set_feature_matcher_options(c_triangle_matcher_options * options)
{
  options_ = options;
  updateControls();
}

c_triangle_matcher_options* QTriangleMatcherOptions::feature_matcher_options() const
{
  return options_;
}

void QTriangleMatcherOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    eps_ctl->setValue(options_->eps);
    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QSnormBasedFeature2dMatcherOptions::QSnormBasedFeature2dMatcherOptions(QWidget * parent) :
    Base("QSnormBasedFeature2dMatcherOptions", parent)
{
  max_acceptable_distance_ctl =
      add_numeric_box<int>("max_acceptable_distance:",
          [this](int value) {
            if ( options_ && options_->max_acceptable_distance != value ) {
              options_->max_acceptable_distance = value;
              emit parameterChanged();
            }
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio:",
          [this](int value) {
            if ( options_ && options_->lowe_ratio != value ) {
              options_->lowe_ratio = value;
              emit parameterChanged();
            }
          });

  updateControls();
}

void QSnormBasedFeature2dMatcherOptions::set_feature_matcher_options( c_snorm_based_feature2d_matcher_options * options)
{
  options_ = options;
  updateControls();
}

c_snorm_based_feature2d_matcher_options * QSnormBasedFeature2dMatcherOptions::feature_matcher_options() const
{
  return options_;
}

void QSnormBasedFeature2dMatcherOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    max_acceptable_distance_ctl->setValue(options_->max_acceptable_distance);
    lowe_ratio_ctl->setValue(options_->lowe_ratio);
    setEnabled(true);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QSparseFeatureMatcherOptions::QSparseFeatureMatcherOptions(QWidget * parent) :
    Base("QSparseFeatureMatcherOptions", parent)
{
  sparseFeatureMatcherType_ctl =
      add_enum_combobox<FEATURE2D_MATCHER_TYPE>(
          "Feature matcher:",
          [this](FEATURE2D_MATCHER_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              show_current_matcher_controls();
              emit parameterChanged();
            }
          });

  hammingDistanceFeature2dMatcherOptions_ctl =
      add_widget<QHammingDistanceFeature2dMatcherOptions>();

  connect(hammingDistanceFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  flannBasedFeature2dMatcherOptions_ctl =
      add_widget<QFlannBasedFeature2dMatcherOptions>();

  connect(flannBasedFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  triangleMatcherOptions_ctl =
      add_widget<QTriangleMatcherOptions>();

  connect(triangleMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  snormBasedFeature2dMatcherOptions_ctl =
      add_widget<QSnormBasedFeature2dMatcherOptions>();

  connect(snormBasedFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  show_current_matcher_controls();
}


void QSparseFeatureMatcherOptions::set_feature_matcher_options(c_feature2d_matcher_options * options)
{
  options_ = options;
  updateControls();
}

c_feature2d_matcher_options * QSparseFeatureMatcherOptions::feature_matcher_options() const
{
  return options_;
}

void QSparseFeatureMatcherOptions::onupdatecontrols()
{
  if( !options_ ) {

    setEnabled(false);
    hammingDistanceFeature2dMatcherOptions_ctl->set_feature_matcher_options(nullptr);
    flannBasedFeature2dMatcherOptions_ctl->set_feature_matcher_options(nullptr);
    triangleMatcherOptions_ctl->set_feature_matcher_options(nullptr);
    snormBasedFeature2dMatcherOptions_ctl->set_feature_matcher_options(nullptr);
  }
  else {

    sparseFeatureMatcherType_ctl->setValue(options_->type);
    hammingDistanceFeature2dMatcherOptions_ctl->set_feature_matcher_options(&options_->hamming);
    flannBasedFeature2dMatcherOptions_ctl->set_feature_matcher_options(&options_->flann);
    triangleMatcherOptions_ctl->set_feature_matcher_options(&options_->triangles);
    snormBasedFeature2dMatcherOptions_ctl->set_feature_matcher_options(&options_->snorm);

    show_current_matcher_controls();

    setEnabled(true);
  }
}

void QSparseFeatureMatcherOptions::show_current_matcher_controls()
{
  if ( !options_ ) {
    hammingDistanceFeature2dMatcherOptions_ctl->setVisible(false);
    flannBasedFeature2dMatcherOptions_ctl->setVisible(false);
    triangleMatcherOptions_ctl->setVisible(false);
    snormBasedFeature2dMatcherOptions_ctl->setVisible(false);
  }
  else {
    switch (options_->type) {
      case FEATURE2D_MATCHER_HAMMING:
        hammingDistanceFeature2dMatcherOptions_ctl->setVisible(true);
        flannBasedFeature2dMatcherOptions_ctl->setVisible(false);
        triangleMatcherOptions_ctl->setVisible(false);
        snormBasedFeature2dMatcherOptions_ctl->setVisible(false);
        break;
      case FEATURE2D_MATCHER_FLANN:
        hammingDistanceFeature2dMatcherOptions_ctl->setVisible(false);
        flannBasedFeature2dMatcherOptions_ctl->setVisible(true);
        triangleMatcherOptions_ctl->setVisible(false);
        snormBasedFeature2dMatcherOptions_ctl->setVisible(false);
        break;
      case FEATURE2D_MATCHER_TRIANGLES:
        hammingDistanceFeature2dMatcherOptions_ctl->setVisible(false);
        flannBasedFeature2dMatcherOptions_ctl->setVisible(false);
        triangleMatcherOptions_ctl->setVisible(true);
        snormBasedFeature2dMatcherOptions_ctl->setVisible(false);
        break;
      case FEATURE2D_MATCHER_SNORM:
        hammingDistanceFeature2dMatcherOptions_ctl->setVisible(false);
        flannBasedFeature2dMatcherOptions_ctl->setVisible(false);
        triangleMatcherOptions_ctl->setVisible(false);
        snormBasedFeature2dMatcherOptions_ctl->setVisible(true);
        break;
      default:
        hammingDistanceFeature2dMatcherOptions_ctl->setVisible(false);
        flannBasedFeature2dMatcherOptions_ctl->setVisible(false);
        triangleMatcherOptions_ctl->setVisible(false);
        snormBasedFeature2dMatcherOptions_ctl->setVisible(false);
        break;
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//QSparseFeatureExtractorOptions::QSparseFeatureExtractorOptions(QWidget * parent) :
//    Base("QSparseFeatureExtractorOptions", parent)
//{
//  add_expandable_groupbox("Feature Detector Options",
//      sparseFeatureDetectorOptions_ctl = new QSparseFeatureDetectorOptions());
//
//  connect(sparseFeatureDetectorOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  add_expandable_groupbox("Feature Descriptor Options",
//      sparseFeatureDescriptorOptions_ctl = new QSparseFeatureDescriptorOptions(this));
//
//   connect(sparseFeatureDescriptorOptions_ctl, &QSettingsWidget::parameterChanged,
//       this, &ThisClass::parameterChanged);
//
//}
//
//void QSparseFeatureExtractorOptions::set_feature_extractor_options(c_sparse_feature_extractor_options * options)
//{
//  options_ = options;
//  updateControls();
//}
//
//c_sparse_feature_extractor_options* QSparseFeatureExtractorOptions::feature_extractor_options() const
//{
//  return options_;
//}
//
//void QSparseFeatureExtractorOptions::onupdatecontrols()
//{
//  if( !options_ ) {
//    setEnabled(false);
//    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(nullptr);
//    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(nullptr);
//  }
//  else {
//    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(&options_->detector);
//    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(&options_->descriptor);
//    setEnabled(true);
//  }
//}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QSparseFeatureDetectorOptions::QSparseFeatureDetectorOptions(QWidget * parent) :
    Base("QSparseFeatureDetectorOptions", parent)
{
  detectorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DETECTOR_TYPE>(
          "DETECTOR_TYPE",
          [this](SPARSE_FEATURE_DETECTOR_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              update_detector_specific_controls();
              emit detectorTypeChanged();
              emit parameterChanged();
            }
          });

  max_keypoints_ctl =
      add_numeric_box<int>("Max. key points:",
          [this](int value) {
            if ( options_ && options_->max_keypoints != value ) {
              options_->max_keypoints = value;
              emit parameterChanged();
            }
          });
}

void QSparseFeatureDetectorOptions::set_feature_detector_options(c_sparse_feature_detector_options * options)
{
  options_ = options;
  updateControls();
}

c_sparse_feature_detector_options* QSparseFeatureDetectorOptions::feature_detector_options() const
{
  return options_;
}

void QSparseFeatureDetectorOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    detectorType_ctl->setValue(options_->type);
    update_detector_specific_controls();
    setEnabled(true);
  }
}

void QSparseFeatureDetectorOptions::update_detector_specific_controls()
{
  for ( QWidget * w : controls ) {
    form->removeRow(w);
  }
  controls.clear();

  if ( options_  ) {

    if( options_->type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
      max_keypoints_ctl->setValue(1);
      max_keypoints_ctl->setEnabled(false);
    }
    else {
      max_keypoints_ctl->setValue(options_->max_keypoints);
      max_keypoints_ctl->setEnabled(true);
    }

#define ADD_CTL(f, name) \
    controls.append(add_ctl<decltype(options_->f.name)>(#name, \
        [this](decltype(options_->f.name) v){ \
          if ( options_ ) { \
            options_->f.name = v; \
          }}, \
        [this](decltype(options_->f.name) * v) -> bool { \
          if ( options_ ) { \
            *v = options_->f.name; \
            return true; \
          } \
          return false; \
        } \
    ))

    switch (options_->type) {
    case SPARSE_FEATURE_DETECTOR_ORB:
      ADD_CTL(orb, nfeatures);
      ADD_CTL(orb, scaleFactor);
      ADD_CTL(orb, nlevels);
      ADD_CTL(orb, edgeThreshold);
      ADD_CTL(orb, firstLevel);
      ADD_CTL(orb, WTA_K);
      ADD_CTL(orb, patchSize);
      ADD_CTL(orb, fastThreshold);
      ADD_CTL(orb, scoreType);
      break;

    case SPARSE_FEATURE_DETECTOR_BRISK:
      ADD_CTL(brisk, thresh);
      ADD_CTL(brisk, octaves);
      ADD_CTL(brisk, patternScale);
      break;

    case SPARSE_FEATURE_DETECTOR_MSER:
      ADD_CTL(mser, delta);
      ADD_CTL(mser, min_area);
      ADD_CTL(mser, max_area);
      ADD_CTL(mser, max_variation);
      ADD_CTL(mser, min_diversity);
      ADD_CTL(mser, max_evolution);
      ADD_CTL(mser, area_threshold);
      ADD_CTL(mser, min_margin);
      ADD_CTL(mser, edge_blur_size);
      break;

    case SPARSE_FEATURE_DETECTOR_FAST:
      ADD_CTL(fast, threshold);
      ADD_CTL(fast, nonmaxSuppression);
      ADD_CTL(fast, type);
      break;

    case SPARSE_FEATURE_DETECTOR_AGAST:
      ADD_CTL(agast, threshold);
      ADD_CTL(agast, nonmaxSuppression);
      ADD_CTL(agast, type);
      break;

    case SPARSE_FEATURE_DETECTOR_GFTT:
      ADD_CTL(gftt, maxCorners);
      ADD_CTL(gftt, qualityLevel);
      ADD_CTL(gftt, minDistance);
      ADD_CTL(gftt, blockSize);
      ADD_CTL(gftt, gradiantSize);
      ADD_CTL(gftt, k);
      ADD_CTL(gftt, useHarrisDetector);
      break;

    case SPARSE_FEATURE_DETECTOR_BLOB:
      ADD_CTL(blob, thresholdStep);
      ADD_CTL(blob, minThreshold);
      ADD_CTL(blob, maxThreshold);
      ADD_CTL(blob, minRepeatability);
      ADD_CTL(blob, minDistBetweenBlobs);
      ADD_CTL(blob, filterByColor);
      ADD_CTL(blob, blobColor);
      ADD_CTL(blob, filterByArea);
      ADD_CTL(blob, minArea);
      ADD_CTL(blob, maxArea);
      ADD_CTL(blob, filterByCircularity);
      ADD_CTL(blob, minCircularity);
      ADD_CTL(blob, maxCircularity);
      ADD_CTL(blob, filterByInertia);
      ADD_CTL(blob, minInertiaRatio);
      ADD_CTL(blob, maxInertiaRatio);
      ADD_CTL(blob, filterByConvexity);
      ADD_CTL(blob, minConvexity);
      ADD_CTL(blob, maxConvexity);
      break;

    case SPARSE_FEATURE_DETECTOR_KAZE:
      ADD_CTL(kaze, extended);
      ADD_CTL(kaze, upright);
      ADD_CTL(kaze, threshold);
      ADD_CTL(kaze, nOctaves);
      ADD_CTL(kaze, nOctaveLayers);
      ADD_CTL(kaze, diffusivity);
      break;

    case SPARSE_FEATURE_DETECTOR_AKAZE:
      ADD_CTL(akaze, descriptor_type);
      ADD_CTL(akaze, descriptor_size);
      ADD_CTL(akaze, descriptor_channels);
      ADD_CTL(akaze, threshold);
      ADD_CTL(akaze, nOctaves);
      ADD_CTL(akaze, nOctaveLayers);
      ADD_CTL(akaze, diffusivity);
      break;

#if HAVE_FEATURE2D_SIFT
    case SPARSE_FEATURE_DETECTOR_SIFT:
      ADD_CTL(sift, nfeatures);
      ADD_CTL(sift, nOctaveLayers);
      ADD_CTL(sift, contrastThreshold);
      ADD_CTL(sift, edgeThreshold);
      ADD_CTL(sift, sigma);
      break;
#endif

#if HAVE_FEATURE2D_SURF
    case SPARSE_FEATURE_DETECTOR_SURF:
      ADD_CTL(surf, hessianThreshold);
      ADD_CTL(surf, nOctaves);
      ADD_CTL(surf, nOctaveLayers);
      ADD_CTL(surf, extended);
      ADD_CTL(surf, upright);
      break;
#endif

#if HAVE_FEATURE2D_STAR
    case SPARSE_FEATURE_DETECTOR_STAR:
      ADD_CTL(star, maxSize);
      ADD_CTL(star, responseThreshold);
      ADD_CTL(star, lineThresholdProjected);
      ADD_CTL(star, lineThresholdBinarized);
      ADD_CTL(star, suppressNonmaxSize);
      break;
#endif

#if HAVE_FEATURE2D_MSD
    case SPARSE_FEATURE_DETECTOR_MSD:
      ADD_CTL(msd, m_patch_radius);
      ADD_CTL(msd, m_search_area_radius);
      ADD_CTL(msd, m_nms_radius);
      ADD_CTL(msd, m_nms_scale_radius);
      ADD_CTL(msd, m_th_saliency);
      ADD_CTL(msd, m_kNN);
      ADD_CTL(msd, m_scale_factor);
      ADD_CTL(msd, m_n_scales);
      ADD_CTL(msd, m_compute_orientation);
      break;
#endif

#if HAVE_FEATURE2D_HL
    case SPARSE_FEATURE_DETECTOR_HL:
      ADD_CTL(hl, numOctaves);
      ADD_CTL(hl, corn_thresh);
      ADD_CTL(hl, DOG_thresh);
      ADD_CTL(hl, maxCorners);
      ADD_CTL(hl, num_layers);
      break;
#endif

    default:
      break;
    }

    emit populatecontrols();

#undef ADD_CTL
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QSparseFeatureDescriptorOptions::QSparseFeatureDescriptorOptions(QWidget * parent) :
    Base("QSparseFeatureDescriptorOptions", parent)
{
  useDetectorSettings_ctl =
      add_checkbox("Use detector settings",
          [this](bool checked) {
            if ( options_ && options_->use_detector_options != checked ) {
              options_->use_detector_options = checked;
              update_descriptor_specific_controls();
              emit parameterChanged();
            }
          });

  descriptorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DESCRIPTOR_TYPE>(
          "DESCRIPTOR_TYPE",
          [this](SPARSE_FEATURE_DESCRIPTOR_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              update_descriptor_specific_controls();
              emit parameterChanged();
            }
          });
}

void QSparseFeatureDescriptorOptions::set_feature_descriptor_options(c_sparse_feature_descriptor_options * options)
{
  options_ = options;
  updateControls();
}

c_sparse_feature_descriptor_options* QSparseFeatureDescriptorOptions::feature_descriptor_options() const
{
  return options_;
}

void QSparseFeatureDescriptorOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    useDetectorSettings_ctl->setChecked(options_->use_detector_options);
    descriptorType_ctl->setValue(options_->type);
    update_descriptor_specific_controls();
    setEnabled(true);
  }
}

void QSparseFeatureDescriptorOptions::update_descriptor_specific_controls()
{
  for ( QWidget * w : controls ) {
    form->removeRow(w);
  }
  controls.clear();

  if ( !options_ || options_->use_detector_options ) {
    descriptorType_ctl->setEnabled(false);
  }
  else {

    descriptorType_ctl->setEnabled(true);

#define ADD_CTL(f, name) \
    controls.append(add_ctl<decltype(options_->f.name)>(#name, \
        [this](decltype(options_->f.name) v){ \
          if ( options_ ) { \
            options_->f.name = v; \
          }}, \
        [this](decltype(options_->f.name) * v) -> bool { \
          if ( options_ ) { \
            *v = options_->f.name; \
            return true; \
          } \
          return false; \
        } \
    ))

    switch (options_->type) {
    case SPARSE_FEATURE_DESCRIPTOR_ORB:
      ADD_CTL(orb, nfeatures);
      ADD_CTL(orb, scaleFactor);
      ADD_CTL(orb, nlevels);
      ADD_CTL(orb, edgeThreshold);
      ADD_CTL(orb, firstLevel);
      ADD_CTL(orb, WTA_K);
      ADD_CTL(orb, patchSize);
      ADD_CTL(orb, fastThreshold);
      ADD_CTL(orb, scoreType);
      break;

    case SPARSE_FEATURE_DESCRIPTOR_BRISK:
      ADD_CTL(brisk, thresh);
      ADD_CTL(brisk, octaves);
      ADD_CTL(brisk, patternScale);
      break;

    case SPARSE_FEATURE_DESCRIPTOR_KAZE:
      ADD_CTL(kaze, extended);
      ADD_CTL(kaze, upright);
      ADD_CTL(kaze, threshold);
      ADD_CTL(kaze, nOctaves);
      ADD_CTL(kaze, nOctaveLayers);
      ADD_CTL(kaze, diffusivity);
      break;

    case SPARSE_FEATURE_DESCRIPTOR_AKAZE:
      ADD_CTL(akaze, descriptor_type);
      ADD_CTL(akaze, descriptor_size);
      ADD_CTL(akaze, descriptor_channels);
      ADD_CTL(akaze, threshold);
      ADD_CTL(akaze, nOctaves);
      ADD_CTL(akaze, nOctaveLayers);
      ADD_CTL(akaze, diffusivity);
      break;

#if HAVE_FEATURE2D_SIFT
    case SPARSE_FEATURE_DESCRIPTOR_SIFT:
      ADD_CTL(sift, nfeatures);
      ADD_CTL(sift, nOctaveLayers);
      ADD_CTL(sift, contrastThreshold);
      ADD_CTL(sift, edgeThreshold);
      ADD_CTL(sift, sigma);
      break;
#endif

#if HAVE_FEATURE2D_SURF
    case SPARSE_FEATURE_DESCRIPTOR_SURF:
      ADD_CTL(surf, hessianThreshold);
      ADD_CTL(surf, nOctaves);
      ADD_CTL(surf, nOctaveLayers);
      ADD_CTL(surf, extended);
      ADD_CTL(surf, upright);
      break;
#endif

#if HAVE_FEATURE2D_FREAK
    case SPARSE_FEATURE_DESCRIPTOR_FREAK:
      ADD_CTL(freak, orientationNormalized);
      ADD_CTL(freak, scaleNormalized);
      ADD_CTL(freak, patternScale);
      ADD_CTL(freak, nOctaves);
      break;
#endif

#if HAVE_FEATURE2D_BRIEF
    case SPARSE_FEATURE_DESCRIPTOR_BRIEF:
      ADD_CTL(brief, bytes);
      ADD_CTL(brief, use_orientation);
      break;
#endif

#if HAVE_FEATURE2D_LUCID
    case SPARSE_FEATURE_DESCRIPTOR_LUCID:
      ADD_CTL(lucid, lucid_kernel);
      ADD_CTL(lucid, blur_kernel);
      break;
#endif
#if HAVE_FEATURE2D_LATCH
    case SPARSE_FEATURE_DESCRIPTOR_LATCH:
      ADD_CTL(latch, bytes);
      ADD_CTL(latch, rotationInvariance);
      ADD_CTL(latch, half_ssd_size);
      ADD_CTL(latch, sigma);
      break;
#endif
#if HAVE_FEATURE2D_DAISY
    case SPARSE_FEATURE_DESCRIPTOR_DAISY:
      ADD_CTL(daisy, radius);
      ADD_CTL(daisy, q_radius);
      ADD_CTL(daisy, q_theta);
      ADD_CTL(daisy, q_hist);
      ADD_CTL(daisy, norm);
      ADD_CTL(daisy, interpolation);
      ADD_CTL(daisy, use_orientation);
      break;
#endif
#if HAVE_FEATURE2D_VGG
    case SPARSE_FEATURE_DESCRIPTOR_VGG:
      ADD_CTL(vgg, desc);
      ADD_CTL(vgg, isigma);
      ADD_CTL(vgg, img_normalize);
      ADD_CTL(vgg, use_scale_orientation);
      ADD_CTL(vgg, scale_factor);
      ADD_CTL(vgg, dsc_normalize);
      break;
#endif
#if HAVE_FEATURE2D_BOOST
    case SPARSE_FEATURE_DESCRIPTOR_BOOST:
      ADD_CTL(boost, desc);
      ADD_CTL(boost, use_scale_orientation);
      ADD_CTL(boost, scale_factor);
      break;
#endif
    default:
      break;
    }

    emit populatecontrols();
#undef ADD_CTL
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFeatureBasedRegistrationOptions::QFeatureBasedRegistrationOptions(QWidget * parent) :
    Base("QFeatureBasedRegistrationOptions", parent)
{
  enableFeatureBasedRegistration_ctl =
      add_checkbox("Enable feature based registration",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              emit parameterChanged();
            }
          });

  controls.append(scale_ctl =
      add_numeric_box<double>("Image scale",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              emit parameterChanged();
            }
          }));

  add_expandable_groupbox("Sparse Feature Detector Options",
      sparseFeatureDetectorOptions_ctl = new QSparseFeatureDetectorOptions());
  controls.append(sparseFeatureDetectorOptions_ctl);

  connect(sparseFeatureDetectorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Sparse Feature Descriptor Options",
      sparseFeatureDescriptorOptions_ctl = new QSparseFeatureDescriptorOptions());
  controls.append(sparseFeatureDescriptorOptions_ctl);

  connect(sparseFeatureDescriptorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Sparse Feature Matcher Options",
      sparseFeatureMatcherOptions_ctl = new QSparseFeatureMatcherOptions());
  controls.append(sparseFeatureMatcherOptions_ctl);

  connect(sparseFeatureMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(sparseFeatureDetectorOptions_ctl, &QSparseFeatureDetectorOptions::detectorTypeChanged,
      this, &ThisClass::onDetectorTypeChanged);

}

void QFeatureBasedRegistrationOptions::set_registration_options(c_feature_based_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_feature_based_registration_options * QFeatureBasedRegistrationOptions::registration_options() const
{
  return options_;
}

void QFeatureBasedRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableFeatureBasedRegistration_ctl->setChecked(options_->enabled);
    scale_ctl->setValue(options_->scale);
    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(&options_->sparse_feature_extractor.detector);
    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(&options_->sparse_feature_extractor.descriptor);
    sparseFeatureMatcherOptions_ctl->set_feature_matcher_options(&options_->sparse_feature_matcher);
    update_controls_state();
    onDetectorTypeChanged();
    setEnabled(true);
  }
}

void QFeatureBasedRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

void QFeatureBasedRegistrationOptions::onDetectorTypeChanged()
{
  if ( options_ ) {

    if ( options_->sparse_feature_extractor.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
      sparseFeatureDescriptorOptions_ctl->setEnabled(false);
      sparseFeatureMatcherOptions_ctl->setEnabled(false);
    }
    else {
      sparseFeatureDescriptorOptions_ctl->setEnabled(true);
      sparseFeatureMatcherOptions_ctl->setEnabled(true);
    }
  }
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccRegistrationOptions::QEccRegistrationOptions(QWidget * parent) :
    Base("QEccRegistrationOptions", parent)
{
  enableEcc_ctl =
      add_checkbox("Enable ECC",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              emit parameterChanged();
            }
          });

  controls.append(scale_ctl =
      add_numeric_box<double>("image scale",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(eps_ctl =
      add_numeric_box<double>("eps",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              emit parameterChanged();
            }
          }));

  controls.append(min_rho_ctl =
      add_numeric_box<double>("min_rho",
          [this](double value) {
            if ( options_ && options_->min_rho != value ) {
              options_->min_rho = value;
              emit parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              emit parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              emit parameterChanged();
            }
          }));

  controls.append(update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          [this](double value) {
            if ( options_ && options_->update_step_scale != value ) {
              options_->update_step_scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(normalization_noise_ctl =
      add_numeric_box<double>("normalization_noise",
          [this](double value) {
            if ( options_ && options_->normalization_noise != value ) {
              options_->normalization_noise = value;
              emit parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          [this](int value) {
            if ( options_ && options_->normalization_scale != value ) {
              options_->normalization_scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              emit parameterChanged();
            }
          }));

  controls.append(enable_ecch_ctl =
      add_checkbox("enable_ecch",
          [this](bool checked) {
            if ( options_ && options_->enable_ecch != checked ) {
              options_->enable_ecch = checked;
              ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);
              emit parameterChanged();
            }
          }));

  controls.append(ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          [this](int value) {
            if ( options_ && options_->ecch_minimum_image_size != value ) {
              options_->ecch_minimum_image_size = value;
              emit parameterChanged();
            }
          }));

  updateControls();
}

void QEccRegistrationOptions::set_registration_options(c_ecc_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_ecc_registration_options* QEccRegistrationOptions::registration_options() const
{
  return options_;
}

void QEccRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableEcc_ctl->setChecked(options_->enabled);
    scale_ctl->setValue(options_->scale);
    eps_ctl->setValue(options_->eps);
    min_rho_ctl->setValue(options_->min_rho);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    update_step_scale_ctl->setValue(options_->update_step_scale);
    normalization_noise_ctl->setValue(options_->normalization_noise);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    max_iterations_ctl->setValue(options_->max_iterations);
    enable_ecch_ctl->setChecked(options_->enable_ecch);
    ecch_minimum_image_size_ctl->setValue(options_->ecch_minimum_image_size);
    ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);

    update_controls_state();

    setEnabled(true);
  }
}

void QEccRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccFlowRegistrationOptions::QEccFlowRegistrationOptions(QWidget * parent) :
    Base("QEccFlowRegistrationOptions", parent)
{
  enableEccFlow_ctl =
      add_checkbox("Enable ECC flow",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              emit parameterChanged();
            }
          });



  controls.append(support_scale_ctl =
      add_numeric_box<int>("support_scale",
          [this](int value) {
            if ( options_ && options_->support_scale != value ) {
              options_->support_scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              emit parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              emit parameterChanged();
            }
          }));

  controls.append(update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          [this](double value) {
            if ( options_ && options_->update_multiplier != value ) {
              options_->update_multiplier = value;
              emit parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              emit parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          [this](int value) {
            if ( options_ && options_->normalization_scale != value ) {
              options_->normalization_scale = value;
              emit parameterChanged();
            }
          }));
}

void QEccFlowRegistrationOptions::set_registration_options(c_eccflow_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_eccflow_registration_options* QEccFlowRegistrationOptions::registration_options() const
{
  return options_;
}

void QEccFlowRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableEccFlow_ctl->setChecked(options_->enabled);
    update_multiplier_ctl->setValue(options_->update_multiplier);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    max_iterations_ctl->setValue(options_->max_iterations);
    support_scale_ctl->setValue(options_->support_scale);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    update_controls_state();
    setEnabled(true);
  }
}

void QEccFlowRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QJovianDerotationOptions::QJovianDerotationOptions(QWidget * parent) :
    Base("QJovianDerotationOptions", parent)
{
  enableJovianDerotation_ctl =
      add_checkbox("Enable Jovian Derotation",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              emit parameterChanged();
            }
          });

  controls.append(min_rotation_ctl =
      add_numeric_box<double>("min_rotation [deg]:",
          [this](double value) {
            if ( options_ && options_->min_rotation != (value *= M_PI / 180) ) {
              options_->min_rotation = value;
              emit parameterChanged();
            }
          }));

  controls.append(max_rotation_ctl =
      add_numeric_box<double>("max_rotation [deg]:",
          [this](double value) {
            if ( options_ && options_->max_rotation != (value *= M_PI / 180) ) {
              options_->max_rotation = value;
              emit parameterChanged();
            }
          }));

  controls.append(eccflow_support_scale_ctl =
      add_numeric_box<int>("eccflow_support_scale:",
          [this](double value) {
            if ( options_ && options_->eccflow_support_scale != value ) {
              options_->eccflow_support_scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(eccflow_normalization_scale_ctl =
      add_numeric_box<int>("eccflow_normalization_scale:",
          [this](double value) {
            if ( options_ && options_->eccflow_normalization_scale != value ) {
              options_->eccflow_normalization_scale = value;
              emit parameterChanged();
            }
          }));

  controls.append(eccflow_max_pyramid_level_ctl =
      add_numeric_box<int>("eccflow_max_pyramid_level:",
          [this](double value) {
            if ( options_ && options_->eccflow_max_pyramid_level != value ) {
              options_->eccflow_max_pyramid_level = value;
              emit parameterChanged();
            }
          }));

  controls.append(align_jovian_disk_horizontally_ctl =
      add_checkbox("align_jovian_disk_horizontally",
          [this](bool checked) {
            if ( options_ && options_->rotate_jovian_disk_horizontally != checked ) {
              options_->rotate_jovian_disk_horizontally = checked;
              emit parameterChanged();
            }
          }));
}

void QJovianDerotationOptions::set_derotation_options(c_jovian_derotation_options * options)
{
  options_ = options;
  updateControls();
}

c_jovian_derotation_options* QJovianDerotationOptions::derotation_options() const
{
  return options_;
}

void QJovianDerotationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableJovianDerotation_ctl->setChecked(options_->enabled);
    min_rotation_ctl->setValue(options_->min_rotation * 180 / M_PI);
    max_rotation_ctl->setValue(options_->max_rotation * 180 / M_PI);
    eccflow_support_scale_ctl->setValue(options_->eccflow_support_scale);
    eccflow_normalization_scale_ctl->setValue(options_->eccflow_normalization_scale);
    eccflow_max_pyramid_level_ctl->setValue(options_->eccflow_max_pyramid_level);
    align_jovian_disk_horizontally_ctl->setChecked(options_->rotate_jovian_disk_horizontally);

    update_controls_state();

    setEnabled(true);
  }
}

void QJovianDerotationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageRegistrationOptions::QImageRegistrationOptions(QWidget * parent) :
    Base("QImageRegistrationOptions", parent)
{
  motion_type_ctl =
      add_enum_combobox<ECC_MOTION_TYPE>("Motion type:",
          [this](ECC_MOTION_TYPE value) {
            if ( options_ && options_->image_registration_options.motion_type != value ) {
              options_->image_registration_options.motion_type = value;
              emit parameterChanged();
            }
          });

  registration_channel_ctl =
      add_enum_combobox<color_channel_type>(
          "Registration channel:",
          [this](color_channel_type value) {
            if ( options_ && options_->image_registration_options.registration_channel != value ) {
              options_->image_registration_options.registration_channel = value;
              emit parameterChanged();
            }
          });

  interpolation_method_ctl =
      add_enum_combobox<ECC_INTERPOLATION_METHOD>(
          "Interpolation method:",
          [this](ECC_INTERPOLATION_METHOD value) {
            if ( options_ && options_->image_registration_options.interpolation != value) {
              options_->image_registration_options.interpolation = value;
              emit parameterChanged();
            }
          });

  border_mode_ctl =
      add_enum_combobox<ECC_BORDER_MODE>(
          "Border mode:",
          [this](ECC_BORDER_MODE value) {
            if ( options_ && options_->image_registration_options.border_mode != value ) {
              options_->image_registration_options.border_mode = value;
              emit parameterChanged();
            }
          });

  border_value_ctl =
      add_numeric_box<cv::Scalar>(
          "Border Value",
          [this](const cv::Scalar & value) {
            if ( options_ && options_->image_registration_options.border_value != value ) {
              options_->image_registration_options.border_value = value;
              emit parameterChanged();
            }
          });

  ///

  add_expandable_groupbox("Master Frame Options",
      masterFrameOptions_ctl = new QMasterFrameOptions(this));

  connect(masterFrameOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Feature Based Registration Options",
      featureRegistrationOptions_ctl = new QFeatureBasedRegistrationOptions(this));
  connect(featureRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("ECC Registration Options",
      eccOptions_ctl = new QEccRegistrationOptions(this));
  connect(eccOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("ECC Flow Registration Options",
      eccFlowOptions_ctl = new QEccFlowRegistrationOptions(this));
  connect(eccFlowOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Jovian Derotation Options",
      jovianDerotationOptions_ctl = new QJovianDerotationOptions(this));
  connect(jovianDerotationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
}

void QImageRegistrationOptions::set_stack_options(const c_image_stacking_options::ptr & stack_options)
{
  stack_options_ = stack_options;
  options_ = stack_options_ ? &stack_options_->frame_registration_options() : nullptr;
  updateControls();
}

const c_image_stacking_options::ptr & QImageRegistrationOptions::stack_options() const
{
  return stack_options_;
}

void QImageRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    motion_type_ctl->setValue(options_->image_registration_options.motion_type);
    registration_channel_ctl->setValue(options_->image_registration_options.registration_channel);
    interpolation_method_ctl->setValue(options_->image_registration_options.interpolation);
    border_mode_ctl->setValue(options_->image_registration_options.border_mode);
    border_value_ctl->setValue(options_->image_registration_options.border_value);

    masterFrameOptions_ctl->set_master_frame_options(&options_->master_frame_options, stack_options_->input_sequence());
    featureRegistrationOptions_ctl->set_registration_options(&options_->image_registration_options.feature_registration);
    eccOptions_ctl->set_registration_options(&options_->image_registration_options.ecc);
    eccFlowOptions_ctl->set_registration_options(&options_->image_registration_options.eccflow);
    jovianDerotationOptions_ctl->set_derotation_options(&options_->image_registration_options.jovian_derotation);

    setEnabled(true);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent) :
    Base("QFrameRegistrationOptions", parent)
{
  enable_frame_registration_ctl =
      add_checkbox("Enable image registration",
          [this](bool checked) {
            if ( options_ && options_->image_registration_options.enable_frame_registration != checked ) {
              options_->image_registration_options.enable_frame_registration = checked;
              update_controls_visibility();
              emit parameterChanged();
            }
          });

  imageRegistrationOptions_ctl =
      add_widget<QImageRegistrationOptions>();

  connect(imageRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  accumulateAndCompensateTurbulentFlow_ctl =
      add_checkbox("accumulate and compensate turbulent flow",
          [this](bool checked) {
            if ( options_ && options_->accumulate_and_compensate_turbulent_flow != checked ) {
              options_->accumulate_and_compensate_turbulent_flow = checked;
              emit parameterChanged();
            }
          });

  alignedFramesPostProcessor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("PostProcess aligned frames:",
          [this](int index) {
            if ( options_ ) {
              options_->aligned_frame_postprocessor =
              alignedFramesPostProcessor_ctl->processor(index);
              emit parameterChanged();
            }
          });

  update_controls_visibility();
}

void QFrameRegistrationOptions::set_stack_options(const c_image_stacking_options::ptr & stack_options)
{
  stack_options_ = stack_options;
  options_ = stack_options_ ? &stack_options_->frame_registration_options() : nullptr;
  updateControls();
}

const c_image_stacking_options::ptr & QFrameRegistrationOptions::stack_options() const
{
  return stack_options_;
}

void QFrameRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    enable_frame_registration_ctl->setChecked(options_->image_registration_options.
        enable_frame_registration);

    imageRegistrationOptions_ctl->set_stack_options(
        stack_options_);

    accumulateAndCompensateTurbulentFlow_ctl->setChecked(
        options_->accumulate_and_compensate_turbulent_flow);

    if( !alignedFramesPostProcessor_ctl->setCurrentProcessor(options_->aligned_frame_postprocessor) ) {
      options_->aligned_frame_postprocessor.reset();
    }

    update_controls_visibility();

    setEnabled(true);
  }
}

void QFrameRegistrationOptions::update_controls_visibility()
{
  if( !options_ ) {
    imageRegistrationOptions_ctl->setVisible(false);
    accumulateAndCompensateTurbulentFlow_ctl->setEnabled(false);
    alignedFramesPostProcessor_ctl->setEnabled(false);
  }
  else {
    imageRegistrationOptions_ctl->setVisible(options_->image_registration_options.enable_frame_registration);
    accumulateAndCompensateTurbulentFlow_ctl->setEnabled(options_->image_registration_options.enable_frame_registration);
    alignedFramesPostProcessor_ctl->setEnabled(options_->image_registration_options.enable_frame_registration);
  }
}




