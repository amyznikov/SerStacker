/*
 * QFeature2dOptions.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */
#include "QFeature2dOptions.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QSparseFeatureDetectorOptions::QSparseFeatureDetectorOptions(QWidget * parent) :
    Base("QSparseFeatureDetectorOptions", parent)
{
  detectorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DETECTOR_TYPE>(
          "DETECTOR_TYPE",
          "",
          [this](SPARSE_FEATURE_DETECTOR_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              update_detector_specific_controls();
              Q_EMIT detectorTypeChanged();
              Q_EMIT parameterChanged();
            }
          });

  max_keypoints_ctl =
      add_numeric_box<int>("Max. key points:",
          "",
          [this](int value) {
            if ( options_ && options_->max_keypoints != value ) {
              options_->max_keypoints = value;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
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
        "", \
        [this](decltype(options_->f.name) v){ \
          if ( options_ ) { \
            options_->f.name = v; \
            Q_EMIT parameterChanged(); \
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

    Q_EMIT populatecontrols();

#undef ADD_CTL
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QSparseFeatureDescriptorOptions::QSparseFeatureDescriptorOptions(QWidget * parent) :
    Base("QSparseFeatureDescriptorOptions", parent)
{
  useDetectorSettings_ctl =
      add_checkbox("Use detector settings",
          "",
          [this](bool checked) {
            if ( options_ && options_->use_detector_options != checked ) {
              options_->use_detector_options = checked;
              update_descriptor_specific_controls();
              Q_EMIT parameterChanged();
            }
          });

  descriptorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DESCRIPTOR_TYPE>(
          "DESCRIPTOR_TYPE",
          "",
          [this](SPARSE_FEATURE_DESCRIPTOR_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              update_descriptor_specific_controls();
              Q_EMIT parameterChanged();
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
        "", \
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

    Q_EMIT populatecontrols();
#undef ADD_CTL
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QHammingDistanceFeature2dMatcherOptions::QHammingDistanceFeature2dMatcherOptions(QWidget * parent) :
    Base("QHammingDistanceFeature2dMatcherOptions", parent)
{
  max_acceptable_distance_ctl =
      add_numeric_box<int>("max_acceptable_distance",
          "",
          [this](int value) {
            if ( options_ && options_->max_acceptable_distance != value ) {
              options_->max_acceptable_distance = value;
              Q_EMIT parameterChanged();
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
          "",
          [this](FlannIndexType value) {
            if ( options_ && options_->index.type != value ) {
              options_->index.type = value;
              update_matcher_specific_controls();
              Q_EMIT parameterChanged();
            }
          });

  flannDistanceType_ctl =
      add_enum_combobox<cvflann::flann_distance_t>("Flann Distance Type:",
          "",
          [this](cvflann::flann_distance_t value) {
            if ( options_ && options_->distance_type != value ) {
              options_->distance_type = value;
              Q_EMIT parameterChanged();
            }
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio",
          "",
          [this](double value) {
            if ( options_ && options_->lowe_ratio != value ) {
              options_->lowe_ratio = value;
              Q_EMIT parameterChanged();
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
        "", \
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

    Q_EMIT populatecontrols();

#undef ADD_FLANN_INDEX_CTL
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QTriangleMatcherOptions::QTriangleMatcherOptions(QWidget * parent) :
    Base("QTriangleMatcherOptions", parent)
{
  eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              Q_EMIT parameterChanged();
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
          "",
          [this](int value) {
            if ( options_ && options_->max_acceptable_distance != value ) {
              options_->max_acceptable_distance = value;
              Q_EMIT parameterChanged();
            }
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio:",
          "",
          [this](int value) {
            if ( options_ && options_->lowe_ratio != value ) {
              options_->lowe_ratio = value;
              Q_EMIT parameterChanged();
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
          "",
          [this](FEATURE2D_MATCHER_TYPE value) {
            if ( options_ && options_->type != value ) {
              options_->type = value;
              show_current_matcher_controls();
              Q_EMIT parameterChanged();
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

