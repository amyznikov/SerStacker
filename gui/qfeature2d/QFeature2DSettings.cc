/*
 * QFeature2DSettings.cc
 *
 *  Created on: Mar 29, 2022
 *      Author: amyznikov
 */
#include "QFeature2DSettings.h"


#define ADD_CTL(f, name) \
    controls_.append(add_ctl<decltype(options_->f.name)>(#name, \
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

#define ADD_FLANN_INDEX_CTL(f, name) \
    controls_.append(add_ctl<decltype(options_->flann.index.f.name)>(#name, \
        [this](decltype(options_->flann.index.f.name) v){ \
          if ( options_ ) { \
            options_->flann.index.f.name = v; \
          }}, \
        [this](decltype(options_->flann.index.f.name) * v) -> bool { \
          if ( options_ ) { \
            *v = options_->flann.index.f.name; \
            return true; \
          } \
          return false; \
        } \
    ))

///////////////////////////////////////////////////////////////////////

QSparseFeatureDetectorSettingsWidget::QSparseFeatureDetectorSettingsWidget(QWidget * parent) :
    Base("QFeatureExtractorSettings", parent)
{
  featureExtractorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DETECTOR_TYPE>(
          "Sparse Feature extractor:",
          [this](SPARSE_FEATURE_DETECTOR_TYPE v) {
            if ( options_ && options_->type != v ) {
              options_->type = v;
              updateFeatureExtractorSpecificControls();
            }
          });
}

void QSparseFeatureDetectorSettingsWidget::set_feature_detector_options(c_sparse_feature_detector_options * opts)
{
  clearFeatureExtractorSpecificControls();
  this->options_ = opts;
  updateControls();
}

const c_sparse_feature_detector_options * QSparseFeatureDetectorSettingsWidget::feature_detector_options() const
{
  return options_;
}

void QSparseFeatureDetectorSettingsWidget::updateFeatureExtractorSpecificControls()
{
  clearFeatureExtractorSpecificControls();
  populateFeatureExtractorSpecificControls();
}

void QSparseFeatureDetectorSettingsWidget::clearFeatureExtractorSpecificControls()
{
  for ( QWidget * w : controls_ ) {
    form->removeRow(w);
  }
  controls_.clear();
}


void QSparseFeatureDetectorSettingsWidget::populateFeatureExtractorSpecificControls()
{
  if( options_ ) {
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
  }
}

void QSparseFeatureDetectorSettingsWidget::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
    clearFeatureExtractorSpecificControls();
  }
  else {
    featureExtractorType_ctl->setCurrentItem(options_->type);
    populateFeatureExtractorSpecificControls();
    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////

QSparseDescriptorExtractorSettingsWidget::QSparseDescriptorExtractorSettingsWidget(QWidget * parent) :
    Base("QSparseDescriporExtractorSettings", parent)
{
  useDetectorSettings_ctl =
      add_checkbox("Use descriptors from feature detector",
          [this](bool checked) {
              if ( options_  ) {
                options_->use_detector_options = checked;
                descriptorExtractorType_ctl->setEnabled(!options_->use_detector_options);
                updateDescriptorExtractorSpecificControls();
              }
          });

  descriptorExtractorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DESCRIPTOR_TYPE>(
          "Sparse Descriptor extractor:",
          [this](SPARSE_FEATURE_DESCRIPTOR_TYPE v) {
            if ( options_ && options_->type != v ) {
              options_->type = v;
              updateDescriptorExtractorSpecificControls();
            }
          });

}

void QSparseDescriptorExtractorSettingsWidget::set_feature_descriptor_options(c_sparse_feature_descriptor_options * opts)
{
  clearDescriptorExtractorSpecificControls();
  options_ = opts;
  updateControls();
}

const c_sparse_feature_descriptor_options * QSparseDescriptorExtractorSettingsWidget::feature_descriptor_options() const
{
  return options_;
}

void QSparseDescriptorExtractorSettingsWidget::clearDescriptorExtractorSpecificControls()
{
  for ( QWidget * w : controls_ ) {
    form->removeRow(w);
  }
  controls_.clear();
}

void QSparseDescriptorExtractorSettingsWidget::populateDescriptorExtractorSpecificControls()
{
  if( options_ && !options_->use_detector_options ) {

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
  }
}

void QSparseDescriptorExtractorSettingsWidget::updateDescriptorExtractorSpecificControls()
{
  clearDescriptorExtractorSpecificControls();
  populateDescriptorExtractorSpecificControls();
}

void QSparseDescriptorExtractorSettingsWidget::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
    clearDescriptorExtractorSpecificControls();
  }
  else {
    useDetectorSettings_ctl->setChecked(options_->use_detector_options);
    descriptorExtractorType_ctl->setEnabled(!options_->use_detector_options);
    descriptorExtractorType_ctl->setCurrentItem(options_->type);
    populateDescriptorExtractorSpecificControls();
    setEnabled(true);
  }
}


///////////////////////////////////////////////////////////////////////

QSparseFeatureExtractorSettingsWidget::QSparseFeatureExtractorSettingsWidget(QWidget * parent) :
    Base("QSparseFeatureExtractorSettings", parent)
{
  add_expandable_groupbox("Sparse Feature Detector Options",
      detectorSettings_ctl = new QSparseFeatureDetectorSettingsWidget(this));

  add_expandable_groupbox("Sparse Feature Descriptor Options",
      descriptorSettings_ctl = new QSparseDescriptorExtractorSettingsWidget(this));
}

void QSparseFeatureExtractorSettingsWidget::set_sparse_feature_extractor_options(c_sparse_feature_extractor_options * opts)
{
  options_ = opts;
  updateControls();
}

const c_sparse_feature_extractor_options* QSparseFeatureExtractorSettingsWidget::sparse_feature_extractor_options() const
{
  return options_;
}

void QSparseFeatureExtractorSettingsWidget::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    detectorSettings_ctl->set_feature_detector_options(&options_->detector);
    descriptorSettings_ctl->set_feature_descriptor_options(&options_->descriptor);

    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////

QSparseFeature2DMatcherSettingsWidget::QSparseFeature2DMatcherSettingsWidget(QWidget * parent) :
    Base("QSparseFeature2DMatcherSettings", parent)
{
  matcherType_ctl =
      add_enum_combobox<FEATURE2D_MATCHER_TYPE>(
          "Sparse Feature Matcher:",
          [this](FEATURE2D_MATCHER_TYPE v) {
            if ( options_ && options_->type != v ) {
              options_->type = v;
              updateMatcherSpecificControls();
            }
          });
}

void QSparseFeature2DMatcherSettingsWidget::set_feature2d_matcher_options(c_feature2d_matcher_options * opts)
{
  clearMatcherSpecificControls();
  options_ = opts;
  updateControls();
}

const c_feature2d_matcher_options * QSparseFeature2DMatcherSettingsWidget::feature2d_matcher_options() const
{
  return options_;
}

void QSparseFeature2DMatcherSettingsWidget::clearMatcherSpecificControls()
{
  for ( QWidget * w : controls_ ) {
    form->removeRow(w);
  }
  controls_.clear();
}

void QSparseFeature2DMatcherSettingsWidget::populateMatcherSpecificControls()
{
  if( options_ ) {

    switch (options_->type) {
    case FEATURE2D_MATCHER_HAMMING:
      ADD_CTL(hamming, max_acceptable_distance);
      break;

    case FEATURE2D_MATCHER_SNORM:
      ADD_CTL(snorm, max_acceptable_distance);
      ADD_CTL(snorm, lowe_ratio);
      break;

    case FEATURE2D_MATCHER_FLANN: {
      QFlannIndexParamsWidget *flannIndexParams_ctl = new QFlannIndexParamsWidget(this);
      flannIndexParams_ctl->set_feature2d_matcher_options(options_);
      form->addRow(flannIndexParams_ctl);
      controls_.append(flannIndexParams_ctl);
      break;
      }
    }

    emit populatecontrols();
  }
}

void QSparseFeature2DMatcherSettingsWidget::updateMatcherSpecificControls()
{
  clearMatcherSpecificControls();
  populateMatcherSpecificControls();
}

void QSparseFeature2DMatcherSettingsWidget::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
    clearMatcherSpecificControls();
  }
  else {
    matcherType_ctl->setCurrentItem(options_->type);
    populateMatcherSpecificControls();
    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////

QFlannIndexParamsWidget::QFlannIndexParamsWidget(QWidget * parent) :
     Base("QFlannIndexParams", parent)
{
  flannIndexType_ctl =
      add_enum_combobox<FlannIndexType>(
          "Flann Index Type:",
          [this](FlannIndexType v) {
            if ( options_ && options_->flann.index.type != v ) {
              options_->flann.index.type = v;
              updateIndexParamsSpecificControls();
            }
          });
}

void QFlannIndexParamsWidget::set_feature2d_matcher_options(c_feature2d_matcher_options * opts)
{
  clearIndexParamsSpecificControls();
  options_ = opts;
  updateControls();
}

const c_feature2d_matcher_options * QFlannIndexParamsWidget::feature2d_matcher_options() const
{
  return options_;
}

void QFlannIndexParamsWidget::clearIndexParamsSpecificControls()
{
  for ( QWidget * w : controls_ ) {
    form->removeRow(w);
  }
  controls_.clear();
}

void QFlannIndexParamsWidget::populateIndexParamsSpecificControls()
{
  if( options_ && options_->type == FEATURE2D_MATCHER_FLANN ) {

    ADD_CTL(flann, distance_type);
    ADD_CTL(flann, lowe_ratio);

    switch (options_->flann.index.type) {
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
  }
}

void QFlannIndexParamsWidget::updateIndexParamsSpecificControls()
{
  clearIndexParamsSpecificControls();
  populateIndexParamsSpecificControls();
}

void QFlannIndexParamsWidget::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
    clearIndexParamsSpecificControls();
  }
  else {
    flannIndexType_ctl->setCurrentItem(options_->flann.index.type);
    populateIndexParamsSpecificControls();
    setEnabled(true);
  }
}

