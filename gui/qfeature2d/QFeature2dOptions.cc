/*
 * QFeature2dOptions.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */
#include "QFeature2dOptions.h"
#include <core/feature2d/feature_extraction.h>
#include <core/ssprintf.h>
#include <core/debug.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ADDCTL2(name) \
  w->add_ctl<decltype(w->opts()->name)>(#name, \
      "", \
      [w](decltype(w->opts()->name) v){ \
        if ( w->opts() ) { \
          w->opts()->name = v; \
          Q_EMIT w->parameterChanged(); \
        }}, \
      [w](decltype(w->opts()->name) * v) -> bool { \
        if ( w->opts() ) { \
          *v = w->opts()->name; \
          return true; \
        } \
        return false; \
      } \
    )


static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_orb::options> * w)
{
  ADDCTL2(nfeatures);
  ADDCTL2(scaleFactor);
  ADDCTL2(nlevels);
  ADDCTL2(edgeThreshold);
  ADDCTL2(firstLevel);
  ADDCTL2(WTA_K);
  ADDCTL2(patchSize);
  ADDCTL2(fastThreshold);
  ADDCTL2(scoreType);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_brisk::options> * w)
{
  ADDCTL2(thresh);
  ADDCTL2(octaves);
  ADDCTL2(patternScale);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_kaze::options> * w)
{
  ADDCTL2(extended);
  ADDCTL2(upright);
  ADDCTL2(threshold);
  ADDCTL2(nOctaves);
  ADDCTL2(nOctaveLayers);
  ADDCTL2(diffusivity);
}

//  akaze
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_akaze::options> * w)
{
  ADDCTL2(descriptor_type);
  ADDCTL2(descriptor_size);
  ADDCTL2(descriptor_channels);
  ADDCTL2(threshold);
  ADDCTL2(nOctaves);
  ADDCTL2(nOctaveLayers);
  ADDCTL2(diffusivity);
}

//  sift
#if HAVE_FEATURE2D_SIFT
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_sift::options> * w)
{
  ADDCTL2(nfeatures);
  ADDCTL2(nOctaveLayers);
  ADDCTL2(contrastThreshold);
  ADDCTL2(edgeThreshold);
  ADDCTL2(sigma);
}
#endif

//  surf
#if HAVE_FEATURE2D_SURF
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_surf::options> * w)
{
  ADDCTL2(hessianThreshold);
  ADDCTL2(nOctaves);
  ADDCTL2(nOctaveLayers);
  ADDCTL2(extended);
  ADDCTL2(upright);
}
#endif

//  mser
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_mser::options> * w)
{
  ADDCTL2(delta);
  ADDCTL2(min_area);
  ADDCTL2(max_area);
  ADDCTL2(max_variation);
  ADDCTL2(min_diversity);
  ADDCTL2(max_evolution);
  ADDCTL2(area_threshold);
  ADDCTL2(min_margin);
  ADDCTL2(edge_blur_size);
}

//fast;
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_fast::options> * w)
{
  ADDCTL2(threshold);
  ADDCTL2(nonmaxSuppression);
  ADDCTL2(type);
}

//agast;
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_agast::options > * w)
{
  ADDCTL2(threshold);
  ADDCTL2(nonmaxSuppression);
  ADDCTL2(type);
}

//gftt;
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_gftt::options > * w)
{
  ADDCTL2(maxCorners);
  ADDCTL2(qualityLevel);
  ADDCTL2(minDistance);
  ADDCTL2(blockSize);
  ADDCTL2(gradiantSize);
  ADDCTL2(k);
  ADDCTL2(useHarrisDetector);
}

//blob;
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_blob::options> * w)
{
  ADDCTL2(thresholdStep);
  ADDCTL2(minThreshold);
  ADDCTL2(maxThreshold);
  ADDCTL2(minRepeatability);
  ADDCTL2(minDistBetweenBlobs);
  ADDCTL2(filterByColor);
  ADDCTL2(blobColor);
  ADDCTL2(filterByArea);
  ADDCTL2(minArea);
  ADDCTL2(maxArea);
  ADDCTL2(filterByCircularity);
  ADDCTL2(minCircularity);
  ADDCTL2(maxCircularity);
  ADDCTL2(filterByInertia);
  ADDCTL2(minInertiaRatio);
  ADDCTL2(maxInertiaRatio);
  ADDCTL2(filterByConvexity);
  ADDCTL2(minConvexity);
  ADDCTL2(maxConvexity);
}

//star;
#if HAVE_FEATURE2D_STAR
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_star::options > * w)
{
  ADDCTL2(maxSize);
  ADDCTL2(responseThreshold);
  ADDCTL2(lineThresholdProjected);
  ADDCTL2(lineThresholdBinarized);
  ADDCTL2(suppressNonmaxSize);
}
#endif

//morph;
#if HAVE_MORPH_EXTRACTOR
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_morph_extractor::options > * w)
{
  ADDCTL2(morph_type);
  ADDCTL2(threshold);
  ADDCTL2(se_radius);
}
#endif

//sex;
#if HAVE_STAR_EXTRACTOR
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_star_extractor::options > * w)
{
  ADDCTL2(median_filter_size);
  ADDCTL2(sigma1);
  ADDCTL2(sigma2);
  ADDCTL2(noise_blur);
  ADDCTL2(noise_threshold);
  ADDCTL2(min_score);
  ADDCTL2(min_pts);
  ADDCTL2(min_b);
  ADDCTL2(min_ba_ratio);
}
#endif

//msd;
#if HAVE_FEATURE2D_MSD
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_msd::options > * w)
{
  ADDCTL2(m_patch_radius);
  ADDCTL2(m_search_area_radius);
  ADDCTL2(m_nms_radius);
  ADDCTL2(m_nms_scale_radius);
  ADDCTL2(m_th_saliency);
  ADDCTL2(m_kNN);
  ADDCTL2(m_scale_factor);
  ADDCTL2(m_n_scales);
  ADDCTL2(m_compute_orientation);
}
#endif

//hl;
#if HAVE_FEATURE2D_HL
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_hl::options > * w)
{
  ADDCTL2(numOctaves);
  ADDCTL2(corn_thresh);
  ADDCTL2(DOG_thresh);
  ADDCTL2(maxCorners);
  ADDCTL2(num_layers);
}
#endif

//planetary_disk_detector;
#if HAVE_SIMPLE_PLANETARY_DISK_DETECTOR
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_planetary_disk_detector::options> * w)
{
  ADDCTL2(gbsigma);
  ADDCTL2(stdev_factor);
  ADDCTL2(se_close_size);
  ADDCTL2(align_planetary_disk_masks);
}
#endif

// freak;
#if HAVE_FEATURE2D_FREAK
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_freak::options> * w)
{
  ADDCTL2(orientationNormalized);
  ADDCTL2(scaleNormalized);
  ADDCTL2(patternScale);
  ADDCTL2(nOctaves);
}
#endif

// brief;
#if HAVE_FEATURE2D_BRIEF
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_brief::options> * w)
{
  ADDCTL2(bytes);
  ADDCTL2(use_orientation);
}
#endif

// lucid;
#if HAVE_FEATURE2D_LUCID
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_lucid::options> * w)
{
  ADDCTL2(lucid_kernel);
  ADDCTL2(blur_kernel);
}
#endif

// latch;
#if HAVE_FEATURE2D_LATCH
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_latch::options> * w)
{
  ADDCTL2(bytes);
  ADDCTL2(rotationInvariance);
  ADDCTL2(half_ssd_size);
  ADDCTL2(sigma);
}
#endif

// daisy;
#if HAVE_FEATURE2D_DAISY
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_daisy::options> * w)
{
  ADDCTL2(radius);
  ADDCTL2(q_radius);
  ADDCTL2(q_theta);
  ADDCTL2(q_hist);
  ADDCTL2(norm);
  ADDCTL2(interpolation);
  ADDCTL2(use_orientation);
}
#endif

// vgg;
#if HAVE_FEATURE2D_VGG
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_vgg::options> * w)
{
  ADDCTL2(desc);
  ADDCTL2(isigma);
  ADDCTL2(img_normalize);
  ADDCTL2(use_scale_orientation);
  ADDCTL2(scale_factor);
  ADDCTL2(dsc_normalize);

}
#endif

// boost;
#if HAVE_FEATURE2D_BOOST
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_boost::options> * w)
{
  ADDCTL2(desc);
  ADDCTL2(use_scale_orientation);
  ADDCTL2(scale_factor);
}
#endif

// triangles;
#if HAVE_TRIANGLE_EXTRACTOR
static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_feature2d_triangle_extractor::options> * w)
{
  ADDCTL2(max_points);
  ADDCTL2(min_side_size);
}
#endif


static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_linear_index_options> * w)
{
  (void)(w);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_kdtree_index_options> * w)
{
  ADDCTL2(trees);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_kmeans_index_options> * w)
{
  ADDCTL2(centers_init);
  ADDCTL2(branching);
  ADDCTL2(iterations);
  ADDCTL2(cb_index);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_composite_index_options> * w)
{
  ADDCTL2(centers_init);
  ADDCTL2(trees);
  ADDCTL2(branching);
  ADDCTL2(iterations);
  ADDCTL2(cb_index);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_hierarchical_index_options> * w)
{
  ADDCTL2(centers_init);
  ADDCTL2(branching);
  ADDCTL2(trees);
  ADDCTL2(leaf_size);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_lsh_index_options> * w)
{
  ADDCTL2(table_number);
  ADDCTL2(key_size);
  ADDCTL2(multi_probe_level);
}

static inline void populate_feature2d_options(QSettingsWidgetTemplate<c_flann_autotuned_index_options> * w)
{
  ADDCTL2(target_precision);
  ADDCTL2(build_weight);
  ADDCTL2(memory_weight);
  ADDCTL2(sample_fraction);
}


template<class F>
void QSparseFeatureDetectorOptions::addStackWidget(F c_sparse_feature_detector_options::* mp)
{
  auto w = new QSparseFeature2DOptionsTemplate<typename F::feature2d_class>(this);
  _stack->addWidget(w);

  QObject::connect(this, &ThisClass::populatecontrols,
      [this, w, mp]() {
        w->setOpts(this->_opts ? &(this->_opts->*mp) : nullptr);
      });
}


QSparseFeatureDetectorOptions::QSparseFeatureDetectorOptions(QWidget * parent) :
    Base(parent)
{

  detectorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DETECTOR_TYPE>(
          "DETECTOR_TYPE",
          "",
          [this](SPARSE_FEATURE_DETECTOR_TYPE value) {
            if ( _opts && _opts->type != value ) {
              _opts->type = value;
              updateDetectorSpecificControls();
              Q_EMIT detectorTypeChanged();
              Q_EMIT parameterChanged();
            }
          },
          [this](SPARSE_FEATURE_DETECTOR_TYPE * value) {
            if ( _opts ) {
              *value = _opts->type;
              return true;
            }
            return false;
          });

  max_keypoints_ctl =
      add_numeric_box<int>("Max. key points:",
          "",
          [this](int value) {
            if ( _opts && _opts->max_keypoints != value ) {
              _opts->max_keypoints = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_keypoints;
              return true;
            }
            return false;
          });



  ThisClass::addRow(_stack = new QStackedWidget(this));

  addStackWidget(&c_sparse_feature_detector_options::orb);
  addStackWidget(&c_sparse_feature_detector_options::brisk);
  addStackWidget(&c_sparse_feature_detector_options::kaze);
  addStackWidget(&c_sparse_feature_detector_options::akaze);
  addStackWidget(&c_sparse_feature_detector_options::mser);
  addStackWidget(&c_sparse_feature_detector_options::fast);
  addStackWidget(&c_sparse_feature_detector_options::agast);
  addStackWidget(&c_sparse_feature_detector_options::gftt);
  addStackWidget(&c_sparse_feature_detector_options::blob);
#if HAVE_FEATURE2D_SIFT
  addStackWidget(&c_sparse_feature_detector_options::sift);
#endif
#if HAVE_FEATURE2D_SURF
  addStackWidget(&c_sparse_feature_detector_options::surf);
#endif
#if HAVE_FEATURE2D_STAR
  addStackWidget(&c_sparse_feature_detector_options::star);
#endif
#if HAVE_MORPH_EXTRACTOR
  addStackWidget(&c_sparse_feature_detector_options::morph);
#endif
#if HAVE_STAR_EXTRACTOR
  addStackWidget(&c_sparse_feature_detector_options::sex);
#endif
#if HAVE_FEATURE2D_MSD
  addStackWidget(&c_sparse_feature_detector_options::msd);
#endif
#if HAVE_FEATURE2D_HL
  addStackWidget(&c_sparse_feature_detector_options::hl);
#endif
#if HAVE_SIMPLE_PLANETARY_DISK_DETECTOR
  addStackWidget(&c_sparse_feature_detector_options::planetary_disk_detector);
#endif

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        QSettingsWidget * currentSettings = dynamic_cast<QSettingsWidget * >(_stack->currentWidget());
        if ( currentSettings ) {
          currentSettings->setEnabled(_opts != nullptr);
          if ( _opts )  {
            currentSettings->enablecontrols();
          }
        }
  });

  QObject::connect(this, &ThisClass::populatecontrols,
      this, &ThisClass::updateDetectorSpecificControls);

  updateControls();
}

void QSparseFeatureDetectorOptions::updateDetectorSpecificControls()
{
  if ( _opts ) {
    QSettingsWidget * currentSettings =
        _stack->findChild<QSettingsWidget*>(QString::fromStdString(toString(_opts->type)),
            Qt::FindDirectChildrenOnly);

    if ( !currentSettings ) {
      _stack->setEnabled(false);
    }
    else {
      _stack->setCurrentWidget(currentSettings);
      _stack->setEnabled(true);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<class F>
void QSparseFeatureDescriptorOptions::addStackWidget(F c_sparse_feature_descriptor_options::* mp)
{
  auto w = new QSparseFeature2DOptionsTemplate<typename F::feature2d_class>(this);
  _stack->addWidget(w);

  QObject::connect(this, &ThisClass::populatecontrols,
      [this, w, mp]() {
        w->setOpts(this->_opts ? &(this->_opts->*mp) : nullptr);
      });
}


QSparseFeatureDescriptorOptions::QSparseFeatureDescriptorOptions(QWidget * parent) :
    Base(parent)
{
  descriptorType_ctl =
      add_enum_combobox<SPARSE_FEATURE_DESCRIPTOR_TYPE>(
          "DESCRIPTOR_TYPE",
          "",
          [this](SPARSE_FEATURE_DESCRIPTOR_TYPE value) {
            if ( _opts && _opts->type != value ) {
              _opts->type = value;
              updateDescriptorSpecificControls();
              Q_EMIT parameterChanged();
            }
          },
          [this](SPARSE_FEATURE_DESCRIPTOR_TYPE * value) {
            if ( _opts ) {
              *value = _opts->type;
              return true;
            }
            return false;
          }
      );

  ThisClass::addRow(_stack = new QStackedWidget(this));

  addStackWidget(&c_sparse_feature_descriptor_options::orb);
  addStackWidget(&c_sparse_feature_descriptor_options::brisk);
  addStackWidget(&c_sparse_feature_descriptor_options::kaze);
  addStackWidget(&c_sparse_feature_descriptor_options::akaze);
#if HAVE_FEATURE2D_SIFT
  addStackWidget(&c_sparse_feature_descriptor_options::sift);
#endif
#if HAVE_FEATURE2D_SURF
  addStackWidget(&c_sparse_feature_descriptor_options::surf);
#endif
#if HAVE_FEATURE2D_FREAK
  addStackWidget(&c_sparse_feature_descriptor_options::freak);
#endif
#if HAVE_FEATURE2D_BRIEF
  addStackWidget(&c_sparse_feature_descriptor_options::brief);
#endif
#if HAVE_FEATURE2D_LUCID
  addStackWidget(&c_sparse_feature_descriptor_options::lucid);
#endif
#if HAVE_FEATURE2D_LATCH
  addStackWidget(&c_sparse_feature_descriptor_options::latch);
#endif
#if HAVE_FEATURE2D_DAISY
  addStackWidget(&c_sparse_feature_descriptor_options::daisy);
#endif
#if HAVE_FEATURE2D_VGG
  addStackWidget(&c_sparse_feature_descriptor_options::vgg);
#endif
#if HAVE_FEATURE2D_BOOST
  addStackWidget(&c_sparse_feature_descriptor_options::boost);
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  addStackWidget(&c_sparse_feature_descriptor_options::triangles);
#endif

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        QSettingsWidget * currentSettings = dynamic_cast<QSettingsWidget * >(_stack->currentWidget());
        if ( currentSettings ) {
          currentSettings->setEnabled(_opts != nullptr);
          if ( _opts )  {
            currentSettings->enablecontrols();
          }
        }
  });

  QObject::connect(this, &ThisClass::populatecontrols,
      this, &ThisClass::updateDescriptorSpecificControls);

  updateControls();
  //updateDescriptorSpecificControls();
}

void QSparseFeatureDescriptorOptions::updateDescriptorSpecificControls()
{
  if ( _opts ) {
    QSettingsWidget * currentSettings =
        _stack->findChild<QSettingsWidget*>(QString::fromStdString(toString(_opts->type)),
            Qt::FindDirectChildrenOnly);

    if ( !currentSettings ) {
      _stack->setEnabled(false);
    }
    else {
      _stack->setCurrentWidget(currentSettings);
      _stack->setEnabled(true);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QHammingDistanceFeature2dMatcherOptions::QHammingDistanceFeature2dMatcherOptions(QWidget * parent) :
    Base(parent)
{
  max_acceptable_distance_ctl =
      add_numeric_box<int>("max_acceptable_distance",
          "",
          [this](int value) {
            if ( _opts && _opts->max_acceptable_distance != value ) {
              _opts->max_acceptable_distance = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts  ) {
              * value = _opts->max_acceptable_distance;
              return true;
            }
            return false;
          });

  octavedif_ctl =
      add_numeric_box<int>("octavedif",
          "",
          [this](int value) {
            if ( _opts && _opts->octavedif != value ) {
              _opts->octavedif = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->octavedif;
              return true;
            }
            return false;
          });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<class F>
QFlannIndexOptions<F> * QFlannBasedFeature2dMatcherOptions::addStackWidget(F c_flann_based_feature2d_matcher_options::* mp)
{
  using flann_index_options = F;
  auto w = new QFlannIndexOptions<F>(this);
  _stack->addWidget(w);

  QObject::connect(this, &ThisClass::populatecontrols,
      [this, w, mp]() {
        w->setOpts(this->_opts ? &(this->_opts->*mp) : nullptr);
  });

  return w;
}


QFlannBasedFeature2dMatcherOptions::QFlannBasedFeature2dMatcherOptions(QWidget * parent) :
    Base(parent)
{
  flannDistanceType_ctl =
      add_enum_combobox<cvflann::flann_distance_t>("Flann Distance Type:",
          "",
          [this](cvflann::flann_distance_t value) {
            if ( _opts && _opts->distance_type != value ) {
              _opts->distance_type = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](cvflann::flann_distance_t * value) {
            if ( _opts ) {
              * value = _opts->distance_type;
              return true;
            }
            return false;
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio",
          "",
          [this](double value) {
            if ( _opts && _opts->lowe_ratio != value ) {
              _opts->lowe_ratio = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->lowe_ratio;
              return true;
            }
            return false;
          });


  flannIndexType_ctl =
      add_enum_combobox<FlannIndexType>("Flann Index Type:",
          "",
          [this](FlannIndexType value) {
            if ( _opts && _opts->index_type != value ) {
              _opts->index_type = value;
              showMatcherSpecificControls();
              Q_EMIT parameterChanged();
            }
          },
          [this](FlannIndexType * value) {
            if ( _opts ) {
              * value = _opts->index_type;
              return true;
            }
            return false;
          });


  Base::addRow(_stack = new QStackedWidget(this));
  _linear_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::linear);
  _kdtree_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::kdtree);
  _kmeans_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::kmeans);
  _composite_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::composite);
  _hierarchical_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::hierarchical);
  _lsh_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::lsh);
  _autotuned_index_options = addStackWidget(&c_flann_based_feature2d_matcher_options::autotuned);

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        QSettingsWidget * currentSettings = dynamic_cast<QSettingsWidget * >(_stack->currentWidget());
        if ( currentSettings ) {
          currentSettings->setEnabled(_opts != nullptr);
          if ( _opts )  {
            currentSettings->enablecontrols();
          }
        }
  });

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        _linear_index_options->setOpts(_opts? &_opts->linear : nullptr);
        _kdtree_index_options->setOpts(_opts? &_opts->kdtree : nullptr);
        _kmeans_index_options->setOpts(_opts? &_opts->kmeans: nullptr);
        _composite_index_options->setOpts(_opts? &_opts->composite : nullptr);
        _hierarchical_index_options->setOpts(_opts? &_opts->hierarchical : nullptr);
        _lsh_index_options->setOpts(_opts? &_opts->lsh : nullptr);
        _autotuned_index_options->setOpts(_opts? &_opts->autotuned : nullptr);
        showMatcherSpecificControls();
      });


  updateControls();
}

void QFlannBasedFeature2dMatcherOptions::showMatcherSpecificControls()
{
  if ( _opts ) {
    switch (_opts->index_type) {
      case FlannIndex_linear:
        _stack->setCurrentWidget(_linear_index_options);
        break;
      case FlannIndex_kdtree:
        _stack->setCurrentWidget(_kdtree_index_options);
        break;
      case FlannIndex_kmeans:
        _stack->setCurrentWidget(_kmeans_index_options);
        break;
      case FlannIndex_composite:
        _stack->setCurrentWidget(_composite_index_options);
        break;
      case FlannIndex_hierarchical:
        _stack->setCurrentWidget(_hierarchical_index_options);
        break;
      case FlannIndex_lsh:
        _stack->setCurrentWidget(_lsh_index_options);
        break;
      case FlannIndex_autotuned:
        _stack->setCurrentWidget(_autotuned_index_options);
        break;
      default:
        _stack->setEnabled(false);
        return;
    }
    _stack->setEnabled(true);
  }
}

//void QFlannBasedFeature2dMatcherOptions::update_matcher_specific_controls()
//{
//  for ( QWidget * w : controls ) {
//    form->removeRow(w);
//  }
//  controls.clear();
//
//  if ( !_opts ) {
//    flannIndexType_ctl->setEnabled(false);
//    flannDistanceType_ctl->setEnabled(false);
//    lowe_ratio_ctl->setEnabled(false);
//  }
//  else {
//
//#define ADD_FLANN_INDEX_CTL(f, name) \
//    controls.append(add_ctl<decltype(_opts->index.f.name)>(#name, \
//        "", \
//        [this](decltype(_opts->index.f.name) v){ \
//          if ( _opts ) { \
//            _opts->index.f.name = v; \
//          }}, \
//        [this](decltype(_opts->index.f.name) * v) -> bool { \
//          if ( _opts ) { \
//            *v = _opts->index.f.name; \
//            return true; \
//          } \
//          return false; \
//        } \
//    ))
//
//
//    switch (_opts->index.type) {
//    case FlannIndex_linear:
//      break;
//
//    case FlannIndex_kdtree:
//      ADD_FLANN_INDEX_CTL(kdtree, trees);
//      break;
//
//    case FlannIndex_kmeans:
//      ADD_FLANN_INDEX_CTL(kmeans, centers_init);
//      ADD_FLANN_INDEX_CTL(kmeans, branching);
//      ADD_FLANN_INDEX_CTL(kmeans, iterations);
//      ADD_FLANN_INDEX_CTL(kmeans, cb_index);
//      break;
//
//    case FlannIndex_composite:
//      ADD_FLANN_INDEX_CTL(composite, centers_init);
//      ADD_FLANN_INDEX_CTL(composite, trees);
//      ADD_FLANN_INDEX_CTL(composite, branching);
//      ADD_FLANN_INDEX_CTL(composite, iterations);
//      ADD_FLANN_INDEX_CTL(composite, cb_index);
//      break;
//
//    case FlannIndex_hierarchical:
//      ADD_FLANN_INDEX_CTL(hierarchical, centers_init);
//      ADD_FLANN_INDEX_CTL(hierarchical, branching);
//      ADD_FLANN_INDEX_CTL(hierarchical, trees);
//      ADD_FLANN_INDEX_CTL(hierarchical, leaf_size);
//      break;
//
//    case FlannIndex_lsh:
//      ADD_FLANN_INDEX_CTL(lsh, table_number);
//      ADD_FLANN_INDEX_CTL(lsh, key_size);
//      ADD_FLANN_INDEX_CTL(lsh, multi_probe_level);
//      break;
//
//    case FlannIndex_autotuned:
//      ADD_FLANN_INDEX_CTL(autotuned, target_precision);
//      ADD_FLANN_INDEX_CTL(autotuned, build_weight);
//      ADD_FLANN_INDEX_CTL(autotuned, memory_weight);
//      ADD_FLANN_INDEX_CTL(autotuned, sample_fraction);
//      break;
//    }
//
//    Q_EMIT populatecontrols();
//
//#undef ADD_FLANN_INDEX_CTL
//  }
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QOptFlowPyrLKMatcherOptions::QOptFlowPyrLKMatcherOptions(QWidget * parent) :
    Base(parent)
{
  maxLevel_ctl =
      add_numeric_box<int>("maxLevel",
          "max pyramid level",
          [this](int v) {
            if ( _opts && _opts->maxLevel != v ) {
              _opts->maxLevel = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->maxLevel;
              return true;
            }
            return false;
          });

  winSize_ctl =
      add_numeric_box<cv::Size>("winSize",
          "winSize",
          [this](const cv::Size & v) {
            if ( _opts && _opts->winSize != v ) {
              _opts->winSize = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * v) {
            if ( _opts ) {
              *v = _opts->winSize;
              return true;
            }
            return false;
          });

  maxIterations_ctl =
      add_numeric_box<int>("maxIterations",
          "maxIterations",
          [this](int v) {
            if ( _opts && _opts->maxIterations != v ) {
              _opts->maxIterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->maxIterations;
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("eps",
          "eps",
          [this](double v) {
            if ( _opts && _opts->eps != v ) {
              _opts->eps = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->eps;
              return true;
            }
            return false;
          });

  flags_ctl =
      add_numeric_box<int>("flags",
          "flags",
          [this](int v) {
            if ( _opts && _opts->flags != v ) {
              _opts->flags = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->flags;
              return true;
            }
            return false;
          });

  minEigThreshold_ctl =
      add_numeric_box<double>("minEigThreshold",
          "minEigThreshold",
          [this](double v) {
            if ( _opts && _opts->minEigThreshold != v ) {
              _opts->minEigThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->minEigThreshold;
              return true;
            }
            return false;
          });

  maxErr_ctl =
      add_numeric_box<double>("maxErr",
          "maxErr",
          [this](double v) {
            if ( _opts && _opts->maxErr != v ) {
              _opts->maxErr = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->maxErr;
              return true;
            }
            return false;
          });

  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QTriangleMatcherOptions::QTriangleMatcherOptions(QWidget * parent) :
    Base(parent)
{
  eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( _opts && _opts->eps != value ) {
              _opts->eps = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              *value = _opts->eps;
              return true;
            }
            return false;
          });

  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QSnormBasedFeature2dMatcherOptions::QSnormBasedFeature2dMatcherOptions(QWidget * parent) :
    Base(parent)
{
  max_acceptable_distance_ctl =
      add_numeric_box<int>("max_acceptable_distance:",
          "",
          [this](int value) {
            if ( _opts && _opts->max_acceptable_distance != value ) {
              _opts->max_acceptable_distance = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_acceptable_distance;
              return true;
            }
            return false;
          });

  lowe_ratio_ctl =
      add_numeric_box<double>("lowe_ratio:",
          "",
          [this](double value) {
            if ( _opts && _opts->lowe_ratio != value ) {
              _opts->lowe_ratio = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->lowe_ratio;
              return true;
            }
            return false;
          });

  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QSparseFeatureMatcherOptions::QSparseFeatureMatcherOptions(QWidget * parent) :
    Base(parent)
{
  sparseFeatureMatcherType_ctl =
      add_enum_combobox<FEATURE2D_MATCHER_TYPE>(
          "Feature matcher:",
          "",
          [this](FEATURE2D_MATCHER_TYPE value) {
            if ( _opts && _opts->type != value ) {
              _opts->type = value;
              showMatcherSpecificControls();
              Q_EMIT parameterChanged();
            }
          },
          [this](FEATURE2D_MATCHER_TYPE * value) {
            if ( _opts ) {
              * value = _opts->type;
              return true;
            }
            return false;
          });

  _stack = new QStackedWidget(this);
  ThisClass::addRow(_stack);

  _stack->addWidget(hammingDistanceFeature2dMatcherOptions_ctl = new QHammingDistanceFeature2dMatcherOptions(this));
  _stack->addWidget(flannBasedFeature2dMatcherOptions_ctl = new QFlannBasedFeature2dMatcherOptions(this));
  _stack->addWidget(optFlowPyrLKMatcherOptions_ctl = new QOptFlowPyrLKMatcherOptions(this));
  _stack->addWidget(triangleMatcherOptions_ctl = new QTriangleMatcherOptions(this));
  _stack->addWidget(snormBasedFeature2dMatcherOptions_ctl = new QSnormBasedFeature2dMatcherOptions(this));

  connect(hammingDistanceFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged, this, &ThisClass::parameterChanged);
  connect(flannBasedFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged, this, &ThisClass::parameterChanged);
  connect(optFlowPyrLKMatcherOptions_ctl, &QSettingsWidget::parameterChanged, this, &ThisClass::parameterChanged);
  connect(triangleMatcherOptions_ctl, &QSettingsWidget::parameterChanged, this, &ThisClass::parameterChanged);
  connect(snormBasedFeature2dMatcherOptions_ctl, &QSettingsWidget::parameterChanged, this, &ThisClass::parameterChanged);

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        QSettingsWidget * currentSettings = dynamic_cast<QSettingsWidget * >(_stack->currentWidget());
        if ( currentSettings ) {
          currentSettings->setEnabled(_opts != nullptr);
          if ( _opts )  {
            currentSettings->enablecontrols();
          }
        }
  });

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        hammingDistanceFeature2dMatcherOptions_ctl->setOpts(_opts? &_opts->hamming : nullptr);
        flannBasedFeature2dMatcherOptions_ctl->setOpts(_opts? &_opts->flann : nullptr);
        optFlowPyrLKMatcherOptions_ctl->setOpts(_opts? &_opts->optflowpyrlk : nullptr);
        triangleMatcherOptions_ctl->setOpts(_opts? &_opts->triangles : nullptr);
        snormBasedFeature2dMatcherOptions_ctl->setOpts(_opts? &_opts->snorm : nullptr);
        showMatcherSpecificControls();
      });

  updateControls();
}

void QSparseFeatureMatcherOptions::showMatcherSpecificControls()
{
  if( _opts ) {
    switch (_opts->type) {
      case FEATURE2D_MATCHER_HAMMING:
        _stack->setCurrentWidget(hammingDistanceFeature2dMatcherOptions_ctl);
        break;
      case FEATURE2D_MATCHER_FLANN:
        _stack->setCurrentWidget(flannBasedFeature2dMatcherOptions_ctl);
        break;
      case FEATURE2D_MATCHER_OptFlowPyrLK:
        _stack->setCurrentWidget(optFlowPyrLKMatcherOptions_ctl);
        break;
      case FEATURE2D_MATCHER_TRIANGLES:
        _stack->setCurrentWidget(triangleMatcherOptions_ctl);
        break;
      case FEATURE2D_MATCHER_SNORM:
        _stack->setCurrentWidget(snormBasedFeature2dMatcherOptions_ctl);
        break;
      default:
        _stack->setEnabled(false);
        return;
    }
    _stack->setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

