/*
 * feature2d_settings.cc
 *
 *  Created on: Jan 8, 2022
 *      Author: amyznikov
 */

#include "feature2d_settings.h"
#include <core/debug.h>

#define SAVE_SETINGS(p)  \
  save_settings(settings, #p, args.p);

bool load_settings(c_config_setting settings, c_feature2d_orb::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, nfeatures);
  LOAD_OPTIONS(settings, *args, scaleFactor);
  LOAD_OPTIONS(settings, *args, nlevels);
  LOAD_OPTIONS(settings, *args, edgeThreshold);
  LOAD_OPTIONS(settings, *args, firstLevel);
  LOAD_OPTIONS(settings, *args, WTA_K);
  LOAD_OPTIONS(settings, *args, scoreType);
  LOAD_OPTIONS(settings, *args, patchSize);
  LOAD_OPTIONS(settings, *args, fastThreshold);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_orb::options & args)
{
  SAVE_SETINGS(nfeatures);
  SAVE_SETINGS(scaleFactor);
  SAVE_SETINGS(nlevels);
  SAVE_SETINGS(edgeThreshold);
  SAVE_SETINGS(firstLevel);
  SAVE_SETINGS(WTA_K);
  SAVE_SETINGS(scoreType);
  SAVE_SETINGS(patchSize);
  SAVE_SETINGS(fastThreshold);
  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_brisk::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, thresh);
  LOAD_OPTIONS(settings, *args, octaves);
  LOAD_OPTIONS(settings, *args, patternScale);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_brisk::options & args)
{
  SAVE_SETINGS(thresh);
  SAVE_SETINGS(octaves);
  SAVE_SETINGS(patternScale);
  return true;
}

bool load_settings(c_config_setting settings, c_feature2d_kaze::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, extended);
  LOAD_OPTIONS(settings, *args, upright);
  LOAD_OPTIONS(settings, *args, threshold);
  LOAD_OPTIONS(settings, *args, nOctaves);
  LOAD_OPTIONS(settings, *args, nOctaveLayers);
  LOAD_OPTIONS(settings, *args, diffusivity);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_kaze::options & args)
{
  SAVE_SETINGS(extended);
  SAVE_SETINGS(upright);
  SAVE_SETINGS(threshold);
  SAVE_SETINGS(nOctaves);
  SAVE_SETINGS(nOctaveLayers);
  SAVE_SETINGS(diffusivity);
  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_akaze::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, descriptor_type);
  LOAD_OPTIONS(settings, *args, descriptor_size);
  LOAD_OPTIONS(settings, *args, descriptor_channels);
  LOAD_OPTIONS(settings, *args, threshold);
  LOAD_OPTIONS(settings, *args, nOctaves);
  LOAD_OPTIONS(settings, *args, nOctaveLayers);
  LOAD_OPTIONS(settings, *args, diffusivity);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_akaze::options & args)
{
  SAVE_SETINGS(descriptor_type);
  SAVE_SETINGS(descriptor_size);
  SAVE_SETINGS(descriptor_channels);
  SAVE_SETINGS(threshold);
  SAVE_SETINGS(nOctaves);
  SAVE_SETINGS(nOctaveLayers);
  SAVE_SETINGS(diffusivity);
  return true;
}


#if HAVE_FEATURE2D_SIFT
bool load_settings(c_config_setting settings, c_feature2d_sift::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, nfeatures);
  LOAD_OPTIONS(settings, *args, nOctaveLayers);
  LOAD_OPTIONS(settings, *args, contrastThreshold);
  LOAD_OPTIONS(settings, *args, edgeThreshold);
  LOAD_OPTIONS(settings, *args, sigma);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_sift::options & args)
{
  SAVE_SETINGS(nfeatures);
  SAVE_SETINGS(nOctaveLayers);
  SAVE_SETINGS(contrastThreshold);
  SAVE_SETINGS(edgeThreshold);
  SAVE_SETINGS(sigma);
  return true;
}
#endif

#if HAVE_FEATURE2D_SURF
bool load_settings(c_config_setting settings, c_feature2d_surf::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, hessianThreshold);
  LOAD_OPTIONS(settings, *args, nOctaves);
  LOAD_OPTIONS(settings, *args, nOctaveLayers);
  LOAD_OPTIONS(settings, *args, extended);
  LOAD_OPTIONS(settings, *args, upright);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_surf::options & args)
{
  SAVE_SETINGS(hessianThreshold);
  SAVE_SETINGS(nOctaves);
  SAVE_SETINGS(nOctaveLayers);
  SAVE_SETINGS(extended);
  SAVE_SETINGS(upright);
  return true;
}
#endif

bool load_settings(c_config_setting settings, c_feature2d_mser::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, delta);
  LOAD_OPTIONS(settings, *args, min_area);
  LOAD_OPTIONS(settings, *args, max_area);
  LOAD_OPTIONS(settings, *args, max_variation);
  LOAD_OPTIONS(settings, *args, min_diversity);
  LOAD_OPTIONS(settings, *args, max_evolution);
  LOAD_OPTIONS(settings, *args, area_threshold);
  LOAD_OPTIONS(settings, *args, min_margin);
  LOAD_OPTIONS(settings, *args, edge_blur_size);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_mser::options & args)
{
  SAVE_SETINGS(delta);
  SAVE_SETINGS(min_area);
  SAVE_SETINGS(max_area);
  SAVE_SETINGS(max_variation);
  SAVE_SETINGS(min_diversity);
  SAVE_SETINGS(max_evolution);
  SAVE_SETINGS(area_threshold);
  SAVE_SETINGS(min_margin);
  SAVE_SETINGS(edge_blur_size);
  return true;
}

bool load_settings(c_config_setting settings, c_feature2d_fast::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, threshold);
  LOAD_OPTIONS(settings, *args, nonmaxSuppression);
  LOAD_OPTIONS(settings, *args, type);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_fast::options & args)
{
  SAVE_SETINGS(threshold);
  SAVE_SETINGS(nonmaxSuppression);
  SAVE_SETINGS(type);
  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_agast::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, threshold);
  LOAD_OPTIONS(settings, *args, nonmaxSuppression);
  LOAD_OPTIONS(settings, *args, type);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_agast::options & args)
{
  SAVE_SETINGS(threshold);
  SAVE_SETINGS(nonmaxSuppression);
  SAVE_SETINGS(type);
  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_gftt::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, maxCorners);
  LOAD_OPTIONS(settings, *args, qualityLevel);
  LOAD_OPTIONS(settings, *args, minDistance);
  LOAD_OPTIONS(settings, *args, blockSize);
  LOAD_OPTIONS(settings, *args, gradiantSize);
  LOAD_OPTIONS(settings, *args, useHarrisDetector);
  LOAD_OPTIONS(settings, *args, k);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_gftt::options & args)
{
  SAVE_SETINGS(maxCorners);
  SAVE_SETINGS(qualityLevel);
  SAVE_SETINGS(minDistance);
  SAVE_SETINGS(blockSize);
  SAVE_SETINGS(gradiantSize);
  SAVE_SETINGS(useHarrisDetector);
  SAVE_SETINGS(k);
  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_blob::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, thresholdStep);
  LOAD_OPTIONS(settings, *args, minThreshold);
  LOAD_OPTIONS(settings, *args, maxThreshold);
  LOAD_OPTIONS(settings, *args, minRepeatability);
  LOAD_OPTIONS(settings, *args, minDistBetweenBlobs);

  LOAD_OPTIONS(settings, *args, filterByColor);
  LOAD_OPTIONS(settings, *args, blobColor);

  LOAD_OPTIONS(settings, *args, filterByArea);
  LOAD_OPTIONS(settings, *args, minArea);
  LOAD_OPTIONS(settings, *args, maxArea);

  LOAD_OPTIONS(settings, *args, filterByCircularity);
  LOAD_OPTIONS(settings, *args, minCircularity);
  LOAD_OPTIONS(settings, *args, maxCircularity);

  LOAD_OPTIONS(settings, *args, filterByInertia);
  LOAD_OPTIONS(settings, *args, minInertiaRatio);
  LOAD_OPTIONS(settings, *args, maxInertiaRatio);

  LOAD_OPTIONS(settings, *args, filterByConvexity);
  LOAD_OPTIONS(settings, *args, minConvexity);
  LOAD_OPTIONS(settings, *args, maxConvexity);
  END_LOAD_OPTIONS(settings)

  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_blob::options & args)
{
  SAVE_SETINGS(thresholdStep);
  SAVE_SETINGS(minThreshold);
  SAVE_SETINGS(maxThreshold);
  SAVE_SETINGS(minRepeatability);
  SAVE_SETINGS(minDistBetweenBlobs);

  SAVE_SETINGS(filterByColor);
  SAVE_SETINGS(blobColor);

  SAVE_SETINGS(filterByArea);
  SAVE_SETINGS(minArea);
  SAVE_SETINGS(maxArea);

  SAVE_SETINGS(filterByCircularity);
  SAVE_SETINGS(minCircularity);
  SAVE_SETINGS(maxCircularity);

  SAVE_SETINGS(filterByInertia);
  SAVE_SETINGS(minInertiaRatio);
  SAVE_SETINGS(maxInertiaRatio);

  SAVE_SETINGS(filterByConvexity);
  SAVE_SETINGS(minConvexity);
  SAVE_SETINGS(maxConvexity);
  return true;
}

#if HAVE_FEATURE2D_STAR
bool load_settings(c_config_setting settings, c_feature2d_star::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, maxSize);
  LOAD_OPTIONS(settings, *args, responseThreshold);
  LOAD_OPTIONS(settings, *args, lineThresholdProjected);
  LOAD_OPTIONS(settings, *args, lineThresholdBinarized);
  LOAD_OPTIONS(settings, *args, suppressNonmaxSize);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_star::options & args)
{
  SAVE_SETINGS(maxSize);
  SAVE_SETINGS(responseThreshold);
  SAVE_SETINGS(lineThresholdProjected);
  SAVE_SETINGS(lineThresholdBinarized);
  SAVE_SETINGS(suppressNonmaxSize);
  return true;
}
#endif

#if HAVE_FEATURE2D_MSD
bool load_settings(c_config_setting settings, c_feature2d_msd::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, m_patch_radius);
  LOAD_OPTIONS(settings, *args, m_search_area_radius);
  LOAD_OPTIONS(settings, *args, m_nms_radius);
  LOAD_OPTIONS(settings, *args, m_nms_scale_radius);
  LOAD_OPTIONS(settings, *args, m_th_saliency);
  LOAD_OPTIONS(settings, *args, m_kNN);
  LOAD_OPTIONS(settings, *args, m_scale_factor);
  LOAD_OPTIONS(settings, *args, m_n_scales);
  LOAD_OPTIONS(settings, *args, m_compute_orientation);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_msd::options & args)
{
  SAVE_SETINGS(m_patch_radius);
  SAVE_SETINGS(m_search_area_radius);
  SAVE_SETINGS(m_nms_radius);
  SAVE_SETINGS(m_nms_scale_radius);
  SAVE_SETINGS(m_th_saliency);
  SAVE_SETINGS(m_kNN);
  SAVE_SETINGS(m_scale_factor);
  SAVE_SETINGS(m_n_scales);
  SAVE_SETINGS(m_compute_orientation);
  return true;
}
#endif

#if HAVE_FEATURE2D_HL
bool load_settings(c_config_setting settings, c_feature2d_hl::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, numOctaves);
  LOAD_OPTIONS(settings, *args, corn_thresh);
  LOAD_OPTIONS(settings, *args, DOG_thresh);
  LOAD_OPTIONS(settings, *args, maxCorners);
  LOAD_OPTIONS(settings, *args, num_layers);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_hl::options & args)
{
  SAVE_SETINGS(numOctaves);
  SAVE_SETINGS(corn_thresh);
  SAVE_SETINGS(DOG_thresh);
  SAVE_SETINGS(maxCorners);
  SAVE_SETINGS(num_layers);
  return true;
}
#endif

#if HAVE_MORPH_EXTRACTOR
bool load_settings(c_config_setting settings, c_feature2d_morph_extractor::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, morph_type);
  LOAD_OPTIONS(settings, *args, threshold);
  LOAD_OPTIONS(settings, *args, se_radius);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_morph_extractor::options & args)
{
  SAVE_SETINGS(morph_type);
  SAVE_SETINGS(threshold);
  SAVE_SETINGS(se_radius);
  return true;
}
#endif

#if HAVE_FEATURE2D_FREAK
bool load_settings(c_config_setting settings, c_feature2d_freak::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, orientationNormalized);
  LOAD_OPTIONS(settings, *args, scaleNormalized);
  LOAD_OPTIONS(settings, *args, patternScale);
  LOAD_OPTIONS(settings, *args, nOctaves);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_freak::options & args)
{
  SAVE_SETINGS(orientationNormalized);
  SAVE_SETINGS(scaleNormalized);
  SAVE_SETINGS(patternScale);
  SAVE_SETINGS(nOctaves);
  return true;
}
#endif

#if HAVE_FEATURE2D_BRIEF
bool load_settings(c_config_setting settings, c_feature2d_brief::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, bytes);
  LOAD_OPTIONS(settings, *args, use_orientation);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_brief::options & args)
{
  SAVE_SETINGS(bytes);
  SAVE_SETINGS(use_orientation);
  return true;
}

#endif

#if HAVE_FEATURE2D_LUCID
bool load_settings(c_config_setting settings, c_feature2d_lucid::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, lucid_kernel);
  LOAD_OPTIONS(settings, *args, blur_kernel);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_lucid::options & args)
{
  SAVE_SETINGS(lucid_kernel);
  SAVE_SETINGS(blur_kernel);
  return true;
}

#endif

#if HAVE_FEATURE2D_LATCH
bool load_settings(c_config_setting settings, c_feature2d_latch::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, bytes);
  LOAD_OPTIONS(settings, *args, rotationInvariance);
  LOAD_OPTIONS(settings, *args, half_ssd_size);
  LOAD_OPTIONS(settings, *args, sigma);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_latch::options & args)
{
  SAVE_SETINGS(bytes);
  SAVE_SETINGS(rotationInvariance);
  SAVE_SETINGS(half_ssd_size);
  SAVE_SETINGS(sigma);
  return true;
}
#endif

#if HAVE_FEATURE2D_DAISY
bool load_settings(c_config_setting settings, c_feature2d_daisy::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, radius);
  LOAD_OPTIONS(settings, *args, q_radius);
  LOAD_OPTIONS(settings, *args, q_theta);
  LOAD_OPTIONS(settings, *args, q_hist);
  LOAD_OPTIONS(settings, *args, norm);
  LOAD_OPTIONS(settings, *args, interpolation);
  LOAD_OPTIONS(settings, *args, use_orientation);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_daisy::options & args)
{
  SAVE_SETINGS(radius);
  SAVE_SETINGS(q_radius);
  SAVE_SETINGS(q_theta);
  SAVE_SETINGS(q_hist);
  SAVE_SETINGS(norm);
  SAVE_SETINGS(interpolation);
  SAVE_SETINGS(use_orientation);
  return true;
}
#endif

#if HAVE_FEATURE2D_VGG
bool load_settings(c_config_setting settings, c_feature2d_vgg::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, desc);
  LOAD_OPTIONS(settings, *args, isigma);
  LOAD_OPTIONS(settings, *args, img_normalize);
  LOAD_OPTIONS(settings, *args, use_scale_orientation);
  LOAD_OPTIONS(settings, *args, scale_factor);
  LOAD_OPTIONS(settings, *args, dsc_normalize);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_vgg::options & args)
{
  SAVE_SETINGS(desc);
  SAVE_SETINGS(isigma);
  SAVE_SETINGS(img_normalize);
  SAVE_SETINGS(use_scale_orientation);
  SAVE_SETINGS(scale_factor);
  SAVE_SETINGS(dsc_normalize);
  return true;
}
#endif

#if HAVE_FEATURE2D_BOOST
bool load_settings(c_config_setting settings, c_feature2d_boost::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, desc);
  LOAD_OPTIONS(settings, *args, use_scale_orientation);
  LOAD_OPTIONS(settings, *args, scale_factor);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_boost::options & args)
{
  SAVE_SETINGS(desc);
  SAVE_SETINGS(use_scale_orientation);
  SAVE_SETINGS(scale_factor);
  return true;
}
#endif

#if HAVE_STAR_EXTRACTOR

bool load_settings(c_config_setting settings, c_feature2d_star_extractor::options * args)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *args, median_filter_size);
  LOAD_OPTIONS(settings, *args, sigma1);
  LOAD_OPTIONS(settings, *args, sigma2);
  LOAD_OPTIONS(settings, *args, noise_blur);
  LOAD_OPTIONS(settings, *args, noise_threshold);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_star_extractor::options & args)
{
  SAVE_SETINGS(median_filter_size);
  SAVE_SETINGS(sigma1);
  SAVE_SETINGS(sigma2);
  SAVE_SETINGS(noise_blur);
  SAVE_SETINGS(noise_threshold);
  return true;
}
#endif

#if HAVE_TRIANGLE_EXTRACTOR
bool load_settings(c_config_setting settings, c_feature2d_triangle_extractor::options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, min_side_size);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_feature2d_triangle_extractor::options & args)
{
  SAVE_SETINGS(min_side_size);
  return true;
}
#endif


bool save_settings(c_config_setting settings, const c_flann_based_feature2d_matcher_options & args)
{
  SAVE_SETINGS(distance_type);
  SAVE_SETINGS(lowe_ratio);
  SAVE_SETINGS(index);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_based_feature2d_matcher_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, distance_type);
  LOAD_OPTIONS(settings, *options, lowe_ratio);
  LOAD_OPTIONS(settings, *options, index);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_optflowpyrlk_feature2d_matcher_options & args)
{
  SAVE_SETINGS(maxLevel);
  SAVE_SETINGS(winSize);
  SAVE_SETINGS(maxIterations);
  SAVE_SETINGS(flags);
  SAVE_SETINGS(eps);
  SAVE_SETINGS(minEigThreshold);
  SAVE_SETINGS(maxErr);

  return true;
}

bool load_settings(c_config_setting settings, c_optflowpyrlk_feature2d_matcher_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, maxLevel);
  LOAD_OPTIONS(settings, *options, winSize);
  LOAD_OPTIONS(settings, *options, maxIterations);
  LOAD_OPTIONS(settings, *options, flags);
  LOAD_OPTIONS(settings, *options, eps);
  LOAD_OPTIONS(settings, *options, minEigThreshold);
  LOAD_OPTIONS(settings, *options, maxErr);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_snorm_based_feature2d_matcher_options & args)
{
  SAVE_SETINGS(max_acceptable_distance);
  SAVE_SETINGS(lowe_ratio);
  return true;
}

bool load_settings(c_config_setting settings, c_snorm_based_feature2d_matcher_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, max_acceptable_distance);
  LOAD_OPTIONS(settings, *options, lowe_ratio);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_triangle_matcher_options & args)
{
  SAVE_SETINGS(eps);
  return true;
}

bool load_settings(c_config_setting settings, c_triangle_matcher_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, eps);
  END_LOAD_OPTIONS(settings)
  return true;
}


bool save_settings(c_config_setting settings, const c_flann_linear_index_options & args)
{
  // has no options
  return true;
}

bool load_settings(c_config_setting settings, c_flann_linear_index_options * options)
{
  // has no options
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_kdtree_index_options & args)
{
  SAVE_SETINGS(trees);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_kdtree_index_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, trees);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_kmeans_index_options & args)
{
  SAVE_SETINGS(centers_init);
  SAVE_SETINGS(branching);
  SAVE_SETINGS(iterations);
  SAVE_SETINGS(cb_index);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_kmeans_index_options* options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, centers_init);
  LOAD_OPTIONS(settings, *options, branching);
  LOAD_OPTIONS(settings, *options, iterations);
  LOAD_OPTIONS(settings, *options, cb_index);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_composite_index_options & args)
{
  SAVE_SETINGS(centers_init);
  SAVE_SETINGS(trees);
  SAVE_SETINGS(branching);
  SAVE_SETINGS(iterations);
  SAVE_SETINGS(cb_index);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_composite_index_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, centers_init);
  LOAD_OPTIONS(settings, *options, trees);
  LOAD_OPTIONS(settings, *options, branching);
  LOAD_OPTIONS(settings, *options, iterations);
  LOAD_OPTIONS(settings, *options, cb_index);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_hierarchical_index_options & args)
{
  SAVE_SETINGS(centers_init);
  SAVE_SETINGS(branching);
  SAVE_SETINGS(trees);
  SAVE_SETINGS(leaf_size);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_hierarchical_index_options  * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, centers_init);
  LOAD_OPTIONS(settings, *options, branching);
  LOAD_OPTIONS(settings, *options, trees);
  LOAD_OPTIONS(settings, *options, leaf_size);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_lsh_index_options & args)
{
  SAVE_SETINGS(table_number);
  SAVE_SETINGS(key_size);
  SAVE_SETINGS(multi_probe_level);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_lsh_index_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, table_number);
  LOAD_OPTIONS(settings, *options, key_size);
  LOAD_OPTIONS(settings, *options, multi_probe_level);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_flann_autotuned_index_options & args)
{
  SAVE_SETINGS(target_precision);
  SAVE_SETINGS(build_weight);
  SAVE_SETINGS(memory_weight);
  SAVE_SETINGS(sample_fraction);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_autotuned_index_options  * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, target_precision);
  LOAD_OPTIONS(settings, *options, build_weight);
  LOAD_OPTIONS(settings, *options, memory_weight);
  LOAD_OPTIONS(settings, *options, sample_fraction);
  END_LOAD_OPTIONS(settings)

  return true;
}

bool save_settings(c_config_setting section, const c_feature2d::sptr & obj)
{
  const c_feature2d * p =
      obj.get();

  if ( !p ) {
    CF_ERROR("No FeatureDetector object specified for saving into config section");
    return false;
  }

  switch ( p->type() ) {
  case FEATURE2D_ORB :
    return save_settings(section,
        *static_cast<const c_feature2d_orb::options*>(p->opts()));
  case FEATURE2D_BRISK :
    return save_settings(section,
        *static_cast<const c_feature2d_brisk::options*>(p->opts()));
  case FEATURE2D_MSER :
    return save_settings(section,
        *static_cast<const c_feature2d_mser::options*>(p->opts()));
  case FEATURE2D_FAST :
    return save_settings(section,
        *static_cast<const c_feature2d_fast::options*>(p->opts()));
  case FEATURE2D_AGAST :
    return save_settings(section,
        *static_cast<const c_feature2d_agast::options*>(p->opts()));
  case FEATURE2D_GFTT :
    return save_settings(section,
        *static_cast<const c_feature2d_gftt::options*>(p->opts()));
  case FEATURE2D_BLOB :
    return save_settings(section,
        *static_cast<const c_feature2d_blob::options*>(p->opts()));
  case FEATURE2D_KAZE :
    return save_settings(section,
        *static_cast<const c_feature2d_kaze::options*>(p->opts()));
  case FEATURE2D_AKAZE :
    return save_settings(section,
        *static_cast<const c_feature2d_akaze::options*>(p->opts()));
#if HAVE_FEATURE2D_BRIEF
  case FEATURE2D_BRIEF :
    return save_settings(section,
        *static_cast<const c_feature2d_brief::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_SIFT
  case FEATURE2D_SIFT :
    return save_settings(section,
        *static_cast<const c_feature2d_sift::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_SURF
  case FEATURE2D_SURF :
    return save_settings(section,
        *static_cast<const c_feature2d_surf::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_FREAK
  case FEATURE2D_FREAK :
    return save_settings(section,
        *static_cast<const c_feature2d_freak::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_STAR
  case FEATURE2D_STAR :
    return save_settings(section,
        *static_cast<const c_feature2d_star::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_LUCID
  case FEATURE2D_LUCID :
    return save_settings(section,
        *static_cast<const c_feature2d_lucid::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_LATCH
  case FEATURE2D_LATCH :
    return save_settings(section,
        *static_cast<const c_feature2d_latch::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_DAISY
  case FEATURE2D_DAISY :
    return save_settings(section,
        *static_cast<const c_feature2d_daisy::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_MSD
  case FEATURE2D_MSD :
    return save_settings(section,
        *static_cast<const c_feature2d_msd::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_VGG
  case FEATURE2D_VGG :
    return save_settings(section,
        *static_cast<const c_feature2d_vgg::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_BOOST
  case FEATURE2D_BOOST :
    return save_settings(section,
        *static_cast<const c_feature2d_boost::options*>(p->opts()));
#endif
#if HAVE_FEATURE2D_HL
  case FEATURE2D_HL :
    return save_settings(section,
        *static_cast<const c_feature2d_hl::options*>(p->opts()));
#endif
#if HAVE_STAR_EXTRACTOR
  case FEATURE2D_STAR_EXTRACTOR :
    return save_settings(section,
        *static_cast<const c_feature2d_star_extractor::options*>(p->opts()));
#endif
#if HAVE_MORPH_EXTRACTOR
  case FEATURE2D_MORPH :
    return save_settings(section,
        *static_cast<const c_feature2d_morph_extractor::options*>(p->opts()));
#endif

  default :
    CF_ERROR("Unknown or not supported feature2d object type=%d (%s)",
        (int )(p->type()), toCString(p->type()));
    break;
  }

  return false;
}




bool load_settings(c_config_setting settings, c_sparse_feature_extractor_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, detector);
  LOAD_OPTIONS(settings, *options, descriptor);
  END_LOAD_OPTIONS(settings)

  return true;
}
//
//c_sparse_feature_extractor::sptr create_sparse_feature_extractor(c_config_setting settings)
//{
//  INSTRUMENT_REGION("");
//
//  c_sparse_feature_extractor_options options;
//  if ( !load_settings(settings, &options) ) {
//    CF_ERROR("load_settings(c_sparse_feature_extractor_options) fails");
//    return nullptr;
//  }
//
//  return create_sparse_feature_extractor(options);
//}

bool save_settings(c_config_setting settings, const c_flann_index_options & args)
{
  SAVE_SETINGS(type);
  SAVE_SETINGS(linear);
  SAVE_SETINGS(kdtree);
  SAVE_SETINGS(kmeans);
  SAVE_SETINGS(composite);
  SAVE_SETINGS(hierarchical);
  SAVE_SETINGS(lsh);
  SAVE_SETINGS(autotuned);
  return true;
}

bool load_settings(c_config_setting settings, c_flann_index_options * options)
{
  if ( !settings ) {
    CF_ERROR("settings pointer is NULL");
    return false;
  }

  std::string objtype;
  if ( !load_settings(settings, "type", &objtype) || objtype.empty() ) {
    if ( options->type == FlannIndex_unknown ) {
      CF_ERROR("No flann index type specified");
      return false;
    }
  }
  else if ( !fromString(objtype, &options->type) || options->type == FlannIndex_unknown ) {
    CF_ERROR("Unknown or not supported flann index type specified: '%s'",
        objtype.c_str());
    return false;
  }

  c_config_setting subsection;

  if( (subsection = settings[toString(FlannIndex_linear)]).isGroup() ) {
    load_settings(subsection, &options->linear);
  }
  if( (subsection = settings[toString(FlannIndex_kdtree)]).isGroup() ) {
    load_settings(subsection, &options->kdtree);
  }
  if( (subsection = settings[toString(FlannIndex_kmeans)]).isGroup() ) {
    load_settings(subsection, &options->kmeans);
  }
  if( (subsection = settings[toString(FlannIndex_composite)]).isGroup() ) {
    load_settings(subsection, &options->composite);
  }
  if( (subsection = settings[toString(FlannIndex_hierarchical)]).isGroup() ) {
    load_settings(subsection, &options->hierarchical);
  }
  if( (subsection = settings[toString(FlannIndex_lsh)]).isGroup() ) {
    load_settings(subsection, &options->lsh);
  }
  if( (subsection = settings[toString(FlannIndex_autotuned)]).isGroup() ) {
    load_settings(subsection, &options->autotuned);
  }

  return true;
}

bool save_settings(c_config_setting settings, const c_hamming_distance_feature2d_matcher_options & args)
{
  SAVE_SETINGS(max_acceptable_distance);
  SAVE_SETINGS(octavedif);
  return true;
}

bool load_settings(c_config_setting settings, c_hamming_distance_feature2d_matcher_options * options)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *options, max_acceptable_distance);
  LOAD_OPTIONS(settings, *options, octavedif);
  END_LOAD_OPTIONS(settings)

  return true;
}


bool save_settings(c_config_setting settings, const c_feature2d_matcher_options & options)
{
  if ( !settings ) {
    CF_ERROR("settings pointer is NULL");
    return false;
  }

  save_settings(settings, "type", options.type);
  save_settings(settings.add_group(toString(FEATURE2D_MATCHER_FLANN)), options.flann);
  save_settings(settings.add_group(toString(FEATURE2D_MATCHER_HAMMING)), options.hamming);
  save_settings(settings.add_group(toString(FEATURE2D_MATCHER_OptFlowPyrLK)), options.optflowpyrlk);
  save_settings(settings.add_group(toString(FEATURE2D_MATCHER_SNORM)), options.snorm);
  save_settings(settings.add_group(toString(FEATURE2D_MATCHER_TRIANGLES)), options.triangles);

  return true;
}


bool load_settings(c_config_setting settings, c_feature2d_matcher_options * options)
{
  if ( !settings ) {
    CF_ERROR("settings pointer is NULL");
    return false;
  }

  std::string objtype;
  if( load_settings(settings, "type", &objtype) && !objtype.empty() ) {
    fromString(objtype, &options->type);
  }

//  if ( !load_settings(settings, "type", &objtype) || objtype.empty() ) {
//    if ( options->type == FEATURE2D_MATCHER_UNKNOWN ) {
//      CF_ERROR("No sparse feature2d matcher type specified");
//      return false;
//    }
//  }
//  else if ( !fromString(objtype, &options->type) || options->type == FEATURE2D_MATCHER_UNKNOWN ) {
//    CF_ERROR("Unknown or not supported No sparse feature2d matcher type specified: '%s'",
//        objtype.c_str());
//    return false;
//  }

  c_config_setting subsection;

  if( (subsection = settings[toString(FEATURE2D_MATCHER_FLANN)]).isGroup() ) {
    load_settings(subsection, &options->flann);
  }
  if( (subsection = settings[toString(FEATURE2D_MATCHER_HAMMING)]).isGroup() ) {
    load_settings(subsection, &options->hamming);
  }
  if( (subsection = settings[toString(FEATURE2D_MATCHER_OptFlowPyrLK)]).isGroup() ) {
    load_settings(subsection, &options->optflowpyrlk);
  }
  if( (subsection = settings[toString(FEATURE2D_MATCHER_SNORM)]).isGroup() ) {
    load_settings(subsection, &options->snorm);
  }
  if( (subsection = settings[toString(FEATURE2D_MATCHER_TRIANGLES)]).isGroup() ) {
    load_settings(subsection, &options->triangles);
  }

  return true;
}

//
//c_feature2d_matcher::sptr create_sparse_feature_matcher(const c_sparse_feature_extractor::sptr & extractor, c_config_setting settings)
//{
//  INSTRUMENT_REGION("");
//
//  c_feature2d_matcher_options options;
//
//  if ( extractor ) {
//
//    switch ( extractor->descriptor()->defaultNorm() ) {
//    case cv::NORM_HAMMING :
//      options.type = FEATURE2D_MATCHER_HAMMING;
//      break;
//
//    default :
//      switch ( CV_MAT_DEPTH(extractor->descriptor()->descriptorType()) ) {
//      case CV_8U :
//        case CV_8S :
//        options.type = FEATURE2D_MATCHER_FLANN;
//        options.flann.index.type = FlannIndex_lsh;
//        break;
//      default :
//        options.type = FEATURE2D_MATCHER_FLANN;
//        options.flann.index.type = FlannIndex_kdtree;
//        break;
//      }
//      break;
//    }
//  }
//
//  if ( !load_settings(settings, &options) ) {
//    CF_ERROR("load_settings(c_feature2d_matcher_options) fails");
//    return nullptr;
//  }
//
//  return create_sparse_feature_matcher(options);
//}
//
//c_feature2d_matcher::sptr create_sparse_feature_matcher(c_config_setting settings)
//{
//  INSTRUMENT_REGION("");
//
//
//  c_feature2d_matcher_options options;
//
//  if ( !load_settings(settings, &options) ) {
//    CF_ERROR("load_settings(c_feature2d_matcher_options) fails");
//    return nullptr;
//  }
//
//  return create_sparse_feature_matcher(options);
//}
//



//c_feature2d_matcher::sptr create_sparse_feature_matcher(const c_sparse_feature_extractor::sptr & feature_extractor,
//    const std::string & matcher_spec)
//{
//  INSTRUMENT_REGION("");
//
////  if ( feature_extractor ) {
////
////    CF_DEBUG("descriptor: \n"
////        "defaultNorm=%d\n"
////        "descriptorType=%d\n"
////        "descriptorSize=%d\n",
////        (int )(feature_extractor->descriptor()->defaultNorm()),
////        (int )(feature_extractor->descriptor()->descriptorType()),
////        (int )(feature_extractor->descriptor()->descriptorSize()));
////  }
//
//
//  //
//  // Parse matcher type and params if provided
//  //
//
//  enum FEATURE2D_MATCHER_TYPE matcher_type =
//      FEATURE2D_MATCHER_UNKNOWN;
//
//  enum FlannIndexType index_type =
//      FlannIndex_unknown;
//
//  typedef std::pair<std::string, std::string> matcher_arg;
//  std::vector<matcher_arg> matcher_args;
//
//  if ( !matcher_spec.empty() ) {
//
//    std::string matcher_type_name;
//
//    if ( !parse_object_type_and_args(matcher_spec, matcher_type_name, matcher_args) ) {
//      CF_ERROR("parse_object_type_and_args() fails");
//      return nullptr;
//    }
//
//    if ( !fromString(matcher_type_name, &matcher_type) ) {
//      CF_ERROR("Unknown or not supported feature2d matcher requested: %s",
//          matcher_type_name.c_str());
//      return nullptr;
//    }
//
//
//    std::vector<matcher_arg>::iterator pos =
//        std::find_if(matcher_args.begin(), matcher_args.end(),
//            [](const matcher_arg & arg) {
//              return arg.first == "index";
//            });
//
//    if ( pos != matcher_args.end() ) {
//
//      if ( !fromString(pos->second, &index_type) ) {
//        CF_ERROR("Invalid or not supported index type '%s' requested",
//            pos->second.c_str());
//        return nullptr;
//      }
//    }
//  }
//
//  if ( matcher_type == FEATURE2D_MATCHER_UNKNOWN ) {
//
//    if ( !feature_extractor ) {
//      CF_ERROR("Can not deduce appropriate feature2d matcher type");
//      return nullptr;
//    }
//
//    switch ( feature_extractor->descriptor()->defaultNorm() ) {
//    case cv::NORM_HAMMING :
//      matcher_type = FEATURE2D_MATCHER_HAMMING;
//      break;
//
//    default :
//      switch ( CV_MAT_DEPTH(feature_extractor->descriptor()->descriptorType()) ) {
//      case CV_8U :
//        case CV_8S :
//        matcher_type = FEATURE2D_MATCHER_FLANN;
//        index_type = FlannIndex_lsh;
//        break;
//      default :
//        matcher_type = FEATURE2D_MATCHER_FLANN;
//        index_type = FlannIndex_kdtree;
//        break;
//      }
//      break;
//    }
//  }
//
//  if ( matcher_type == FEATURE2D_MATCHER_FLANN && index_type == FlannIndex_unknown ) {
//
//    if ( !feature_extractor ) {
//      CF_ERROR("Can not deduce appropriate feature2d flann index type");
//      return nullptr;
//    }
//
//    switch ( CV_MAT_DEPTH(feature_extractor->descriptor()->descriptorType()) ) {
//    case CV_8U :
//      case CV_8S :
//      index_type = FlannIndex_lsh;
//      break;
//    default :
//      index_type = FlannIndex_kdtree;
//      break;
//    }
//  }
//
//  CF_DEBUG("matcher_type=%s index_type=%s",
//      toString(matcher_type),
//      toString(index_type));
//
//#define PARSE_PARAM(p)  \
//    if ( strcasecmp(name, #p) == 0 ) { \
//      if ( *value && fromString(value, &p) != 1 ) { \
//        CF_ERROR("Syntax error: can not parse value '%s' for parameter %s", value, name); \
//        return nullptr; \
//      } \
//      continue; \
//    }
//
//  switch ( matcher_type ) {
//  case FEATURE2D_MATCHER_HAMMING : {
//
//    double max_acceptable_distance = -1;
//
//    for ( const matcher_arg & arg : matcher_args ) {
//      const char * name = arg.first.c_str();
//      const char * value = arg.second.c_str();
//
//      PARSE_PARAM(max_acceptable_distance);
//
//      CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//          name, value, toString(matcher_type));
//
//      return nullptr;
//    }
//
//    c_hamming_distance_feature2d_matcher::ptr matcher(
//        new c_hamming_distance_feature2d_matcher());
//
//    if ( max_acceptable_distance >= 0 ) {
//      matcher->set_max_acceptable_distance(
//          max_acceptable_distance);
//    }
//
//    return matcher;
//  }
//
//  case FEATURE2D_MATCHER_SNORM : {
//
//    double max_acceptable_distance = -1;
//    double lowe_ratio = -1;
//
//    for ( const matcher_arg & arg : matcher_args ) {
//      const char * name = arg.first.c_str();
//      const char * value = arg.second.c_str();
//
//      PARSE_PARAM(max_acceptable_distance);
//      PARSE_PARAM(lowe_ratio);
//
//      CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//          name, value, toString(matcher_type));
//
//      return nullptr;
//    }
//
//    c_snorm_based_feature2d_matcher::ptr matcher(
//        new c_snorm_based_feature2d_matcher());
//
//    if ( max_acceptable_distance >= 0 ) {
//      matcher->set_max_acceptable_distance(
//          max_acceptable_distance);
//    }
//
//    if ( lowe_ratio >= 0 ) {
//      matcher->set_lowe_ratio(
//          lowe_ratio);
//    }
//
//    return matcher;
//  }
//
//  case FEATURE2D_MATCHER_FLANN :
//
//    switch ( index_type ) {
//    case FlannIndex_linear : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::LinearIndexParams()));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_kdtree : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      int trees = 1;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(trees);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::KDTreeIndexParams(
//                  trees)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_kmeans : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      int branching = 32;
//      int iterations = 11;
//      cvflann::flann_centers_init_t centers_init = cvflann::FLANN_CENTERS_RANDOM;
//      float cb_index = 0.2f;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(branching);
//        PARSE_PARAM(iterations);
//        PARSE_PARAM(centers_init);
//        PARSE_PARAM(cb_index);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::KMeansIndexParams(
//                  branching,
//                  iterations,
//                  centers_init,
//                  cb_index)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_composite : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      int trees = 1;
//      int branching = 32;
//      int iterations = 11;
//      cvflann::flann_centers_init_t centers_init = cvflann::FLANN_CENTERS_RANDOM;
//      float cb_index = 0.2f;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(trees);
//        PARSE_PARAM(branching);
//        PARSE_PARAM(iterations);
//        PARSE_PARAM(centers_init);
//        PARSE_PARAM(cb_index);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::CompositeIndexParams(
//                  trees,
//                  branching,
//                  iterations,
//                  centers_init,
//                  cb_index)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_hierarchical : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      int branching = 32;
//      cvflann::flann_centers_init_t centers_init = cvflann::FLANN_CENTERS_RANDOM;
//      int trees = 4;
//      int leaf_size = 100;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(branching);
//        PARSE_PARAM(centers_init);
//        PARSE_PARAM(trees);
//        PARSE_PARAM(leaf_size);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::HierarchicalClusteringIndexParams(
//                  branching,
//                  centers_init,
//                  trees,
//                  leaf_size)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_lsh : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      int table_number = 8;
//      int key_size = 12;
//      int multi_probe_level = 1;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(table_number);
//        PARSE_PARAM(key_size);
//        PARSE_PARAM(multi_probe_level);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::LshIndexParams(
//                  table_number,
//                  key_size,
//                  multi_probe_level)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    case FlannIndex_autotuned : {
//
//      double lowe_ratio = -1;
//      cvflann::flann_distance_t distance_type = cvflann::FLANN_DIST_L2;
//      float target_precision = 0.8f;
//      float build_weight = 0.01f;
//      float memory_weight = 0;
//      float sample_fraction = 0.1f;
//
//      for ( const matcher_arg & arg : matcher_args ) {
//        const char * name = arg.first.c_str();
//        const char * value = arg.second.c_str();
//
//        PARSE_PARAM(lowe_ratio);
//        PARSE_PARAM(distance_type);
//        PARSE_PARAM(target_precision);
//        PARSE_PARAM(build_weight);
//        PARSE_PARAM(memory_weight);
//        PARSE_PARAM(sample_fraction);
//
//        CF_ERROR("WARNING: Unknown parameter '%s = %s' specified for feature2d matcher %s. Ignored",
//            name, value, toString(matcher_type));
//
//        return nullptr;
//      }
//
//      c_flann_based_feature2d_matcher::ptr matcher(
//          new c_flann_based_feature2d_matcher(
//              new cv::flann::AutotunedIndexParams(
//                  target_precision,
//                  build_weight,
//                  memory_weight,
//                  sample_fraction)));
//
//      matcher->set_distance_type(
//          distance_type);
//
//      if ( lowe_ratio >= 0 ) {
//        matcher->set_lowe_ratio(
//            lowe_ratio);
//      }
//
//      return matcher;
//    }
//
//    default :
//      break;
//    }
//
//    break;
//  default :
//    break;
//  }
//
//
//
//#undef PARSE_PARAM
//
//
//  CF_ERROR("Invalid or not supported matcher_type=%d index_type=%d requested",
//      matcher_type, index_type);
//
//  return nullptr;
//}
//


bool load_settings(c_config_setting settings, c_sparse_feature_detector_options * options)
{
  if ( !settings ) {
    CF_ERROR("libconfig settings is null in load_settings(c_sparse_feature_detector_options)");
    return false;
  }

  std::string detector_type;

  if ( load_settings(settings, "type", &detector_type) && !detector_type.empty() ) {
    if ( !fromString(detector_type, &options->type) || options->type == SPARSE_FEATURE_DETECTOR_UNKNOWN ) {
      CF_ERROR("Invalid or not supported feature descriptor type specified : %s",
          detector_type.c_str());
    }
  }

  load_settings(settings, "max_keypoints",
      &options->max_keypoints);


  c_config_setting group;
#define LOAD_GROUP(name) \
  if ( (group = settings[#name]).isGroup() ) { \
    load_settings(group, &options->name); \
  }

  LOAD_GROUP(orb);
  LOAD_GROUP(brisk);
  LOAD_GROUP(kaze);
  LOAD_GROUP(akaze);
#if HAVE_FEATURE2D_SIFT
  LOAD_GROUP(sift);
#endif
#if HAVE_FEATURE2D_SURF
  LOAD_GROUP(surf);
#endif
  LOAD_GROUP(mser);
  LOAD_GROUP(fast);
  LOAD_GROUP(agast);
  LOAD_GROUP(gftt);
  LOAD_GROUP(blob);
#if HAVE_FEATURE2D_STAR
  LOAD_GROUP(star);
#endif
#if HAVE_MORPH_EXTRACTOR
  LOAD_GROUP(morph);
#endif
#if HAVE_FEATURE2D_MSD
  LOAD_GROUP(msd);
#endif
#if HAVE_FEATURE2D_HL
  LOAD_GROUP(hl);
#endif

#undef LOAD_GROUP

  return true;
}

bool save_settings(c_config_setting settings, const c_sparse_feature_detector_options & options)
{
  if ( !settings ) {
    CF_ERROR("libconfig settings is null in save_settings(c_sparse_feature_detector_options)");
    return false;
  }

  if ( !save_settings(settings, "type", toString(options.type)) ) {
    CF_ERROR("save_settings(type='%s') fails in save_settings(c_sparse_feature_detector_options)", toString(options.type).c_str() );
    return false;
  }

  save_settings(settings, "max_keypoints",
      options.max_keypoints);

#define SAVE_GROUP(name) \
    save_settings(settings.add_group(#name), \
        options.name);

  SAVE_GROUP(orb);
  SAVE_GROUP(brisk);
  SAVE_GROUP(kaze);
  SAVE_GROUP(akaze);
#if HAVE_FEATURE2D_SIFT
  SAVE_GROUP(sift);
#endif
#if HAVE_FEATURE2D_SURF
  SAVE_GROUP(surf);
#endif
  SAVE_GROUP(mser);
  SAVE_GROUP(fast);
  SAVE_GROUP(agast);
  SAVE_GROUP(gftt);
  SAVE_GROUP(blob);
#if HAVE_FEATURE2D_STAR
  SAVE_GROUP(star);
#endif
#if HAVE_FEATURE2D_MSD
  SAVE_GROUP(msd);
#endif
#if HAVE_FEATURE2D_HL
  SAVE_GROUP(hl);
#endif

#undef SAVE_GROUP

  return true;
}



bool load_settings(c_config_setting settings, c_sparse_feature_descriptor_options * options)
{
  INSTRUMENT_REGION("");

  if ( !settings ) {
    CF_ERROR("libconfig settings is null in load_settings(c_sparse_feature_descriptor_options)");
    return false;
  }

  std::string descriptor_type;

  if ( load_settings(settings, "type", &descriptor_type) && !descriptor_type.empty() ) {
    if ( !fromString(descriptor_type, &options->type) ) {
      CF_ERROR("Invalid or not supported feature descriptor type specified : %s",
          descriptor_type.c_str());
    }
  }

//  load_settings(settings, "use_detector_options",
//      &options->use_detector_options);

  c_config_setting group;
#define LOAD_GROUP(name) \
  if ( (group = settings[#name]).isGroup() ) { \
    load_settings(group, &options->name); \
  }

  LOAD_GROUP(orb);
  LOAD_GROUP(brisk);
  LOAD_GROUP(kaze);
  LOAD_GROUP(akaze);
#if HAVE_FEATURE2D_SIFT
  LOAD_GROUP(sift);
#endif
#if HAVE_FEATURE2D_SURF
  LOAD_GROUP(surf);
#endif
#if HAVE_FEATURE2D_FREAK
  LOAD_GROUP(freak);
#endif
#if HAVE_FEATURE2D_BRIEF
  LOAD_GROUP(brief);
#endif
#if HAVE_FEATURE2D_LUCID
  LOAD_GROUP(lucid);
#endif
#if HAVE_FEATURE2D_LATCH
  LOAD_GROUP(latch);
#endif
#if HAVE_FEATURE2D_DAISY
  LOAD_GROUP(daisy);
#endif
#if HAVE_FEATURE2D_VGG
  LOAD_GROUP(vgg);
#endif
#if HAVE_FEATURE2D_BOOST
  LOAD_GROUP(boost);
#endif
#if HAVE_TRIANGLE_EXTRACTOR
  LOAD_GROUP(triangles);
#endif

#undef LOAD_GROUP

  return true;
}

bool save_settings(c_config_setting settings, const c_sparse_feature_descriptor_options & options)
{
  save_settings(settings, "type",
      toString(options.type));

//  save_settings(settings, "use_detector_options",
//      options.use_detector_options);

#define SAVE_GROUP(name) \
    save_settings(settings.add_group(#name), \
        options.name);

  SAVE_GROUP(orb);
  SAVE_GROUP(brisk);
  SAVE_GROUP(kaze);
  SAVE_GROUP(akaze);
#if HAVE_FEATURE2D_SIFT
  SAVE_GROUP(sift);
#endif
#if HAVE_FEATURE2D_SURF
  SAVE_GROUP(surf);
#endif
#if HAVE_FEATURE2D_FREAK
  SAVE_GROUP(freak);
#endif
#if HAVE_FEATURE2D_BRIEF
  SAVE_GROUP(brief);
#endif
#if HAVE_FEATURE2D_LUCID
  SAVE_GROUP(lucid);
#endif
#if HAVE_FEATURE2D_LATCH
  SAVE_GROUP(latch);
#endif
#if HAVE_FEATURE2D_DAISY
  SAVE_GROUP(daisy);
#endif
#if HAVE_FEATURE2D_VGG
  SAVE_GROUP(vgg);
#endif
#if HAVE_FEATURE2D_BOOST
  SAVE_GROUP(boost);
#endif

#undef SAVE_GROUP

  return true;
}


bool save_settings(c_config_setting settings, const c_sparse_feature_extractor_options & options)
{
  save_settings(settings.add_group("detector"),
      options.detector);

  save_settings(settings.add_group("descriptor"),
      options.descriptor);

  return true;
}



