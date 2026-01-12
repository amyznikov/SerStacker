/*
 * feature2d_settings.h
 *
 *  Created on: Jan 8, 2022
 *      Author: amyznikov
 */
#pragma once
#ifndef __feature2d_settings_h__
#define __feature2d_settings_h__

#include "feature2d.h"
#include <core/settings/opencv_settings.h>

bool save_settings(c_config_setting settings, const c_feature2d_orb::options & args);
bool load_settings(c_config_setting settings, c_feature2d_orb::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_brisk::options & args);
bool load_settings(c_config_setting settings, c_feature2d_brisk::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_kaze::options & args);
bool load_settings(c_config_setting settings, c_feature2d_kaze::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_akaze::options & args);
bool load_settings(c_config_setting settings, c_feature2d_akaze::options * args);

#if HAVE_FEATURE2D_SIFT
bool save_settings(c_config_setting settings, const c_feature2d_sift::options & args);
bool load_settings(c_config_setting settings, c_feature2d_sift::options * args);
#endif

#if HAVE_FEATURE2D_SURF
bool save_settings(c_config_setting settings, const c_feature2d_surf::options & args);
bool load_settings(c_config_setting settings, c_feature2d_surf::options * args);
#endif

bool save_settings(c_config_setting settings, const c_feature2d_mser::options & args);
bool load_settings(c_config_setting settings, c_feature2d_mser::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_fast::options & args);
bool load_settings(c_config_setting settings, c_feature2d_fast::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_agast::options & args);
bool load_settings(c_config_setting settings, c_feature2d_agast::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_gftt::options & args);
bool load_settings(c_config_setting settings, c_feature2d_gftt::options * args);

bool save_settings(c_config_setting settings, const c_feature2d_blob::options & args);
bool load_settings(c_config_setting settings, c_feature2d_blob::options * args);

#if HAVE_FEATURE2D_STAR
bool save_settings(c_config_setting settings, const c_feature2d_star::options & args);
bool load_settings(c_config_setting settings, c_feature2d_star::options * args);
#endif

#if HAVE_FEATURE2D_MSD
bool save_settings(c_config_setting settings, const c_feature2d_msd::options & args);
bool load_settings(c_config_setting settings, c_feature2d_msd::options * args);
#endif

#if HAVE_FEATURE2D_HL
bool save_settings(c_config_setting settings, const c_feature2d_hl::options & args);
bool load_settings(c_config_setting settings, c_feature2d_hl::options * args);
#endif

#if HAVE_FEATURE2D_FREAK
bool save_settings(c_config_setting settings, const c_feature2d_freak::options & args);
bool load_settings(c_config_setting settings, c_feature2d_freak::options * args);
#endif

#if HAVE_FEATURE2D_BRIEF
bool save_settings(c_config_setting settings, const c_feature2d_brief::options & args);
bool load_settings(c_config_setting settings, c_feature2d_brief::options * args);
#endif

#if HAVE_FEATURE2D_LUCID
bool save_settings(c_config_setting settings, const c_feature2d_lucid::options & args);
bool load_settings(c_config_setting settings, c_feature2d_lucid::options * args);
#endif

#if HAVE_FEATURE2D_LATCH
bool save_settings(c_config_setting settings, const c_feature2d_latch::options & args);
bool load_settings(c_config_setting settings, c_feature2d_latch::options * args);
#endif

#if HAVE_FEATURE2D_DAISY
bool save_settings(c_config_setting settings, const c_feature2d_daisy::options & args);
bool load_settings(c_config_setting settings, c_feature2d_daisy::options * args);
#endif

#if HAVE_FEATURE2D_VGG
bool save_settings(c_config_setting settings, const c_feature2d_vgg::options & args);
bool load_settings(c_config_setting settings, c_feature2d_vgg::options * args);
#endif

#if HAVE_FEATURE2D_BOOST
bool save_settings(c_config_setting settings, const c_feature2d_boost::options & args);
bool load_settings(c_config_setting settings, c_feature2d_boost::options * args);
#endif

#if HAVE_STAR_EXTRACTOR
bool save_settings(c_config_setting settings, const c_feature2d_star_extractor::options & args);
bool load_settings(c_config_setting settings, c_feature2d_star_extractor::options * args);
#endif

#if HAVE_MORPH_EXTRACTOR
bool save_settings(c_config_setting settings, const c_feature2d_morph_extractor::options & args);
bool load_settings(c_config_setting settings, c_feature2d_morph_extractor::options * args);
#endif

#if HAVE_TRIANGLE_EXTRACTOR
bool load_settings(c_config_setting settings, c_feature2d_triangle_extractor::options * args);
bool save_settings(c_config_setting settings, const c_feature2d_triangle_extractor::options & args);
#endif

bool save_settings(c_config_setting settings, const c_sparse_feature_detector_options & args);
bool load_settings(c_config_setting settings, c_sparse_feature_detector_options * opts);

bool save_settings(c_config_setting settings, const c_sparse_feature_descriptor_options & args);
bool load_settings(c_config_setting settings, c_sparse_feature_descriptor_options * opts);

bool save_settings(c_config_setting settings, const c_sparse_feature_extractor_options & args);
bool load_settings(c_config_setting settings, c_sparse_feature_extractor_options * opts);


bool save_settings(c_config_setting settings, const c_flann_linear_index_options & args);
bool load_settings(c_config_setting settings, c_flann_linear_index_options * opts);

bool save_settings(c_config_setting settings, const c_flann_kdtree_index_options & args);
bool load_settings(c_config_setting settings, c_flann_kdtree_index_options * opts);

bool save_settings(c_config_setting settings, const c_flann_kmeans_index_options & args);
bool load_settings(c_config_setting settings, c_flann_kmeans_index_options * opts);

bool save_settings(c_config_setting settings, const c_flann_composite_index_options & args);
bool load_settings(c_config_setting settings, c_flann_composite_index_options * opts);

bool save_settings(c_config_setting settings, const c_flann_hierarchical_index_options & args);
bool load_settings(c_config_setting settings, c_flann_hierarchical_index_options  * opts);

bool save_settings(c_config_setting settings, const c_flann_lsh_index_options & args);
bool load_settings(c_config_setting settings, c_flann_lsh_index_options * opts);

bool save_settings(c_config_setting settings, const c_flann_autotuned_index_options & args);
bool load_settings(c_config_setting settings, c_flann_autotuned_index_options  * opts);

bool save_settings(c_config_setting settings, const c_flann_index_options & args);
bool load_settings(c_config_setting settings, c_flann_index_options * opts);

bool save_settings(c_config_setting settings, const c_hamming_distance_feature2d_matcher_options & args);
bool load_settings(c_config_setting settings, c_hamming_distance_feature2d_matcher_options * opts);

bool save_settings(c_config_setting settings, const c_flann_based_feature2d_matcher_options & args);
bool load_settings(c_config_setting settings, c_flann_based_feature2d_matcher_options * opts);

bool save_settings(c_config_setting settings, const c_optflowpyrlk_feature2d_matcher_options & args);
bool load_settings(c_config_setting settings, c_optflowpyrlk_feature2d_matcher_options * opts);

bool save_settings(c_config_setting settings, const c_snorm_based_feature2d_matcher_options & args);
bool load_settings(c_config_setting settings, c_snorm_based_feature2d_matcher_options * opts);

bool save_settings(c_config_setting settings, const c_triangle_matcher_options & args);
bool load_settings(c_config_setting settings, c_triangle_matcher_options * opts);

bool save_settings(c_config_setting settings, const c_feature2d_matcher_options & args);
bool load_settings(c_config_setting settings, c_feature2d_matcher_options * opts);


bool save_settings(c_config_setting settings, const c_feature2d::sptr & obj);
bool save_settings(c_config_setting settings, const c_feature2d_matcher::sptr & obj);

///////////////////////////////////////////////////////////////////////

c_feature2d_matcher::sptr create_sparse_feature_matcher(
    const c_sparse_feature_extractor::sptr & extractor,
    const std::string & matcher_spec);

#endif /* __feature2d_settings_h__ */
