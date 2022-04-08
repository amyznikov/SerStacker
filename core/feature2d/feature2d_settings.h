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

bool load_settings(c_config_setting settings,
    c_feature2d_orb::options * args);
bool save_settings(c_config_setting settings,
    c_feature2d_orb::options & args);

bool load_settings(c_config_setting settings,
    c_feature2d_brisk::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_brisk::options & args);

bool load_settings(c_config_setting settings,
    c_feature2d_kaze::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_kaze::options &);

bool load_settings(c_config_setting settings,
    c_feature2d_akaze::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_akaze::options & );

#if HAVE_FEATURE2D_SIFT
bool load_settings(c_config_setting settings,
    c_feature2d_sift::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_sift::options & );
#endif

#if HAVE_FEATURE2D_SURF
bool load_settings(c_config_setting settings,
    c_feature2d_surf::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_surf::options & );
#endif

bool load_settings(c_config_setting settings,
    c_feature2d_mser::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_mser::options & );

bool load_settings(c_config_setting settings,
    c_feature2d_fast::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_fast::options & );

bool load_settings(c_config_setting settings,
    c_feature2d_agast::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_agast::options & );

bool load_settings(c_config_setting settings,
    c_feature2d_gftt::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_gftt::options & );

bool load_settings(c_config_setting settings,
    c_feature2d_blob::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_blob::options & );

#if HAVE_FEATURE2D_STAR
bool load_settings(c_config_setting settings,
    c_feature2d_star::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_star::options & );
#endif

#if HAVE_FEATURE2D_MSD
bool load_settings(c_config_setting settings,
    c_feature2d_msd::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_msd::options & );
#endif

#if HAVE_FEATURE2D_HL
bool load_settings(c_config_setting settings,
    c_feature2d_hl::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_hl::options & );
#endif

#if HAVE_FEATURE2D_FREAK
bool load_settings(c_config_setting settings,
    c_feature2d_freak::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_freak::options & );
#endif

#if HAVE_FEATURE2D_BRIEF
bool load_settings(c_config_setting settings,
    c_feature2d_brief::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_brief::options & );
#endif

#if HAVE_FEATURE2D_LUCID
bool load_settings(c_config_setting settings,
    c_feature2d_lucid::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_lucid::options & );
#endif

#if HAVE_FEATURE2D_LATCH
bool load_settings(c_config_setting settings,
    c_feature2d_latch::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_latch::options & );
#endif

#if HAVE_FEATURE2D_DAISY
bool load_settings(c_config_setting settings,
    c_feature2d_daisy::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_daisy::options & );
#endif

#if HAVE_FEATURE2D_VGG
bool load_settings(c_config_setting settings,
    c_feature2d_vgg::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_vgg::options & );
#endif

#if HAVE_FEATURE2D_BOOST
bool load_settings(c_config_setting settings,
    c_feature2d_boost::options * args);
bool save_settings(c_config_setting settings,
    const c_feature2d_boost::options & );
#endif

bool save_settings(c_config_setting settings,
    const c_feature2d::ptr & obj);


bool load_settings(c_config_setting settings,
    c_sparse_feature_detector_options * options);

bool save_settings(c_config_setting settings,
    const c_sparse_feature_detector_options & options);

bool load_settings(c_config_setting settings,
    c_sparse_feature_descriptor_options * options);

bool save_settings(c_config_setting settings,
    const c_sparse_feature_descriptor_options & options);

bool load_settings(c_config_setting settings,
    c_sparse_feature_extractor_options * options);

bool save_settings(c_config_setting settings,
    const c_sparse_feature_extractor_options & options);



bool save_settings(c_config_setting settings,
    const c_flann_linear_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_linear_index_options * options);

bool save_settings(c_config_setting settings,
    const c_flann_kdtree_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_kdtree_index_options * options);

bool save_settings(c_config_setting settings,
    const c_flann_kmeans_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_kmeans_index_options* options);

bool save_settings(c_config_setting settings,
    const c_flann_composite_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_composite_index_options * options);

bool save_settings(c_config_setting settings,
    const c_flann_hierarchical_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_hierarchical_index_options  * options);

bool save_settings(c_config_setting settings,
    const c_flann_lsh_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_lsh_index_options * options);

bool save_settings(c_config_setting settings,
    const c_flann_autotuned_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_autotuned_index_options  * options);

bool save_settings(c_config_setting settings,
    const c_flann_index_options & options);

bool load_settings(c_config_setting settings,
    c_flann_index_options * options);

bool save_settings(c_config_setting settings,
    const c_hamming_distance_feature2d_matcher_options & options);

bool load_sett3ings(c_config_setting settings,
    c_hamming_distance_feature2d_matcher_options * options);

bool save_settings(c_config_setting settings,
    const c_flann_based_feature2d_matcher_options & options);

bool load_settings(c_config_setting settings,
    c_flann_based_feature2d_matcher_options * options);

bool save_settings(c_config_setting settings,
    const c_snorm_based_feature2d_matcher_options & options);

bool load_settings(c_config_setting settings,
    c_snorm_based_feature2d_matcher_options * options);

bool save_settings(c_config_setting settings,
    const c_feature2d_matcher_options & options);

bool load_settings(c_config_setting settings,
    c_feature2d_matcher_options * options);



bool save_settings(c_config_setting settings,
    const c_feature2d_matcher::ptr & obj);


///////////////////////////////////////////////////////////////////////

c_sparse_feature_extractor::ptr create_sparse_feature_extractor(
    c_config_setting settings);

c_feature2d_matcher::ptr create_sparse_feature_matcher(
    c_config_setting settings);

c_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_sparse_feature_extractor::ptr & extractor,
    c_config_setting settings);

c_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_sparse_feature_extractor::ptr & extractor,
    const std::string & matcher_spec);

#endif /* __feature2d_settings_h__ */
