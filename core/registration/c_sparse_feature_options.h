/*
 * c_sparse_feature_options.h
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sparse_feature_options_h__
#define __c_sparse_feature_options_h__

#include <core/feature2d/feature2d.h>

struct c_sparse_feature_options {
  bool enable_sparse_features = true;
  c_sparse_feature_extractor_options sparse_feature_extractor;
  c_feature2d_matcher_options sparse_feature_matcher;
};




#endif /* __c_sparse_feature_options_h__ */
