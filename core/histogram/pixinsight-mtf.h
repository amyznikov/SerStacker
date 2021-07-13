/*
 * pixinsight-mtf.h
 *
 *  Created on: Dec 14, 2020
 *      Author: amyznikov
 *
 * Midtones Transfer Function (MTF) as it defined in PixInsight
 *
 *   See section 'Midtones balance' at
 *     <http://pixinsight.com/doc/tools/HistogramTransformation/HistogramTransformation.html>
 *
 * The formal definition of an MTF function with midtones balance parameter 'm' is given by
 *
 *  MTF(x,m) =
 *           0.0   for x == 0
 *           0.5   for x == m
 *           1.0   for x == 1
 *           (m - 1) * x / ( (2 * m - 1) * x - m) otherwise
 */
#pragma once
#ifndef __mtf_pixinsight_h__
#define __mtf_pixinsight_h__

#include <opencv2/opencv.hpp>

/** @brief Pixinsight MTF for nominal range 0 <= x <= 1, 0 <= m <= 1

Midtones Transfer Function (MTF) as it is defined in PixInsight
    mtf(x,m) =  (m - 1) * x / ((2 * m - 1) * x - m)

See section 'Midtones balance' at
  <http://pixinsight.com/doc/tools/HistogramTransformation/HistogramTransformation.html>
*/
double mtf_pixinsight(double x,
    double m);

/** @brief Apply Pixinsight Midtones Transfer Function to image
 */
bool apply_mtf_pixinsight(cv::InputArray src,
    cv::OutputArray dst,
    double src_min, double src_max,
    double dst_min, double dst_max,
    double shadows,
    double highlights,
    double midtones,
    int ddepth=-1);

/** @brief Auto adjust midtones balance for given image histogram.
Based on findMidtonesBalance() from siril source code.

Out[ut results are normalized to range [0..1]
*/
bool find_midtones_balance_pixinsight(cv::InputArray input_image_histogram,
    double * output_shadows,
    double * output_highlights,
    double * output_midtones);


#endif /* __mtf_pixinsight_h__ */
