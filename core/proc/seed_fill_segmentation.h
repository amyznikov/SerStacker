/*
 * seed_fill_segmentation.h
 *
 *  Created on: Oct 31, 2020
 *      Author: amyznikov
 *
 *
 *  C++/OpenCV implementation of Connected component labeling algorithm from paper by M. Emre Celebi,
 *    "A Simple and Efficient Algorithm for Connected Component Labeling in Color Images"
 *
 *  https://ui.adsabs.harvard.edu/abs/2012SPIE.8295E..1HC/abstract
 *
 */

#ifndef __seedfill_segmentation_h__
#define __seedfill_segmentation_h__

#include <opencv2/opencv.hpp>


/**
 * args:
 *    src           : input image of any opencv type (up to 4 channels)
 *    output_labels : output CV_32S labels image of the same size as src
 *    threshold     : minimum difference between pixels (equclidian norm of pixel difference is used).
 *                    Label indexes are in inclusive range [1..max_label].
 *
 * return:
 *    int max_label:  maximal output label in output_labels[][] image.
 */
int seed_fill_segmentation(cv::InputArray src,
    cv::Mat1i & output_labels,
    double threshold);


#endif /* __seedfill_segmentation_h__ */
