/*
 * mtf-histogram.h
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __mtf_histogram_h__
#define __mtf_histogram_h__

#include "c_midtones_transfer_function.h"



// @brief build histogram for given multi-channel image.
// Output is single-channel CV_32FC1 matrix of size 'nbins rows' x 'image channels columns'.
// if input mask is not empty then it must be sigle-channel CV_8U matrix of the same size as input image.
bool create_output_histogram(const c_midtones_transfer_function * mtf,
    cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    /*[in, out]*/ double * minval,
    /*[in, out]*/ double * maxval,
    int nbins = -1,
    bool cumulative = false,
    bool scaled = false);



#endif /* __mtf_histogram_h__ */
