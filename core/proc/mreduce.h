/*
 * mreduce.h
 *
 *  Created on: Aug 12, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __mreduce_h__
#define __mreduce_h__

#include <opencv2/opencv.hpp>

/**
 * @brief Analog of reduce() with operation mask support.
 *
 * @param src input 2D matrix.
 * @param dst output vector. Its size and type is defined by dim and dtype parameters.
 * @param dim dimension index along which the matrix is reduced. 0 means that the matrix is reduced to
 * a single row. 1 means that the matrix is reduced to a single column.
 * @param rtype reduction operation that could be one of #ReduceTypes
 * @param dtype when non-negative the output vector type will be CV_MAKE_TYPE(CV_MAT_DEPTH(dtype), src.channels()).
 * , when negative the output will have the same type as dst.type() if dst.isFixedType(),
 *   std::max(src.depth(), CV_32F) for cv::REDUCE_SUM and cv::REDUCE_SUM2, or src.depth() otherwise.
 */
bool mreduce(cv::InputArray src, cv::OutputArray dst, int dim, cv::ReduceTypes rtype,
    int ddepth = -1, cv::InputArray mask = cv::noArray());



#endif /* __mreduce_h__ */
