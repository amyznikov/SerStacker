/*
 * threshold.h
 *
 *  Created on: May 21, 2017
 *      Author: amyznikov
 */

#pragma once

#ifndef __threshold_h__
#define __threshold_h__

#include <opencv2/opencv.hpp>

///////////////////////////////////////////////////////////////////////////////

double get_otsu_threshold_8u(const cv::Mat& _src);
double get_otsu_threshold(cv::InputArray _src, cv::InputArray _mask = cv::noArray());
void otsu_threshold(const cv::Mat & _src, cv::Mat & _dst, double ts = 1, cv::InputArray _mask = cv::noArray() );


double get_minimum_threshold(cv::InputArray src, cv::InputArray _mask = cv::noArray());
int get_minimum_threshold_histogram_bin(const int data[], int hlen);
int get_minimum_threshold_histogram_bin(const float data[], int hlen);


double get_triangle_threshold(cv::InputArray src, cv::InputArray _mask = cv::noArray());
double get_moments_threshold(cv::InputArray src, cv::InputArray _mask = cv::noArray());
double get_yen_threshold(cv::InputArray src, cv::InputArray _mask = cv::noArray());




double get_isodata_threshold(cv::InputArray _src, cv::InputArray _mask = cv::noArray());
void isodata_threshold(const cv::Mat & src, cv::Mat & dst, double maxval, cv::InputArray _mask = cv::noArray(), int ttype = cv::THRESH_BINARY);

double get_huang_threshold(cv::InputArray src, cv::InputArray _mask = cv::noArray());
void huang_threshold(const cv::Mat & _src, cv::Mat & _dst, cv::InputArray _mask = cv::noArray());

void mean_threshold(cv::InputArray  _src, cv::Mat & _dst, cv::InputArray _mask = cv::noArray());


enum THRESHOLD_TYPE {
  THRESHOLD_TYPE_VALUE,
  THRESHOLD_TYPE_OTSU,
  THRESHOLD_TYPE_TRIANGLE,
  THRESHOLD_TYPE_MOMENTS,
  THRESHOLD_TYPE_ISODATA,
  THRESHOLD_TYPE_HUANG,
  THRESHOLD_TYPE_YEN,
  THRESHOLD_TYPE_MEAN,
  THRESHOLD_TYPE_MINIMUM,
  THRESHOLD_TYPE_NOISE
};

double get_threshold_value(cv::InputArray image, cv::InputArray mask,
    THRESHOLD_TYPE threshold_type,
    double value = 0);

///////////////////////////////////////////////////////////////////////////////
#endif /* __glv_threshold_h__*/
