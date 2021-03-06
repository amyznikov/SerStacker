/*
 * debayer.h
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#ifndef __debayer_h__
#define __debayer_h__

#include <opencv2/opencv.hpp>

/// @brief COLORID
/// SER file color id.
/// The numercal values are selected from paper "SER format description version 3",
///     by Heiko Wilkens and Grischa Hahn, at 2014 Feb 06.
/// Addidional special values starting from optical flow added by me.
enum COLORID : int32_t {
  COLORID_UNKNOWN = -1,
  COLORID_MONO = 0,
  COLORID_BAYER_RGGB = 8,
  COLORID_BAYER_GRBG = 9,
  COLORID_BAYER_GBRG = 10,
  COLORID_BAYER_BGGR = 11,
  COLORID_BAYER_CYYM = 16,
  COLORID_BAYER_YCMY = 17,
  COLORID_BAYER_YMCY = 18,
  COLORID_BAYER_MYYC = 19,
  COLORID_RGB = 100,
  COLORID_BGR = 101,
  COLORID_BGRA = 201,

  COLORID_OPTFLOW = 501,
};


enum DEBAYER_ALGORITHM {
  DEBAYER_DISABLE,  // Don't debayer
  DEBAYER_NN,       // Use OpenCV nearest-neighboor interpolation with cv::demosaicing()
  DEBAYER_VNG,      // Use OpenCV VNG interpolation with cv::demosaicing()
  DEBAYER_EA,       // Use OpenCV EA (edge aware) interpolation with cv::demosaicing()
  DEBAYER_GB,       // Use GaussianBlur() interpolation
  DEBAYER_GBNR      // Use GaussianBlur() interpolation with simple bad pixels filtering
};
//
//const extern struct DEBAYER_ALGORITHM_desc {
//  const char * name;
//  enum DEBAYER_ALGORITHM value;
//} debayer_algorithms[];
//
//std::string toStdString(enum DEBAYER_ALGORITHM m);
//enum DEBAYER_ALGORITHM fromStdString(const std::string & s,
//    enum DEBAYER_ALGORITHM defval);


/** @brief
 * Return true if colorid is one from known bayer patterns (COLORID_BAYER_XXXX)
 */
bool is_bayer_pattern(enum COLORID colorid);


/** @brief
 * Extract src into dense 4-channel dst matrix with 4 bayer planes ordered as[ R G1 B G2 ].
 * The output size of dst is twice smaller than src
 */
bool extract_bayer_planes(cv::InputArray src, cv::OutputArray dst,
    enum COLORID colorid);


/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
bool bayer_planes_to_bgr(cv::InputArray src, cv::OutputArray dst,
    int ddepth = -1);


/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix using GB interpolaton.
 * The output size of dst is twce large to src
 */
bool gbinterpolation(cv::InputArray src, cv::OutputArray dst,
    enum COLORID colorid);

/** @brief
 * Bayer Demosaicing
 */
bool debayer(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid,
    enum DEBAYER_ALGORITHM algo = DEBAYER_GB);




#endif /* __debayer_h__ */
