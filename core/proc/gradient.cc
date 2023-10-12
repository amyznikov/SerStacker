/*
 * gradient.cc
 *
 *  Created on: May 1, 2023
 *      Author: amyznikov
 *
 *  Compute image gradient using central finite differences
 *    <https://en.wikipedia.org/wiki/Finite_difference_coefficient>
 *
 *  Very similar to cv::Sobel() except kernel size is always 1 x (2 * kradius + 1)
 *
 */

#include "gradient.h"
#include <core/debug.h>

/**
 * @see <https://en.wikipedia.org/wiki/Finite_difference_coefficient>
 */

static cv::Mat select_kernel(int order, int kradius)
{
  static float k11[] = { -0.5, +0., 0.5 };
  static const cv::Matx<float, 1, 3> K11 = cv::Matx<float, 1, 3>(k11);

  static float k12[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> K12 = cv::Matx<float, 1, 5>(k12);// / 1.5;

  static float k13[] = { -1. / 60, +3. / 20, -3. / 4, +0., 3. / 4, -3. / 20, +1. / 60 };
  static const cv::Matx<float, 1, 7> K13 = cv::Matx<float, 1, 7>(k13);// / 1.83333333333333333333;

  static float k14[] = { +1. / 280, -4. / 105, +1. / 5, -4. / 5, +0., +4. / 5, -1. / 5, +4. / 105, -1. / 280 };
  static const cv::Matx<float, 1, 9> K14 = cv::Matx<float, 1, 9>(k14);// / 2.08333333333333333333;

  /////////////////////////////////////////////////////////

  static float k21[] = { +1., -2., +1. };
  static const cv::Matx<float, 1, 3> K21 = cv::Matx<float, 1, 3>(k21);// / 4.0;

  static float k22[] = { -1. / 12, +4. / 3, -5. / 2, +4. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> K22 = cv::Matx<float, 1, 5>(k22);// / 5.33333333333333333333;

  static float k23[] = { +1. / 90, -3. / 20, +3. / 2, -49. / 18, +3. / 2, -3. / 20, +1 / 90 };
  static const cv::Matx<float, 1, 7> K23 = cv::Matx<float, 1, 7>(k23);// / 6.04444444444444444444;

  static float k24[] = { -1. / 560, +8. / 315, -1. / 5, +8. / 5, -205. / 72, +8. / 5, -1. / 5, +8. / 315, -1. / 560 };
  static const cv::Matx<float, 1, 9> K24 = cv::Matx<float, 1, 9>(k24);// / 6.50158730158730158730;

  /////////////////////////////////////////////////////////

  static float k32[] = { -1. / 2, +1., +0., -1., +1. / 2 };
  static const cv::Matx<float, 1, 5> K32 = cv::Matx<float, 1, 5>(k32);// / 3.;

  static float k33[] = { +1. / 8, -1., +13. / 8, +0., -13. / 8, +1., -1. / 8 };
  static const cv::Matx<float, 1, 7> K33 = cv::Matx<float, 1, 7>(k33);// / 5.5;

  static float k34[] = { -7. / 240, +3. / 10, -169. / 120, +61. / 30, +0., -61. / 30, +169. / 120, -3. / 10, +7. / 240 };
  static const cv::Matx<float, 1, 9> K34 = cv::Matx<float, 1, 9>(k34);// / 7.54166666666666666667;

  /////////////////////////////////////////////////////////

  static float k42[] = { +1., -4., +.6, -4., +1. };
  static const cv::Matx<float, 1, 5> K42 = cv::Matx<float, 1, 5>(k42);// / 10.6;

  static float k43[] = { -1. / 6, +2., -13. / 2, +28. / 3, -13. / 2, +2., -1. / 6 };
  static const cv::Matx<float, 1, 7> K43 = cv::Matx<float, 1, 7>(k43);// / 26.66666666666666666667;

  static float k44[] =
      { 7. / 240, -2. / 5, +169. / 60, -122. / 15, +91. / 8, -122. / 15, +169. / 60, -2. / 5, +7. / 240 };
  static const cv::Matx<float, 1, 9> K44 = cv::Matx<float, 1, 9>(k44);// / 34.13333333333333333333;

  /////////////////////////////////////////////////////////

  static float k53[] = { -1. / 2, +2., -5. / 2, +0., +5. / 2, -2., +1. / 2 };
  static const cv::Matx<float, 1, 7> K53 = cv::Matx<float, 1, 7>(k53);// / 10.;

  static float k54[] = { +1. / 6, -3. / 2, +13. / 3, -29. / 6, +0., +29. / 6, -13. / 3, +3. / 2, -1. / 6 };
  static const cv::Matx<float, 1, 9> K54 = cv::Matx<float, 1, 9>(k54);// / 21.66666666666666666667;

  static float k55[] = { -13. / 288, +19. / 36, -87. / 32, +13. / 2, -323. / 48, +0., +323. / 48, -13. / 2, +87. / 32,
      -19. / 36, +13. / 288 };
  static const cv::Matx<float, 1, 11> K55 = cv::Matx<float, 1, 11>(k55);// / 33.04166666666666666667;

  /////////////////////////////////////////////////////////

  static float k63[] = { +1., -6., +15., -20., +15., -6., +1. };
  static const cv::Matx<float, 1, 7> K63 = cv::Matx<float, 1, 7>(k63);// / 64.;

  static float k64[] = { -1. / 4, +3., -13., +29., -75. / 2, +29., -13., +3., -1. / 4 };
  static const cv::Matx<float, 1, 9> K64 = cv::Matx<float, 1, 9>(k64);// / 128.;

  static float k65[] = { +13. / 240, -19. / 24, +87. / 16, -39. / 2, +323. / 8, -1023. / 20, +323. / 8, -39. / 2, +87.
      / 16, -19. / 24, +13. / 240 };
  static const cv::Matx<float, 1, 11> K65 = cv::Matx<float, 1, 11>(k65);// / 183.46666666666666666667;

  switch (order) {
    case 1:
      if( kradius < 1 ) {
        kradius = 1;
      }
      else if( kradius > 4 ) {
        kradius = 4;
      }
      switch (kradius) {
        case 1:
          return cv::Mat(K11);
        case 2:
          return cv::Mat(K12);
        case 3:
          return cv::Mat(K13);
        case 4:
          return cv::Mat(K14);
      }
      break;

    case 2:
      if( kradius < 1 ) {
        kradius = 1;
      }
      else if( kradius > 4 ) {
        kradius = 4;
      }
      switch (kradius) {
        case 1:
          return cv::Mat(K21);
        case 2:
          return cv::Mat(K22);
        case 3:
          return cv::Mat(K23);
        case 4:
          return cv::Mat(K24);
      }
      break;

    case 3:
      if( kradius < 2 ) {
        kradius = 2;
      }
      else if( kradius > 4 ) {
        kradius = 4;
      }
      switch (kradius) {
        case 2:
          return cv::Mat(K32);
        case 3:
          return cv::Mat(K33);
        case 4:
          return cv::Mat(K34);
      }
      break;

    case 4:
      if( kradius < 2 ) {
        kradius = 2;
      }
      else if( kradius > 4 ) {
        kradius = 4;
      }
      switch (kradius) {
        case 2:
          return cv::Mat(K42);
        case 3:
          return cv::Mat(K43);
        case 4:
          return cv::Mat(K44);
      }
      break;

    case 5:
      if( kradius < 3 ) {
        kradius = 3;
      }
      else if( kradius > 5 ) {
        kradius = 5;
      }
      switch (kradius) {
        case 3:
          return cv::Mat(K53);
        case 4:
          return cv::Mat(K54);
        case 5:
          return cv::Mat(K55);
      }
      break;

    case 6:
      if( kradius < 3 ) {
        kradius = 3;
      }
      else if( kradius > 5 ) {
        kradius = 5;
      }
      switch (kradius) {
        case 3:
          return cv::Mat(K63);
        case 4:
          return cv::Mat(K64);
        case 5:
          return cv::Mat(K65);
      }
      break;
  }

  return cv::Mat();
}

bool compute_gradient(cv::InputArray src, cv::OutputArray dst,
    int dx, int dy,
    int kradius,
    int ddepth ,
    double delta,
    double scale)
{

  cv::Mat Kx, Ky;

  if( dx > 0 && (Kx = select_kernel(dx, kradius)).empty() ) {
    CF_ERROR("select_kernel() fails for dx order %d kradius %d", dx, kradius);
    return false;
  }

  if( dy > 0 && (Ky = select_kernel(dy, kradius)).empty() ) {
    CF_ERROR("select_kernel() fails for dy order %d kradius %d", dy, kradius);
    return false;
  }

  if( dx > 0 && dy <= 0 ) {
    if( scale == 1 ) {
      cv::filter2D(src, dst, ddepth, Kx, cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
    else {
      cv::filter2D(src, dst, ddepth, Kx * scale, cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
  }
  else if( dx <= 0 && dy > 0 ) {
    if( scale == 1 ) {
      cv::filter2D(src, dst, ddepth, Ky.t(), cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
    else {
      cv::filter2D(src, dst, ddepth, Ky.t() * scale, cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
  }
  else if( dx > 0 && dy > 0 )   {
    if( scale == 1 ) {
      cv::sepFilter2D(src, dst, ddepth, Kx, Ky.t(), cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
    else {
      cv::sepFilter2D(src, dst, ddepth, Kx * scale, Ky.t() * scale, cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
    }
  }
  else {
    CF_ERROR("Invalid argument: the order of derivative is not specified: dx=%d dy=%d kradius=%d", dx, dy, kradius);
    return false;
  }

  return true;
}
