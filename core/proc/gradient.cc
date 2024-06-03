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
 *
 *  TODO: <https://bartwronski.com/2021/02/28/computing-gradients-on-grids-forward-central-and-diagonal-differences>
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


bool compute_sobel_gradients(cv::InputArray src,
    cv::OutputArray gx,
    cv::OutputArray gy,
    int ddepth,
    int borderType)
{

  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  if( ddepth < 0 ) {
    ddepth = std::max(src.depth(), CV_32F);
  }

  cv::sepFilter2D(src, gx, ddepth, Kx, Ky, cv::Point(-1, -1), 0, borderType);
  cv::sepFilter2D(src, gy, ddepth, Ky, Kx, cv::Point(-1, -1), 0, borderType);

  return true;
}

// Image gradient estimate using Diagonal differences in 3x3 window
// https://bartwronski.com/2021/02/28/computing-gradients-on-grids-forward-central-and-diagonal-differences/
bool compute_diagonal_gradients(cv::InputArray src, cv::OutputArray gx, cv::OutputArray gy, int ddepth, int borderType)
{
  static const float S = M_SQRT1_2 / 2;

  static const thread_local cv::Matx33f K1(
      -S,  -S,   0,
      -S,   0,  +S,
       0,  +S,  +S);

  static const thread_local cv::Matx33f K2(
       0,   +S,   +S,
      -S,    0,   +S,
      -S,   -S,    0);

  cv::Mat g1, g2;

  cv::filter2D(src, g1, ddepth, K1, cv::Point(1, 1), 0, borderType);
  cv::filter2D(src, g2, ddepth, K2, cv::Point(1, 1), 0, borderType);

//  [gx] = [ca -sa]  [g1]
//  [gy]   [sa  ca]  [g2]

  static const float sa = sin(-M_PI_4);
  static const float ca = cos(-M_PI_4);

  cv::addWeighted(g1, ca, g2, -sa, 0, gx); // gx = ca * g1 - sa * g2
  cv::addWeighted(g1, -sa, g2, -ca, 0, gy); //  gy = -(sa * g1 + ca * g2)


  return true;
}

bool compute_second_sobel_derivatives(cv::InputArray src,
    cv::OutputArray gxx,
    cv::OutputArray gxy,
    cv::OutputArray gyy,
    int ddepth,
    int borderType)
{
  static thread_local cv::Mat Kx1, Ky1, Kx2, Ky2;
  if( Kx1.empty() ) {

    cv::getDerivKernels(Kx1, Ky1, 1, 1, 3, true, CV_32F);
    cv::getDerivKernels(Kx2, Ky2, 2, 0, 3, true, CV_32F);
    //    Kx1 *= M_SQRT2;
    //    Ky1 *= M_SQRT2;
  }

  if( ddepth < 0 ) {
    ddepth = std::max(src.depth(), CV_32F);
  }

  cv::sepFilter2D(src, gxx, ddepth, Kx2, Ky2, cv::Point(-1, -1), 0, borderType);
  cv::sepFilter2D(src, gyy, ddepth, Ky2, Kx2, cv::Point(-1, -1), 0, borderType);
  cv::sepFilter2D(src, gxy, ddepth, Kx1, Ky1, cv::Point(-1, -1), 0, borderType);

  return true;
}


static inline void compute_eigen_values(const float a, const float b, const float d, float & mu1, float & mu2)
{
  // @see <Eigenvectors and eigenvalues of real symmetric matrices.pdf>
  // A = | a b |
  //     | b d |
  //
  const float T =
      a + d;

  const float D =
       std::sqrt((a - d) * (a - d) + 4 * b * b);

  mu1 =
      (T + D) * 0.5f;

  mu2 =
      (T - D) * 0.5f;
}

template<class _Tp>
static void compute_hessian_eigenvalues_(cv::InputArray _gxx, cv::InputArray _gxy, cv::InputArray _gyy,
    cv::OutputArray _mu1, cv::OutputArray _mu2)
{
  const cv::Size image_size =
      _gxx.size();

  const int cn =
      _gxx.channels();

  const cv::Mat_<_Tp> gxx =
      _gxx.getMat();

  const cv::Mat_<_Tp> gxy =
      _gxy.getMat();

  const cv::Mat_<_Tp> gyy =
      _gyy.getMat();

  if( _mu1.needed() && _mu2.needed() ) {

    _mu1.create(image_size,
        CV_MAKETYPE(CV_32F, cn));

    _mu2.create(image_size,
        CV_MAKETYPE(CV_32F, cn));

    cv::Mat_<float> mu1 =
        _mu1.getMatRef();

    cv::Mat_<float> mu2 =
        _mu2.getMatRef();

    for( int y = 0; y < image_size.height; ++y ) {

      const _Tp * gxxp = gxx[y];
      const _Tp * gxyp = gxy[y];
      const _Tp * gyyp = gyy[y];
      float * mu1p = mu1[y];
      float * mu2p = mu2[y];

      for( int x = 0; x < image_size.width; ++x ) {

        for( int c = 0; c < cn; ++c ) {

          const int i =
              x * cn + c;

          const float a =
              gxxp[i];

          const float b =
              gxyp[i];

          const float d =
              gyyp[i];

          float mu1, mu2;

          compute_eigen_values(a, b, d, mu1, mu2);

          if( std::abs(mu1) > std::abs(mu2) ) {
            mu1p[i] = mu1;
            mu2p[i] = mu2;
          }
          else {
            mu1p[i] = mu2;
            mu2p[i] = mu1;
          }
        }
      }
    }

  }
  else if( _mu1.needed() ) {

    _mu1.create(image_size,
        CV_MAKETYPE(CV_32F, cn));

    cv::Mat_<float> mu1 =
        _mu1.getMatRef();

    for( int y = 0; y < image_size.height; ++y ) {

      const _Tp * gxxp = gxx[y];
      const _Tp * gxyp = gxy[y];
      const _Tp * gyyp = gyy[y];
      float * mu1p = mu1[y];

      for( int x = 0; x < image_size.width; ++x ) {

        for( int c = 0; c < cn; ++c ) {

          const int i =
              x * cn + c;

          const float a =
              gxxp[i];

          const float b =
              gxyp[i];

          const float d =
              gyyp[i];

          float mu1, mu2;

          compute_eigen_values(a, b, d, mu1, mu2);

          if( std::abs(mu1) > std::abs(mu2) ) {
            mu1p[i] = mu1;
          }
          else {
            mu1p[i] = mu2;
          }
        }
      }
    }

  }
  else if( _mu2.needed() ) {

    _mu2.create(image_size,
        CV_MAKETYPE(CV_32F, cn));

    cv::Mat_<float> mu2 =
        _mu2.getMatRef();

    for( int y = 0; y < image_size.height; ++y ) {

      const _Tp * gxxp = gxx[y];
      const _Tp * gxyp = gxy[y];
      const _Tp * gyyp = gyy[y];
      float * mu2p = mu2[y];

      for( int x = 0; x < image_size.width; ++x ) {

        for( int c = 0; c < cn; ++c ) {

          const int i =
              x * cn + c;

          const float a =
              gxxp[i];

          const float b =
              gxyp[i];

          const float d =
              gyyp[i];

          float mu1, mu2;

          compute_eigen_values(a, b, d, mu1, mu2);

          if( std::abs(mu1) > std::abs(mu2) ) {
            mu2p[i] = mu2;
          }
          else {
            mu2p[i] = mu1;
          }
        }
      }
    }

  }
}


bool compute_hessian_eigenvalues(cv::InputArray gxx, cv::InputArray gxy, cv::InputArray gyy,
    cv::OutputArray mu1, cv::OutputArray mu2)
{
  if ( gxx.type() != gxy.type() || gxx.type() != gyy.type() ) {
    CF_ERROR("INPUT ERROR: derivative images have different types");
    return false;
  }

  if ( gxx.size() != gxy.size() || gxx.size() != gyy.size() ) {
    CF_ERROR("INPUT ERROR: derivative images have different sizes");
    return false;
  }

  switch (gxx.depth()) {
    case CV_8U:
      compute_hessian_eigenvalues_<uint8_t>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_8S:
      compute_hessian_eigenvalues_<int8_t>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_16U:
      compute_hessian_eigenvalues_<uint16_t>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_16S:
      compute_hessian_eigenvalues_<int16_t>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_32S:
      compute_hessian_eigenvalues_<int32_t>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_32F:
      compute_hessian_eigenvalues_<float>(gxx, gxy, gyy, mu1, mu2);
      break;
    case CV_64F:
      compute_hessian_eigenvalues_<double>(gxx, gxy, gyy, mu1, mu2);
      break;
    default:
      CF_ERROR("Invalid or not supported input image depth=%d", gxx.depth());
      return false;
  }

  return true;
}
