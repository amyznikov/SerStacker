/*
 * cdiffs.cc
 *
 *  Created on: Sep 22, 2017
 *      Author: amyznikov
 */

#include "cdiffs.h"
//#include <opencv2/ximgproc.hpp>

/*
 * See also:
 *  Tony Lindeberg, Principles for Automatic scale selection.pdf
 *
 */


void cdiffs(cv::InputArray _src, cv::Mat * _Gx, cv::Mat * _Gy,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy,
    cv::Mat * _Gxxx, cv::Mat * _Gyyy,
    cv::Mat * _Gxxy, cv::Mat * _Gyyx,
    cv::Mat * _Gxyx, cv::Mat * _Gxyy)
{
  cv::Mat Gx, Gy;
  cv::Mat Gxx, Gyy, Gxy;
  cv::Mat Gxxx, Gyyy, Gxxy, Gyyx, Gxyx, Gxyy;
  cv::Mat Kx, Ky, I;
  int ddepth;

  static const float Cf[3] = { -0.5f, 0.0f, 0.5f };
  static const double Cd[3] = { -0.5, 0.0, 0.5 };

  const cv::Mat src = _src.getMat();

  if ( src.depth() == CV_64F ) {
    ddepth = CV_64F;
    Kx = cv::Mat(1, 3, CV_64F, (void*) Cd);
    Ky = cv::Mat(3, 1, CV_64F, (void*) Cd);
    I = cv::Mat::ones(1, 1, CV_64F);
  }
  else {
    ddepth = CV_32F;
    Kx = cv::Mat(1, 3, CV_32F, (void*) Cf);
    Ky = cv::Mat(3, 1, CV_32F, (void*) Cf);
    I = cv::Mat::ones(1, 1, CV_32F);
  }


  if ( _Gx || _Gxx || _Gxxx || _Gxxy ) {
    cv::sepFilter2D(src, Gx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gxx || _Gxxx || _Gxxy ) {
      cv::sepFilter2D(Gx, Gxx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxxx ) {
      cv::sepFilter2D(Gxx, Gxxx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxxy ) {
      cv::sepFilter2D(Gxx, Gxxy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gy || _Gyy || _Gyyy ) {
    cv::sepFilter2D(src, Gy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gyy || _Gyyy || _Gyyx ) {
      cv::sepFilter2D(Gy, Gyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gyyy ) {
      cv::sepFilter2D(Gyy, Gyyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gyyx ) {
      cv::sepFilter2D(Gyy, Gyyx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gxy || _Gxyy || _Gxyx ) {
    cv::sepFilter2D(src, Gxy, ddepth, Kx, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gxyy ) {
      cv::sepFilter2D(Gxy, Gxyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxyx ) {
      cv::sepFilter2D(Gxy, Gxyx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gx ) {
    *_Gx = Gx;
  }
  if ( _Gy ) {
    *_Gy = Gy;
  }
  if ( _Gxy ) {
    *_Gxy = Gxy;
  }
  if ( _Gxx ) {
    *_Gxx = Gxx;
  }
  if ( _Gyy ) {
    *_Gyy = Gyy;
  }
  if ( _Gxxx ) {
    *_Gxxx = Gxxx;
  }
  if ( _Gxxy ) {
    *_Gxxy = Gxxy;
  }
  if ( _Gyyy ) {
    *_Gyyy = Gyyy;
  }
  if ( _Gyyx ) {
    *_Gyyx = Gyyx;
  }
  if ( _Gxyx ) {
    * _Gxyx = Gxyx;
  }
  if ( _Gxyy ) {
    * _Gxyy = Gxyy;
  }
}


void cdiffs_s(cv::InputArray src, int s,
    cv::Mat * _Gx, cv::Mat * _Gy,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy,
    cv::Mat * _Gxxx, cv::Mat * _Gyyy,
    cv::Mat * _Gxxy, cv::Mat * _Gyyx,
    cv::Mat * _Gxyx, cv::Mat * _Gxyy)
{
  cv::Mat Gx, Gy;
  cv::Mat Gxx, Gyy, Gxy;
  cv::Mat Gxxx, Gyyy, Gxxy, Gyyx, Gxyx, Gxyy;

  const int ddepth = CV_32F;
  const int ksize = 2 * s + 1;

  std::vector<float> C(ksize);

  for ( int i = 0; i < s; ++i ) {
    C[i] = -1.0 / (ksize - 1);
  }
  C[s] = 0;
  for ( int i = s + 1; i < ksize; ++i ) {
    C[i] = +1.0 / (ksize - 1);
  }


  cv::Mat Kx = cv::Mat(1, ksize, CV_32F, (void*) &C[0]);
  cv::Mat Ky = cv::Mat(ksize, 1, CV_32F, (void*) &C[0]);

  static const cv::Mat1f I = cv::Mat1f::ones(1, 1);

  if ( _Gx || _Gxx || _Gxxx || _Gxxy ) {
    cv::sepFilter2D(src, Gx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gxx || _Gxxx || _Gxxy ) {
      cv::sepFilter2D(Gx, Gxx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxxx ) {
      cv::sepFilter2D(Gxx, Gxxx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxxy ) {
      cv::sepFilter2D(Gxx, Gxxy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gy || _Gyy || _Gyyy ) {
    cv::sepFilter2D(src, Gy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gyy || _Gyyy || _Gyyx ) {
      cv::sepFilter2D(Gy, Gyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gyyy ) {
      cv::sepFilter2D(Gyy, Gyyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gyyx ) {
      cv::sepFilter2D(Gyy, Gyyx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gxy || _Gxyy || _Gxyx ) {
    cv::sepFilter2D(src, Gxy, ddepth, Kx, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    if ( _Gxyy ) {
      cv::sepFilter2D(Gxy, Gxyy, ddepth, I, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
    if ( _Gxyx ) {
      cv::sepFilter2D(Gxy, Gxyx, ddepth, Kx, I, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    }
  }

  if ( _Gx ) {
    *_Gx = Gx;
  }
  if ( _Gy ) {
    *_Gy = Gy;
  }
  if ( _Gxy ) {
    *_Gxy = Gxy;
  }
  if ( _Gxx ) {
    *_Gxx = Gxx;
  }
  if ( _Gyy ) {
    *_Gyy = Gyy;
  }
  if ( _Gxxx ) {
    *_Gxxx = Gxxx;
  }
  if ( _Gxxy ) {
    *_Gxxy = Gxxy;
  }
  if ( _Gyyy ) {
    *_Gyyy = Gyyy;
  }
  if ( _Gyyx ) {
    *_Gyyx = Gyyx;
  }
  if ( _Gxyx ) {
    * _Gxyx = Gxyx;
  }
  if ( _Gxyy ) {
    * _Gxyy = Gxyy;
  }
}


void cdiffs(cv::InputArray src, cv::Mat * _Gx, cv::Mat * _Gy, cv::Mat * _Gxx, cv::Mat * _Gyy, cv::Mat * _Gxy)
{
  cdiffs(src, _Gx, _Gy, _Gxx, _Gyy, _Gxy, NULL, NULL, NULL, NULL, NULL, NULL);
}

void hessian(cv::InputArray src, cv::Mat * _Gxx, cv::Mat * _Gyy, cv::Mat * _Gxy)
{
  cdiffs(src, NULL, NULL, _Gxx, _Gyy, _Gxy, NULL, NULL, NULL, NULL, NULL, NULL);
}

void gradient(cv::InputArray src, cv::Mat * _Gx, cv::Mat * _Gy, cv::Mat * _G)
{
  cv::Mat Gx, Gy;

  cdiffs(src, &Gx, &Gy, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  if ( _Gx ) {
    *_Gx = Gx;
  }

  if ( _Gy ) {
    *_Gy = Gy;
  }

  if ( _G ) {
    cv::magnitude(Gx, Gy, *_G);
  }
}


void gradient(cv::InputArray src, double sigma, cv::Mat * _Gx, cv::Mat * _Gy, cv::Mat * _G)
{
  if ( sigma <= 0 ) {
    gradient(src, _Gx, _Gy, _G);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(src, tmp, cv::Size(0, 0), sigma);
    gradient(tmp, _Gx, _Gy, _G);
  }
}

void gradientvec(cv::InputArray src, cv::Mat * _GVec)
{
  cv::Mat G[2];
  cdiffs(src, &G[0], &G[1], NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
  cv::merge(G, 2, *_GVec);
}

void gradientvec(cv::InputArray src, double sigma, cv::Mat * _GVec)
{
  if ( sigma <= 0 ) {
    gradientvec(src, _GVec);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(src, tmp, cv::Size(0, 0), sigma);
    gradientvec(tmp, _GVec);
  }
}


void gradient_s(cv::InputArray src, int s, cv::Mat * _Gx, cv::Mat * _Gy, cv::Mat * _G)
{
  cv::Mat Gx, Gy;

  cdiffs_s(src, s, &Gx, &Gy, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  if ( _Gx ) {
    *_Gx = Gx;
  }

  if ( _Gy ) {
    *_Gy = Gy;
  }

  if ( _G ) {
    cv::magnitude(Gx, Gy, *_G);
  }
}


void gradient_s(cv::InputArray _src, int s, double sigma, cv::Mat * _Gx, cv::Mat * _Gy, cv::Mat * _G)
{
  cv::Mat src;

  if ( sigma <= 0 ) {
    src = _src.getMat();
  }
  else {
    cv::GaussianBlur(_src, src, cv::Size(0, 0), sigma);
  }
  gradient_s(src, s, _Gx, _Gy, _G);
}



void laplace(cv::InputArray src, cv::OutputArray L)
{
  cv::Mat Gxx, Gyy;
  hessian(src, &Gxx, &Gyy, NULL);
  cv::add(Gxx, Gyy, L);
}

void modlaplace(cv::InputArray src, cv::OutputArray L)
{
  cv::Mat Gxx, Gyy;
  hessian(src, &Gxx, &Gyy, NULL);
  cv::add(Gxx.mul(Gxx), Gyy.mul(Gyy), L);
  //magnitude(Gxx, Gyy, L);
}

void modlaplace2(cv::InputArray src,
    cv::OutputArray L, int step, int box)
{
  cv::Mat K, Gxx, Gyy;
  const int ddepth = CV_32F;


  K = cv::Mat::zeros(1, 2 * step + 1, CV_32F);
  K.at<float>(0) = -1;
  K.at<float>(K.cols - 1) = -1;
  K.at<float>(K.cols / 2) = 2;

  cv::filter2D(src, Gxx, ddepth, K);
  cv::filter2D(src, Gyy, ddepth, K.t());

  cv::magnitude(Gxx, Gyy, L);
  cv::sqrBoxFilter(L, L, ddepth, cv::Size(box, box), cv::Point(-1, -1), true);
  cv::sqrt(L, L);
}


void ridgeness(cv::InputArray _Gxx, cv::InputArray _Gyy, cv::InputArray _Gxy, cv::OutputArray R )
{
  cv::Mat T1, T2, T3, mu1, mu2;

  cv::Mat Gxx, Gyy, Gxy;

  Gxx = _Gxx.getMat();
  Gyy = _Gyy.getMat();
  Gxy = _Gxy.getMat();

  // R = (Lxx + Lyy)*sqrt((Lxx-Lyy)^2 + 4 * Lxy^2 )
  add(Gxx, Gyy, T1);
  magnitude(Gxx - Gyy, 2 * Gxy, T2);
  multiply(T1, T2, T3); // abs(

  // eigenvalues abs(mu1) < abs(mu2)
  mu1 = (T1 - T2);
  mu2 = (T1 + T2);
  mu1.copyTo(mu2, abs(mu1) > abs(mu2));
  T3.setTo(0, mu2 > 0);

  sqrt(abs(T3), R);
}

void ridgeness(cv::InputArray src, cv::OutputArray R )
{
  cv::Mat Gxx, Gxy, Gyy;
  hessian(src, &Gxx, &Gyy, &Gxy);
  ridgeness(Gxx, Gyy, Gxy, R);
}



template<class T>
static void computeEigenVectors(cv::Mat & _Nx, cv::Mat & _Ny,
    const cv::Mat & a, const cv::Mat & b, const cv::Mat & mu,
    const cv::Mat & Gx, const cv::Mat & Gy)
{
  cv::Mat mu2a, D;

  cv::subtract(mu, a, mu2a);
  cv::magnitude(b, mu2a, D);

  cv::divide(b, D, _Nx);
  cv::divide(mu2a, D, _Ny);

  for ( int y = 0, xmax = _Nx.cols * _Nx.channels(); y < _Nx.rows; ++y ) {

    const float * Dp = D.ptr<const float>(y);
    const float * ap = a.ptr<const float>(y);
    const float * Gxp = Gx.empty() ? NULL : Gx.ptr<const float>(y);
    const float * Gyp = Gy.empty() ? NULL : Gy.ptr<const float>(y);

    float * Pxp = _Nx.ptr<float>(y);
    float * Pyp = _Ny.ptr<float>(y);

    for ( int x = 0; x < xmax; ++x ) {

      if ( Dp[x] < FLT_EPSILON ) {
        Pyp[x] = 0;
        if ( ap[x] != 0 ) {
          Pxp[x] = ap[x] > 0 ? 1 : -1;
        }
      }

      if ( Gxp && Gyp && (Gyp[x] > 0 || (Gyp[x] == 0 && Gxp[x] > 0)) ) {
        Pxp[x] = -Pxp[x];
        Pyp[x] = -Pyp[x];
      }
    }
  }
}


/* See:
 *  Eigenvectors and eigenvalues of real symmetric matrices.pdf
 *  Diagonalization of a 2 × 2 real symmetric matrix.pdf
 */
void eigen2d(cv::InputArray _Dxx, cv::InputArray _Dyy, cv::InputArray _Dxy,
    cv::Mat * _mu1, cv::Mat * _mu2, cv::Mat * _Nx, cv::Mat * _Ny,
    cv::InputArray _Dx, cv::InputArray _Dy)
{
  cv::Mat D, T, mu1, mu2;

  // Eigenvectors and eigenvalues of real symmetric matrices.pdf
  // A = | a b |
  //     | b d |
  //

  cv::Mat a = _Dxx.getMat();
  cv::Mat b = _Dxy.getMat();
  cv::Mat d = _Dyy.getMat();

  cv::add(a, d, T); //  T = a + d;
  cv::magnitude(a - d, 2 * b, D);

  cv::Mat mu[2] = {
      (T + D) * 0.5,
      (T - D) * 0.5
  };

  // sort eigenvalues to get abs(mu2) > abs(mu1)
  //  const cv::Mat mask = abs(mu[0]) > abs(mu[1]);

  cv::absdiff(mu[0], 0.0, mu1), absdiff(mu[1], 0.0, mu2);
  cv::compare(mu1, mu2, T, cv::CMP_GT);
  mu[1].copyTo(mu2), mu[0].copyTo(mu2, T);
  mu[0].copyTo(mu1), mu[1].copyTo(mu1, T);

  if ( _mu1 ) {
    *_mu1 = mu1;
  }
  if ( _mu2 ) {
    *_mu2 = mu2;
  }

  T.release();
  D.release();

  if ( _Nx && _Ny ) {
    switch ( mu2.depth() ) {
    case CV_32F :
      computeEigenVectors<float>(*_Nx, *_Ny, a, b, mu2, _Dx.getMat(), _Dy.getMat());
      break;
    case CV_64F :
      computeEigenVectors<double>(*_Nx, *_Ny, a, b, mu2, _Dx.getMat(), _Dy.getMat());
      break;
    }
  }
}

void eigen2d(cv::InputArray src, cv::Mat * _mu1, cv::Mat * _mu2, cv::Mat * _Nx, cv::Mat * _Ny, bool fixNormals)
{
  cv::Mat Gx, Gy, Gxx, Gyy, Gxy;

  if ( fixNormals && _Nx && _Ny ) {
    cdiffs(src, &Gx, &Gy, &Gxx, &Gyy, &Gxy);
  }
  else {
    cdiffs(src, NULL, NULL, &Gxx, &Gyy, &Gxy);
  }

  eigen2d(Gxx, Gyy, Gxy, _mu1, _mu2, _Nx, _Ny, Gx, Gy);
}






void eigenridges(cv::InputArray _Dxx, cv::InputArray _Dyy, cv::InputArray _Dxy, cv::OutputArray _R,
    cv::Mat * _Nx, cv::Mat * _Ny)
{
  cv::Mat mu1, mu2;
  eigen2d(_Dxx, _Dyy, _Dxy, &mu1, &mu2, _Nx, _Ny);
  cv::subtract(cv::abs(mu2), cv::abs(mu1), _R);
  _R.getMat().setTo(0, mu2 > 0);
}

void eigenridges(cv::InputArray src, cv::OutputArray R, cv::Mat * _Nx, cv::Mat * _Ny)
{
  cv::Mat Dxx, Dxy, Dyy;
  hessian(src, &Dxx, &Dyy, &Dxy);
  eigenridges(Dxx, Dyy, Dxy, R, _Nx, _Ny);
}

void eigenridges(cv::InputArray src, double sigma, cv::OutputArray R, cv::Mat * _Nx, cv::Mat * _Ny )
{
  if ( sigma <= 0 ) {
    eigenridges(src, sigma, R, _Nx, _Ny);
  }
  else {
    cv::Mat src1;
    cv::GaussianBlur(src, src1, cv::Size(0,0), sigma);
    eigenridges(src1, R, _Nx, _Ny);
  }
}

void eigenridges2(cv::InputArray _src, double sigma, cv::Mat & R, cv::Mat * _Nx, cv::Mat * _Ny)
{
  cv::Mat src, mu1, mu2, mu, mask;

  if ( sigma <= 0 ) {
    src = _src.getMat();
  }
  else {
    cv::GaussianBlur(_src, src, cv::Size(0, 0), sigma);
  }

  eigen2d(src, &mu1, &mu2, _Nx, _Ny);
  mask = (mu2 > 0);

  cv::absdiff(mu1, 0, mu1);
  cv::absdiff(mu2, 0, mu2);

  cv::divide(mu2, mu1 + 1, R);
  R.setTo(0, mask);
}



/*
 * See:
 *  Tony Lindeberg,
 *    Principles for Automatic scale selection.pdf, formula (1.14)
 *
 *  */
void cLvv(const cv::Mat & Lx, const cv::Mat & Ly,
    const cv::Mat & Lxx, const cv::Mat & Lyy, const cv::Mat & Lxy,
    cv::Mat & _Lvv)
{
  cv::Mat tmp1, tmp2, tmp3;
  cv::Mat Lv, Lvv;

  cv::magnitude(Lx, Ly, Lv);
  cv::multiply(Lx.mul(Lx), Lxx, tmp1);
  cv::multiply(Lx.mul(Ly), Lxy, tmp2, 2);
  cv::multiply(Ly.mul(Ly), Lyy, tmp3);
  cv::add(tmp1, tmp2, Lvv);
  cv::add(tmp3, Lvv, Lvv);
  cv::divide(Lvv, Lv.mul(Lv), _Lvv);
}


/*
 * See:
 *  Tony Lindeberg,
 *    Principles for Automatic scale selection.pdf, formula (1.15)
 *  */
void cLvvv(const cv::Mat & Lx, const cv::Mat & Ly,
    const cv::Mat & Lxxx, const cv::Mat & Lyyy,
    const cv::Mat & Lxyy, const cv::Mat & Lxxy,
    cv::Mat & _Lvvv)
{
  cv::Mat Lv;
  cv::Mat T1, T2, Lvvv;
  cv::Mat tmp1, tmp2;

  cv::multiply(Lx.mul(Lx), Lxxx, tmp1);
  cv::multiply(Ly.mul(Ly), Lxyy, tmp2, 3);
  cv::add(tmp1, tmp2, T1);
  cv::multiply(Lx, T1, T1);


  cv::multiply(Lx.mul(Lx), Lxxy, tmp1, 3);
  cv::multiply(Ly.mul(Ly), Lyyy, tmp2);
  cv::add(tmp1, tmp2, T2);
  cv::multiply(Ly, T2, T2);

  cv::add(T1, T2, Lvvv);

  cv::magnitude(Lx, Ly, Lv);
  cv::multiply(Lv.mul(Lv), Lv, Lv);

  cv::divide(Lvvv, Lv, Lvvv);

  _Lvvv = Lvvv;
}


/*
 * Use of cv::sepFilter2D with differential kernel { -1, +1 } anchored to {0,1}
 * for gradient computation
 * */
void ggrad(cv::InputArray src, cv::Mat * gx, cv::Mat * gy, cv::Mat * g, int borderType)
{
  static float C[2] = { -1, +1 };
  static const thread_local cv::Mat I(1, 1, CV_32F, 1.0);
  const int ddepth = src.depth() == CV_64F ? CV_64F : CV_32F;

  cv::Mat  gx_, gy_;

  if ( (gx || g) ) {
    if ( !gx ) {
      gx = &gx_;
    }
    static const thread_local cv::Mat1f Kx(1, 2, C);
    cv::sepFilter2D(src, *gx, ddepth, Kx, I, cv::Point(1, 0), 0, borderType);
  }

  if ( (gy || g) ) {
    if ( !gy ) {
      gy = &gy_;
    }
    static const thread_local cv::Mat1f Ky(2, 1, C);
    cv::sepFilter2D(src, *gy, ddepth, I, Ky, cv::Point(0, 1), 0, borderType);
  }

  if ( g ) {
    cv::magnitude(*gx, *gy, *g);
  }
}

/*
 * Use of cv::sepFilter2D with user specified kernels and anchors
 * for gradient computation
 * */
void ggrad(cv::InputArray src,
    const cv::Mat1f & Kx, const cv::Mat1f & Ky,
    cv::Point anchor_x, cv::Point anchor_y,
    cv::Mat * gx, cv::Mat * gy, cv::Mat * g,
    int borderType)
{
  static const thread_local cv::Mat I(1, 1, CV_32F, 1.0);
  const int ddepth = src.depth() == CV_64F ? CV_64F : CV_32F;

  cv::Mat  gx_, gy_;

  if ( (gx || g) ) {
    if ( !gx ) {
      gx = &gx_;
    }
    cv::sepFilter2D(src, *gx, ddepth, Kx, I, anchor_x, 0, borderType);
  }

  if ( (gy || g) ) {
    if ( !gy ) {
      gy = &gy_;
    }
    cv::sepFilter2D(src, *gy, ddepth, I, Ky, anchor_y, 0, borderType);
  }

  if ( g ) {
    cv::magnitude(*gx, *gy, *g);
  }
}



/*
 * Use of cv::sepFilter2D with differential kernel {  0, -1, +1 };
 * for gradient computation of pre-smoothed image src
 * */
void ggrad(cv::InputArray src, double pre_smooth_sigma, cv::Mat * gx, cv::Mat * gy, cv::Mat * g, int borderType)
{
  if ( pre_smooth_sigma <= 0 ) {
    ggrad(src, gx, gy, g, borderType);
  }
  else {
    cv::Mat tmp;
    cv::GaussianBlur(src, tmp, cv::Size(0,0), pre_smooth_sigma);
    ggrad(tmp, gx, gy, g, borderType);
  }

}


/**
 * Generalized Laplacian filter L is defined as
 *    L = Dx' W Dx   +   Dy' W Dy
 */
void glap(cv::InputArray _src, cv::InputArray weight, cv::Mat & dst, int borderType)
{
  cv::Mat Dxx, Dyy;
  int ddepth;

  const cv::Mat src = _src.getMat();
  const cv::Mat W = weight.getMat();

  static const float C1[3] = { 0, -1, +1 };
  static const float C2[3] = { +1, -1, 0 };
  static const thread_local cv::Mat I(1, 1, CV_32F, 1);

  if ( src.depth() == CV_64F ) {
    ddepth = CV_64F;
  }
  else {
    ddepth = CV_32F;
  }

  // H-filter
  cv::sepFilter2D(_src, Dxx, ddepth, cv::Mat(1, 3, CV_32F, (void*) C1), I, cv::Point(1, 0), 0, borderType);
  if ( !W.empty() ) {
    cv::multiply(Dxx, W, Dxx);
  }
  cv::sepFilter2D(Dxx, Dxx, ddepth, cv::Mat(1, 3, CV_32F, (void*) C2), I, cv::Point(1, 0), 0, borderType);


  // V- filter
  cv::sepFilter2D(_src, Dyy, ddepth, I, cv::Mat(3, 1, CV_32F, (void*) C1), cv::Point(0, 1), 0, borderType);
  if ( !W.empty() ) {
    cv::multiply(Dyy, W, Dyy);
  }
  cv::sepFilter2D(Dyy, Dyy, ddepth, I, cv::Mat(3, 1, CV_32F, (void*) C2), cv::Point(0, 1), 0, borderType);

  // sum
  cv::add(Dxx, Dyy, dst);
}


/*
 * divergence of gradient of image X
 * */
void divergence(const cv::Mat & src, cv::Mat & dst)
{
  static const float C[3 * 3] = {
      0,     -1./8,   0,
    -1./8,    4./8, -1./8,
      0,     -1./8,   0,
  };

  cv::filter2D(src, dst, -1,
      cv::Mat(3, 3, CV_32F, (void*) C),
      cv::Point(1, 1));
}

