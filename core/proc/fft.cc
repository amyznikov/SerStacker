/*
 * fft.cc
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#include "fft.h"
#include <tbb/tbb.h>
#include <core/debug.h>

static inline double hyp(double x, double y)
{
  return sqrt(x * x + y * y);
}

static inline double square (double x)
{
  return x * x;
}


cv::Size fftGetOptimalSize(cv::Size imageSize, cv::Size psfSize, bool forceEvenSize )
{
  int w, h;

  imageSize += psfSize * 2;

  if ( !forceEvenSize ) {
    w = cv::getOptimalDFTSize(imageSize.width);
  }
  else {
    for ( int i = 0;; ++i ) {
      if ( !((w = cv::getOptimalDFTSize(imageSize.width + i)) & 0x7) ) {
        break;
      }
    }
  }

  if ( !forceEvenSize ) {
    h = cv::getOptimalDFTSize(imageSize.height);
  }
  else {
    for ( int i = 0;; ++i ) {
      if ( !((h = cv::getOptimalDFTSize(imageSize.height + i)) & 0x7) ) {
        break;
      }
    }
  }

  return cv::Size(w, h);
}


bool fftCopyMakeBorder(cv::InputArray src, cv::OutputArray dst, cv::Size fftSize, cv::Rect * outrc)
{
  const cv::Size src_size = src.size();

  if ( fftSize.width < src_size.width || fftSize.height < src_size.height ) {
    CF_ERROR("Invalid argument: fftSize (%dx%d) must be >= src.size() (%dx%d)",
        fftSize.width, fftSize.height, src_size.width, src_size.height );
    return false;
  }

  if ( src_size.width == fftSize.width && src_size.height == fftSize.height ) {
    if ( src.getMat().data != dst.getMatRef().data ) {
      src.copyTo(dst);
    }
    if ( outrc ) {
      *outrc = cv::Rect(0, 0, src_size.width, src_size.height);
    }
  }
  else {
    const int border_top = (fftSize.height - src_size.height) / 2;
    const int border_bottom = (fftSize.height - src_size.height - border_top);
    const int border_left = (fftSize.width - src_size.width ) / 2;
    const int border_right = (fftSize.width - src_size.width - border_left);
    cv::copyMakeBorder(src, dst, border_top, border_bottom, border_left, border_right, cv::BORDER_REFLECT);
    if ( outrc ) {
      * outrc = cv::Rect(border_left, border_top, src_size.width, src_size.height);
    }
  }

  return true;
}

void fftImageToSpectrum(cv::InputArray src, std::vector<cv::Mat2f> & complex_channels)
{
  const int cn = src.channels();
  complex_channels.resize(cn);

  if ( cn == 1 ) {
    cv::dft(src, complex_channels[0], cv::DFT_COMPLEX_OUTPUT);
  }
  else {
    cv::Mat channels[cn];
    cv::split(src.getMat(), channels);

    for ( int i = 0; i < cn; ++i ) {
      cv::dft(channels[i], complex_channels[i], cv::DFT_COMPLEX_OUTPUT);
    }
  }
}


void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels, cv::OutputArray dst)
{
  const int cn = complex_channels.size();

  if ( cn == 1 ) {
    cv::idft(complex_channels[0], dst, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  }
  else {
    cv::Mat channels[cn];

    for ( int i = 0; i < cn; ++i ) {
      cv::idft(complex_channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
    }

    cv::merge(channels, cn, dst);
  }
}


void fftImageToSpectrum(cv::InputArray image,
    std::vector<cv::Mat2f> & complex_channels,
    const cv::Size & psfSize,
    cv::Rect * outrc)
{
  if ( psfSize.empty() ) {

    fftImageToSpectrum(image,
        complex_channels);

    if ( outrc ) {
      *outrc = cv::Rect(0, 0,
          image.cols(),
          image.rows());
    }

  }
  else {

    cv::Mat tmp;
    cv::Rect rc;

    fftCopyMakeBorder(image, tmp,
        fftGetOptimalSize(image.size(),
            psfSize),
        &rc);

    fftImageToSpectrum(tmp,
        complex_channels);

    if ( outrc ) {
      *outrc = rc;
    }
  }

}

void fftImageFromSpectrum(const std::vector<cv::Mat2f> & complex_channels,
    cv::OutputArray dst,
    const cv::Rect & rc)
{
  if ( rc.empty() ) {
    fftImageFromSpectrum(complex_channels, dst);
  }
  else {
    cv::Mat tmp;
    fftImageFromSpectrum(complex_channels, tmp);
    dst.move(tmp = tmp(rc));
  }
}


void fftSwapQuadrants(cv::InputOutputArray _spec)
{
  using namespace cv;

  Mat spec = _spec.getMat();

  if ( spec.rows < 2 && spec.cols < 2 ) {
    // empty or trivially shifted.
    return;
  }

  int xMid = spec.cols >> 1;
  int yMid = spec.rows >> 1;

  const bool is_1d = xMid == 0 || yMid == 0;

  if ( is_1d ) {

    const int is_odd = (xMid > 0 && spec.cols % 2 == 1) || (yMid > 0 && spec.rows % 2 == 1);

    xMid = xMid + yMid;

    Mat half0(spec, Rect(0, 0, xMid + is_odd, 1));
    Mat half1(spec, Rect(xMid + is_odd, 0, xMid, 1));

    Mat tmp;
    half0.copyTo(tmp);
    half1.copyTo(spec(Rect(0, 0, xMid, 1)));
    tmp.copyTo(spec(Rect(xMid, 0, xMid + is_odd, 1)));

  }
  else {

    const int isXodd = spec.cols % 2 == 1;
    const int isYodd = spec.rows % 2 == 1;

    // perform quadrant swaps...
    Mat q0(spec, Rect(0, 0, xMid + isXodd, yMid + isYodd));
    Mat q1(spec, Rect(xMid + isXodd, 0, xMid, yMid + isYodd));
    Mat q2(spec, Rect(0, yMid + isYodd, xMid + isXodd, yMid));
    Mat q3(spec, Rect(xMid + isXodd, yMid + isYodd, xMid, yMid));

    if ( !(isXodd || isYodd) ) {
      Mat tmp;
      q0.copyTo(tmp);
      q3.copyTo(q0);
      tmp.copyTo(q3);

      q1.copyTo(tmp);
      q2.copyTo(q1);
      tmp.copyTo(q2);
    }
    else {
      Mat tmp0, tmp1, tmp2, tmp3;
      q0.copyTo(tmp0);
      q1.copyTo(tmp1);
      q2.copyTo(tmp2);
      q3.copyTo(tmp3);

      tmp0.copyTo(spec(Rect(xMid, yMid, xMid + isXodd, yMid + isYodd)));
      tmp3.copyTo(spec(Rect(0, 0, xMid, yMid)));

      tmp1.copyTo(spec(Rect(0, yMid, xMid, yMid + isYodd)));
      tmp2.copyTo(spec(Rect(xMid, 0, xMid + isXodd, yMid)));
    }
  }
}


bool fftSpectrumPower(cv::InputArray _src, cv::OutputArray _dst)
{
  if ( _src.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input image expected ");
    return false;
  }

  if ( _dst.fixedType() && _dst.type() != CV_32FC1 ) {
    CF_ERROR("Invalid output argument: CV_32FC1 output image expected ");
    return false;
  }

  const cv::Mat2f src = _src.getMat();

  cv::Mat1f tmp;
  cv::Mat1f dst;
  if ( src.data != _dst.getMatRef().data ) {
    _dst.create(src.size(), CV_32F);
    dst = _dst.getMatRef();
  }
  else {
    tmp.create(src.size());
    dst = tmp;
  }


  tbb::parallel_for(0, src.rows,
      [&src, &dst](int y) {
        for ( int x = 0; x < src.cols; ++x ) {
          const double a = src[y][x][0];
          const double b = src[y][x][1];
          dst[y][x] = (a * a + b * b);
        }
      });

  if ( !tmp.empty() ) {
    _dst.move(tmp);
  }

  return true;
}

bool fftSpectrumModule(cv::InputArray _src, cv::OutputArray _dst)
{
  if ( _src.type() != CV_32FC2 ) {
    CF_ERROR("Invalid argument: CV_32FC2 input image expected ");
    return false;
  }

  if ( _dst.fixedType() && _dst.type() != CV_32FC1 ) {
    CF_ERROR("Invalid output argument: CV_32FC1 output image expected ");
    return false;
  }

  const cv::Mat2f src = _src.getMat();

  cv::Mat1f tmp;
  cv::Mat1f dst;
  if ( src.data != _dst.getMatRef().data ) {
    _dst.create(src.size(), CV_32F);
    dst = _dst.getMatRef();
  }
  else {
    tmp.create(src.size());
    dst = tmp;
  }


  tbb::parallel_for(0, src.rows,
      [&src, &dst](int y) {
        for ( int x = 0; x < src.cols; ++x ) {
          const double a = src[y][x][0];
          const double b = src[y][x][1];
          dst[y][x] = sqrt(a * a + b * b);
        }
      });

  if ( !tmp.empty() ) {
    _dst.move(tmp);
  }

  return true;
}


bool fftSpectrumToPolar(const cv::Mat & src, cv::Mat & magnitude, cv::Mat & phase)
{
  if ( src.channels() != 2 || src.depth() != CV_32F ) {
    CF_DEBUG("invalid arg: FP32 2-channel input image expected");
    return false;
  }

  const cv::Mat2f csrc = src;

  magnitude.create(src.size(), CV_32F);
  cv::Mat1f cmag = magnitude;

  phase.create(src.size(), CV_32F);
  cv::Mat1f cphase = phase;

  tbb::parallel_for(0, src.rows,
      [&]( int y) {
        for ( int x = 0; x < csrc.cols; ++x ) {
          cmag[y][x] = sqrt(csrc[y][x][0]*csrc[y][x][0] + csrc[y][x][1]*csrc[y][x][1] );
          cphase[y][x] = atan2(csrc[y][x][1], csrc[y][x][0]);
        }
      });

  return true;
}

bool fftSpectrumFromPolar(const cv::Mat & magnitude, const cv::Mat & phase, cv::Mat & dst )
{
  const cv::Mat1f cmag = magnitude;
  const cv::Mat1f cphase = phase;

  dst.create(cmag.size(), CV_32FC2);
  cv::Mat2f cdst = dst;

  tbb::parallel_for(0, cdst.rows,
      [&]( int y) {
        for ( int x = 0; x < cdst.cols; ++x ) {
          double sa, ca;
          sincos(cphase[y][x], &sa, &ca);
          cdst[y][x][0] = cmag[y][x] * ca;
          cdst[y][x][1] = cmag[y][x] * sa;
        }
      });

  return true;
}


void fftRadialPolySharp(cv::InputArray src, cv::OutputArray dst,
    const std::vector<double> & coeffs,
    std::vector<double> * profile_before,
    std::vector<double> * profile_after)
{
  cv::Mat image;
  cv::Mat2f F;
  cv::Rect rc;

  cv::Size fftSize(cv::getOptimalDFTSize(src.cols()),
      cv::getOptimalDFTSize(src.rows()));

  if ( src.size() == fftSize ) {
    //image = src.getMat();
    src.copyTo(image);
  }
  else {
    fftCopyMakeBorder(src, image, fftSize, &rc);
  }



  const cv::Point centre = cv::Point(fftSize.width / 2, fftSize.height / 2);
  F.create(fftSize);

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, F.rows, 256),
      [&F, &coeffs, centre](const range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0, nx = F.cols; x < nx; ++x ) {

            const double D = 2 * hyp(((double)x - centre.x) / F.cols, ((double)y - centre.y) / F.rows);

            F[y][x][0] = 1;
            F[y][x][1] = 0;

            double DD = D;
            for ( int i = 0, n = coeffs.size(); i < n; ++i) {
              F[y][x][0] = std::max(0., F[y][x][0] + coeffs[i] * DD);
              if ( i < n - 1 ) {
                DD *= D;
              }
            }

          }
        }
      });

  fftSwapQuadrants(F);

  const auto applyF = [](const cv::Mat & src, cv::Mat & dst, const cv::Mat2f & F) -> void {
    cv::Mat S;
    cv::dft(src, S, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(S, F, S, 0, false);
    cv::idft(S, dst, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
  };


  const int nc = image.channels();
  if ( nc == 1 ) {
    applyF(image, image, F);
  }
  else {
    cv::Mat1f channels[nc];
    cv::split(image, channels);

    if ( image.size().area() > 1024 * 124 ) {
      for ( int i = 0; i < nc; ++i ) {
        applyF(channels[i], channels[i], F);
      }
    }
    else {
      tbb::parallel_for(0, nc, 1,
          [&F, &channels, &applyF](int i) {
            applyF(channels[i], channels[i], F);
          });
    }
    cv::merge(channels, nc, image);
  }

  if ( rc.empty() ) {
    dst.move(image);
  }
  else {
    image(rc).copyTo(dst);
  }
}


void fftSharpenR1(cv::InputArray image, cv::OutputArray dst, double scale, bool preserve_l2_norm)
{
  typedef tbb::blocked_range<int> range;

  std::vector<cv::Mat2f> channels;
  double saved_image_norm = 1;
  cv::Rect rc;

  if ( preserve_l2_norm ) {
    saved_image_norm = cv::norm(image, cv::NORM_L2);
  }

  fftImageToSpectrum(image, channels, cv::Size(32, 32), &rc);

  for ( int c = 0, cn = channels.size(); c < cn; ++c ) {

    cv::Mat2f & spec = channels[c];

    fftSwapQuadrants(spec);

    if ( scale >= 0 ) {
      // sharpen

      tbb::parallel_for(range(0, spec.rows, 64),
          [&spec, scale] (const range & r) {

            const double x0 = spec.cols / 2;
            const double y0 = spec.rows / 2;

            for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
              for ( int x = 0; x < spec.cols; ++x ) {
                const double r1 = 1. + scale * hyp((x - x0) / x0, (y - y0) / y0);
                spec[y][x][0] *= r1;
                spec[y][x][1] *= r1;
              }
            }
          });
    }
    else {
      // smoothing

      tbb::parallel_for(range(0, spec.rows, 64),
          [&spec, scale] (const range & r) {

            const double x0 = spec.cols / 2;
            const double y0 = spec.rows / 2;

            for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
              for ( int x = 0; x < spec.cols; ++x ) {
                const double r1 = 1./(1. - scale * hyp((x - x0) / x0, (y - y0) / y0));
                spec[y][x][0] *= r1;
                spec[y][x][1] *= r1;
              }
            }
          });
    }

    fftSwapQuadrants(spec);
  }

  fftImageFromSpectrum(channels, dst, rc);

  if ( preserve_l2_norm ) {
    cv::multiply(dst, saved_image_norm / cv::norm(image, cv::NORM_L2), dst);
  }
}



bool fftAccumulatePowerSpectrum(const cv::Mat & src,  cv::Mat & acc, float & cnt)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Mat img;
  cv::Size fft_size;

  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() && cnt > 0 ) {
    fft_size = acc.size();
  }
  else {
    fft_size = fftGetOptimalSize(src.size(), cv::Size(32,32));
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
    cnt = 0;
  }

  fftCopyMakeBorder(src, img, fft_size, nullptr);

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }

  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }
  else {
    img = channels[0];
  }

  cv::add(acc, img, acc);
  ++cnt;

  return true;
}

bool fftMaxPowerSpectrum(const cv::Mat & src, cv::Mat & acc)
{
  static const auto compute_magnitue =
      [](cv::Mat & src) {

        typedef std::complex<float> complex;

        const cv::Mat_<complex> spec = src;
        cv::Mat1f mag(spec.size());

        for ( int y = 0; y < spec.rows; ++y ) {
          for ( int x = 0; x < spec.cols; ++x ) {
            mag[y][x] = std::abs(spec[y][x]);
          }
        }

        src = std::move(mag);
      };


  cv::Size fft_size;
  cv::Mat img, spec;

  const int cn = src.channels();
  cv::Mat channels[cn];

  if ( !acc.empty() ) {
    fft_size = acc.size();
  }
  else {
    fft_size = fftGetOptimalSize(src.size(), cv::Size(32, 32));
    acc.create(fft_size, CV_MAKETYPE(CV_32F, src.channels()));
    acc.setTo(0);
  }

  if ( !fftCopyMakeBorder(src, img, fft_size, nullptr) ) {
    CF_ERROR("copyMakeFFTBorder(src=%dx%d, img, fft_size=%dx%d) fails", src.cols, src.rows, fft_size.width, fft_size.height);
    return false;
  }

  if ( cn == 1 ) {
    channels[0] = img;
  }
  else {
    cv::split(img, channels);
  }


  for ( int i = 0; i < cn; ++i ) {
    cv::dft(channels[i], channels[i], cv::DFT_COMPLEX_OUTPUT);
    compute_magnitue(channels[i]);
  }

  if ( cn > 1 ) {
    cv::merge(channels, cn, img);
  }
  else {
    img = channels[0];
  }

  cv::max(acc, img, acc);

  return true;
}


