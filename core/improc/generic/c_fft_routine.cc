/*
 * c_fft_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_fft_routine.h"
#include <core/ssprintf.h>


namespace {

static void magSpectrums(cv::InputArray _src, cv::OutputArray _dst)
{
  cv::Mat src = _src.getMat();
  int depth = src.depth(), cn = src.channels(), type = src.type();
  int rows = src.rows, cols = src.cols;
  int j, k;

  CV_Assert(type == CV_32FC1 || type == CV_32FC2 );

  if( src.depth() == CV_32F )
    _dst.create(src.rows, src.cols, CV_32FC1);
  else
    _dst.create(src.rows, src.cols, CV_64FC1);

  cv::Mat dst = _dst.getMat();
  dst.setTo(0); //Mat elements are not equal to zero by default!

  bool is_1d = (rows == 1 || (cols == 1 && src.isContinuous() && dst.isContinuous()));

  if( is_1d )
    cols = cols + rows - 1, rows = 1;

  int ncols = cols * cn;
  int j0 = cn == 1;
  int j1 = ncols - (cols % 2 == 0 && cn == 1);

  const float * dataSrc = src.ptr<float>();
  float * dataDst = dst.ptr<float>();

  size_t stepSrc = src.step / sizeof(dataSrc[0]);
  size_t stepDst = dst.step / sizeof(dataDst[0]);

  if( !is_1d && cn == 1 ) {
    for( k = 0; k < (cols % 2 ? 1 : 2); k++ ) {
      if( k == 1 )
        dataSrc += cols - 1, dataDst += cols - 1;
      dataDst[0] = dataSrc[0] * dataSrc[0];
      if( rows % 2 == 0 )
        dataDst[(rows - 1) * stepDst] = dataSrc[(rows - 1) * stepSrc] * dataSrc[(rows - 1) * stepSrc];

      for( j = 1; j <= rows - 2; j += 2 ) {
        dataDst[j * stepDst] = (float) std::sqrt((double) dataSrc[j * stepSrc] * dataSrc[j * stepSrc] +
            (double) dataSrc[(j + 1) * stepSrc] * dataSrc[(j + 1) * stepSrc]);
      }

      if( k == 1 )
        dataSrc -= cols - 1, dataDst -= cols - 1;
    }
  }

  for( ; rows--; dataSrc += stepSrc, dataDst += stepDst ) {
    if( is_1d && cn == 1 ) {
      dataDst[0] = dataSrc[0] * dataSrc[0];
      if( cols % 2 == 0 )
        dataDst[j1] = dataSrc[j1] * dataSrc[j1];
    }

    for( j = j0; j < j1; j += 2 ) {
      dataDst[j] = (float) std::sqrt((double) dataSrc[j] * dataSrc[j] + (double) dataSrc[j + 1] * dataSrc[j + 1]);
    }
  }
}

static void divSpectrums(cv::InputArray _srcA, cv::InputArray _srcB, cv::OutputArray _dst, int flags, bool conjB)
{
  cv::Mat srcA = _srcA.getMat(), srcB = _srcB.getMat();
  int depth = srcA.depth(), cn = srcA.channels(), type = srcA.type();
  int rows = srcA.rows, cols = srcA.cols;
  int j, k;

  CV_Assert(type == srcB.type() && srcA.size() == srcB.size());
  CV_Assert(type == CV_32FC1 || type == CV_32FC2);

  _dst.create(srcA.rows, srcA.cols, type);
  cv::Mat dst = _dst.getMat();

  CV_Assert(dst.data != srcA.data); // non-inplace check
  CV_Assert(dst.data != srcB.data); // non-inplace check

  bool is_1d = (flags & cv::DFT_ROWS) || (rows == 1 || (cols == 1 &&
      srcA.isContinuous() && srcB.isContinuous() && dst.isContinuous()));

  if( is_1d && !(flags & cv::DFT_ROWS) )
    cols = cols + rows - 1, rows = 1;

  int ncols = cols * cn;
  int j0 = cn == 1;
  int j1 = ncols - (cols % 2 == 0 && cn == 1);

  const float * dataA = srcA.ptr<float>();
  const float * dataB = srcB.ptr<float>();
  float * dataC = dst.ptr<float>();
  float eps = FLT_EPSILON; // prevent div0 problems

  size_t stepA = srcA.step / sizeof(dataA[0]);
  size_t stepB = srcB.step / sizeof(dataB[0]);
  size_t stepC = dst.step / sizeof(dataC[0]);

  if( !is_1d && cn == 1 ) {
    for( k = 0; k < (cols % 2 ? 1 : 2); k++ ) {
      if( k == 1 )
        dataA += cols - 1, dataB += cols - 1, dataC += cols - 1;
      dataC[0] = dataA[0] / (dataB[0] + eps);
      if( rows % 2 == 0 )
        dataC[(rows - 1) * stepC] = dataA[(rows - 1) * stepA] / (dataB[(rows - 1) * stepB] + eps);
      if( !conjB ) {
        for( j = 1; j <= rows - 2; j += 2 ) {
          double denom = (double) dataB[j * stepB] * dataB[j * stepB] +
              (double) dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] + (double) eps;

          double re = (double) dataA[j * stepA] * dataB[j * stepB] +
              (double) dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

          double im = (double) dataA[(j + 1) * stepA] * dataB[j * stepB] -
              (double) dataA[j * stepA] * dataB[(j + 1) * stepB];

          dataC[j * stepC] = (float) (re / denom);
          dataC[(j + 1) * stepC] = (float) (im / denom);
        }
      }
      else {
        for( j = 1; j <= rows - 2; j += 2 ) {
          double denom = (double) dataB[j * stepB] * dataB[j * stepB] +
              (double) dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] + (double) eps;

          double re = (double) dataA[j * stepA] * dataB[j * stepB] -
              (double) dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

          double im = (double) dataA[(j + 1) * stepA] * dataB[j * stepB] +
              (double) dataA[j * stepA] * dataB[(j + 1) * stepB];

          dataC[j * stepC] = (float) (re / denom);
          dataC[(j + 1) * stepC] = (float) (im / denom);
        }
      }
      if( k == 1 )
        dataA -= cols - 1, dataB -= cols - 1, dataC -= cols - 1;
    }
  }

  for( ; rows--; dataA += stepA, dataB += stepB, dataC += stepC ) {
    if( is_1d && cn == 1 ) {
      dataC[0] = dataA[0] / (dataB[0] + eps);
      if( cols % 2 == 0 )
        dataC[j1] = dataA[j1] / (dataB[j1] + eps);
    }

    if( !conjB )
      for( j = j0; j < j1; j += 2 ) {
        double denom = (double) (dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps);
        double re = (double) (dataA[j] * dataB[j] + dataA[j + 1] * dataB[j + 1]);
        double im = (double) (dataA[j + 1] * dataB[j] - dataA[j] * dataB[j + 1]);
        dataC[j] = (float) (re / denom);
        dataC[j + 1] = (float) (im / denom);
      }
    else
      for( j = j0; j < j1; j += 2 ) {
        double denom = (double) (dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps);
        double re = (double) (dataA[j] * dataB[j] - dataA[j + 1] * dataB[j + 1]);
        double im = (double) (dataA[j + 1] * dataB[j] + dataA[j] * dataB[j + 1]);
        dataC[j] = (float) (re / denom);
        dataC[j + 1] = (float) (im / denom);
      }
  }
}

}
template<>
const c_enum_member * members_of<c_fft_routine::DisplayType>()
{

  static const c_enum_member members[] = {
      {c_fft_routine::DisplayModule, "Module", "Display spectrum module"},
      {c_fft_routine::DisplayPower, "Power", "Display power spectrum"},
      {c_fft_routine::DisplayPhase, "Phase", "Display phase spectrum"},
      {c_fft_routine::DisplayReal, "Real", "Display Real part of spectrum"},
      {c_fft_routine::DisplayImag, "Imag", "Display Imaginary part of spectrum"},
      {c_fft_routine::DisplayCCSTest1, "CCSTest1", "CCSTest1"},
      {c_fft_routine::DisplayCCSTest2, "CCSTest2", "CCSTest2"},
      {c_fft_routine::DisplayCCSTest3, "CCSTest3", "CCSTest3"},
      {c_fft_routine::DisplayCCSTest4, "CCSTest4", "CCSTest4"},
      {c_fft_routine::DisplayCCSTest5, "CCSTest5", "CCSTest5"},
      {c_fft_routine::DisplayCCSTest6, "CCSTest6", "CCSTest6"},
      {c_fft_routine::DisplayModule},
  };

  return members;
}

void c_fft_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "output_display", ctx(&this_class::_output_display), "Output image display");
   ctlbind(ctls, "dft_scale", ctx(&this_class::_dft_scale), "Set cv::DFT_SCALE flag");
}

bool c_fft_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _output_display);
    SERIALIZE_OPTION(settings, save, *this, _dft_scale);
    return true;
  }
  return false;
}

bool c_fft_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const int cn = image.channels();

  cv::Mat channels[cn];

  if( cn == 1 ) {
    image.getMat().convertTo(channels[0], CV_32F);
  }
  else {
    cv::split(image.getMat(), channels);
    for( int c = 0; c < cn; ++c ) {
      channels[c].convertTo(channels[c], CV_32F);
    }
  }

  if( _output_display == DisplayCCSTest1 ) {
    CF_DEBUG("CCSTest1");
    cv::Mat1f realspec;
    cv::Mat2f cmplxspec;
    cv::dft(channels[0], realspec, cv::DFT_REAL_OUTPUT);
    fftUnpackCCSSpectrum(realspec, cmplxspec);
    image.move(cmplxspec);
    //fftSwapQuadrants(image);
    mask.release();
    return true;
  }

  if( _output_display == DisplayCCSTest2 ) {
    CF_DEBUG("CCSTest2");
    cv::Mat2f cmplxspec;
    cv::dft(channels[0], cmplxspec, cv::DFT_COMPLEX_OUTPUT);
    image.move(cmplxspec);
    //fftSwapQuadrants(image);
    mask.release();
    return true;
  }

  if ( _output_display == DisplayCCSTest3 ) {
    CF_DEBUG("CCSTest3");
    cv::Mat1f realspec;
    cv::Mat2f cmplxspec1, cmplxspec2, cmplxdiff;

    cv::dft(channels[0], realspec, cv::DFT_REAL_OUTPUT);
    fftUnpackCCSSpectrum(realspec, cmplxspec1);

    cv::dft(channels[0], cmplxspec2, cv::DFT_COMPLEX_OUTPUT);
    cv::subtract(cmplxspec1, cmplxspec2, cmplxdiff);
    //fftSpectrumToPolar(cmplxdiff, cmplxdiff);
    image.move(cmplxdiff);
    //fftSwapQuadrants(image);

    mask.release();
    return true;
  }

  if ( _output_display == DisplayCCSTest4 ) {
    CF_DEBUG("CCSTest4");
    const cv::Mat1f filter = fftGenerateGaussianFilter(channels[0].size(), 1, 1, false);
    cv::dft(channels[0], channels[0], cv::DFT_REAL_OUTPUT);
    fftMulSpectrumCCS(channels[0], filter, channels[0]);
    fftUnpackCCSSpectrum(channels[0], channels[0]);
    fftSpectrumToPolar(channels[0], image);
    mask.release();
    return true;
  }



  if ( _output_display == DisplayCCSTest5 ) {
    CF_DEBUG("CCSTest5(cmplx)");

    cv::Mat1f synth(channels[0].size(), 0.f);
    synth(channels[0].rows/2,channels[0].cols/2) = 1;

    cv::Mat2f cmplxspec;
    cv::dft(synth, cmplxspec, cv::DFT_COMPLEX_OUTPUT);
    image.move(cmplxspec);
    mask.release();
    return  true;
  }


  if ( _output_display == DisplayCCSTest6 ) {
    CF_DEBUG("CCSTest5(real)");
    cv::Mat1f synth(channels[0].size(), 0.f);

    synth(channels[0].rows/2,channels[0].cols/2) = 1;

    cv::Mat1f realspec;
    cv::Mat2f cmplxspec;
    cv::dft(synth, realspec, cv::DFT_REAL_OUTPUT);
    fftUnpackCCSSpectrum(realspec, cmplxspec);

    image.move(cmplxspec);

    mask.release();
    return  true;
  }

  int flags = cv::DFT_COMPLEX_OUTPUT;
  if( _dft_scale ) {
    flags |= cv::DFT_SCALE;
  }


  for( int c = 0; c < cn; ++c ) {
    cv::dft(channels[c], channels[c], flags);
  }

  switch (_output_display) {
    case DisplayModule:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumModule(channels[c], channels[c]);
      }
      break;

    case DisplayPower:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumPower(channels[c], channels[c]);
      }
      break;

    case DisplayPhase:
      for( int c = 0; c < cn; ++c ) {
        fftSpectrumPhase(channels[c], channels[c]);
      }
      break;

    case DisplayReal:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 0);
      }
      break;

    case DisplayImag:
      for( int c = 0; c < cn; ++c ) {
        cv::extractChannel(channels[c], channels[c], 1);
      }
      break;
  }

  for( int c = 0; c < cn; ++c ) {
    fftSwapQuadrants(channels[c]);
  }

  if ( cn == 1 ) {
    channels[0].copyTo(image);
  }
  else {
    cv::merge(channels, cn, image);
  }

  mask.release();

  return true;
}

