/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/




#include "c_image_undistort.h"
#include "globalFuncs.h"
#include <core/io/load_image.h>
#include <core/io/c_stdio_file.h>
#include <core/debug.h>


namespace dso
{
c_photometric_undistort * c_photometric_undistort::load(const std::string & calibfile,
    const std::string & noiseImage,
    const std::string & vignetteImage,
    int w_, int h_)
{

  std::vector<float> Gvec;
  cv::Mat vmimage;

  //////////////////////////////
  // Read G
  if ( calibfile.empty() ) {
    CF_WARNING("No photometric Calibration file provided");
  }
  else {

    c_stdio_file f;

    if ( !f.open(calibfile, "rt") ) {
      CF_ERROR("Can not read photometric calibration file '%s'", calibfile.c_str());
      return nullptr;
    }

    Gvec.reserve(256);
    float gv;

    while (fscanf(f, "%f", &gv) == 1) {

      if( !Gvec.empty() && gv <= Gvec.back() ) {
        CF_ERROR("invalid photometric calibration file format: it has to be strictly increasing");
        return nullptr;
      }

      Gvec.emplace_back(gv);
    }

    if( Gvec.size() < 256 ) {
      CF_ERROR("invalid photometric calibration file format: got %zu entries in first line, expected at least 256!",
          Gvec.size());
      return nullptr;
    }

    // make it to 0..255 => 0..255.
    float min = Gvec[0];
    float max = Gvec.back(); // [GDepth - 1];
    for( size_t i = 0, n = Gvec.size(); i < n; i++ ) {
      Gvec[i] = 255.0f * (Gvec[i] - min) / (max - min);
    }
  }



  if( !setting_photometricCalibration ) {
    for( int i = 0, n = (int) Gvec.size(); i < n; i++ ) {
      Gvec[i] = 255.0f * i / (float) (n - 1);
    }
  }


  //////////////////////////////
  // Read Vignette
  if ( vignetteImage.empty() ) {
    CF_WARNING("No vignette image provided");
  }
  else if ( !load_image(vignetteImage, vmimage) ) {
    CF_ERROR("load_image('%s') fails", vignetteImage.c_str());
    return nullptr;
  }
  else if ( vmimage.channels() != 1 ) {
    CF_ERROR("Multi-channel vignette image '%s' not supported", vignetteImage.c_str());
    return nullptr;
  }
  else if ( vmimage.cols != w_ || vmimage.rows != h_ ) {
    CF_ERROR("Invalid vignette image size %dx%d. Expected %dx%d from '%s'",
        vmimage.cols, vmimage.rows,  w_, h_, vignetteImage.c_str());
    return nullptr;
  }
  else {

    double vmin, vmax;
    cv::minMaxLoc(vmimage, &vmin, &vmax);

    if( vmimage.depth() != CV_32F ) {
      vmimage.convertTo(vmimage, CV_32F, 1. / vmax);
    }
    else if ( vmax != 1 ) {
      cv::multiply(vmimage, 1./vmax, vmimage);
    }

    cv::divide(1., vmimage, vmimage);
  }


  this_class * obj = new this_class();
  obj->output_.reset(new c_image_and_exposure(h_, w_));
  obj->G_ = std::move(Gvec);
  obj->vignetteMapInv_ = std::move(vmimage);
  obj->w = w_;
  obj->h = h_;

  return obj;
}

void c_photometric_undistort::unMapFloatImage(float * image)
{
  if( !G_.empty() ) {

    int wh = w * h;

    const float GDepth =
        (float) (G_.size());

    for( int i = 0; i < wh; i++ ) {

      float BinvC;
      float color = image[i];

      if( color < 1e-3 ) {
        BinvC = 0.0f;
      }
      else if( color > GDepth - 1.01f ) {
        BinvC = GDepth - 1.1f;
      }
      else {
        int c = color;
        float a = color - c;
        BinvC = G_[c] * (1 - a) + G_[c + 1] * a;
      }

      float val = BinvC;
      if( val < 0 ) {
        val = 0;
      }

      image[i] = val;
    }
  }
}

template<typename T>
void c_photometric_undistort::processFrame(const T * image_in, float exposure_time, float factor)
{
  // removes readout noise, and converts to irradiance.
  // affine normalizes values to 0 <= I < 256.
  // raw irradiance = a*I + b.
  // output will be written in [output].

  const int wh =
      w * h;

  float * data =
      output_->data();

  assert(output_->cols() == w && output_->rows() == h);
  assert(data != 0);

  if( G_.empty() || exposure_time <= 0 || setting_photometricCalibration == 0 ) {

    // disable full photometric calibration.
    for( int i = 0; i < wh; i++ ) {
      data[i] = factor * image_in[i];
    }

    output_->set_exposure(exposure_time);
    output_->set_timestamp(0);
  }
  else {

    for( int i = 0; i < wh; i++ ) {
      data[i] = G_[image_in[i]];
    }

    if( setting_photometricCalibration == 2 && !vignetteMapInv_.empty() ) {

      const float * vmp =
          (const float*) vignetteMapInv_.data;

      for( int i = 0; i < wh; i++ ) {
        data[i] *= vmp[i];
      }
    }

    output_->set_exposure(exposure_time);
    output_->set_timestamp(0);
  }

  if( !setting_useExposure ) {
    output_->set_exposure(1);
  }
}

template void c_photometric_undistort::processFrame<uint8_t>(const uint8_t* image_in, float exposure_time, float factor);
template void c_photometric_undistort::processFrame<uint16_t>(const uint16_t* image_in, float exposure_time, float factor);


c_image_undistort * c_image_undistort::load(const std::string & configFilename, const std::string & gammaFilename, const std::string & vignetteFilename)
{
  static const auto read_first_line =
      [] (const std::string & fname, char buf[], int bufsize) -> bool {

    c_stdio_file fp;

    if ( fname.empty() ) {
      CF_ERROR("Calibration file name not provided");
      return false;
    }

    if( !fp.open(fname.c_str(), "rt") ) {
      CF_ERROR("Can not read calibration file '%s': %s", fname.c_str(), strerror(errno));
      return false;
    }

    if ( !fgets(buf, bufsize, fp) ) {
      CF_ERROR("Can not read text line from calibration file '%s': %s", fname.c_str(), strerror(errno));
      return false;
    }

    return true;
  };

  char l1[2048] = "";

  if ( !read_first_line(configFilename, l1, sizeof(l1)) )  {
    CF_ERROR("Can not read text line from calibration file '%s': %s",
        configFilename.c_str(), strerror(errno));
    return nullptr;
  }

  CF_DEBUG("Reading calibration from '%s'",
      configFilename.c_str());


  if( strncasecmp(l1, "kitti", 5) == 0 ) {
    return new c_image_undistort_kitti();
  }


  float ic[10] = {0};
  this_class * u = nullptr;

  // for backwards-compatibility: Use RadTan model for 8 parameters.
  if( std::sscanf(l1, "%f %f %f %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]) == 8 ) {
    CF_DEBUG("found RadTan (OpenCV) camera model, building rectifier.");
    if( !(u = c_image_undistort_radtan::load(configFilename.c_str(), true)) ) {
      CF_ERROR("c_image_undistort_radtan::load() fails");
      return nullptr;
    }
  }

  // for backwards-compatibility: Use Pinhole / FoV model for 5 parameter.
  else if( std::sscanf(l1, "%f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4]) == 5 ) {
    if( ic[4] == 0 ) {
      CF_DEBUG("found PINHOLE camera model, building rectifier.");
      if( !(u = c_image_undistort_pinhole::load(configFilename.c_str(), true)) ) {
        CF_ERROR("c_image_undistort_pinhole::load() fails");
        return nullptr;
      }
    }
    else {
      CF_DEBUG("found ATAN camera model, building rectifier.");
      if( !(u = c_image_undistort_fov::load(configFilename.c_str(), true)) ) {
        CF_ERROR("c_image_undistort_fov::load() fails");
        return nullptr;
      }
    }
  }

  // clean model selection implementation.
  else if( std::sscanf(l1, "KannalaBrandt %f %f %f %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]) == 8 ) {
    if( !(u = c_image_undistort_kb::load(configFilename.c_str(), false)) ) {
      CF_ERROR("c_image_undistort_kb::load() fails");
      return nullptr;
    }
  }

  else if( std::sscanf(l1, "RadTan %f %f %f %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]) == 8 ) {
    if( !(u = c_image_undistort_radtan::load(configFilename.c_str(), false)) ) {
      CF_ERROR("c_image_undistort_radtan::load() fails");
      return nullptr;
    }
  }

  else if( std::sscanf(l1, "EquiDistant %f %f %f %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]) == 8 ) {
    if( !(u = c_image_undistort_equidistant::load(configFilename.c_str(), false)) ) {
      CF_ERROR("c_image_undistort_equidistant::load() fails");
      return nullptr;
    }
  }

  else if( std::sscanf(l1, "FOV %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4]) == 5 ) {
    if( !(u = c_image_undistort_fov::load(configFilename.c_str(), false)) ) {
      CF_ERROR("c_image_undistort_fov::load() fails");
      return nullptr;
    }
  }

  else if( std::sscanf(l1, "Pinhole %f %f %f %f %f", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4]) == 5 ) {
    if( !(u = c_image_undistort_pinhole::load(configFilename.c_str(), false)) ) {
      CF_ERROR("c_image_undistort_pinhole::load() fails");
      return nullptr;
    }
  }

  else {
    CF_ERROR("Can not parse calibration from file '%s'", configFilename.c_str());
    return nullptr;
  }

  if( !u->load_photometric_calibration(gammaFilename, "", vignetteFilename) ) {
    CF_ERROR("loadPhotometricCalibration() fails");
    delete u;
    return nullptr;
  }

  return u;
}

bool c_image_undistort::load_photometric_calibration(const std::string & file, const std::string & noiseImage,
    const std::string & vignetteImage)
{
  const Eigen::Vector2i size =
      getOriginalSize();

  photometricUndist_.reset(c_photometric_undistort::load(file, noiseImage, vignetteImage,size[0], size[1]));
  if ( !photometricUndist_ ) {
    CF_ERROR("c_photometric_undistort::load() fails");
    return false;
  }

  return true;
}

void c_image_undistort::undistort_impl(const float in_data[], float out_data[]) const
{
  float * noiseMapX = 0;
  float * noiseMapY = 0;

  if( benchmark_varNoise > 0 ) {

    int numnoise = (benchmark_noiseGridsize + 8) * (benchmark_noiseGridsize + 8);
    noiseMapX = new float[numnoise];
    noiseMapY = new float[numnoise];
    memset(noiseMapX, 0, sizeof(float) * numnoise);
    memset(noiseMapY, 0, sizeof(float) * numnoise);

    for( int i = 0; i < numnoise; i++ ) {
      noiseMapX[i] = 2 * benchmark_varNoise * (rand() / (float) RAND_MAX - 0.5f);
      noiseMapY[i] = 2 * benchmark_varNoise * (rand() / (float) RAND_MAX - 0.5f);
    }
  }

  for( int idx = w * h - 1; idx >= 0; idx-- ) {
    // get interp. values
    float xx = remapX[idx];
    float yy = remapY[idx];

    if( benchmark_varNoise > 0 ) {
      float deltax = getInterpolatedElement11BiCub(noiseMapX, 4 + (xx / (float) wOrg) * benchmark_noiseGridsize,
          4 + (yy / (float) hOrg) * benchmark_noiseGridsize, benchmark_noiseGridsize + 8);
      float deltay = getInterpolatedElement11BiCub(noiseMapY, 4 + (xx / (float) wOrg) * benchmark_noiseGridsize,
          4 + (yy / (float) hOrg) * benchmark_noiseGridsize, benchmark_noiseGridsize + 8);
      float x = idx % w + deltax;
      float y = idx / w + deltay;
      if( x < 0.01 )
        x = 0.01;
      if( y < 0.01 )
        y = 0.01;
      if( x > w - 1.01 )
        x = w - 1.01;
      if( y > h - 1.01 )
        y = h - 1.01;

      xx = getInterpolatedElement(remapX.data(), x, y, w);
      yy = getInterpolatedElement(remapY.data(), x, y, w);
    }

    if( xx < 0 ) {
      out_data[idx] = 0;
    }
    else {

      // get integer and rational parts
      int xxi = xx;
      int yyi = yy;
      xx -= xxi;
      yy -= yyi;
      float xxyy = xx * yy;

      // get array base pointer
      const float * src = in_data + xxi + yyi * wOrg;

      // interpolate (bilinear)
      out_data[idx] = xxyy * src[1 + wOrg]
          + (yy - xxyy) * src[wOrg]
          + (xx - xxyy) * src[1]
          + (1 - xx - yy + xxyy) * src[0];
    }
  }

  if( benchmark_varNoise > 0 ) {
    delete[] noiseMapX;
    delete[] noiseMapY;
  }

}

bool c_image_undistort::undistort(const cv::Mat & image_raw, c_image_and_exposure * output_image, float exposure, double timestamp, float factor) const
{
  if( image_raw.cols != wOrg || image_raw.rows != hOrg ) {
    CF_ERROR("Undistort::undistort: wrong image size (%d %d instead of %d %d) ", image_raw.cols, image_raw.rows, wOrg, hOrg);
    return false;
  }

  if ( dynamic_cast<const c_image_undistort_kitti * >(this) ) {

    output_image->create(h, w,
        timestamp);

    if ( image_raw.channels() == 1 ) {
      image_raw.convertTo(output_image->image(), CV_32F);
    }
    else {
      cv::Mat tmp;
      cv::cvtColor(image_raw, tmp, cv::COLOR_BGR2GRAY);
      tmp.convertTo(output_image->image(), CV_32F);
    }

    return true;
  }




  if( image_raw.type() != CV_8UC1 && image_raw.type() != CV_16UC1 ) {
    CF_ERROR("Undistort::undistort: wrong image type %d ,  must be CV_8UC1 or CV_16UC1", image_raw.type());
    return false;
  }




  output_image->create(h, w,
      timestamp);

  switch (image_raw.type()) {
    case CV_8UC1:
      photometricUndist_->processFrame((const uint8_t*) image_raw.data, exposure, factor);
      break;
    case CV_16UC1:
      photometricUndist_->processFrame((const uint16_t*) image_raw.data, exposure, factor);
      break;
  }

  photometricUndist_->output()->copyMetaTo(*output_image);

  if( passthrough ) {
    output_image->copy_from(photometricUndist_->output()->image());
  }
  else {
    undistort_impl(photometricUndist_->output()->data(), output_image->data());
  }

  applyBlurNoise(output_image->data());

  return true;
}


void c_image_undistort::applyBlurNoise(float* img) const
{
	if(benchmark_varBlurNoise==0) return;

	int numnoise=(benchmark_noiseGridsize+8)*(benchmark_noiseGridsize+8);
	float* noiseMapX=new float[numnoise];
	float* noiseMapY=new float[numnoise];
	float* blutTmp=new float[w*h];

	if(benchmark_varBlurNoise>0)
	{
		for(int i=0;i<numnoise;i++)
		{
				noiseMapX[i] =  benchmark_varBlurNoise  * (rand()/(float)RAND_MAX);
				noiseMapY[i] =  benchmark_varBlurNoise  * (rand()/(float)RAND_MAX);
		}
	}


	float gaussMap[1000];
	for(int i=0;i<1000;i++)
		gaussMap[i] = expf((float)(-i*i/(100.0*100.0)));

	// x-blur.
	for(int y=0;y<h;y++)
		for(int x=0;x<w;x++) {
			float xBlur = getInterpolatedElement11BiCub(noiseMapX,
					4+(x/(float)w)*benchmark_noiseGridsize,
					4+(y/(float)h)*benchmark_noiseGridsize,
					benchmark_noiseGridsize+8 );

			if(xBlur < 0.01) {
			  xBlur=0.01;
			}


			int kernelSize = 1 + (int)(1.0f+xBlur*1.5);
			float sumW=0;
			float sumCW=0;
			for(int dx=0; dx <= kernelSize; dx++) {

			  int gmid = 100.0f*dx/xBlur + 0.5f;
				if(gmid > 900 ) gmid = 900;
				float gw = gaussMap[gmid];

				if(x+dx>0 && x+dx<w) {
					sumW += gw;
					sumCW += gw * img[x+dx+y*this->w];
				}

				if(x-dx>0 && x-dx<w && dx!=0) {
					sumW += gw;
					sumCW += gw * img[x-dx+y*this->w];
				}
			}

			blutTmp[x+y*this->w] = sumCW / sumW;
		}

	// y-blur.
	for(int x=0;x<w;x++)
		for(int y=0;y<h;y++) {
			float yBlur = getInterpolatedElement11BiCub(noiseMapY,
					4+(x/(float)w)*benchmark_noiseGridsize,
					4+(y/(float)h)*benchmark_noiseGridsize,
					benchmark_noiseGridsize+8 );

			if(yBlur < 0.01) yBlur=0.01;

			int kernelSize = 1 + (int)(0.9f+yBlur*2.5);
			float sumW=0;
			float sumCW=0;
			for(int dy=0; dy <= kernelSize; dy++) {
				int gmid = 100.0f*dy/yBlur + 0.5f;
				if(gmid > 900 ) gmid = 900;
				float gw = gaussMap[gmid];

				if(y+dy>0 && y+dy<h) {
					sumW += gw;
					sumCW += gw * blutTmp[x+(y+dy)*this->w];
				}

				if(y-dy>0 && y-dy<h && dy!=0) {
					sumW += gw;
					sumCW += gw * blutTmp[x+(y-dy)*this->w];
				}
			}
			img[x+y*this->w] = sumCW / sumW;
		}



	delete[] noiseMapX;
	delete[] noiseMapY;
}

bool c_image_undistort::makeOptimalK_crop()
{
	CF_DEBUG("finding CROP optimal new model!");
	K.setIdentity();

	// 1. stretch the center lines as far as possible, to get initial coarse quess.
	float* tgX = new float[100000];
	float* tgY = new float[100000];
	float minX = 0;
	float maxX = 0;
	float minY = 0;
	float maxY = 0;

	for(int x=0; x<100000;x++) {
	  tgX[x] = (x-50000.0f) / 10000.0f;
	  tgY[x] = 0;
	}

	distort(tgX, tgY,tgX, tgY,100000);

  for( int x = 0; x < 100000; x++ ) {
    if( tgX[x] > 0 && tgX[x] < wOrg - 1 ) {
      if( minX == 0 ) {
        minX = (x - 50000.0f) / 10000.0f;
      }
      maxX = (x - 50000.0f) / 10000.0f;
    }
  }

  for( int y = 0; y < 100000; y++ ) {
    tgY[y] = (y - 50000.0f) / 10000.0f;
    tgX[y] = 0;
  }

	distort(tgX, tgY,tgX, tgY,100000);

  for( int y = 0; y < 100000; y++ ) {
    if( tgY[y] > 0 && tgY[y] < hOrg - 1 ) {
      if( minY == 0 ) {
        minY = (y - 50000.0f) / 10000.0f;
      }
      maxY = (y - 50000.0f) / 10000.0f;
    }
  }

	delete[] tgX;
	delete[] tgY;

	minX *= 1.01;
	maxX *= 1.01;
	minY *= 1.01;
	maxY *= 1.01;

	CF_DEBUG("initial range: x: %.4f - %.4f; y: %.4f - %.4f!",
	    minX, maxX, minY, maxY);



	// 2. while there are invalid pixels at the border: shrink square at the side that has invalid pixels,
  // if several to choose from, shrink the wider dimension.
  bool oobLeft = true;
  bool oobRight = true;
  bool oobTop = true;
  bool oobBottom = true;
  int iteration = 0;

  while (oobLeft || oobRight || oobTop || oobBottom) {

    oobLeft = oobRight = oobTop = oobBottom = false;

    for( int y = 0; y < h; y++ ) {
      remapX[y * 2] = minX;
      remapX[y * 2 + 1] = maxX;
      remapY[y * 2] = remapY[y * 2 + 1] = minY + (maxY - minY) * (float) y / ((float) h - 1.0f);
    }

    distort(remapX.data(), remapY.data(),
        remapX.data(), remapY.data(),
        2 * h);

    for( int y = 0; y < h; y++ ) {
      if( !(remapX[2 * y] > 0 && remapX[2 * y] < wOrg - 1) ) {
        oobLeft = true;
      }
      if( !(remapX[2 * y + 1] > 0 && remapX[2 * y + 1] < wOrg - 1) ) {
        oobRight = true;
      }
    }

    for( int x = 0; x < w; x++ ) {
      remapY[x * 2] = minY;
      remapY[x * 2 + 1] = maxY;
      remapX[x * 2] = remapX[x * 2 + 1] = minX + (maxX - minX) * (float) x / ((float) w - 1.0f);
    }

    distort(remapX.data(), remapY.data(),
        remapX.data(), remapY.data(),
        2 * w);

    for( int x = 0; x < w; x++ ) {
      if( !(remapY[2 * x] > 0 && remapY[2 * x] < hOrg - 1) ) {
        oobTop = true;
      }

      if( !(remapY[2 * x + 1] > 0 && remapY[2 * x + 1] < hOrg - 1) ) {
        oobBottom = true;
      }
    }

    if( (oobLeft || oobRight) && (oobTop || oobBottom) ) {

      if( (maxX - minX) > (maxY - minY) ) {
        oobBottom = oobTop = false;	// only shrink left/right
      }
      else {
        oobLeft = oobRight = false; // only shrink top/bottom
      }
    }

    if( oobLeft ) {
      minX *= 0.995;
    }

    if( oobRight ) {
      maxX *= 0.995;
    }

    if( oobTop ) {
      minY *= 0.995;
    }

    if( oobBottom ) {
      maxY *= 0.995;
    }

    iteration++;

    CF_DEBUG("iteration %05d: range: x: %.4f - %.4f; y: %.4f - %.4f!", iteration, minX, maxX, minY, maxY);
    if( iteration > 500 ) {
      CF_ERROR("FAILED TO COMPUTE GOOD CAMERA MATRIX - SOMETHING IS SERIOUSLY WRONG. ABORTING ");
      return false;
    }
  }

  K(0, 0) = ((float) w - 1.0f) / (maxX - minX);
  K(1, 1) = ((float) h - 1.0f) / (maxY - minY);
  K(0, 2) = -minX * K(0, 0);
  K(1, 2) = -minY * K(1, 1);

  return true;
}

bool c_image_undistort::makeOptimalK_full()
{
  CF_ERROR("c_image_undistort::makeOptimalK_full() is not implemented");
	return false;
}

bool c_image_undistort::load_parameters(const std::string & configFileName, int nPars, const std::string & prefix)
{
  c_stdio_file fp(configFileName, "rt");

  if ( !fp.is_open() )  {
    CF_ERROR("Can not read config file '%s': %s",
        configFileName.c_str(),
        strerror(errno));
    return false;
  }

  // read parameters

  char lines[4][2048] = {0};

  for( int i = 0; i < 4; ++i ) {
    if( !fgets(lines[i], sizeof(lines[i]), fp) ) {
      CF_ERROR("Can not read config line %d from '%s': %s", i,
          configFileName.c_str(),
          strerror(errno));
      return false;
    }
  }

  fp.close();


  photometricUndist_ = 0;
  passthrough = false;
  remapX.clear();
  remapY.clear();

  float outputCalibration[5];

  parsOrg = VecX(nPars);

  // lines[0] & lines[1]
  if( nPars == 5 ) { // fov model

    char buf[1000];
    snprintf(buf, 1000, "%s%%lf %%lf %%lf %%lf %%lf", prefix.c_str());

    if( std::sscanf(lines[0], buf, &parsOrg[0], &parsOrg[1], &parsOrg[2], &parsOrg[3], &parsOrg[4]) == 5
        && std::sscanf(lines[1], "%d %d", &wOrg, &hOrg) == 2 ) {
      CF_DEBUG("Input resolution: %d %d\nIn: %f %f %f %f %f", wOrg, hOrg,
          parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3], parsOrg[4]);
    }
    else {
      CF_ERROR("Failed to read camera calibration (invalid format?)\nCalibration file: %s", configFileName.c_str());
      return false;
    }
  }
  else if( nPars == 8 ) { // KB, equi & radtan model

    char buf[1000];
    snprintf(buf, 1000, "%s%%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf", prefix.c_str());

    if( std::sscanf(lines[0], buf, &parsOrg[0], &parsOrg[1], &parsOrg[2], &parsOrg[3], &parsOrg[4], &parsOrg[5],
        &parsOrg[6], &parsOrg[7]) == 8 && std::sscanf(lines[1], "%d %d", &wOrg, &hOrg) == 2 ) {
      CF_DEBUG("Input resolution: %d %d\nIn: %s%f %f %f %f %f %f %f %f", wOrg, hOrg, prefix.c_str(),
          parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3], parsOrg[4], parsOrg[5], parsOrg[6], parsOrg[7]);
    }
    else {
      CF_ERROR("Failed to read camera calibration (invalid format?)\nCalibration file: %s", configFileName.c_str());
      return false;
    }
  }
  else {
    CF_ERROR("called with invalid number of parameters.... forgot to implement me?");
    return false;
  }

  if( parsOrg[2] < 1 && parsOrg[3] < 1 ) {
    CF_DEBUG("\n\nFound fx=%f, fy=%f, cx=%f, cy=%f.\n I'm assuming this is the \"relative\" calibration file format,"
        "and will rescale this by image width / height to fx=%f, fy=%f, cx=%f, cy=%f.\n",
        parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3],
        parsOrg[0] * wOrg, parsOrg[1] * hOrg, parsOrg[2] * wOrg - 0.5, parsOrg[3] * hOrg - 0.5);

    // rescale and substract 0.5 offset.
    // the 0.5 is because I'm assuming the calibration is given such that the pixel at (0,0)
    // contains the integral over intensity over [0,0]-[1,1], whereas I assume the pixel (0,0)
    // to contain a sample of the intensity ot [0,0], which is best approximated by the integral over
    // [-0.5,-0.5]-[0.5,0.5]. Thus, the shift by -0.5.
    parsOrg[0] = parsOrg[0] * wOrg;
    parsOrg[1] = parsOrg[1] * hOrg;
    parsOrg[2] = parsOrg[2] * wOrg - 0.5;
    parsOrg[3] = parsOrg[3] * hOrg - 0.5;
  }



  // lines[2]
  if( strncasecmp(lines[2], "crop", 4) == 0 ) {
    outputCalibration[0] = -1;
    CF_DEBUG("Out: Rectify Crop");
  }
  else if( strncasecmp(lines[2], "full", 4) == 0 ) {
    outputCalibration[0] = -2;
    CF_DEBUG("Out: Rectify Full");
  }
  else if( strncasecmp(lines[2], "none", 4) == 0 ) {
    outputCalibration[0] = -3;
    CF_DEBUG("Out: No Rectification");
  }
  else if(std::sscanf(lines[2], "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5) {
    CF_DEBUG("Out: %f %f %f %f %f",
        outputCalibration[0], outputCalibration[1],
        outputCalibration[2], outputCalibration[3],
        outputCalibration[4]);

  }
  else {
    CF_ERROR("Out: Failed to Read Output pars... not rectifying.");
    return false;
  }


  // lines[3]
  if( std::sscanf(lines[3], "%d %d", &w, &h) == 2 ) {

    if( benchmarkSetting_width != 0 ) {
      w = benchmarkSetting_width;
      if( outputCalibration[0] == -3 ) {
        outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
      }
    }

    if( benchmarkSetting_height != 0 ) {
      h = benchmarkSetting_height;
      if( outputCalibration[0] == -3 ) {
        outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
      }
    }

    CF_DEBUG("Output resolution: %d %d", w, h);
  }
  else {
    CF_ERROR("Out: Failed to Read Output resolution... not rectifying.");
    return false;
  }

  remapX.resize(w * h);
  remapY.resize(w * h);

  if( outputCalibration[0] == -1 ) {
    if( !makeOptimalK_crop() ) {
      CF_ERROR("makeOptimalK_crop() fails");
      return false;
    }
  }
  else if( outputCalibration[0] == -2 ) {
    if( !makeOptimalK_full() ) {
      CF_ERROR("makeOptimalK_full() fails");
      return false;
    }
  }
  else if( outputCalibration[0] == -3 ) {

    if( w != wOrg || h != hOrg ) {
      CF_ERROR("ERROR: rectification mode none requires input and output dimenstions to match!");
      return false;
    }

    K.setIdentity();
    K(0, 0) = parsOrg[0];
    K(1, 1) = parsOrg[1];
    K(0, 2) = parsOrg[2];
    K(1, 2) = parsOrg[3];

    passthrough = true;
  }
  else {

    if( outputCalibration[2] > 1 || outputCalibration[3] > 1 ) {
      CF_WARNING("WARNING: given output calibration (%f %f %f %f) seems wrong.\n"
          "It needs to be relative to image width / height!",
          outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3]);
    }

    K.setIdentity();
    K(0, 0) = outputCalibration[0] * w;
    K(1, 1) = outputCalibration[1] * h;
    K(0, 2) = outputCalibration[2] * w - 0.5;
    K(1, 2) = outputCalibration[3] * h - 0.5;
  }

  if(benchmarkSetting_fxfyfac != 0) {

    K(0,0) = fmax(benchmarkSetting_fxfyfac, (float)K(0,0));
    K(1,1) = fmax(benchmarkSetting_fxfyfac, (float)K(1,1));
    passthrough = false; // cannot pass through when fx / fy have been overwritten.
  }


  for( int y = 0; y < h; y++ ) {
    for( int x = 0; x < w; x++ ) {
      remapX[x + y * w] = x;
      remapY[x + y * w] = y;
    }
  }

  distort(remapX.data(), remapY.data(),
      remapX.data(), remapY.data(),
      h * w);

  for( int y = 0; y < h; y++ ) {
    for( int x = 0; x < w; x++ ) {

      // make rounding resistant.
      float ix =
          remapX[x + y * w];

      float iy =
          remapY[x + y * w];

      if( ix == 0 ) {
        ix = 0.001;
      }

      if( iy == 0 ) {
        iy = 0.001;
      }

      if( ix == wOrg - 1 ) {
        ix = wOrg - 1.001;
      }

      if( iy == hOrg - 1 ) {
        ix = hOrg - 1.001;
      }

      if( ix > 0 && iy > 0 && ix < wOrg - 1 && iy < wOrg - 1 ) {
        remapX[x + y * w] = ix;
        remapY[x + y * w] = iy;
      }
      else {
        remapX[x + y * w] = -1;
        remapY[x + y * w] = -1;
      }
    }
  }

  return true;
}

//
//void c_image_undistort::readFromFile(const char* configFileName, int nPars, std::string prefix)
//{
//	photometricUndist_=0;
//	valid = false;
//	passthrough=false;
//	remapX = 0;
//	remapY = 0;
//
//	float outputCalibration[5];
//
//	parsOrg = VecX(nPars);
//
//	// read parameters
//	std::ifstream infile(configFileName);
//	assert(infile.good());
//
//    std::string l1,l2,l3,l4;
//
//	std::getline(infile,l1);
//	std::getline(infile,l2);
//    std::getline(infile,l3);
//    std::getline(infile,l4);
//
//    // l1 & l2
//    if(nPars == 5) // fov model
//	{
//		char buf[1000];
//		snprintf(buf, 1000, "%s%%lf %%lf %%lf %%lf %%lf", prefix.c_str());
//
//		if(std::sscanf(l1.c_str(), buf, &parsOrg[0], &parsOrg[1], &parsOrg[2], &parsOrg[3], &parsOrg[4]) == 5 &&
//				std::sscanf(l2.c_str(), "%d %d", &wOrg, &hOrg) == 2)
//		{
//			printf("Input resolution: %d %d\n",wOrg, hOrg);
//			printf("In: %f %f %f %f %f\n",
//					parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3], parsOrg[4]);
//		}
//		else
//		{
//			printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
//			infile.close();
//			return;
//		}
//	}
//	else if(nPars == 8) // KB, equi & radtan model
//	{
//		char buf[1000];
//		snprintf(buf, 1000, "%s%%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf", prefix.c_str());
//
//		if(std::sscanf(l1.c_str(), buf,
//				&parsOrg[0], &parsOrg[1], &parsOrg[2], &parsOrg[3], &parsOrg[4],
//				&parsOrg[5], &parsOrg[6], &parsOrg[7]) == 8 &&
//				std::sscanf(l2.c_str(), "%d %d", &wOrg, &hOrg) == 2)
//		{
//			printf("Input resolution: %d %d\n",wOrg, hOrg);
//			printf("In: %s%f %f %f %f %f %f %f %f\n",
//					prefix.c_str(),
//					parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3], parsOrg[4], parsOrg[5], parsOrg[6], parsOrg[7]);
//		}
//		else
//		{
//			printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
//			infile.close();
//			return;
//		}
//	}
//	else
//	{
//		printf("called with invalid number of parameters.... forgot to implement me?\n");
//		infile.close();
//		return;
//	}
//
//
//
//    if(parsOrg[2] < 1 && parsOrg[3] < 1)
//    {
//        printf("\n\nFound fx=%f, fy=%f, cx=%f, cy=%f.\n I'm assuming this is the \"relative\" calibration file format,"
//               "and will rescale this by image width / height to fx=%f, fy=%f, cx=%f, cy=%f.\n\n",
//               parsOrg[0], parsOrg[1], parsOrg[2], parsOrg[3],
//               parsOrg[0] * wOrg, parsOrg[1] * hOrg, parsOrg[2] * wOrg - 0.5, parsOrg[3] * hOrg - 0.5 );
//
//        // rescale and substract 0.5 offset.
//        // the 0.5 is because I'm assuming the calibration is given such that the pixel at (0,0)
//        // contains the integral over intensity over [0,0]-[1,1], whereas I assume the pixel (0,0)
//        // to contain a sample of the intensity ot [0,0], which is best approximated by the integral over
//        // [-0.5,-0.5]-[0.5,0.5]. Thus, the shift by -0.5.
//        parsOrg[0] = parsOrg[0] * wOrg;
//        parsOrg[1] = parsOrg[1] * hOrg;
//        parsOrg[2] = parsOrg[2] * wOrg - 0.5;
//        parsOrg[3] = parsOrg[3] * hOrg - 0.5;
//    }
//
//
//
//	// l3
//	if(l3 == "crop")
//	{
//		outputCalibration[0] = -1;
//        printf("Out: Rectify Crop\n");
//	}
//	else if(l3 == "full")
//	{
//		outputCalibration[0] = -2;
//        printf("Out: Rectify Full\n");
//	}
//	else if(l3 == "none")
//	{
//		outputCalibration[0] = -3;
//        printf("Out: No Rectification\n");
//	}
//	else if(std::sscanf(l3.c_str(), "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
//	{
//		printf("Out: %f %f %f %f %f\n",
//				outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3], outputCalibration[4]);
//
//	}
//	else
//	{
//		printf("Out: Failed to Read Output pars... not rectifying.\n");
//		infile.close();
//		return;
//	}
//
//
//	// l4
//	if(std::sscanf(l4.c_str(), "%d %d", &w, &h) == 2)
//	{
//		if(benchmarkSetting_width != 0)
//        {
//			w = benchmarkSetting_width;
//            if(outputCalibration[0] == -3)
//                outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
//        }
//        if(benchmarkSetting_height != 0)
//        {
//			h = benchmarkSetting_height;
//            if(outputCalibration[0] == -3)
//                outputCalibration[0] = -1;  // crop instead of none, since probably resolution changed.
//        }
//
//		printf("Output resolution: %d %d\n",w, h);
//	}
//	else
//	{
//		printf("Out: Failed to Read Output resolution... not rectifying.\n");
//		valid = false;
//    }
//
//    remapX = new float[w*h];
//    remapY = new float[w*h];
//
//	if(outputCalibration[0] == -1)
//		makeOptimalK_crop();
//	else if(outputCalibration[0] == -2)
//		makeOptimalK_full();
//	else if(outputCalibration[0] == -3)
//	{
//		if(w != wOrg || h != hOrg)
//		{
//			printf("ERROR: rectification mode none requires input and output dimenstions to match!\n\n");
//			exit(1);
//		}
//		K.setIdentity();
//        K(0,0) = parsOrg[0];
//        K(1,1) = parsOrg[1];
//        K(0,2) = parsOrg[2];
//        K(1,2) = parsOrg[3];
//		passthrough = true;
//	}
//	else
//	{
//
//
//        if(outputCalibration[2] > 1 || outputCalibration[3] > 1)
//        {
//            printf("\n\n\nWARNING: given output calibration (%f %f %f %f) seems wrong. It needs to be relative to image width / height!\n\n\n",
//                   outputCalibration[0],outputCalibration[1],outputCalibration[2],outputCalibration[3]);
//        }
//
//
//		K.setIdentity();
//        K(0,0) = outputCalibration[0] * w;
//        K(1,1) = outputCalibration[1] * h;
//        K(0,2) = outputCalibration[2] * w - 0.5;
//        K(1,2) = outputCalibration[3] * h - 0.5;
//	}
//
//	if(benchmarkSetting_fxfyfac != 0)
//	{
//		K(0,0) = fmax(benchmarkSetting_fxfyfac, (float)K(0,0));
//		K(1,1) = fmax(benchmarkSetting_fxfyfac, (float)K(1,1));
//        passthrough = false; // cannot pass through when fx / fy have been overwritten.
//	}
//
//
//	for(int y=0;y<h;y++)
//		for(int x=0;x<w;x++)
//		{
//			remapX[x+y*w] = x;
//			remapY[x+y*w] = y;
//		}
//
//	distortCoordinates(remapX, remapY, remapX, remapY, h*w);
//
//
//	for(int y=0;y<h;y++)
//		for(int x=0;x<w;x++)
//		{
//			// make rounding resistant.
//			float ix = remapX[x+y*w];
//			float iy = remapY[x+y*w];
//
//			if(ix == 0) ix = 0.001;
//			if(iy == 0) iy = 0.001;
//			if(ix == wOrg-1) ix = wOrg-1.001;
//			if(iy == hOrg-1) ix = hOrg-1.001;
//
//			if(ix > 0 && iy > 0 && ix < wOrg-1 &&  iy < wOrg-1)
//			{
//				remapX[x+y*w] = ix;
//				remapY[x+y*w] = iy;
//			}
//			else
//			{
//				remapX[x+y*w] = -1;
//				remapY[x+y*w] = -1;
//			}
//		}
//
//	valid = true;
//
//
//
//
//	printf("\nRectified Kamera Matrix:\n");
//	// std::cout << K << "\n\n";
//
//}

c_image_undistort_kitti::c_image_undistort_kitti()
{
  passthrough = true;
  w = wOrg = wUp = 1242;
  h = hOrg = hUp = 375;
  upsampleUndistFactor = 1;

  const Mat33 m {
    {7.215377e+02, 0.000000e+00, 6.095593e+02},
    {0.000000e+00, 7.215377e+02, 1.728540e+02},
    {0.000000e+00, 0.000000e+00, 1.000000e+00}
  };

  K = m;
}


void c_image_undistort_kitti::distort(float * in_x, float * in_y, float * out_x, float * out_y, int n) const
{

}


c_image_undistort_fov* c_image_undistort_fov::load(const std::string & configFileName, bool noprefix)
{
  CF_DEBUG("Creating FOV undistorter");

  this_class * obj = new this_class();

  if( !obj->load_parameters(configFileName, 5, noprefix ? "" : "FOV ") ) {
    CF_ERROR("c_image_undistort_fov: load_parameters() fails");
    delete obj;
    obj = nullptr;
  }

  return obj;
}

void c_image_undistort_fov::distort(float * in_x, float * in_y, float * out_x, float * out_y, int n) const
{
  float dist = parsOrg[4];
  float d2t = 2.0f * tan(dist / 2.0f);

  // current camera parameters
  float fx = parsOrg[0];
  float fy = parsOrg[1];
  float cx = parsOrg[2];
  float cy = parsOrg[3];

  float ofx = K(0, 0);
  float ofy = K(1, 1);
  float ocx = K(0, 2);
  float ocy = K(1, 2);

  for( int i = 0; i < n; i++ ) {
    float x = in_x[i];
    float y = in_y[i];
    float ix = (x - ocx) / ofx;
    float iy = (y - ocy) / ofy;

    float r = sqrtf(ix * ix + iy * iy);
    float fac = (r == 0 || dist == 0) ? 1 : atanf(r * d2t) / (dist * r);

    ix = fx * fac * ix + cx;
    iy = fy * fac * iy + cy;

    out_x[i] = ix;
    out_y[i] = iy;
  }
}


c_image_undistort_radtan * c_image_undistort_radtan::load(const char * configFileName, bool noprefix)
{
  CF_DEBUG("Creating RadTan undistorter");

  this_class * obj = new this_class();

  if( !obj->load_parameters(configFileName, 8, noprefix ? "" : "RadTan ") ) {
    CF_ERROR("c_image_undistort_radtan: load_parameters() fails");
    delete obj;
    obj = nullptr;
  }

  return obj;
}

void c_image_undistort_radtan::distort(float * in_x, float * in_y, float * out_x, float * out_y, int n) const
{
  // RADTAN
  float fx = parsOrg[0];
  float fy = parsOrg[1];
  float cx = parsOrg[2];
  float cy = parsOrg[3];
  float k1 = parsOrg[4];
  float k2 = parsOrg[5];
  float r1 = parsOrg[6];
  float r2 = parsOrg[7];

  float ofx = K(0, 0);
  float ofy = K(1, 1);
  float ocx = K(0, 2);
  float ocy = K(1, 2);

  for( int i = 0; i < n; i++ ) {

    float x = in_x[i];
    float y = in_y[i];

    // RADTAN
    float ix = (x - ocx) / ofx;
    float iy = (y - ocy) / ofy;
    float mx2_u = ix * ix;
    float my2_u = iy * iy;
    float mxy_u = ix * iy;
    float rho2_u = mx2_u + my2_u;
    float rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    float x_dist = ix + ix * rad_dist_u + 2.0 * r1 * mxy_u + r2 * (rho2_u + 2.0 * mx2_u);
    float y_dist = iy + iy * rad_dist_u + 2.0 * r2 * mxy_u + r1 * (rho2_u + 2.0 * my2_u);
    float ox = fx * x_dist + cx;
    float oy = fy * y_dist + cy;

    out_x[i] = ox;
    out_y[i] = oy;
  }

}



c_image_undistort_equidistant* c_image_undistort_equidistant::load(const char * configFileName, bool noprefix)
{
  CF_DEBUG("Creating Equidistant undistorter");

  this_class * obj = new this_class();

  if( !obj->load_parameters(configFileName, 8, noprefix ? "" : "EquiDistant ") ) {
    CF_ERROR("c_image_undistort_equidistant: load_parameters() fails");
    delete obj;
    obj = nullptr;
  }

  return obj;
}

void c_image_undistort_equidistant::distort(float * in_x, float * in_y, float * out_x, float * out_y,
    int n) const
{
  // EQUI
  float fx = parsOrg[0];
  float fy = parsOrg[1];
  float cx = parsOrg[2];
  float cy = parsOrg[3];
  float k1 = parsOrg[4];
  float k2 = parsOrg[5];
  float k3 = parsOrg[6];
  float k4 = parsOrg[7];

  float ofx = K(0, 0);
  float ofy = K(1, 1);
  float ocx = K(0, 2);
  float ocy = K(1, 2);

  for( int i = 0; i < n; i++ ) {
    float x = in_x[i];
    float y = in_y[i];

    // EQUI
    float ix = (x - ocx) / ofx;
    float iy = (y - ocy) / ofy;
    float r = sqrt(ix * ix + iy * iy);
    float theta = atan(r);
    float theta2 = theta * theta;
    float theta4 = theta2 * theta2;
    float theta6 = theta4 * theta2;
    float theta8 = theta4 * theta4;
    float thetad = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    float scaling = (r > 1e-8) ? thetad / r : 1.0;
    float ox = fx * ix * scaling + cx;
    float oy = fy * iy * scaling + cy;

    out_x[i] = ox;
    out_y[i] = oy;
  }
}

c_image_undistort_kb* c_image_undistort_kb::load(const char * configFileName, bool noprefix)
{
  CF_DEBUG("Creating KannalaBrandt undistorter");

  this_class * obj = new this_class();

  if( !obj->load_parameters(configFileName, 8, noprefix ? "" : "KannalaBrandt ") ) {
    CF_ERROR("c_image_undistort_kb: load_parameters() fails");
    delete obj;
    obj = nullptr;
  }

  return obj;
}

void c_image_undistort_kb::distort(float * in_x, float * in_y, float * out_x, float * out_y, int n) const
{
  const float fx = parsOrg[0];
  const float fy = parsOrg[1];
  const float cx = parsOrg[2];
  const float cy = parsOrg[3];
  const float k0 = parsOrg[4];
  const float k1 = parsOrg[5];
  const float k2 = parsOrg[6];
  const float k3 = parsOrg[7];

  const float ofx = K(0, 0);
  const float ofy = K(1, 1);
  const float ocx = K(0, 2);
  const float ocy = K(1, 2);

  for( int i = 0; i < n; i++ ) {

    float x = in_x[i];
    float y = in_y[i];

    // RADTAN
    float ix = (x - ocx) / ofx;
    float iy = (y - ocy) / ofy;

    const float Xsq_plus_Ysq = ix * ix + iy * iy;
    const float sqrt_Xsq_Ysq = sqrtf(Xsq_plus_Ysq);
    const float theta = atan2f(sqrt_Xsq_Ysq, 1);
    const float theta2 = theta * theta;
    const float theta3 = theta2 * theta;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + k0 * theta3 + k1 * theta5 + k2 * theta7 + k3 * theta9;

    if( sqrt_Xsq_Ysq < 1e-6 ) {
      out_x[i] = fx * ix + cx;
      out_y[i] = fy * iy + cy;
    }
    else {
      out_x[i] = (r / sqrt_Xsq_Ysq) * fx * ix + cx;
      out_y[i] = (r / sqrt_Xsq_Ysq) * fy * iy + cy;
    }
  }
}



c_image_undistort_pinhole * c_image_undistort_pinhole::load(const char * configFileName, bool noprefix)
{
  CF_DEBUG("Creating pinhole undistorter");

  this_class * obj = new this_class();

  if( !obj->load_parameters(configFileName, 5, noprefix ? "" : "Pinhole ") ) {
    CF_ERROR("c_image_undistort_radtan: load_parameters() fails");
    delete obj;
    obj = nullptr;
  }

  return obj;
}

void c_image_undistort_pinhole::distort(float * in_x, float * in_y, float * out_x, float * out_y,
    int n) const
{
  // current camera parameters
  float fx = parsOrg[0];
  float fy = parsOrg[1];
  float cx = parsOrg[2];
  float cy = parsOrg[3];

  float ofx = K(0, 0);
  float ofy = K(1, 1);
  float ocx = K(0, 2);
  float ocy = K(1, 2);

  for( int i = 0; i < n; i++ ) {
    float x = in_x[i];
    float y = in_y[i];
    float ix = (x - ocx) / ofx;
    float iy = (y - ocy) / ofy;
    ix = fx * ix + cx;
    iy = fy * iy + cy;
    out_x[i] = ix;
    out_y[i] = iy;
  }
}


}
