/*
 * c_edgebox_routine.cc
 *
 *  Created on: Apr 15, 2023
 *      Author: amyznikov
 *
 *  See /opencv_contrib/modules/ximgproc/samples/edgeboxes_demo.cpp
 */

#include "c_edgebox_routine.h"
#include <core/proc/pyrscale.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_edgebox_routine::DisplayType>()
{
  static const c_enum_member members[] = {

  { c_edgebox_routine::DisplayEdgeMap, "EdgeMap",
      "The output from cv::ximgproc::StructuredEdgeDetection::detectEdges()" },

  { c_edgebox_routine::DisplayEdgeOrientation, "EdgeOrientation",
      "The output from cv::ximgproc::StructuredEdgeDetection::computeOrientation()" },

  { c_edgebox_routine::DisplayEdgeNMS, "EdgeNMS",
      "The output from cv::ximgproc::StructuredEdgeDetection::edgesNms()" },

  { c_edgebox_routine::DisplayBoxes, "Boxes",
      "The output from cv::ximgproc::EdgeBoxes::getBoundingBoxes(EdgeNMS)" },

  { c_edgebox_routine::DisplayEdgeMap }
  };

  return members;
}

template<>
const c_enum_member* members_of<c_edgebox_routine::GradientType>()
{
  static const c_enum_member members[] = {
      { c_edgebox_routine::GradientMagnitude, "GradientMagnitude", "" },
      { c_edgebox_routine::StructuredEdgeDetection, "StructuredEdgeDetection", ""  },
      { c_edgebox_routine::GradientMagnitude },
  };
  return members;
}


static void compute_gradient(const cv::Mat & src, cv::Mat & g, int pscale )
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  constexpr int ddepth = CV_32F;
  const cv::Size src_size = src.size();

  cv::Mat gx, gy;
  cv::Mat image;

  if( pscale <= 0 ) {
    image = src;
  }
  else {
    pyramid_downscale(src, image, pscale);
  }

  cv::filter2D(image, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::filter2D(image, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  cv::add(gx.mul(gx), gy.mul(gy), g);

  if ( g.channels() > 0 ) {
    cv::cvtColor(g, g, cv::COLOR_BGR2GRAY);
  }

  cv::sqrt(g, g);

  if( pscale > 0 ) {
    pyramid_upscale(g, src_size);
  }
}

static void compute_gradients_orientation(cv::InputArray _src, cv::OutputArray _dst)
{
  CV_Assert(_src.type() == CV_32FC1);

  cv::Mat Oxx, Oxy, Oyy;

  _dst.createSameSize(_src, _src.type());
  _dst.setTo(0);

  //cv::Mat src = _src.getMat();
  //cv::Mat E_conv = imsmooth(src, __rf.options.gradientNormalizationRadius);
  cv::Mat E_conv = _src.getMat();

  cv::Sobel(E_conv, Oxx, -1, 2, 0);
  cv::Sobel(E_conv, Oxy, -1, 1, 1);
  cv::Sobel(E_conv, Oyy, -1, 0, 2);

  cv::Mat dst = _dst.getMat();
  float *o = dst.ptr<float>();
  float *oxx = Oxx.ptr<float>();
  float *oxy = Oxy.ptr<float>();
  float *oyy = Oyy.ptr<float>();
  for( int i = 0; i < dst.rows * dst.cols; ++i ) {
    int xysign = -((oxy[i] > 0) - (oxy[i] < 0));
    o[i] = (atan((oyy[i] * xysign / (oxx[i] + 1e-5))) > 0) ? (float) fmod(
        atan((oyy[i] * xysign / (oxx[i] + 1e-5))), CV_PI) : (float) fmod(
        atan((oyy[i] * xysign / (oxx[i] + 1e-5))) + CV_PI, CV_PI);
  }
}


static int imskeleton_morph(cv::InputArray _src, cv::Mat & dst, int ksize, enum cv::MorphShapes shape, int max_iters)
{
  cv::Mat src, K, eroded, temp;
  int num_iters = 0;

  src = _src.getMat().clone();
  dst = cv::Mat::zeros(src.size(), src.type());
  K = cv::getStructuringElement(shape, cv::Size(ksize, ksize));

  if ( max_iters < 1 ) {
    max_iters = INT_MAX;
  }

  do {
    erode(src, eroded, K);
    dilate(eroded, temp, K); // temp = imopen(img)
    subtract(src, temp, temp);
    if ( dst.depth() == CV_8U ) {
      bitwise_or(dst, temp, dst);
    }
    else {
      add(dst, temp, dst);
    }
    src = eroded;
    eroded.release();
  } while ( ++num_iters < max_iters && countNonZero(src) != 0 );

  return num_iters;
}

static void nms(const cv::Mat & src, cv::Mat & dst)
{
  imskeleton_morph(src, dst, 3, cv::MorphShapes::MORPH_RECT, 1000);
}

void c_edgebox_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "gradient_type", ctx(&this_class::_gradient_type), "Method for computing image gradients");
  ctlbind(ctls, "gradient_pscale", ctx(&this_class::_gradient_pscale), "Gradient pyramid scale");
  ctlbind(ctls, "gradient_threshold", ctx(&this_class::_gradient_threshold), "Gradient threshold method");
  ctlbind(ctls, "display", ctx(&this_class::_display), "Display image");
  ctlbind_browse_for_file(ctls, "model", ctx, &this_class::model, &this_class::set_model, "Model file for createStructuredEdgeDetection()\n" "https://github.com/opencv/opencv_extra/blob/master/testdata/cv/ximgproc/model.yml.gz\n");
  ctlbind_expandable_group(ctls, "EdgeBoxes", "Options for cv::ximgproc::EdgeBoxes");
    ctlbind(ctls, "MaxBoxes", ctx, &this_class::MaxBoxes, &this_class::set_MaxBoxes, "max number of boxes to detect");
    ctlbind(ctls, "EdgeMinMag", ctx, &this_class::EdgeMinMag, &this_class::set_EdgeMinMag, "the edge min magnitude");
    ctlbind(ctls, "EdgeMergeThr", ctx, &this_class::EdgeMergeThr, &this_class::set_EdgeMergeThr, "Sets the edge merge threshold");
    ctlbind(ctls, "ClusterMinMag", ctx, &this_class::ClusterMinMag, &this_class::set_ClusterMinMag, "the cluster min magnitude");
    ctlbind(ctls, "MaxAspectRatio", ctx, &this_class::MaxAspectRatio, &this_class::set_MaxAspectRatio, "the max aspect ratio of boxes");
    ctlbind(ctls, "MinBoxArea", ctx, &this_class::MinBoxArea, &this_class::set_MinBoxArea, "the minimum area of boxes");
    ctlbind(ctls, "Gamma", ctx, &this_class::Gamma, &this_class::set_Gamma, "the affinity sensitivity");
    ctlbind(ctls, "Kappa", ctx, &this_class::Kappa, &this_class::set_Kappa, "the scale sensitivity");
  ctlbind_end_group(ctls);
}

bool c_edgebox_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, gradient_type);
    SERIALIZE_PROPERTY(settings, save, *this, gradient_pscale);
    SERIALIZE_PROPERTY(settings, save, *this, gradient_threshold);
    SERIALIZE_PROPERTY(settings, save, *this, model);
    SERIALIZE_PROPERTY(settings, save, *this, display);
    SERIALIZE_PROPERTY(settings, save, *this, MaxBoxes);
    SERIALIZE_PROPERTY(settings, save, *this, EdgeMinMag);
    SERIALIZE_PROPERTY(settings, save, *this, EdgeMergeThr);
    SERIALIZE_PROPERTY(settings, save, *this, ClusterMinMag);
    SERIALIZE_PROPERTY(settings, save, *this, MaxAspectRatio);
    SERIALIZE_PROPERTY(settings, save, *this, MinBoxArea);
    SERIALIZE_PROPERTY(settings, save, *this, Gamma);
    SERIALIZE_PROPERTY(settings, save, *this, Kappa);
    return true;
  }
  return false;
}

bool c_edgebox_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  try {

    if( !_dollar ) {

      if( _model.empty() ) {
        CF_ERROR("Model file not specified");
        return false;
      }

      _dollar =
          cv::ximgproc::createStructuredEdgeDetection(_model);

      if( !_dollar ) {
        CF_ERROR("cv::ximgproc::createStructuredEdgeDetection(model=%s) fails",
            _model.c_str());
        return false;
      }
    }

    cv::Mat img;

    switch (image.depth()) {
      case CV_8U:
        image.getMat().convertTo(img, CV_32F, 1.0 / UINT8_MAX);
        break;
      case CV_8S:
        image.getMat().convertTo(img, CV_32F, 1.0 / INT8_MAX);
        break;
      case CV_16U:
        image.getMat().convertTo(img, CV_32F, 1.0 / UINT16_MAX);
        break;
      case CV_16S:
        image.getMat().convertTo(img, CV_32F, 1.0 / INT16_MAX);
        break;
      case CV_32S:
        image.getMat().convertTo(img, CV_32F, 1.0 / INT32_MAX);
        break;
      case CV_32F:
        image.copyTo(img);
        break;
      case CV_64F:
        image.getMat().convertTo(img, CV_32F);
        break;
    }

    // compute edge map
    switch (_gradient_type) {
      case GradientMagnitude:
        compute_gradient(img, _edges, _gradient_pscale);
        _edges.setTo(0, _edges < EdgeMinMag());
        break;
      case StructuredEdgeDetection:
        _dollar->detectEdges(img, _edges);
        break;
    }
    if( _display == DisplayEdgeMap ) {
      image.move(_edges);
      return true;
    }


    // compute orientation from edge map
    switch (_gradient_type) {
      case GradientMagnitude:
        compute_gradients_orientation(_edges, _orientations);
        break;
      case StructuredEdgeDetection:
        _dollar->computeOrientation(_edges, _orientations);
        break;
    }

    if( _display == DisplayEdgeOrientation ) {
      image.move(_orientations);
      return true;
    }

    // apply edge nms
    switch (_gradient_type) {
      case GradientMagnitude:
        nms(_edges, _edgeNms);
        break;
      case StructuredEdgeDetection:
        _dollar->edgesNms(_edges, _orientations, _edgeNms, 2, 0, 1, true);
        break;
    }

    if( _gradient_threshold != THRESHOLD_TYPE_VALUE ) {
      _edgeNms.setTo(0, _edgeNms < get_threshold_value(_edgeNms, cv::noArray(), _gradient_threshold, 0));
    }

    if( _display == DisplayEdgeNMS ) {
      image.move(_edgeNms);
      return true;
    }


    // apply EdgeBoxes
    _edgeboxes->getBoundingBoxes(_edgeNms, _orientations, _boxes, _scores);

//  CF_DEBUG("\n");

    for( int i = 0, n = (int) _boxes.size(); i < n; ++i ) {

      const cv::Point p1(_boxes[i].x, _boxes[i].y);

      const cv::Point p2(_boxes[i].x + _boxes[i].width, _boxes[i].y + _boxes[i].height);

      const cv::Scalar color(std::max(32, rand() % 255),
          std::max(32, rand() % 255),
          std::max(32, rand() % 255));

      cv::rectangle(image, p1, p2, color, 1);

      //    cv::putText(image,
      //        ssprintf("%g", scores_[i]),
      //        p1,
      //        cv::FONT_HERSHEY_PLAIN,
      //        2,
      //        color,
      //        1,
      //        cv::LINE_AA,
      //        false);

      //  CF_DEBUG("box[%d]: %g", i, scores_[i]);

    }
  }

  catch( const cv::Exception &e ) {
    CF_ERROR("Exception in c_edgebox_routine::process() :\n%s",
        e.msg.c_str());
    return false;
  }

//  CF_DEBUG("\n");

  return true;
}
