/*
 * QDSOCloudView.cc
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#include "QDSOCloudView.h"
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/debug.h>

namespace qdso {


//template<class PointType>
//static inline cv::Vec3f compute_world_pos(const PointType * p, const cv::Matx44f & m, float fxi, float fyi, float cxi, float cyi)
//{
//  const float u = p->u;
//  const float v = p->v;
//  const float idpeth = p->idepth_scaled;
//  const float relObsBaseline = p->maxRelBaseline;
//  const float idepth_hessian = p->idepth_hessian;
//  const float depth = 10.0f / idpeth;
//  const float dx = 0;
//  const float dy = 0;
//
//  cv::Vec4f cp = m * cv::Vec4f(depth * ((u + dx) * fxi + cxi),
//      depth * ((v + dy) * fyi + cyi),
//      depth * (1 + 2 * fxi),
//      1);
//
////  cv::Vec4f cp =
////      m * cv::Vec4f(+depth * (1 + 2 * fxi),
////      -depth * ((u + dx) * fxi + cxi),
////      -depth * ((v + dy) * fyi + cyi),
////      1);
//
//  // xnew = z
//  // ynew = -x
//  // znew = -y
//
//  return cv::Vec3f(cp[2], -cp[0], -cp[1] );
//}

static inline cv::Vec3f compute_world_pos(const cv::Vec3f & cp, const cv::Matx44f & m)
{
  const cv::Vec4f wp =
      m * cv::Vec4f(cp[0], cp[1], cp[2], 1);

  // xnew = z
  // ynew = -x
  // znew = -y

  return cv::Vec3f(wp[2], -wp[0], -wp[1]);
}

// copies points from KF over to internal buffer,
// keeping some additional information so we can render it differently.
void QDSOCloudView::c_keyframe_display::setFromF(const dso::c_frame_shell * fs, const dso::CalibHessian * HCalib)
{
  id = fs->id;
  fx = HCalib->fxl();
  fy = HCalib->fyl();
  cx = HCalib->cxl();
  cy = HCalib->cyl();
  //  width = wG[0];
  //  height = hG[0];
  fxi = 1 / fx;
  fyi = 1 / fy;
  cxi = -cx / fx;
  cyi = -cy / fy;

  const Sophus::Matrix4f sm =
      fs->camToWorld.matrix().transpose().cast<float>();

  this->camToWorld =
      cv::Matx44f(sm.data());

  //camToWorld = fs->camToWorld;
  needRefresh = true;
}

// copies points from KF over to internal buffer,
// keeping some additional information so we can render it differently.
void QDSOCloudView::c_keyframe_display::setFromKF(const dso::FrameHessian * fh, const dso::CalibHessian * HCalib)
{
  setFromF(fh->shell, HCalib);

  // add all traces, inlier and outlier points.
  const size_t npoints =
      fh->immaturePoints.size() +
          fh->pointHessians.size() +
          fh->pointHessiansMarginalized.size() +
          fh->pointHessiansOut.size();

  pc.reserve(npoints);

//  for( const dso::ImmaturePoint * p : fh->immaturePoints ) {
//
//    pc.emplace_back();
//
//    InputPointSparse & pt =
//        pc.back();
//
//    pt.color = p->color[0];
//    pt.u = p->u;
//    pt.v = p->v;
//    pt.idpeth = (p->idepth_max + p->idepth_min) * 0.5f;
//    pt.idepth_hessian = 1000;
//    pt.relObsBaseline = 0;
//    pt.numGoodRes = 1;
//    pt.status = 0;
//  }

  for( const dso::PointHessian * p : fh->pointHessians ) {

    pc.emplace_back();

    InputPointSparse & pt =
        pc.back();

    pt.color = p->color[0];
    pt.u = p->u;
    pt.v = p->v;
    pt.idpeth = p->idepth_scaled;
    pt.relObsBaseline = p->maxRelBaseline;
    pt.idepth_hessian = p->idepth_hessian;
    pt.numGoodRes = 0;
    pt.status = 1;
  }

  for( const dso::PointHessian * p : fh->pointHessiansMarginalized ) {

    pc.emplace_back();

    InputPointSparse & pt =
        pc.back();

    pt.color = p->color[0];
    pt.u = p->u;
    pt.v = p->v;
    pt.idpeth = p->idepth_scaled;
    pt.relObsBaseline = p->maxRelBaseline;
    pt.idepth_hessian = p->idepth_hessian;
    pt.numGoodRes = 0;
    pt.status = 2;
  }

  for( const dso::PointHessian * p : fh->pointHessiansOut ) {

    pc.emplace_back();

    InputPointSparse & pt =
        pc.back();

    pt.color = p->color[0];
    pt.u = p->u;
    pt.v = p->v;
    pt.idpeth = p->idepth_scaled;
    pt.relObsBaseline = p->maxRelBaseline;
    pt.idepth_hessian = p->idepth_hessian;
    pt.numGoodRes = 0;
    pt.status = 3;
  }

  //camToWorld = fh->PRE_camToWorld;

//  const Sophus::Matrix4f sm =
//      fh->PRE_camToWorld.matrix().transpose().cast<float>();

//  const Sophus::Matrix4f sm =
//      fh->PRE_camToWorld.matrix().cast<float>();
//
//  this->camToWorld =
//      cv::Matx44f(sm.data());

  needRefresh = true;
}



// copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
bool QDSOCloudView::c_keyframe_display::refreshPC(bool canRefresh, float scaledTH, float absTH,
    int mode, float minBS, int sparsity)
{
  if( canRefresh ) {

    needRefresh = needRefresh ||
        my_scaledTH != scaledTH ||
        my_absTH != absTH ||
        my_displayMode != mode ||
        my_minRelBS != minBS ||
        my_sparsifyFactor != sparsity;
  }

  if( !needRefresh ) {
    return false;
  }


  needRefresh = false;

  my_scaledTH = scaledTH;
  my_absTH = absTH;
  my_displayMode = mode;
  my_minRelBS = minBS;
  my_sparsifyFactor = sparsity;


  // if there are no vertices, done!
  if( pc.empty() ) {
    return false;
  }

  vertex.clear();
  //vertex.reserve(pc.size() * patternNum);
  vertex.reserve(pc.size());

  for(const auto & sp : pc ) {
    /* display modes:
     * my_displayMode==0 - all pts, color-coded
     * my_displayMode==1 - normal points
     * my_displayMode==2 - active only
     * my_displayMode==3 - nothing
     */
    if( my_displayMode > 2 ) {
      continue;
    }

    if( my_displayMode == 1 && sp.status != 1 && sp.status != 2 ) {
      continue;
    }

    if( my_displayMode == 2 && sp.status != 1 ) {
      continue;
    }

    if( sp.idpeth < 0 ) {
      continue;
    }

    if( sp.relObsBaseline < my_minRelBS ) {
      continue;
    }

    const float depth =
        1.0f / sp.idpeth;

    const float depth4 =
        depth * depth * depth * depth;

    const float var =
        (1.0f / (sp.idepth_hessian + 0.01));

    if(var > my_absTH) {
      continue;
    }

    if( var * depth4 > my_scaledTH ) {
      continue;
    }

    //for( int pnt = 0; pnt < patternNum; pnt++ )
    {

//      if( my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0 ) {
//        continue;
//      }

      const int dx = 0;
          // dso::patternP[pnt][0];

      const int dy = 0;
          // dso::patternP[pnt][1];

//      vertex.emplace_back(depth * ((sp.u + dx) * fxi + cxi),
//          depth * ((sp.v + dy) * fyi + cyi),
//          depth * (1 + 2 * fxi * (rand() / (float) RAND_MAX - 0.5f)));

      vertex.emplace_back(depth * ((sp.u + dx) * fxi + cxi),
          depth * ((sp.v + dy) * fyi + cyi),
          depth);

      if( my_displayMode != 0 ) {
        color.emplace_back(sp.color, sp.color, sp.color);
      }
      else if( sp.status == 0 ) {
        color.emplace_back(0, 255, 555);
      }
      else if( sp.status == 1 ) {
        color.emplace_back(0, 255, 0);
      }
      else if( sp.status == 2 ) {
        color.emplace_back(0, 0, 255);
      }
      else if( sp.status == 3 ) {
        color.emplace_back(255, 0, 0);
      }
      else {
        color.emplace_back(255, 255, 255);
      }
    }
  }

  if( vertex.empty() ) {
    return true;
  }

//  numGLBufferGoodPoints = vertexBufferNumPoints;
//  if(numGLBufferGoodPoints > numGLBufferPoints) {
//    numGLBufferPoints = vertexBufferNumPoints*1.3;
//    vertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
//    colorBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW );
//  }
//  vertexBuffer.Upload(tmpVertexBuffer, sizeof(float)*3*numGLBufferGoodPoints, 0);
//  colorBuffer.Upload(tmpColorBuffer, sizeof(unsigned char)*3*numGLBufferGoodPoints, 0);
//  bufferValid=true;
//  delete[] tmpColorBuffer;
//  delete[] tmpVertexBuffer;


  return true;
}


QDSOCloudView::QDSOCloudView(QWidget * parent) :
    Base(parent)
{
  setDisplayFunction(this);

  connect(this, &ThisClass::redrawRequired,
      this, &ThisClass::updateDisplayPoints,
      Qt::QueuedConnection);

  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::updateDisplayPoints,
      Qt::QueuedConnection);

  addDisplay("default", 0, 255);
  setDisplayChannel("default");
}

void QDSOCloudView::loadParameters()
{
  Base::loadParameters();
}

void QDSOCloudView::saveParameters()
{
  Base::saveParameters();
}

void QDSOCloudView::displayKeyframes(const std::vector<dso::FrameHessian*> & frames, const dso::CalibHessian * HCalib)
{
  const float settings_absVarTH = 0.001;
  const float settings_scaledVarTH = 0.001;
  const int settings_pointCloudMode = 1;
  const float settings_minRelBS = 0.1;
  const int settings_sparsity = 0;

  for( const dso::FrameHessian * fh : frames ) {

    auto ii =
        keyframes.find(fh->frameID);

    if ( ii == keyframes.end() ) {
      ii = keyframes.emplace(fh->frameID, new c_keyframe_display()).first;
    }

    const auto & kf =
        ii->second;

    kf->setFromKF(fh, HCalib);
  }

  int total_points = 0;
  for ( auto ii = keyframes.begin(); ii != keyframes.end(); ++ii ) {

    const auto & kf =
        ii->second;

    kf->refreshPC(true, settings_scaledVarTH, settings_absVarTH,
        settings_pointCloudMode, settings_minRelBS, settings_sparsity);

    total_points +=
        kf->vertex.size();
  }




  display_lock_.lock();

  if ( total_points < 1 ) {
    currentPoints_.release();
    currentColors_.release();
  }
  else {

    currentPoints_.create(total_points, 1, CV_32FC3);
    currentColors_.create(total_points, 1, CV_8UC3);

    cv::Mat3f pts =
          currentPoints_;

    cv::Mat3b clrs =
          currentColors_;

    total_points = 0;

    for ( auto ii = keyframes.begin(); ii != keyframes.end(); ++ii ) {

      const auto & kf =
          ii->second;

      const cv::Matx44f & m =
          kf->camToWorld;

      for ( int i = 0, n = kf->vertex.size(); i < n; ++i, ++total_points ) {

        pts[total_points][0] =
            compute_world_pos(kf->vertex[i], m);

        clrs[total_points][0] =
            kf->color[i];

      }
    }

  }



  display_lock_.unlock();

  Q_EMIT redrawRequired();

}
//
//void QDSOCloudView::displayKeyframes(const std::vector<dso::FrameHessian*>  & frames, const dso::CalibHessian * HCalib)
//{
////  const float fx = HCalib->fxl();
////  const float fy = HCalib->fyl();
////  const float cx = HCalib->cxl();
////  const float cy = HCalib->cyl();
////  const float fxi = 1 / fx;
////  const float fyi = 1 / fy;
////  const float cxi = -cx / fx;
////  const float cyi = -cy / fy;
////
////  size_t total_points = 0;
////
////  for( const auto * fh : frames ) {
////    total_points +=
////        fh->pointHessians.size() +
////            fh->pointHessiansMarginalized.size();
////  }
////
////  display_lock_.lock();
////
////  currentPoints_.create(total_points, 1, CV_32FC3);
////
////  cv::Mat3f pts =
////      currentPoints_;
////
////  total_points = 0;
////  for( const auto * fh : frames ) {
////
////    const Sophus::Matrix4f m =
////        fh->PRE_camToWorld.matrix().inverse().cast<float>();
////
////    const cv::Matx44f cam2world =
////        cv::Matx44f(m.data());
////
////    for( const auto * p : fh->pointHessians ) {
////      pts[total_points++][0] = compute_world_pos(p, cam2world, fxi, fyi, cxi, cyi);
////    }
////
////    for( const auto * p : fh->pointHessiansMarginalized ) {
////      pts[total_points++][0] = compute_world_pos(p, cam2world, fxi, fyi, cxi, cyi);
////    }
////  }
////
////  currentColors_.create(currentPoints_.size(), CV_8UC3);
////  currentColors_.setTo(cv::Scalar::all(255));
////
////  display_lock_.unlock();
////
////  Q_EMIT redrawRequired();
//
//}

//
//void QDSOCloudView::addKeyframe(const dso::FrameHessian * fh, bool _final, const dso::CalibHessian * HCalib)
//{
//  if ( _final ) {
//
//    keyframes.emplace_back(new c_keyframe());
//
//    const c_keyframe::uptr & frame =
//        keyframes.back();
//
//    Sophus::Matrix4f m =
//        fh->PRE_camToWorld.matrix().inverse().cast<float>();
////
////    Sophus::Matrix4f m =
////        fh->PRE_worldToCam.matrix().cast<float>();
//
//    frame->cam2world =
//        cv::Matx44f(m.data());
//
//    const size_t npoints =
//            fh->pointHessians.size() +
//            fh->pointHessiansMarginalized.size();
//
//    const float fx = HCalib->fxl();
//    const float fy = HCalib->fyl();
//    const float cx = HCalib->cxl();
//    const float cy = HCalib->cyl();
//    const float fxi = 1 / fx;
//    const float fyi = 1 / fy;
//    const float cxi = -cx / fx;
//    const float cyi = -cy / fy;
//
//    frame->points.reserve(npoints);
//
//    for( const auto * p : fh->pointHessians ) {
//      frame->points.emplace_back(compute_world_pos(p, frame->cam2world, fxi, fyi, cxi, cyi));
//    }
//
//    for( const auto * p : fh->pointHessiansMarginalized ) {
//      frame->points.emplace_back(compute_world_pos(p, frame->cam2world, fxi, fyi, cxi, cyi));
//    }
//
//
//    size_t total_points = 0 ;
//    for ( const auto & f : keyframes ) {
//      total_points += f->points.size();
//    }
//
//    display_lock_.lock();
//
//    currentPoints_.create(total_points, 1, CV_32FC3);
//
//    cv::Mat3f pts =
//        currentPoints_;
//
//    total_points = 0;
//    for ( const auto & f : keyframes ) {
//      for ( const auto & p : f->points ) {
//        pts[total_points++][0] = p;
//      }
//    }
//
//    currentColors_.create(currentPoints_.size(), CV_8UC3);
//    currentColors_.setTo(cv::Scalar::all(255));
//
//    display_lock_.unlock();
//
//    Q_EMIT redrawRequired();
//  }
//
//
//}

//void QDSOCloudView::showPoints(cv::InputArray points)
//{
//  display_lock_.lock();
//
//  points.getMat().copyTo(currentPoints_);
//  currentColors_.create(currentPoints_.size(), CV_8UC3);
//  currentColors_.setTo(cv::Scalar::all(255));
//
//  display_lock_.unlock();
//
//  Q_EMIT redrawCloud();
//}



void QDSOCloudView::createDisplayPoints(cv::InputArray currentPoints,
    cv::InputArray currentColors,
    cv::InputArray currentMask,
    cv::OutputArray displayPoints,
    cv::OutputArray mtfColors,
    cv::OutputArray displayColors)
{
  if ( currentPoints.empty() ) {

    displayPoints.release();
    mtfColors.release();
    displayColors.release();

    Q_EMIT displayImageChanged();

    return;
  }

  DisplayParams & opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  cv::Mat mtfcolors, displaycolors;

  const bool needColormap =
      opts.colormap != COLORMAP_NONE;// &&
          //currentColors.channels() == 1;


  currentPoints.getMat().convertTo(displayPoints, CV_32F);


  adjustMtfRange(mtf, needColormap ? currentColors : cv::noArray(), currentMask, &a);
  mtf->apply(currentColors, mtfcolors, CV_8U);
  restoreMtfRange(mtf, a);

  if ( !mtfColors.fixedType() || mtfColors.type() == mtfcolors.type() ) {
    mtfcolors.copyTo(mtfColors);
  }
  else {
    mtfcolors.convertTo(mtfColors, mtfColors.type());
  }

  if ( needColormap ) {

    if( mtfcolors.channels() != 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_RGB2GRAY);
    }

    cv::applyColorMap(mtfcolors,
        displaycolors,
        opts.lut);

    if( currentMask.size() == displaycolors.size() ) {
      displaycolors.setTo(0, ~currentMask.getMat());
    }

    cv::cvtColor(displaycolors, displaycolors, cv::COLOR_BGR2RGB);

    displaycolors.copyTo(displayColors);

  }

  else {

    if( mtfcolors.channels() == 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_GRAY2BGR);
    }

    mtfcolors.copyTo(displayColors);
  }

  Q_EMIT displayImageChanged();
}

void QDSOCloudView::getInputDataRange(double * minval, double * maxval) const
{
  getminmax(currentColors(), minval, maxval, currentMask());
}

void QDSOCloudView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(currentColors(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}

void QDSOCloudView::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(mtfColors(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}

} /* namespace qdso */
