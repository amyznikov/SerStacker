/*
 * QDSOCloudView.h
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDSOCloudView_h__
#define __QDSOCloudView_h__

#include <gui/qglview/QGLPointCloudView.h>
#include <gui/qmtf/QMtfDisplay.h>
#include <dso/FullSystem.h>

namespace qdso {

class QDSOCloudView :
    public QGLPointCloudView,
    public IMtfDisplay,
    public QCloudViewDisplayFunction
{
  Q_OBJECT;
  Q_INTERFACES(IMtfDisplay);
public:
  typedef QDSOCloudView ThisClass;
  typedef QGLPointCloudView Base;

  QDSOCloudView(QWidget * parent = nullptr);

  void loadParameters();
  void saveParameters();

  //void showPoints(cv::InputArray pts);
  //void addKeyframe(const dso::FrameHessian * fh, bool _final, const dso::CalibHessian * HCalib);
  void displayKeyframes(const std::vector<dso::FrameHessian*>  & frames, const dso::CalibHessian * HCalib);

Q_SIGNALS: // IMtfDisplay
  void displayChannelsChanged();
  void parameterChanged();
  void displayImageChanged();

protected: // IMtfDisplay
  //QStringList displayChannels() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

protected: // QCloudViewDisplayFunction
  void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMask,
      cv::OutputArray displayPoints,
      cv::OutputArray mtfColors,
      cv::OutputArray displayColors) override;

Q_SIGNALS:
  void redrawRequired(QPrivateSignal sig = QPrivateSignal());

protected:

  struct c_keyframe_display
  {
    typedef c_keyframe_display this_class;
    typedef std::shared_ptr<this_class> sptr;
    typedef std::unique_ptr<this_class> uptr;

    struct InputPointSparse
    {
      float u;
      float v;
      float idpeth;
      float idepth_hessian;
      float relObsBaseline;
      int numGoodRes;
      uint8_t color;
      uint8_t status;
    };

    std::vector<InputPointSparse> pc;
    std::vector<cv::Vec3f> vertex;
    std::vector<cv::Vec3b> color;

    int id = 0;
    bool active = false;
    cv::Matx44f camToWorld;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float fxi = 0, fyi = 0, cxi = 0, cyi = 0;

    float my_scaledTH = 0;
    float my_absTH = 0;
    float my_scale = 0;
    int my_sparsifyFactor = 0;
    int my_displayMode = 0;
    float my_minRelBS = 0;
    bool needRefresh = true;

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    void setFromF(const dso::c_frame_shell * fs, const dso::CalibHessian * HCalib);

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    void setFromKF(const dso::FrameHessian* fh, const dso::CalibHessian* HCalib);


    // copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
    bool refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity);

  };

   std::map<int, c_keyframe_display::uptr> keyframes;

};



} /* namespace qdso */

#endif /* __QDSOCloudView_h__ */
