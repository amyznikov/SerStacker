/*
 * QInputPointCloudSourceView.h
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputPointCloudSourceView_h__
#define __QInputPointCloudSourceView_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include <gui/qglview/QGLView.h>

namespace serstacker {


class QCloudViewDisplayFunction
{
public:
  typedef QCloudViewDisplayFunction ThisClass;

  virtual ~QCloudViewDisplayFunction() = default;

  virtual void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMask,
      cv::OutputArray displayPoints,
      cv::OutputArray mtfColors,
      cv::OutputArray displayColors) = 0;
};


class QGLPointCloudView :
    public QGLView
{
  Q_OBJECT;
public:
  typedef QGLPointCloudView ThisClass;
  typedef QGLView Base;

  QGLPointCloudView(QWidget* parent = nullptr);

  void setDisplayFunction(QCloudViewDisplayFunction * displayFunc);
  QCloudViewDisplayFunction * displayFunction() const;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QVector3D & v);
  QVector3D sceneOrigin() const;

  void rotateToShowCloud();

  void setPoints(cv::InputArray points, cv::InputArray colors, cv::InputArray mask, bool make_copy = true);
  void clearPoints();

  const cv::Mat & currentPoints() const;
  const cv::Mat & currentColors() const;
  const cv::Mat & currentMask() const;
  const cv::Mat & mtfColors() const;

  const std::vector<cv::Vec3f> & displayPoints() const;
  const std::vector<cv::Vec3b> & displayColors() const;

  void updateDisplayPoints();
  void updateDisplayColors();

protected:
  void glInit() override;
  void glPreDraw() override;
  void glDraw() override;
  void glPostDraw() override;
  void computeDisplayPoints();
  void onLoadParameters(QSettings & settings) override;
  void onSaveParameters(QSettings & settings) override;

protected:
  cv::Mat currentPoints_;
  cv::Mat currentColors_;
  cv::Mat currentMask_;
  cv::Mat mtfColors_;

  std::vector<cv::Vec3f> displayPoints_;
  std::vector<cv::Vec3b> displayColors_;

  QCloudViewDisplayFunction * displayFunction_ = nullptr;

  QVector3D sceneOrigin_;
  double pointSize_ = 2;
  double pointBrightness_ = 0;

  bool update_display_points_ = false;
  bool update_display_colors_ = false;
  int display_color_channels_ = 0;
};


class QPointCloudSourceView :
    public QGLPointCloudView
{
  Q_OBJECT;
public:
  typedef QPointCloudSourceView ThisClass;
  typedef QGLPointCloudView Base;

  QPointCloudSourceView(QWidget * parent = nullptr);

protected:
};


/////////////////

class QPointCloudViewSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QPointCloudViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QPointCloudViewSettings(QWidget * parent = nullptr);

  void setCloudView(QPointCloudSourceView * v);
  QPointCloudSourceView * cloudView() const;

  void refreshCloudList();


protected:
  //void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QPointCloudSourceView * cloudView_ = nullptr;
  QEnumComboBox<QGLView::Projection> * projection_ctl = nullptr;
  QNumericBox * nearPlane_ctl = nullptr;
  QNumericBox * farPlane_ctl = nullptr;
  QNumericBox * fov_ctl = nullptr;
  QNumericBox * mainAxesLength_ctl = nullptr;
  QNumericBox * sceneTarget_ctl = nullptr;
  QNumericBox * upDirection_ctl = nullptr;
  QNumericBox * sceneOrigin_ctl = nullptr;
  QCheckBox   * autoShowViewTarget_ctl = nullptr;
  QNumericBox * pointSize_ctl = nullptr;
  QNumericBox * pointBrightness_ctl = nullptr;
  QColorPickerButton * bgColor_ctl = nullptr;
};

class QPointCloudViewSettingsWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QPointCloudViewSettingsWidget ThisClass;
  typedef QWidget Base;

  QPointCloudViewSettingsWidget(QWidget * parent = nullptr);

  void setCloudView(QPointCloudSourceView * v);
  QPointCloudSourceView * cloudView() const;

protected:
  QPointCloudSourceView * cloudView_ = nullptr;
  QPointCloudViewSettings * settings_ctl = nullptr;
  QToolButton * rotateCameraToShowCloud_ctl = nullptr;
  QToolButton * moveCameraToShowCloud_ctl = nullptr;
  QToolButton * showKeyBindings_ctl = nullptr;
};

class QPointCloudViewSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QPointCloudViewSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QPointCloudViewSettingsDialogBox(QWidget * parent = nullptr);

  void setCloudViewer(QPointCloudSourceView * v);
  QPointCloudSourceView * cloudViewer() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = nullptr;
  QPointCloudViewSettingsWidget * cloudViewSettingsWidget_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;

};

/////////////////
} /* namespace serstacker */

#endif /* __QInputPointCloudSourceView_h__ */
