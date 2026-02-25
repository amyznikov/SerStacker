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
#include <map>

namespace serstacker {


class QCloudViewDisplayFunction
{
public:
  typedef QCloudViewDisplayFunction ThisClass;

  virtual ~QCloudViewDisplayFunction() = default;

  virtual void createDisplayPoints(cv::OutputArray mtfColors,
      std::vector<cv::Vec3f> & displayPoints,
      std::vector<cv::Vec3b> & displayColors) = 0;
};


class QGLPointCloudView :
    public QGLView
{
  Q_OBJECT;
public:
  typedef QGLPointCloudView ThisClass;
  typedef QGLView Base;
  typedef std::map<uint64_t, size_t, std::less<uint64_t>> PID2POSMapping;


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
  void setPoints(cv::Mat && points, cv::Mat && colors, cv::Mat && mask, std::vector<uint64_t> && pids);
  void clearPoints();

  const cv::Mat & currentPoints() const;
  const cv::Mat & currentColors() const;
  const cv::Mat & currentMask() const;
  const std::vector<uint64_t> & currentPids() const;
  const PID2POSMapping & currentPid2PosMapping() const;


  const std::vector<cv::Vec3f> & displayPoints() const;
  const std::vector<cv::Vec3b> & displayColors() const;
  const cv::Mat & mtfColors() const;

  bool findPointID(double objX, double objY, double objZ, uint64_t * pid) const;

  void updateDisplayPoints();
  void updateDisplayColors();

Q_SIGNALS:
  void glPointMouseEvent(const QPointF & mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ);

protected:
  void onload(const QSettings & settings, const QString & prefix) final;
  void onsave(QSettings & settings, const QString & prefix) final;
  void glInit() final;
  void glPreDraw() final;
  void glDraw() final;
  void glMouseEvent(const QPointF & mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ) final;

protected:
  // bool findSelectionPid(const QPointF & click_pos, double objX, double objY, double objZ, uint64_t * pid) const;
  void computeDisplayPoints();


protected:
  cv::Mat _currentPoints;
  cv::Mat _currentColors;
  cv::Mat _currentMask;
  cv::Mat _mtfColors;
  std::vector<uint64_t> _currentPids;
  PID2POSMapping _currentPid2PosMapping;

  std::vector<cv::Vec3f> _displayPoints;
  std::vector<cv::Vec3b> _displayColors;

  QCloudViewDisplayFunction * _displayFunction = nullptr;

  QVector3D _sceneOrigin;
  double _pointSize = 2;
  double _pointBrightness = 0;

  bool _update_display_points = false;
  bool _update_display_colors = false;
  int _display_color_channels = 0;
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
  void keyPressEvent(QKeyEvent *event) final;
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
  //void onupdatecontrols() override;

protected:
  QPointCloudSourceView * cloudView_ = nullptr;
  QEnumComboBox<QGLView::Projection> * projection_ctl = nullptr;
  QNumericBox * nearPlane_ctl = nullptr;
  QNumericBox * farPlane_ctl = nullptr;
  QNumericBox * fov_ctl = nullptr;
  QCheckBox *  showMainAxes_ctl = nullptr;
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

  void setCloudView(QPointCloudSourceView * v);
  QPointCloudSourceView * cloudView() const;

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
