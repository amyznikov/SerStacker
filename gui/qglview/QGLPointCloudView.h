/*
 * QGLPointCloudView.h
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGLPointCloudView_h__
#define __QGLPointCloudView_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>
#include <gui/qglview/QGLView.h>



class QCloudViewDisplayFunction
{
public:
  typedef QCloudViewDisplayFunction ThisClass;

  virtual ~QCloudViewDisplayFunction() = default;

  virtual void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMasks,
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

  struct CloudSettings {
    cv::Vec3f translation;
    cv::Vec3f rotation;
    cv::Vec3f scale;
    cv::Vec3b color = cv::Vec3b(255,255,255);
    bool visible = true;
    bool override_color = false;
  };


  QGLPointCloudView(QWidget* parent = nullptr);

  void setDisplayFunction(QCloudViewDisplayFunction * displayFunc);
  QCloudViewDisplayFunction * displayFunction() const;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QVector3D & v);
  QVector3D sceneOrigin() const;

  std::vector<CloudSettings> & cloudSettings();
  const std::vector<CloudSettings> & cloudSettings() const;

  void rotateToShowCloud();

  void setPoints(cv::InputArrayOfArrays points, cv::InputArrayOfArrays colors, cv::InputArrayOfArrays masks, bool make_copy = true);
  void clearPoints();

  const std::vector<cv::Mat> & currentPoints() const;
  const std::vector<cv::Mat> & currentColors() const;
  const std::vector<cv::Mat> & currentMasks() const;

  const cv::Mat & currentPoints(uint32_t index) const;
  const cv::Mat & currentColors(uint32_t index) const;
  const cv::Mat & currentMasks(uint32_t index) const;

  const std::vector<std::vector<cv::Vec3f>> & displayPoints() const;
  const std::vector<std::vector<cv::Vec3b>> & displayColors() const;
  const std::vector<cv::Mat> & mtfColors() const;

  void updateDisplayPoints();
  void updateDisplayColors();

protected:
  void glInit() override;
  void glPreDraw() override;
  void glDraw() override;
  void computeDisplayPoints();
  void onLoadParameters(QSettings & settings) override;
  void onSaveParameters(QSettings & settings) override;
 // void mousePressEvent(QMouseEvent *e) override;

protected:
  std::vector<cv::Mat> _currentPoints;
  std::vector<cv::Mat> _currentColors;
  std::vector<cv::Mat> _currentMasks;

  std::vector<std::vector<cv::Vec3f>> _displayPoints;
  std::vector<std::vector<cv::Vec3b>> _displayColors;
  std::vector<cv::Mat> _mtfColors;
  std::mutex _displayLock;

  std::vector<CloudSettings> _cloudSettings;

  QCloudViewDisplayFunction * _displayFunction = nullptr;

  QVector3D _sceneOrigin;
  double _pointSize = 2;
  double _pointBrightness = 0;

  int _displayColorChannels = 0;
  bool _updateDisplayPoints = false;
  bool _updateDisplayColors = false;

};


class QGlPointCloudViewSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGlPointCloudViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QGlPointCloudViewSettings(QWidget * parent = nullptr);

  void setCloudView(QGLPointCloudView * v);
  QGLPointCloudView * cloudView() const;

  void refreshCloudList();


protected:
  //void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QGLPointCloudView * cloudView_ = nullptr;
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

class QGlPointCloudViewSettingsWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QGlPointCloudViewSettingsWidget ThisClass;
  typedef QWidget Base;

  QGlPointCloudViewSettingsWidget(QWidget * parent = nullptr);

  void setCloudView(QGLPointCloudView * v);
  QGLPointCloudView * cloudView() const;

protected:
  QGLPointCloudView * cloudView_ = nullptr;
  QGlPointCloudViewSettings * settings_ctl = nullptr;
  QToolButton * rotateCameraToShowCloud_ctl = nullptr;
  QToolButton * moveCameraToShowCloud_ctl = nullptr;
  QToolButton * showKeyBindings_ctl = nullptr;
};

class QGlPointCloudViewSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QGlPointCloudViewSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGlPointCloudViewSettingsDialogBox(QWidget * parent = nullptr);

  void setCloudViewer(QGLPointCloudView * v);
  QGLPointCloudView * cloudViewer() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = nullptr;
  QGlPointCloudViewSettingsWidget * cloudViewSettingsWidget_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};


class QGlPointCloudSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGlPointCloudSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QGlPointCloudSettingsWidget(QWidget * parent = nullptr);

  void setCloudView(QGLPointCloudView * v);
  QGLPointCloudView * cloudView() const;

protected:
  void onupdatecontrols() override;
  void updatecontrolstates();
  void populatecombobox();
  void onAddSettingsItem();
  void onDeleteSettingsItem();
  void onCurrentItemIndexChanged();

protected:
  QGLPointCloudView::CloudSettings * currentItem() const;

protected:
  QGLPointCloudView * cloudView_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QLabel * itemSelectionLb_ctl = nullptr;
  QComboBox * itemSelection_ctl = nullptr;
  QAction * addItem_action = nullptr;
  QAction * removeItem_action = nullptr;

  QCheckBox * itemVisible_ctl = nullptr;
  QCheckBox * overrideColor_ctl = nullptr;
  QNumericBox * itemColor_ctl = nullptr;
  QNumericBox * itemTranslation_ctl = nullptr;
  QNumericBox * itemRotation_ctl = nullptr;
  QNumericBox * itemScale_ctl = nullptr;

};


class QGlPointCloudSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QGlPointCloudSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGlPointCloudSettingsDialogBox(QWidget * parent = nullptr);

  void setCloudViewer(QGLPointCloudView * v);
  QGLPointCloudView * cloudViewer() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = nullptr;
  QGlPointCloudSettingsWidget * cloudSettingsWidget_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};


#endif /* __QGLPointCloudView_h__ */
