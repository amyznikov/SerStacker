/*
 * QPointSelectionMode.h
 *
 *  Created on: Oct 21, 2024
 *      Author: gandriim
 */

#ifndef __QPointSelectionMode_h__
#define __QPointSelectionMode_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qglshapes/QGLLineShape.h>
#include <gui/qglshapes/QGLRectangleShape.h>
#include "QInputSourceView.h"
#include "QDataAnnotation.h"


namespace serstacker {

class QPointSelectionDefaultMode :
    public QPointSelectionMode
{
public:
  typedef QPointSelectionDefaultMode ThisClass;
  typedef QPointSelectionMode Base;

  QPointSelectionDefaultMode(QObject * parent = nullptr);
};


class QPointSelection3DRulerMode :
    public QPointSelectionMode
{
  Q_OBJECT;

public:
  typedef QPointSelection3DRulerMode ThisClass;
  typedef QPointSelectionMode Base;

  QPointSelection3DRulerMode(QObject * parent = nullptr);

  const QGLLineShape & ruler() const;

  void setActive(QInputSourceView* sourceView, bool activate) final;

  void glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ) final;


Q_SIGNALS:
  void rulerChanged();

protected:
  QGLLineShape _rulerLine;
};

class QPointSelection3DAnnotationMode :
    public QPointSelectionMode
{
public:
  typedef QPointSelection3DAnnotationMode ThisClass;
  typedef QPointSelectionMode Base;

  QPointSelection3DAnnotationMode(QObject * parent = nullptr);

  void setBrushRadius(int v);
  int brushRadius() const;

  void setDepthTolerance(float v);
  float depthTolerance() const;

  void setCDTPlus(float v);
  float cdtplus() const;

  void setCDTMinus(float v);
  float cdtminus() const;

  void setPenColor(const QColor &v);
  QColor penColor() const;

  void setOverwriteExistingLabels(bool v);
  bool overwriteExistingLabels() const;

  void setCurrentColormap(int v);
  int currentColormap() const;

  void setCurrentLabel(uint8_t v);
  uint8_t currentLabel() const;


  void setActive(QInputSourceView* sourceView, bool activate) final;

  void glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ) override;

protected:
  virtual bool annotatePoints(QInputSourceView * sourceView);

protected:
  QGLRectangleShape _rect;
  int _currentColormap = 0;
  int _brushRadius = 15;
  float _depthTolerance = 0.75;
  float _cdtplus = 150;
  float _cdtminus = 50;

  uint8_t _currentLabel = 0;
  bool _overwriteExistingLabels = false;

  cv::Point3f _lastClickPos;
  bool _haveLastClickPos = false;
  bool _hadMouseTracking = false;
};


class QPointSelectionBloomSegmentAnnotationMode :
    public QPointSelectionMode
{
public:
  typedef QPointSelectionBloomSegmentAnnotationMode ThisClass;
  typedef QPointSelectionMode Base;

  QPointSelectionBloomSegmentAnnotationMode(QObject * parent = nullptr);

  void glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ) final;

};

class QDataAnnotationWidget :
    public QSettingsWidgetTemplate<QPointSelection3DAnnotationMode>
{
  Q_OBJECT;
public:
  typedef QDataAnnotationWidget ThisClass;
  typedef QSettingsWidgetTemplate<QPointSelection3DAnnotationMode> Base;

  QDataAnnotationWidget(QWidget * parent = nullptr);

  void set_options(OptionsType * options) final;

  void set_data_annotation_labels(const c_data_annotation_labels *v);
  const c_data_annotation_labels* data_annotation_labels() const;

  void setInputSourceView(QInputSourceView * v);
  QInputSourceView * inputSourceView() const;

Q_SIGNALS:
  void colormapVisibilityChanged(bool visible);

protected:
  void onupdatecontrols() final;
  void populateColormaps();
  void onCurrentColormapMapChanged(int cursel);
  void onCurrentLabelSelectionChanged(int cursel);

protected:
  const c_data_annotation_labels * _data_annotation_labels = nullptr;
  QInputSourceView * _inputSourceView = nullptr;
  QComboBox * colorMapSelection_ctl = nullptr;
  QComboBox * labelSelection_ctl = nullptr;
  QCheckBox * visibilty_ctl = nullptr;
  QCheckBox * overwriteExistingLabels_ctl = nullptr;
  QDoubleSpinBox * blendAlpha_ctl = nullptr;
  QSpinBox * brushRadius_ctl = nullptr;
  QDoubleSpinBox * depthTolerance_ctl = nullptr;
  QDoubleSpinBox * cdtplus_ctl = nullptr;
  QDoubleSpinBox * cdtminus_ctl = nullptr;
};

class QDataAnnotationsDialogBox :
    public QSettingsDialogBoxTemplate<QDataAnnotationWidget>
{
  Q_OBJECT;
public:
  typedef QDataAnnotationsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QDataAnnotationWidget> Base;

  QDataAnnotationsDialogBox(QWidget * parent = nullptr) :
    Base(parent)
  {
    setWindowTitle("3D Data Annotation");

    connect(settings_ctl, &QDataAnnotationWidget::colormapVisibilityChanged,
        this, &ThisClass::colormapVisibilityChanged);
  }

  void set_data_annotation_labels(const c_data_annotation_labels *v)
  {
    settings_ctl->set_data_annotation_labels(v);
  }

  const c_data_annotation_labels* data_annotation_labels() const
  {
    return settings_ctl->data_annotation_labels();
  }

  void setInputSourceView(QInputSourceView * v)
  {
    settings_ctl->setInputSourceView(v);
  }

  QInputSourceView * inputSourceView() const
  {
    return settings_ctl->inputSourceView();
  }

Q_SIGNALS:
  void colormapVisibilityChanged(bool visible);

protected:
};


QToolButton* createPointSelectionModeToolButton(QWidget * parent);

} // namespace serstacker

#endif /* __QPointSelectionMode_h__ */
