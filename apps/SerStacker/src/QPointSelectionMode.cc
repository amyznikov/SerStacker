/*
 * QPointSelectionMode.cc
 *
 *  Created on: Oct 21, 2024
 *      Author: gandriim
 */

#include "QPointSelectionMode.h"
#include <gui/widgets/style.h>
#include <core/debug.h>

#define ICON_annotation         ":/serstacker/icons/annotation3.png"

namespace serstacker
{

using ColorMap =
    c_data_annotation_labels::ColorMap::uptr;

using Label =
    c_data_annotation_labels::Label;


QToolButton* createPointSelectionModeToolButton(QWidget *parent)
{
  QToolButton *tb = new QToolButton(parent);
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(getIcon(ICON_annotation));
  tb->setText("Annotation");
  tb->setToolTip("Point annotation options");
  //tb->setCheckable(true);
  //tb->setPopupMode(QToolButton::ToolButtonPopupMode::MenuButtonPopup);
  tb->setPopupMode(QToolButton::ToolButtonPopupMode::InstantPopup);

  return tb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QPointSelectionDefaultMode::QPointSelectionDefaultMode(QObject *parent) :
    Base(parent)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QPointSelection3DRulerMode::QPointSelection3DRulerMode(QObject *parent) :
    Base(parent)
{
  _rulerLine.setTopLevel(true);
  _rulerLine.setEnabled(false);
  _rulerLine.setVisible(false);

}

const QGLLineShape & QPointSelection3DRulerMode::ruler() const
{
  return _rulerLine;
}

void QPointSelection3DRulerMode::setActive(QInputSourceView* sourceView, bool activate)
{
  if ( activate ) {
    _rulerLine.setEnabled(true);
    _rulerLine.setVisible(false);
    sourceView->cloudView()->addShape(&_rulerLine);
  }
  else {
    _rulerLine.setEnabled(false);
    _rulerLine.setVisible(false);
    sourceView->cloudView()->removeShape(&_rulerLine);
  }

  Base::setActive(sourceView, activate);
}

void QPointSelection3DRulerMode::glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{
  //CF_DEBUG("QPointSelection3DRulerMode: currentViewType=%d", sourceView->currentViewType());
  if (mouseEventType == QEvent::MouseButtonRelease ) {
    if (_rulerLine.isVisible()) {
      //_rulerLine.setVisible(false);
      //sourceView->cloudView()->update();
      _rulerLine.setEnableTooltip(false);
    }
  }
  else {

    if (objHit && mouseButtons == Qt::LeftButton /*&& keyboardModifiers == Qt::ControlModifier*/) {

      uint64_t pid;
      if (sourceView->cloudView()->findPointID(objX, objY, objZ, &pid)) {
        Q_EMIT sourceView->glPointClick(pid, mousePos, mouseEventType,
            mouseButtons, keyboardModifiers);
      }

      switch(mouseEventType)  {

      case QEvent::MouseButtonPress:
        if (_rulerLine.isEnabled()) {
          _rulerLine.setStart(objX, objY, objZ);
          _rulerLine.setEnd(objX, objY, objZ);
          sourceView->cloudView()->update();

          Q_EMIT rulerChanged();

        }
        break;

      case QEvent::MouseMove:
        if (_rulerLine.isEnabled()) {

          _rulerLine.setEnd(objX, objY, objZ);

          if (!_rulerLine.isVisible()) {
            _rulerLine.setVisible(true);
          }

          _rulerLine.setEnableTooltip(true);

          sourceView->cloudView()->update();

          Q_EMIT rulerChanged();
        }
        break;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QPointSelection3DAnnotationMode::QPointSelection3DAnnotationMode(QObject *parent) :
    Base(parent)
{
  _rect.setVisible(false);
  _rect.setEnabled(false);
  _rect.setTopLevel(true);
}

void QPointSelection3DAnnotationMode::setBrushRadius(int v)
{
  _brushRadius = v;
}

int QPointSelection3DAnnotationMode::brushRadius() const
{
  return _brushRadius;
}


void QPointSelection3DAnnotationMode::setDepthTolerance(float v)
{
  _depthTolerance = v;
}

float QPointSelection3DAnnotationMode::depthTolerance() const
{
  return _depthTolerance;
}

void QPointSelection3DAnnotationMode::setCDTPlus(float v)
{
  _cdtplus = v;
}

float QPointSelection3DAnnotationMode::cdtplus() const
{
  return _cdtplus;
}

void QPointSelection3DAnnotationMode::setCDTMinus(float v)
{
  _cdtminus = v;
}

float QPointSelection3DAnnotationMode::cdtminus() const
{
  return _cdtminus;
}

void QPointSelection3DAnnotationMode::setPenColor(const QColor &v)
{
  _rect.setPenColor(v);
}

QColor QPointSelection3DAnnotationMode::penColor() const
{
  return _rect.penColor();
}

void QPointSelection3DAnnotationMode::setOverwriteExistingLabels(bool v)
{
  _overwriteExistingLabels = v;
}

bool QPointSelection3DAnnotationMode::overwriteExistingLabels() const
{
  return _overwriteExistingLabels;
}

void QPointSelection3DAnnotationMode::setCurrentColormap(int v)
{
  _currentColormap = v;
}

int QPointSelection3DAnnotationMode::currentColormap() const
{
  return _currentColormap;
}

void QPointSelection3DAnnotationMode::setCurrentLabel(uint8_t v)
{
  _currentLabel = v;
}

uint8_t QPointSelection3DAnnotationMode::currentLabel() const
{
  return _currentLabel;
}

void QPointSelection3DAnnotationMode::setActive(QInputSourceView* sourceView, bool activate)
{
  if ( activate ) {
    _rect.setEnabled(true);
    _rect.setVisible(false);
    sourceView->cloudView()->addShape(&_rect);

    _hadMouseTracking = sourceView->cloudView()->hasMouseTracking();
    sourceView->cloudView()->setMouseTracking(true);
 }
  else {
    _rect.setEnabled(false);
    _rect.setVisible(false);
    sourceView->cloudView()->removeShape(&_rect);
    sourceView->cloudView()->setMouseTracking(_hadMouseTracking);
  }

  Base::setActive(sourceView, activate);
}

void QPointSelection3DAnnotationMode::glMouseEvent(QInputSourceView *sourceView, const QPointF &mousePos,
    QEvent::Type eventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{
  //CF_DEBUG("QPointSelection3DAnnotationMode: currentViewType=%d", sourceView->currentViewType());

  if (eventType == QEvent::MouseButtonRelease
      || (eventType == QEvent::KeyRelease && keyboardModifiers != Qt::ControlModifier)) {

    if (_rect.isVisible()) {
      _rect.setVisible(false);
      sourceView->cloudView()->update();
    }
  }
  else if (eventType == QEvent::MouseMove && mouseButtons == Qt::NoButton) {
    if (keyboardModifiers == Qt::ControlModifier) {

      const QPoint mpos((int) mousePos.x(),
          (int) mousePos.y());

      _rect.setRect(mpos.x() - _brushRadius, mpos.y() - _brushRadius,
          2 * _brushRadius + 1, 2 * _brushRadius + 1);

      if (!_rect.isVisible()) {
        _rect.setVisible(true);
      }

      sourceView->cloudView()->update();
    }
  }

  else {

    uint64_t pid = 0;
    bool havepid = false;

    if (objHit && (mouseButtons == Qt::LeftButton)) {
      if ((havepid = sourceView->cloudView()->findPointID(objX, objY, objZ, &pid))) {
        Q_EMIT sourceView->glPointClick(pid, mousePos, eventType,
            mouseButtons, keyboardModifiers);
      }
    }

    if (_rect.isEnabled()) {

      const c_data_frame::sptr &currentFrame =
          sourceView->currentFrame();

      if (currentFrame && currentFrame->supports_point_annotations()) {

        const QPoint mpos((int) mousePos.x(),
            (int) mousePos.y());

        switch (eventType) {

        case QEvent::MouseButtonPress: {

          cv::Point3f cp;
          if ( sourceView->cloudView()->projectToScreen(cv::Vec3f(objX, objY, objZ), &cp) )  {
            _lastClickPos = cp;
            _haveLastClickPos = true;
          }


//          _haveLastClickPos =
//              sourceView->cloudView()->projectToScreen(cv::Vec3f(objX, objY, objZ),
//                  &_lastClickPos);

          //CF_DEBUG("MouseButtonPress: _have_last_click_pos=%d", _have_last_click_pos);

          break;
        }

        case QEvent::MouseButtonDblClick: {
//          _haveLastClickPos =
//              sourceView->cloudView()->projectToScreen(cv::Vec3f(objX, objY, objZ),
//                  &_lastClickPos);

          cv::Point3f cp;
          if ( sourceView->cloudView()->projectToScreen(cv::Vec3f(objX, objY, objZ), &cp) )  {
            _lastClickPos = cp;
            _haveLastClickPos = true;
          }

          if (havepid) {
          }
          break;
        }

        case QEvent::MouseMove:
          //CF_DEBUG("MouseMove: _have_last_click_pos=%d", _have_last_click_pos);
          if (_haveLastClickPos ) {

            _rect.setRect(mpos.x() - _brushRadius, mpos.y() - _brushRadius,
                2 * _brushRadius + 1, 2 * _brushRadius + 1);

            if (!_rect.isVisible()) {
              _rect.setVisible(true);
            }

            if (annotatePoints(sourceView)) {
              sourceView->cloudView()->updateDisplayColors();
            }
            else {
              sourceView->cloudView()->update();
            }
          }
          break;
        }
      }
    }
  }

}


bool QPointSelection3DAnnotationMode::annotatePoints(QInputSourceView * sourceView)
{
  static const auto contains =
      [](const cv::Rect &rc, const cv::Point3f &pt) -> bool {
        return rc.x <= pt.x && pt.x < rc.x + rc.width && rc.y <= pt.y && pt.y < rc.y + rc.height;
      };


//  CF_DEBUG("_brushDepth=%g", _brushDepth);

  int nhits = 0;

  const c_data_frame::sptr & currentFrame =
      sourceView->currentFrame();

  // already checked by caller
  //if ( currentFrame && currentFrame->supports_point_annotations() )
  {

    QPointCloudSourceView *cloudView =
        sourceView->cloudView();

    const std::vector<cv::Vec3f> &displayPoints =
        cloudView->displayPoints();

    const std::vector<uint64_t> &pids =
        cloudView->currentPids();

    if (displayPoints.size() == pids.size()) {

      //cloudView->depthBuffer();


      const QRect &qrc =
          _rect.rect();

      const cv::Rect rc(qrc.x(), qrc.y(),
          qrc.width(), qrc.height());

      cv::Point3f spos;


      for (size_t i = 0, n = displayPoints.size(); i < n; ++i) {

        const cv::Vec3f &pos =
            displayPoints[i];

        if (cloudView->projectToScreen(pos, &spos) && contains(rc, spos)) {
          if ( std::abs(_lastClickPos.z - spos.z)  < _depthTolerance ) {

            if (_overwriteExistingLabels || !currentFrame->point_annotation(pids[i], _currentColormap)) {
              currentFrame->set_point_annotation(pids[i],
                  _currentColormap,
                  _currentLabel);

            }

            ++nhits;
          }
        }
      }

    }
  }

  return nhits > 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QPointSelectionBloomSegmentAnnotationMode::QPointSelectionBloomSegmentAnnotationMode(QObject *parent) :
    Base(parent)
{
}

void QPointSelectionBloomSegmentAnnotationMode::glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{
  // CF_DEBUG("QPointSelectionBloomSegmentAnnotationMode: currentViewType=%d", sourceView->currentViewType());
  return Base::glMouseEvent(sourceView, mousePos, mouseEventType, mouseButtons, keyboardModifiers,
      objHit, objX, objY, objZ);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QDataAnnotationWidget::QDataAnnotationWidget(QWidget * parent) :
    Base(parent)
{
  colorMapSelection_ctl =
      add_combobox<QComboBox>("Select Colormap:",
          "",
          false,
          [this](int cursel, QComboBox */*combo*/) {
            onCurrentColormapMapChanged(cursel);
          });

  labelSelection_ctl =
      add_combobox<QComboBox>("Select Label:",
          "",
          false,
          [this](int cursel, QComboBox*) {
            onCurrentLabelSelectionChanged(cursel);
          });

  visibilty_ctl =
      add_checkbox("Visible",
          "",
          [this](bool checked) {
            if ( _data_annotation_labels ) {
              const int colormapIndex = colorMapSelection_ctl->currentIndex();
              if ( colormapIndex >= 0 && colormapIndex < _data_annotation_labels->num_colormaps() ) {
                _data_annotation_labels->colormap(colormapIndex)->set_visible(checked);
                Q_EMIT colormapVisibilityChanged(checked);// parameterChanged();
              }
            }
          },
          [this](bool *checked) {
            if ( _data_annotation_labels ) {
              const int colormapIndex = colorMapSelection_ctl->currentIndex();
              if ( colormapIndex >= 0 && colormapIndex < _data_annotation_labels->num_colormaps() ) {
                *checked = _data_annotation_labels->colormap(colormapIndex)->visible();
                return true;
              }
            }
            return false;
          });


  overwriteExistingLabels_ctl =
      add_checkbox("Overwrite existing labels:",
          "",
          [this](bool checked) {
            if ( options_ && options_->overwriteExistingLabels() != checked ) {
              options_->setOverwriteExistingLabels(checked);
              Q_EMIT colormapVisibilityChanged(checked);  // parameterChanged();
          }
        },
          [this](bool *checked) {
            if ( options_ ) {
              *checked = options_->overwriteExistingLabels();
              return true;
            }
            return false;
          });

  blendAlpha_ctl =
      add_double_spinbox("Color Blend Alpha:",
          "Brush depth in [m]",
          [this](double value) {
            if ( _inputSourceView ) {
              _inputSourceView->setDataAnnotationBlendAlpha(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](double *value) {
            if ( _inputSourceView ) {
              * value = _inputSourceView->dataAnnotationBlendAlpha();
              return true;
            }
            return false;
          });


  brushRadius_ctl =
      add_spinbox("Brush Radius:",
          "",
          [this](int value) {
            if ( options_ && options_->brushRadius() != value ) {
              options_->setBrushRadius(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int *value) {
            if ( options_ ) {
              * value = options_->brushRadius();
              return true;
            }
            return false;
          });


  depthTolerance_ctl =
      add_double_spinbox("Depth Tolerance [m]:",
          "Brush depth in [m]",
          [this](double value) {
            if ( options_ ) {
              options_->setDepthTolerance(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](double *value) {
            if ( options_ ) {
              * value = options_->depthTolerance();
              return true;
            }
            return false;
          });

  cdtplus_ctl =
      add_double_spinbox("cdtplus [m]:",
          "cluster forwad depth tolerance in [m] ",
          [this](double value) {
            if ( options_ ) {
              options_->setCDTPlus(value * 1e2);
              Q_EMIT parameterChanged();
            }
          },
          [this](double *value) {
            if ( options_ ) {
              * value = 1e-2 * options_->cdtplus();
              return true;
            }
            return false;
          });


  cdtminus_ctl =
      add_double_spinbox("cdtminus [m]:",
          "cluster backward depth tolerance in [m] ",
          [this](double value) {
            if ( options_ ) {
              options_->setCDTMinus(value * 1e2);
              Q_EMIT parameterChanged();
            }
          },
          [this](double *value) {
            if ( options_ ) {
              * value = 1e-2 * options_->cdtminus();
              return true;
            }
            return false;
          });

  blendAlpha_ctl->setRange(0, 1);
  blendAlpha_ctl->setSingleStep(1e-2);
  blendAlpha_ctl->setDecimals(2);
  blendAlpha_ctl->setValue(0.8);

  depthTolerance_ctl->setRange(0, 10);
  depthTolerance_ctl->setSingleStep(1e-2);
  depthTolerance_ctl->setDecimals(2);


  cdtplus_ctl->setRange(0, 10);
  cdtplus_ctl->setSingleStep(1e-2);
  cdtplus_ctl->setDecimals(2);

  cdtminus_ctl->setRange(0, 10);
  cdtminus_ctl->setSingleStep(1e-2);
  cdtminus_ctl->setDecimals(2);

  updateControls();
}


void QDataAnnotationWidget::onupdatecontrols()
{
  if ( !options_ || !_data_annotation_labels) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    update_control_states();
    setEnabled(true);
  }
}

void QDataAnnotationWidget::set_options(OptionsType * options)
{
  Base::set_options(options);
  populateColormaps();
}

void QDataAnnotationWidget::set_data_annotation_labels(const c_data_annotation_labels *v)
{
  _data_annotation_labels = v;
  updateControls();
  populateColormaps();
}

const c_data_annotation_labels* QDataAnnotationWidget::data_annotation_labels() const
{
  return _data_annotation_labels;
}

void QDataAnnotationWidget::setInputSourceView(QInputSourceView * v)
{
  if ((_inputSourceView = v)) {
    blendAlpha_ctl->setValue(v->dataAnnotationBlendAlpha());
  }
}

QInputSourceView * QDataAnnotationWidget::inputSourceView() const
{
  return _inputSourceView;
}

void QDataAnnotationWidget::populateColormaps()
{
  colorMapSelection_ctl->blockSignals(true);

  colorMapSelection_ctl->clear();
  if ( _data_annotation_labels ) {
    for ( const auto & colormap : _data_annotation_labels->colormaps() ) {
      colorMapSelection_ctl->addItem(colormap->name().c_str());
    }
  }

  if (colorMapSelection_ctl->count() > 0) {
    colorMapSelection_ctl->setCurrentIndex(0);
    onCurrentColormapMapChanged(colorMapSelection_ctl->currentIndex());
  }

  colorMapSelection_ctl->blockSignals(false);
}

void QDataAnnotationWidget::onCurrentColormapMapChanged(int cursel)
{
  labelSelection_ctl->blockSignals(true);
  visibilty_ctl->blockSignals(true);

  labelSelection_ctl->clear();

  if ( !_data_annotation_labels || cursel < 0 || cursel >= _data_annotation_labels->num_colormaps() ) {
    labelSelection_ctl->setEnabled(false);
    visibilty_ctl->setEnabled(false);
  }
  else {

    const ColorMap & colormap =
        _data_annotation_labels->colormap(cursel);

    visibilty_ctl->setChecked(colormap->visible());
    Q_EMIT colormapVisibilityChanged(colormap->visible());

    for (auto ii = colormap->begin(); ii != colormap->end(); ++ii) {

      const auto & label_name =
          ii->second.name;

      const uint8_t label_value =
          ii->first;

      labelSelection_ctl->addItem(label_name.c_str(),
          QVariant::fromValue(label_value));

    }

    if ( labelSelection_ctl->count()  < 1 ) {
      labelSelection_ctl->setEnabled(false);
    }
    else {
      labelSelection_ctl->setCurrentIndex(0);
      labelSelection_ctl->setEnabled(true);
    }
  }

  labelSelection_ctl->blockSignals(false);
  visibilty_ctl->blockSignals(false);

  if (options_) {
    options_->setCurrentColormap(colorMapSelection_ctl->currentIndex());
  }


  onCurrentLabelSelectionChanged(labelSelection_ctl->currentIndex());
}



void QDataAnnotationWidget::onCurrentLabelSelectionChanged(int currentLabelIndex)
{
  if (_data_annotation_labels && options_ ) {

    const int currentColormapIndex =
        colorMapSelection_ctl->currentIndex();

    if (currentColormapIndex >= 0 && currentColormapIndex < _data_annotation_labels->num_colormaps()) {

      const ColorMap &colormap =
          _data_annotation_labels->colormap(currentColormapIndex);

      if (currentLabelIndex >= 0 && currentLabelIndex < colormap->size()) {

        const uint8_t label_value =
            labelSelection_ctl->currentData().value<uint8_t>();

        options_->setCurrentLabel(label_value);

        // Q_EMIT parameterChanged();
      }
    }
  }
}

}// namespace serstacker
