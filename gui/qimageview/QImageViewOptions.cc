/*
 * QImageViewOptions.cc
 *
 *  Created on: Jul 1, 2022
 *      Author: amyznikov
 */

#include "QImageViewOptions.h"
#include <gui/widgets/style.h>

#define ICON_pen     ":/qimageview/icons/pen16.png"
#define ICON_circle  ":/qimageview/icons/circle.png"
#define ICON_square  ":/qimageview/icons/square.png"


QImageViewOptions::QImageViewOptions(QWidget * parent) :
  Base(parent) // "QImageViewModeOptions",
{

  Q_INIT_RESOURCE(qimageview_resources);

  displayType_ctl =
      add_enum_combobox<QImageViewer::DisplayType>("Display",
          "",
          [this](QImageViewer::DisplayType v) {
            if ( _opts ) {
              _opts->setDisplayType(v);
              // Q_EMIT parameterChanged();
            }
          },
          [this](QImageViewer::DisplayType * v) {
            if ( _opts ) {
              * v = _opts->displayType();
              return true;
            }
            return false;
          });

  transparentMask_ctl =
      add_checkbox("Transparent mask",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setTransparentMask(checked);
              // Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->transparentMask();
              return true;
            }
            return false;
          });


  keepMaskOnMaskEditMode_ctl =
      add_checkbox("Keep mask",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setKeepMaskOnMaskEditMode(checked);
              // Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->keepMaskOnMaskEditMode();
              return true;
            }
            return false;
          });


  blendAlpha_ctl =
      add_numeric_box<double>("Bleand alpha",
          "",
          [this](double value) {
            if ( _opts ) {
              _opts->setMaskBlendAlpha(value);
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->maskBlendAlpha();
              return true;
            }
            return false;
          });

  penOptions_ctl =
      add_widget<QPenOptionsControl>();

  connect(penOptions_ctl, &QPenOptionsControl::enableEditMaskChanged,
      [this]() {
        if ( _opts ) {
          _opts->setEnableEditMask(penOptions_ctl->enableEditMask());
        }
      });

  connect(penOptions_ctl, &QPenOptionsControl::editMaskPenRadiusChanged,
      [this]() {
        if ( _opts ) {
          _opts->setEditMaskPenRadius(penOptions_ctl->editMaskPenRadius());
        }
      });

  connect(penOptions_ctl, &QPenOptionsControl::editMaskPenShapeChanged,
      [this]() {
        if ( _opts ) {
          _opts->setEditMaskPenShape(penOptions_ctl->editMaskPenShape());
        }
      });

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _opts ) {
          QSignalBlocker block(penOptions_ctl);
          penOptions_ctl->setEnableEditMask(_opts->enableEditMask());
          penOptions_ctl->setEditMaskPenRadius(_opts->editMaskPenRadius());
          penOptions_ctl->setEditMaskPenShape(_opts->editMaskPenShape());
        }
      });

  updateControls();
}


void QImageViewOptions::setImageViewer(QImageViewer * imageViewer)
{
  Base::setOpts(imageViewer);
}

QImageViewer * QImageViewOptions::imageViewer() const
{
  return _opts;
}

void QImageViewOptions::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  if( _opts ) {
    _opts->setEnableEditMask(false);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageViewOptionsDlgBox::QImageViewOptionsDlgBox(QWidget * parent) :
    Base("Image View Options", parent)
{
}

QImageViewOptions * QImageViewOptionsDlgBox::viewOptions() const
{
  return _settings;
}

void QImageViewOptionsDlgBox::setImageViewer(QImageViewer * imageViewer)
{
  _settings->setImageViewer(imageViewer);
}

QImageViewer * QImageViewOptionsDlgBox::imageViewer() const
{
  return _settings->imageViewer();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QPenOptionsControl::QPenOptionsControl(QWidget * parent)
{
  Q_INIT_RESOURCE(qimageview_resources);

  enableEdit_ctl = new QToolButton(this);
  enableEdit_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
  enableEdit_ctl->setText("Edit");
  enableEdit_ctl->setToolTip("Edit mask");
  enableEdit_ctl->setIcon(getIcon(ICON_pen));
  enableEdit_ctl->setCheckable(true);
  addWidget(enableEdit_ctl);



  penShape_ctl = new QComboBox(this);
  penShape_ctl->setEditable(false);
  penShape_ctl->addItem(getIcon(ICON_square), "square", (int)(QImageViewer::PenShape_square));
  penShape_ctl->addItem(getIcon(ICON_circle), "circle", (int)(QImageViewer::PenShape_circle));
  penShape_ctl->setCurrentIndex(0);
  addWidget(penShape_ctl);



  penSize_ctl = new QComboBox(this);
  penSize_ctl->setEditable(true);
  for ( int i = 0; i < 51; ++i ) {
    penSize_ctl->addItem(QString("%1px").arg(i), i);
  }
  penShape_ctl->setCurrentIndex(0);
  addWidget(penSize_ctl);

  connect(enableEdit_ctl, &QToolButton::toggled,
      [this]() {
        if ( !updatingControls_ ) {
          Q_EMIT enableEditMaskChanged();
        }
      });

  connect(penShape_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      [this]() {
        if ( !updatingControls_ ) {
          Q_EMIT editMaskPenShapeChanged();
        }
      });

  connect(penSize_ctl, &QComboBox::currentTextChanged,
      [this]() {
        if ( !updatingControls_ ) {
          Q_EMIT editMaskPenRadiusChanged();
        }
      });
}

void QPenOptionsControl::setEnableEditMask(bool enable)
{
  const bool inUpdatingControls = this->updatingControls_;
  this->updatingControls_ = true;

  enableEdit_ctl->setChecked(enable);

  if ( inUpdatingControls != this->updatingControls_ ) {
    this->updatingControls_ = inUpdatingControls;
  }
}

bool QPenOptionsControl::enableEditMask() const
{
  return enableEdit_ctl->isChecked();
}

void QPenOptionsControl::setEditMaskPenRadius(int v)
{
  const bool inUpdatingControls = this->updatingControls_;
  this->updatingControls_ = true;

  int itemIndex =
      penSize_ctl->findData(v);

  if ( itemIndex >= 0 ) {
    penSize_ctl->setCurrentIndex(itemIndex);
  }
  else {
    penSize_ctl->setCurrentText(QString("%dpx").arg(v));
  }

  if ( inUpdatingControls != this->updatingControls_ ) {
    this->updatingControls_ = inUpdatingControls;
  }
}

int QPenOptionsControl::editMaskPenRadius() const
{
  int v = 0;
  fromString(penSize_ctl->currentText(), &v);
  return v;
}

void QPenOptionsControl::setEditMaskPenShape(QImageViewer::PenShape v)
{
  const bool inUpdatingControls = this->updatingControls_;
  this->updatingControls_ = true;

  int index = penShape_ctl->findData(v);
  if ( index < 0 ) {
    index  = 0;
  }

  penShape_ctl->setCurrentIndex(index);

  if ( inUpdatingControls != this->updatingControls_ ) {
    this->updatingControls_ = inUpdatingControls;
  }
}

QImageViewer::PenShape QPenOptionsControl::editMaskPenShape() const
{
  int cursel = (std::max)(0, penShape_ctl->currentIndex());
  return (QImageViewer::PenShape) (penShape_ctl->itemData(cursel).toInt());
}

