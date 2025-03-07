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
  Base("QImageViewModeOptions", parent)
{

  Q_INIT_RESOURCE(qimageview_resources);

  displayType_ctl =
      add_enum_combobox<QImageViewer::DisplayType>("Display",
          "",
          [this](QImageViewer::DisplayType v) {
            if ( imageViewer_ ) {
              imageViewer_->setDisplayType(v);
              // Q_EMIT parameterChanged();
            }
          },
          [this](QImageViewer::DisplayType * v) {
            if ( imageViewer_ ) {
              * v = imageViewer_->displayType();
              return true;
            }
            return false;
          });

  transparentMask_ctl =
      add_checkbox("Transparent mask",
          "",
          [this](bool checked) {
            if ( imageViewer_ ) {
              imageViewer_->setTransparentMask(checked);
              // Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( imageViewer_ ) {
              * checked = imageViewer_->transparentMask();
              return true;
            }
            return false;
          });


  keepMaskOnMaskEditMode_ctl =
      add_checkbox("Keep mask",
          "",
          [this](bool checked) {
            if ( imageViewer_ ) {
              imageViewer_->setKeepMaskOnMaskEditMode(checked);
              // Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( imageViewer_ ) {
              * checked = imageViewer_->keepMaskOnMaskEditMode();
              return true;
            }
            return false;
          });


  blendAlpha_ctl =
      add_numeric_box<double>("Bleand alpha",
          "",
          [this](double value) {
            if ( imageViewer_ ) {
              imageViewer_->setMaskBlendAlpha(value);
            }
          },
          [this](double * value) {
            if ( imageViewer_ ) {
              * value = imageViewer_->maskBlendAlpha();
              return true;
            }
            return false;
          });

  penOptions_ctl =
      add_widget<QPenOptionsControl>();

  connect(penOptions_ctl, &QPenOptionsControl::enableEditMaskChanged,
      [this]() {
        if ( imageViewer_ ) {
          imageViewer_->setEnableEditMask(penOptions_ctl->enableEditMask());
        }
      });

  connect(penOptions_ctl, &QPenOptionsControl::editMaskPenRadiusChanged,
      [this]() {
        if ( imageViewer_ ) {
          imageViewer_->setEditMaskPenRadius(penOptions_ctl->editMaskPenRadius());
        }
      });

  connect(penOptions_ctl, &QPenOptionsControl::editMaskPenShapeChanged,
      [this]() {
        if ( imageViewer_ ) {
          imageViewer_->setEditMaskPenShape(penOptions_ctl->editMaskPenShape());
        }
      });

  updateControls();
}


void QImageViewOptions::setImageViewer(QImageViewer * imageViewer)
{
  imageViewer_ = imageViewer;
  updateControls();
}

QImageViewer * QImageViewOptions::imageViewer() const
{
  return imageViewer_;
}

void QImageViewOptions::onupdatecontrols()
{
  if ( !imageViewer_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    penOptions_ctl->setEnableEditMask(imageViewer_->enableEditMask());
    penOptions_ctl->setEditMaskPenRadius(imageViewer_->editMaskPenRadius());
    penOptions_ctl->setEditMaskPenShape(imageViewer_->editMaskPenShape());

    setEnabled(true);
  }
}

void QImageViewOptions::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  if( imageViewer_ ) {
    imageViewer_->setEnableEditMask(false);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageViewOptionsDlgBox::QImageViewOptionsDlgBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Image View Options");

  QVBoxLayout * layout =
      new QVBoxLayout(this);

  layout->addWidget(viewOptions_ctl =
      new QImageViewOptions(this));
}

QImageViewOptions * QImageViewOptionsDlgBox::viewOptions() const
{
  return viewOptions_ctl;
}

void QImageViewOptionsDlgBox::setImageViewer(QImageViewer * imageViewer)
{
  viewOptions_ctl->setImageViewer(imageViewer);
}

QImageViewer * QImageViewOptionsDlgBox::imageViewer() const
{
  return viewOptions_ctl->imageViewer();
}

void QImageViewOptionsDlgBox::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QImageViewOptionsDlgBox::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);
  Q_EMIT visibilityChanged(isVisible());
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

