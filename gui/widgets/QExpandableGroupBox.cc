/*
 * QExpandableGroupBox.cc
 *
 *  Created on: Mar 29, 2022
 *      Author: amyznikov
 */

#include "QExpandableGroupBox.h"
#include <gui/widgets/style.h>
#include <core/debug.h>

#define __STYLE_TEXT(x) #x

static constexpr char chkbox_style[] =
  __STYLE_TEXT(

      QCheckBox {
        spacing: 15px;
      }
      QCheckBox::indicator {
        width: 13px;
        height: 13px;
      }
      QCheckBox::indicator:unchecked {
        image: url(:/gui/icons/${style}/double-arrow-right.png);
      }
      QCheckBox::indicator:checked {
        image: url(:/gui/icons/${style}/double-arrow-up.png);
      }
  );

#undef __STYLE_TEXT

QExpandableGroupBox::QExpandableGroupBox(const QString & title, QWidget * view, int stretch, Qt::Alignment alignment, QWidget * parent) :
    Base(parent),
    _view(view)
{
  Q_INIT_RESOURCE(gui_resources);

  _layout = new QVBoxLayout(this);


  _chkBox = new QCheckBox(title, this);
  _chkBox->setStyleSheet(QString(chkbox_style).replace("${style}", iconStyleSelector()));
  _layout->addWidget(_chkBox, 0, Qt::AlignTop);

  _frame = new QGroupBox(this);
  _frame->setCheckable(false);

  _frameLayout = new QVBoxLayout(_frame);

  if ( _view ) {
    _frameLayout->addWidget(_view);
  }

  _frame->setVisible(false);

  QObject::connect(_chkBox, &QCheckBox::stateChanged,
      [this, stretch, alignment](int state) {

        if ( state == Qt::Checked ) {
          _layout->addWidget(_frame, stretch, alignment);
          _frame->setVisible(true);
        }
        else {
          _frame->setVisible(false);
          _layout->removeWidget(_frame);
        }

        this->parentWidget()->updateGeometry();

        if ( _frame->isVisible() ) {
          Q_EMIT expanded();
        }
        else {
          Q_EMIT collapsed();
        }
      });

}

void QExpandableGroupBox::setView(QWidget * view)
{
  if ( this->_view ) {
    _frameLayout->removeWidget(this->_view);
  }

  if ( (this->_view = view) ) {
    _frameLayout->addWidget(this->_view);
  }

  updateGeometry();
}

QWidget * QExpandableGroupBox::view() const
{
  return _view;
}

QCheckBox * QExpandableGroupBox::checkbox() const
{
  return _chkBox;
}

QVBoxLayout * QExpandableGroupBox::boxlayout() const
{
  return _layout;
}

void QExpandableGroupBox::expand()
{
  _chkBox->setChecked(true);
}

void QExpandableGroupBox::collapse()
{
  _chkBox->setChecked(false);
}

void QExpandableGroupBox::toggle()
{
  _chkBox->toggle();
}

