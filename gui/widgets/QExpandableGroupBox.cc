/*
 * QExpandableGroupBox.cc
 *
 *  Created on: Mar 29, 2022
 *      Author: amyznikov
 */

#include "QExpandableGroupBox.h"

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
        image: url(:/gui/icons/double-arrow-right.png);
      }
      QCheckBox::indicator:checked {
        image: url(:/gui/icons/double-arrow-up.png);
      }
  );

#undef __STYLE_TEXT

QExpandableGroupBox::QExpandableGroupBox(const QString & title, QWidget * view, QWidget * parent) :
    Base(parent)
{
  Q_INIT_RESOURCE(gui_resources);

  layout_ =  new QFormLayout(this);

  chkBox_ = new QCheckBox(title);
  chkBox_->setStyleSheet(chkbox_style);
  layout_->addRow(chkBox_);

  gBox_ = new QGroupBox();
  layout_->addRow(gBox_);

  if( view ) {
    (new QVBoxLayout(gBox_))->addWidget(view);
    gBox_->setVisible(chkBox_->isChecked());
  }

  QObject::connect(chkBox_, &QCheckBox::stateChanged,
      [this](int state) {
        gBox_->setVisible(state == Qt::Checked);
      });
}

QWidget * QExpandableGroupBox::view() const
{
  return view_;
}

//void QExpandableGroupBox::expand()
//{
//
//}
//
//void QExpandableGroupBox::collapse()
//{
//
//}
//
//void QExpandableGroupBox::toggle()
//{
//
//}

