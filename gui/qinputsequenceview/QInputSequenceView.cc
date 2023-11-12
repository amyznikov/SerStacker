/*
 * QInputSequenceView.cc
 *
 *  Created on: Nov 12, 2023
 *      Author: amyznikov
 */

#include "QInputSequenceView.h"
#include <core/debug.h>


static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}

QInputSequenceView::QInputSequenceView(QWidget * parent) :
  Base(parent)
{
  mainLayout_ = new QVBoxLayout(this);
  mainLayout_->setAlignment(Qt::AlignTop);

  toolbarLayout_ = new QHBoxLayout();
  toolbarLayout_->addWidget(toolbar_ = createToolbar(this), 100, Qt::AlignLeft);
  toolbarLayout_->addWidget(imageViewToolbar_ = createToolbar(this), 1, Qt::AlignRight);
  toolbarLayout_->addWidget(cloudViewToolbar_ = createToolbar(this), 1, Qt::AlignRight);

  mainLayout_->addLayout(toolbarLayout_);
  mainLayout_->addWidget(stackWidget_ = new QStackedWidget(this));
  stackWidget_->addWidget(imageView_ = new QImageEditor(this));
  stackWidget_->addWidget(cloudView_ = new QCloudViewer(this));

  mainLayout_->addWidget(playControls_ = new QPlaySequenceControl(this));

  input_sequence_ = c_input_sequence::create();
  input_sequence_->set_auto_apply_color_matrix(true);

  connect(playControls_, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  connect(stackWidget_, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  setCurrentStackedWidget(imageView_);
}

QToolBar * QInputSequenceView::toolbar() const
{
  return toolbar_;
}

QToolBar * QInputSequenceView::imageViewToolbar() const
{
  return imageViewToolbar_;
}

QToolBar * QInputSequenceView::cloudViewToolbar() const
{
  return cloudViewToolbar_;
}

void QInputSequenceView::setCurrentToolbar(QToolBar * toolbar)
{
  if( toolbar == imageViewToolbar_ ) {
    cloudViewToolbar_->hide();
    imageViewToolbar_->show();
  }
  else if( toolbar == cloudViewToolbar_ ) {
    imageViewToolbar_->hide();
    cloudViewToolbar_->show();
  }
  else {
    imageViewToolbar_->hide();
    cloudViewToolbar_->hide();
  }
  toolbarLayout_->update();
}

void QInputSequenceView::setCurrentStackedWidget(QWidget * w)
{
  stackWidget_->setCurrentWidget(w);
}

QWidget * QInputSequenceView::currentStackedWidget() const
{
  return stackWidget_->currentWidget();
}

void QInputSequenceView::onStackedWidgetCurrentIndexChanged()
{
  const QWidget *currentWidget =
      currentStackedWidget();

  if( currentWidget == imageView_ ) {
    setCurrentToolbar(imageViewToolbar_);
  }
  else if( currentWidget == cloudView_ ) {
    setCurrentToolbar(cloudViewToolbar_);
  }
}


void QInputSequenceView::onSeek(int pos)
{
}
