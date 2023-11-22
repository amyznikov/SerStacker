/*
 * QCloudDatasetView.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "QCloudSequenceView.h"


static void init_resources()
{
  Q_INIT_RESOURCE(gui_resources);
}

namespace cloudview {

static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}


QCloudSequenceView::QCloudSequenceView(QWidget * parent) :
    ThisClass(parent, nullptr, nullptr, nullptr)
{
}

QCloudSequenceView::QCloudSequenceView(QWidget * parent,
    QImageEditor * imageView,
    QCloudViewer * cloudView,
    QTextFileViewer * textView) :
    Base(parent),
    imageView_(imageView),
    cloudView_(cloudView),
    textView_(textView)
{
  init_resources();

  mainLayout_ = new QVBoxLayout(this);
  mainLayout_->setAlignment(Qt::AlignTop);
  mainLayout_->setContentsMargins(0,0,0,0);
  mainLayout_->setMargin(0);
  mainLayout_->setSpacing(0);

  mainLayout_->addLayout(toolbarLayout_ = new QHBoxLayout());
  toolbarLayout_->setContentsMargins(0,0,0,0);
  toolbarLayout_->setMargin(0);
  toolbarLayout_->addWidget(mainToolbar_ = createToolbar(this), 10, Qt::AlignLeft);
  toolbarLayout_->addWidget(imageViewToolbar_ = createToolbar(this), 10, Qt::AlignRight);
  toolbarLayout_->addWidget(cloudViewToolbar_ = createToolbar(this), 10, Qt::AlignRight);
  toolbarLayout_->addWidget(textViewToolbar_ = createToolbar(this), 10, Qt::AlignRight);
  toolbarLayout_->addWidget(rightToolbar_ = createToolbar(this), 0, Qt::AlignRight);


  mainLayout_->addWidget(stackWidget_ = new QStackedWidget(this), 10000);
  stackWidget_->addWidget(imageView_ ? imageView_ : imageView_ = new QImageEditor(this));
  stackWidget_->addWidget(cloudView_ ? cloudView_ : cloudView_ = new QCloudViewer(this));
  stackWidget_->addWidget(textView_ ? textView_ : textView_ = new QTextFileViewer(this));

  mainLayout_->addWidget(playControls_ = new QPlaySequenceControl(this), 0, Qt::AlignBottom);

//  currentSequence_ = c_input_sequence::create();
//  currentSequence_->set_auto_apply_color_matrix(true);

  connect(playControls_, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  connect(stackWidget_, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  setCurrentView(imageView_);
  setCurrentToolbar(imageViewToolbar_);
}

QToolBar * QCloudSequenceView::toolbar() const
{
  return mainToolbar_;
}

QToolBar * QCloudSequenceView::imageViewToolbar() const
{
  return imageViewToolbar_;
}

QToolBar * QCloudSequenceView::cloudViewToolbar() const
{
  return cloudViewToolbar_;
}

QToolBar * QCloudSequenceView::textViewToolbar() const
{
  return textViewToolbar_;
}

QToolBar * QCloudSequenceView::rightToolbar() const
{
  return rightToolbar_;
}

QImageEditor * QCloudSequenceView::imageView() const
{
  return imageView_;
}

QCloudViewer * QCloudSequenceView::cloudView() const
{
  return cloudView_;
}

QTextFileViewer * QCloudSequenceView::textView() const
{
  return textView_;
}

void QCloudSequenceView::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QCloudSequenceView::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  const bool isVisible =
      Base::isVisible();

  if ( !isVisible ) {
    // closeInputSequence();
  }

  Q_EMIT visibilityChanged(isVisible);
}

void QCloudSequenceView::setCurrentView(QWidget * w)
{
  stackWidget_->setCurrentWidget(w);
}

QWidget * QCloudSequenceView::currentView() const
{
  return stackWidget_->currentWidget();
}

void QCloudSequenceView::onStackedWidgetCurrentIndexChanged()
{
  const QWidget *currentWidget =
      currentView();

  if( currentWidget == imageView_ ) {
    setCurrentToolbar(imageViewToolbar_);
  }
  else if( currentWidget == cloudView_ ) {
    setCurrentToolbar(cloudViewToolbar_);
  }
  else if( currentWidget == textView_ ) {
    setCurrentToolbar(textViewToolbar_);
  }

  Q_EMIT currentViewChanged();
}


void QCloudSequenceView::setCurrentToolbar(QToolBar * toolbar)
{
  if( toolbar == imageViewToolbar_ ) {
    cloudViewToolbar_->hide();
    textViewToolbar_->hide();
    imageViewToolbar_->show();
  }
  else if( toolbar == cloudViewToolbar_ ) {
    imageViewToolbar_->hide();
    textViewToolbar_->hide();
    cloudViewToolbar_->show();
  }
  else if( toolbar == textViewToolbar_ ) {
    imageViewToolbar_->hide();
    cloudViewToolbar_->hide();
    textViewToolbar_->show();
  }
  else {
    imageViewToolbar_->hide();
    cloudViewToolbar_->hide();
    textViewToolbar_->hide();
  }

  toolbarLayout_->update();
}

QToolBar* QCloudSequenceView::currentToolbar() const
{
  if( cloudViewToolbar_->isVisible() ) {
    return cloudViewToolbar_;
  }

  if( imageViewToolbar_->isVisible() ) {
    return imageViewToolbar_;
  }

  if( textViewToolbar_->isVisible() ) {
    return textViewToolbar_;
  }

  return nullptr;
}


void QCloudSequenceView::setInputSource(const c_cloudview_input_source::sptr & current_source)
{
  closeCurrentSource();

  current_source_ = current_source;

  startDisplay();
}

const c_cloudview_input_source::sptr & QCloudSequenceView::inputSource() const
{
  return current_source_;
}


void QCloudSequenceView::closeCurrentSource()
{
  if ( current_source_ ) {
    current_source_->close();
  }
}

void QCloudSequenceView::startDisplay()
{

}

void QCloudSequenceView::loadNextFrame()
{

}

void QCloudSequenceView::onSeek(int pos)
{

}


bool QCloudSequenceView::openFile(const QString & abspath)
{
//  closeCurrentSource();
//
//  c_cloudview_input_source::sptr source =
//      c_cloudview_input_source::load(abspath.toStdString());
//
  return false;
}


} // namespace cloudview
