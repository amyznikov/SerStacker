/*
 * QCloudDatasetView.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "QCloudSequenceView.h"
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>

#define ICON_eye  ":/cloudview/icons/eye"


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

static inline bool isOpen(const c_cloudview_input_source::sptr & source)
{
  return source && source->is_open();
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

  connect(playControls_, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  connect(stackWidget_, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  setupMainToolbar();
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

void QCloudSequenceView::setupMainToolbar()
{
  mainToolbar_->addWidget(dataSelectionToolbutton_ctl =
      createToolButton(getIcon(ICON_eye), "",
          "Select View",
          [this](QToolButton * tb) {

          if ( dataItems_.empty() ) {
            return;
          }

          QMenu menu;

          for ( const auto & item : dataItems_ ) {

            const int itemId =
                item.dataid();

            menu.addAction(createCheckableAction(
                QIcon(),
                item.cname(),
                item.ctooltip(),
                itemId == curretDataItemId_,
                [this, itemId]() {
                  showFrameData(itemId);
                }));
          }

          if ( !menu.isEmpty() ) {
            menu.exec(tb->mapToGlobal(QPoint(tb->width()-4, tb->height()-4)));
          }

      }));

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

  currentSource_ = current_source;

  startDisplay();
}

const c_cloudview_input_source::sptr & QCloudSequenceView::inputSource() const
{
  return currentSource_;
}

bool QCloudSequenceView::openFile(const QString & abspath)
{
  closeCurrentSource();

  if ( !abspath.isEmpty() ) {

    const std::string filename =
        abspath.toStdString();

    if( !(currentSource_ = c_cloudview_input_source::load(filename)) ) {
      CF_ERROR("c_cloudview_input_source::load('%s') fails", filename.c_str());
      return false;
    }

    startDisplay();
  }

  return true;
}

void QCloudSequenceView::onSeek(int pos)
{
  if( isOpen(currentSource_) ) {
    if( pos != currentSource_->curpos() ) {
      currentSource_->seek(pos);
    }
    loadNextFrame();
  }
}

void QCloudSequenceView::startDisplay()
{
  imageView_->setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  playControls_->setState(QPlaySequenceControl::Stopped);
  playControls_->hide();

  if ( !isOpen(currentSource_) ) {
    CF_ERROR("currentSource_ is not open");
    return;
  }

  const int num_frames =
      currentSource_->size();

  if ( num_frames < 1 ) {
    QMessageBox::critical(this, "ERROR",
        "Can not determine number of frames and seek range for given source.\n"
        "Image can not be displayed correctly, seems such input source is not supported");
    return;
  }

  if( num_frames > 1 ) {
    playControls_->show();
    playControls_->setSeekRange(0, num_frames - 1);
    playControls_->setCurpos(0);
  }

  loadNextFrame();
}

void QCloudSequenceView::loadNextFrame()
{
  if( isOpen(currentSource_) ) {

    QWaitCursor wait(this, currentSource_->size() == 1);

    if( !(currentFrame_ = currentSource_->read()) ) {
      CF_ERROR("currentSource_->read(currentFrame_) fails");
      return;
    }

    if( currentProcessor_ && !currentProcessor_->process(currentFrame_) ) {
      CF_ERROR("currentProcessor_->process(currentFrame_) fails");
    }

    dataItems_ =
        currentFrame_->items();

    showFrameData(curretDataItemId_);
  }

}


void QCloudSequenceView::closeCurrentSource()
{
  currentFrame_.reset();

  if ( currentSource_ ) {
    currentSource_->close();
  }
}


void QCloudSequenceView::showFrameData(int data_id)
{
  if( currentFrame_ ) {

    const c_cloudview_data_item * displayItem =
        currentFrame_->item(curretDataItemId_ = data_id);

    if( !displayItem && !(displayItem = currentFrame_->item(curretDataItemId_ = -1)) ) {
      CF_ERROR("currentFrame_->item(curretDataItemId_=%d) returned NULL pointer",
          curretDataItemId_);
    }
    else {

      switch (displayItem->type()) {
        case c_cloudview_data_item::image: {

          setCurrentView(imageView_);

          if( !currentFrame_->get_image(curretDataItemId_, imageView_->inputImage(), imageView_->inputMask()) ) {
            imageView_->inputImage().release();
            imageView_->inputMask().release();
          }

          imageView_->updateImage();

          break;
        }

        case c_cloudview_data_item::text: {
          setCurrentView(textView_);
          textView_->showTextFile(currentFrame_->get_filename());
          break;
        }

        case c_cloudview_data_item::point_cloud_3d: {

          cv::Mat3f points;
          cv::Mat colors;

          setCurrentView(cloudView_);
          cloudView_->clear();
          currentFrame_->get_point_cloud(curretDataItemId_, points, colors);
          cloudView_->add(QPointCloud::create(points, colors, false));

          break;
        }

        case c_cloudview_data_item::structured_point_cloud_3d: {
          CF_DEBUG("structured_cloud3d");
          break;
        }

      }
    }
  }
}


} // namespace cloudview
