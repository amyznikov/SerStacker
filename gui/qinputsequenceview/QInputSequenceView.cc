/*
 * QInputSequenceView.cc
 *
 *  Created on: Nov 12, 2023
 *      Author: amyznikov
 */

#include "QInputSequenceView.h"
#include <gui/widgets/createAction.h>
#include <gui/qthumbnailsview/QThumbnails.h>
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/bad_pixels.h>
#include <core/debug.h>


static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  // toolbar->setContentsMargins(0,0,0,0);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}

QInputSequenceView::QInputSequenceView(QWidget * parent) :
    ThisClass(parent, nullptr, nullptr, nullptr)
{
}


QInputSequenceView::QInputSequenceView(QWidget * parent, QImageEditor * imageView, QCloudViewer * cloudView, QTextFileViewer * textView) :
  Base(parent),
  imageView_(imageView),
  cloudView_(cloudView),
  textView_(textView)
{
  Q_INIT_RESOURCE(gui_resources);

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

  currentSequence_ = c_input_sequence::create();
  currentSequence_->set_auto_apply_color_matrix(true);

  connect(playControls_, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  connect(stackWidget_, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  setCurrentView(imageView_);
  setCurrentToolbar(imageViewToolbar_);
}

QToolBar * QInputSequenceView::toolbar() const
{
  return mainToolbar_;
}

QToolBar * QInputSequenceView::imageViewToolbar() const
{
  return imageViewToolbar_;
}

QToolBar * QInputSequenceView::cloudViewToolbar() const
{
  return cloudViewToolbar_;
}

QToolBar * QInputSequenceView::textViewToolbar() const
{
  return textViewToolbar_;
}

QToolBar * QInputSequenceView::rightToolbar() const
{
  return rightToolbar_;
}

QImageEditor * QInputSequenceView::imageView() const
{
  return imageView_;
}

QCloudViewer * QInputSequenceView::cloudView() const
{
  return cloudView_;
}

QTextFileViewer * QInputSequenceView::textView() const
{
  return textView_;
}

void QInputSequenceView::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QInputSequenceView::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  const bool isVisible =
      Base::isVisible();

  if ( !isVisible ) {
    closeInputSequence();
  }

  Q_EMIT visibilityChanged(isVisible);
}


const QString & QInputSequenceView::currentFileName() const
{
  return currentFileName_;
}

void QInputSequenceView::setCurrentFileName(const QString & filename)
{
  currentFileName_ = filename;
  Q_EMIT currentFileNameChanged();
}

const c_input_sequence::sptr & QInputSequenceView::currentSequence() const
{
  return currentSequence_;
}

void QInputSequenceView::setDebayerAlgorithm(DEBAYER_ALGORITHM algo)
{
  debayerAlgorithm_ = algo;
  Q_EMIT debayerAlgorithmChanged();
}

DEBAYER_ALGORITHM QInputSequenceView::debayerAlgorithm() const
{
  return debayerAlgorithm_;
}

void QInputSequenceView::setDropBadPixels(bool v)
{
  filterBadPixels_ = v;
  Q_EMIT dropBadPixelsChanged();
}

bool QInputSequenceView::dropBadPixels() const
{
  return filterBadPixels_;
}

void QInputSequenceView::setBadPixelsVariationThreshold(double v)
{
  badPixelsVariationThreshold_ = v;
  Q_EMIT badPixelsVariationThresholdChanged();
}

double QInputSequenceView::badPixelsVariationThreshold() const
{
  return badPixelsVariationThreshold_;
}

void QInputSequenceView::setSourceOutputType(c_input_source::OUTPUT_TYPE v)
{
  sourceOutputType_ = v;

  if( currentSequence_ && currentSequence_->is_open() ) {

    if( currentSequence_->current_pos() > 0 ) {
      currentSequence_->seek(currentSequence_->current_pos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT sourceOutputTypeChanged();
}

c_input_source::OUTPUT_TYPE QInputSequenceView::sourceOutputType() const
{
  return sourceOutputType_;
}

#if HAVE_VLO_FILE
void QInputSequenceView::setVloDataChannel(c_vlo_file::DATA_CHANNEL channel)
{
  vlo_data_channel_ = channel;

  if( currentSequence_ && currentSequence_->is_open() ) {

    if( currentSequence_->current_pos() > 0 ) {
      currentSequence_->seek(currentSequence_->current_pos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT vloDataChannelChanged();
}

c_vlo_file::DATA_CHANNEL QInputSequenceView::vloDataChannel() const
{
  return vlo_data_channel_;
}
#endif



void QInputSequenceView::setCurrentToolbar(QToolBar * toolbar)
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

void QInputSequenceView::showImageView()
{
  stackWidget_->setCurrentWidget(imageView_);
}

void QInputSequenceView::setCurrentView(QWidget * w)
{
  stackWidget_->setCurrentWidget(w);
}

QWidget * QInputSequenceView::currentView() const
{
  return stackWidget_->currentWidget();
}

void QInputSequenceView::onStackedWidgetCurrentIndexChanged()
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


bool QInputSequenceView::openFile(const std::string & filename)
{
  return openFile(QString::fromStdString(filename));
}

bool QInputSequenceView::openFile(const QString & filename)
{
  closeInputSequence();

  setCurrentFileName(filename);

  const QString suffix =
      QFileInfo(filename).suffix();

  if ( isTextFileSuffix(suffix) ) {
    setCurrentView(textView_);
    textView_->showTextFile(filename);
    return true;
  }

  if( isPlyFileSuffix(suffix) ) {
    if( cloudView_->openPlyFile(filename) ) {
      setCurrentView(cloudView_);
    }
    else {
      setCurrentView(textView_);
      textView_->showTextFile(filename);
    }
    return true;
  }

  setCurrentView(imageView_);

  currentSequence_->add_source(filename.toStdString());
  startDisplay();

  return true;
}

void QInputSequenceView::closeInputSequence()
{
  if( playControls_->state() != QPlaySequenceControl::Stopped ) {
    playControls_->setState(QPlaySequenceControl::Stopped);
  }

  playControls_->hide();

  if( currentSequence_->is_open() ) {
    currentSequence_->close(true);
  }

  imageView_->clear();
  cloudView_->clear();
  textView_->clear();
}

void QInputSequenceView::startDisplay()
{
  QWaitCursor wait(this);

  imageView_->setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  playControls_->setState(QPlaySequenceControl::Stopped);
  playControls_->hide();

  if ( !currentSequence_->is_open() && !currentSequence_->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return;
  }

  const int num_frames =
      currentSequence_->size();

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

void QInputSequenceView::loadNextFrame()
{
  if ( currentSequence_ && currentSequence_->is_open() ) {

    c_input_source::sptr current_source =
        currentSequence_->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

#if HAVE_VLO_FILE
      if( c_vlo_input_source *vlo = dynamic_cast<c_vlo_input_source*>(current_source.get()) ) {

        vlo->set_read_channel(vlo_data_channel_);

        if ( sourceOutputType_ == c_input_source::OUTPUT_TYPE_CLOUD3D  ) {

          cv::Mat3f points;
          cv::Mat colors;

          cloudView_->clear();

          if ( !vlo->read_cloud3d(points, colors) ) {
            CF_ERROR("vlo->read_cloud3d() fails");
          }
          else {
            cloudView_->add(QPointCloud::create(points, colors, false));
          }

          currentSequence_->update_current_pos();

          if( currentView() != cloudView_ ) {
            setCurrentView(cloudView_);
          }

          cloudView_->redraw();

          return;
        }
      }
#endif


      if( currentView() != imageView_ ) {
        setCurrentView(imageView_);
      }

      cv::Mat & inputImage =
          imageView_->inputImage();

      cv::Mat & inputMask =
          imageView_->inputMask();

      if ( !currentSequence_->read(inputImage, &inputMask) ) {
        CF_ERROR("currentSequence_->read() fails");
        inputImage.release();
        inputMask.release();
      }
      else {

        //      CF_DEBUG("inputImage_: %dx%d channels=%d depth=%d inputMask_: %dx%d channels=%d depth=%d",
        //          inputImage.cols, inputImage.rows, inputImage.channels(), inputImage.depth(),
        //          inputMask.cols, inputMask.rows, inputMask.channels(), inputMask.depth());
        //      Q_EMIT onInputImageLoad(inputImage_, inputMask_,
        //          input_sequence_->colorid(),
        //          input_sequence_->bpp());


        if ( filterBadPixels_ && badPixelsVariationThreshold_ > 0 ) {

          if( !is_bayer_pattern(currentSequence_->colorid()) ) {
            median_filter_hot_pixels(inputImage, badPixelsVariationThreshold_, false);
          }
          else if( !extract_bayer_planes(inputImage, inputImage, currentSequence_->colorid()) ) {
            CF_ERROR("ERROR: extract_bayer_planes() fails");
          }
          else {
            median_filter_hot_pixels(inputImage, badPixelsVariationThreshold_, true);
            if( !nninterpolation(inputImage, inputImage, currentSequence_->colorid()) ) {
              CF_ERROR("nninterpolation() fails");
            }
          }
        }
        else if( is_bayer_pattern(currentSequence_->colorid()) ) {
          debayer(inputImage, inputImage, currentSequence_->colorid(),
              debayerAlgorithm_);
        }
      }

      setCurrentFileName(current_source->filename().c_str());
      imageView_->updateImage();
    }
  }

}

void QInputSequenceView::onSeek(int pos)
{
  if ( currentSequence_ && currentSequence_->is_open() ) {
    if ( pos != currentSequence_->current_pos() ) {
      currentSequence_->seek(pos);
    }
    loadNextFrame();
  }
}

