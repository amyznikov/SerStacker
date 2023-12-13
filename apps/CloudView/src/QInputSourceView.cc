/*
 * QCloudDatasetView.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "QInputSourceView.h"

#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>
#include <core/proc/minmax.h>


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

QInputSourceView::QInputSourceView(QWidget * parent) :
    Base(parent),
    IMtfDisplay("QInputSourceView")
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
  stackWidget_->addWidget(imageView_ = new QImageSourceView(this));
  stackWidget_->addWidget(cloudView_ = new QPointCloudSourceView(this));
  stackWidget_->addWidget(textView_ = new QTextSourceView(this));

  mainLayout_->addWidget(playControls_ = new QPlaySequenceControl(this), 0, Qt::AlignBottom);


  displayTypes_.emplace_back(c_enum_member {
      -1, nullptr, nullptr
  });

  QObject::connect(playControls_, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  QObject::connect(stackWidget_, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  //  imageView_->setDisplayFunction(&mtfDisplayFunction_);
  //  cloudView_->setM

  setupMainToolbar();
  setupMtfDisplayFunction();
  setCurrentView(imageView_);
  setCurrentToolbar(imageViewToolbar_);
}

QToolBar * QInputSourceView::toolbar() const
{
  return mainToolbar_;
}

QToolBar * QInputSourceView::imageViewToolbar() const
{
  return imageViewToolbar_;
}

QToolBar * QInputSourceView::cloudViewToolbar() const
{
  return cloudViewToolbar_;
}

QToolBar * QInputSourceView::textViewToolbar() const
{
  return textViewToolbar_;
}

QToolBar * QInputSourceView::rightToolbar() const
{
  return rightToolbar_;
}

QImageSourceView * QInputSourceView::imageView() const
{
  return imageView_;
}

QPointCloudSourceView * QInputSourceView::cloudView() const
{
  return cloudView_;
}

QTextSourceView * QInputSourceView::textView() const
{
  return textView_;
}

void QInputSourceView::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

void QInputSourceView::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  const bool isVisible =
      Base::isVisible();

  if ( !isVisible ) {
    // closeInputSequence();
  }

  Q_EMIT visibilityChanged(isVisible);
}

void QInputSourceView::setCurrentView(QWidget * w)
{
  stackWidget_->setCurrentWidget(w);
}

QWidget * QInputSourceView::currentView() const
{
  return stackWidget_->currentWidget();
}

void QInputSourceView::setupMainToolbar()
{
  mainToolbar_->addWidget(viewSelectionToolbutton_ctl =
      createToolButton(getIcon(ICON_eye), "",
          "Select View",
          [this](QToolButton * tb) {

            if ( !supportedViewTypes_.empty() ) {

              QMenu menu;

              for ( const auto & viewType : supportedViewTypes_ ) {

                const c_enum_member * m = enum_member(viewType);
                if ( m ) {

                  menu.addAction(createCheckableAction(QIcon(),
                      m->name,
                      m->comment,
                      viewType == selectedViewType_,
                      [this, viewType]() {
                        setViewType(viewType);
                      }));
                }
              }

              if ( !menu.isEmpty() ) {
                menu.exec(tb->mapToGlobal(QPoint(tb->width()-4, tb->height()-4)));
              }
            }
          }));

}


void QInputSourceView::setupMtfDisplayFunction()
{
  imageView_->setDisplayFunction(this);
  cloudView_->setDisplayFunction(this);

  connect(imageView_, &QImageViewer::displayImageChanged,
      this, &ThisClass::displayImageChanged);

  connect(this, &ThisClass::displayTypeChanged,
      this, &ThisClass::displayCurrentFrame);

  connect(this, &ThisClass::parameterChanged,
      [this]() {
        if ( imageView_->isVisible() ) {
          imageView_->updateDisplay();
        }
        else if ( cloudView_->isVisible() ) {
          cloudView_->updateDisplayColors();
        }
      });


}

void QInputSourceView::onStackedWidgetCurrentIndexChanged()
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


void QInputSourceView::setCurrentToolbar(QToolBar * toolbar)
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

QToolBar* QInputSourceView::currentToolbar() const
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


QString QInputSourceView::currentFileName() const
{
  return currentSource_ ? currentSource_->filename().c_str() : QString();
}


IMtfDisplay * QInputSourceView::mtfDisplay()
{
  return this;
}

const IMtfDisplay * QInputSourceView::mtfDisplay() const
{
  return this;
}

void QInputSourceView::setCurrentProcessor(const c_cloudview_processor::sptr & processor)
{
  currentProcessor_ = processor;
}

const c_cloudview_processor::sptr & QInputSourceView::currentProcessor() const
{
  return currentProcessor_;
}

void QInputSourceView::setInputSource(const c_cloudview_input_source::sptr & current_source)
{
  closeCurrentSource();

  currentSource_ = current_source;

  startDisplay();
}

const c_cloudview_input_source::sptr & QInputSourceView::inputSource() const
{
  return currentSource_;
}

bool QInputSourceView::openFile(const QString & abspath)
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

void QInputSourceView::onSeek(int pos)
{
  if( isOpen(currentSource_) ) {
    if( pos != currentSource_->curpos() ) {
      currentSource_->seek(pos);
    }
    loadNextFrame();
  }
}

void QInputSourceView::startDisplay()
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

void QInputSourceView::loadNextFrame()
{
  if( isOpen(currentSource_) ) {

    QWaitCursor wait(this, currentSource_->size() == 1);

    if( !(currentFrame_ = currentSource_->read()) ) {
      CF_ERROR("currentSource_->read(currentFrame_) fails");
      return;
    }

    processCurrentFrame();

    setViewType(selectedViewType_);
  }

}

void QInputSourceView::processCurrentFrame()
{
  supportedViewTypes_.clear();

  if( currentFrame_ ) {

    if ( currentProcessor_ && !currentProcessor_->process(currentFrame_) ) {
      CF_ERROR("currentProcessor_->process(currentFrame_) fails");
    }

    currentFrame_->getSupportedViewTypes(&supportedViewTypes_);
  }

  if ( supportedViewTypes_.empty() ) {
    selectedViewType_ = ViewType_Image;
  }
  else if ( supportedViewTypes_.find(selectedViewType_) == supportedViewTypes_.end() ) {
    selectedViewType_ = *supportedViewTypes_.begin();
  }

  viewSelectionToolbutton_ctl->setEnabled(supportedViewTypes_.size()  > 1);
}


void QInputSourceView::closeCurrentSource()
{
  currentFrame_.reset();

  if ( currentSource_ ) {
    currentSource_->close();
  }
}


void QInputSourceView::setViewType(ViewType viewType)
{
  if ( viewType != selectedViewType_ ) {
    selectedViewType_ = viewType;
  }

  if( currentFrame_ ) {

    const std::map<int, DisplayChannel> & displayChannels =
        currentFrame_->getDisplayChannels(selectedViewType_);


    for( auto ii = displayParams_.begin(); ii != displayParams_.end(); ) {
      if( displayChannels.find(ii->first) != displayChannels.end() ) {
        ++ii;
      }
      else {
        ii = displayParams_.erase(ii);
      }
    }

    displayTypes_.clear();

    for( auto ii = displayChannels.begin(); ii != displayChannels.end(); ++ii ) {

      const int displayId =
          ii->first;

      const DisplayChannel & c =
          ii->second;

      addDisplay(displayParams_, displayId,
          c.minval,
          c.maxval);

      displayTypes_.emplace_back(c_enum_member {
          displayId,
          c.name.c_str(),
          c.tooltip.c_str()
      });

    }

    displayTypes_.emplace_back(c_enum_member {
        -1, nullptr, nullptr
    });

    displayCurrentFrame();
  }

}

void QInputSourceView::displayCurrentFrame()
{
  cv::Mat image, data, mask;

  if ( displayTypes_.size() > 1 && !enum_member(displayType_, displayTypes_.data())  ) {
    displayType_ = displayTypes_[0].value;
  }

  if( currentFrame_ ) {

    currentFrame_->getViewData(&selectedViewType_,
        displayType(),
        image, data, mask);
  }

  switch (selectedViewType_) {
    case ViewType_Image:
      setCurrentView(imageView_);
      imageView_->inputImage() = image;
      imageView_->inputMask() = mask;
      imageView_->updateImage();
      break;
    case ViewType_PointCloud:
      setCurrentView(cloudView_);
      cloudView_->setPoints(image, data, mask, false);
      break;
    default:
      break;
  }

}


bool QInputSourceView::applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  if( currentImage.empty() ) {
    return false;
  }

  DisplayParams &opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  adjustMtfRange(mtf, currentImage, currentMask, &a);

  mtf->apply(currentImage, displayImage, ddepth);

  restoreMtfRange(mtf, a);

  if ( currentMask.size() == currentImage.size() ) {
    displayImage.setTo(0, ~currentMask.getMat());
  }

  return true;
}


bool QInputSourceView::applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
    cv::OutputArray colormapImage)
{
  if( displayImage.empty() || displayImage.type() != CV_8UC1 ) {
    return false;
  }

  DisplayParams &opts =
      displayParams();

  if( opts.colormap == COLORMAP_NONE || opts.lut.empty() ) {
    return false;
  }

  cv::applyColorMap(displayImage,
      colormapImage,
      opts.lut);

  if( displayMask.size() == colormapImage.size() ) {
    colormapImage.setTo(0, ~displayMask.getMat());
  }

  return true;
}

const c_enum_member * QInputSourceView::displayTypes() const
{
  return displayTypes_.data();
}

void QInputSourceView::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;

  switch (selectedViewType_) {
    case ViewType_Image:
      getminmax(imageView_->currentImage(), minval, maxval, imageView_->currentMask());
      break;
    case ViewType_PointCloud:
      getminmax(cloudView_->currentColors(), minval, maxval, cloudView_->currentMask());
      break;
    default:
      break;
  }
}


void QInputSourceView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  switch (selectedViewType_) {
    case ViewType_Image:

      create_histogram(imageView_->currentImage(),
          imageView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);


      break;

    case ViewType_PointCloud:

      create_histogram(cloudView_->currentColors(),
          cloudView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);

      break;

    default:
      H.release();
      break;
  }

}


void QInputSourceView::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{

  switch (selectedViewType_) {
    case ViewType_Image:

      create_histogram(imageView_->mtfImage(),
          imageView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);

      break;

    case ViewType_PointCloud:

      create_histogram(cloudView_->mtfColors(),
          cloudView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);

      break;

    default:
      H.release();
      break;
  }

//  if ( imageViewer_ ) {
//
//    cv::Mat image, mask;
//
//    double scale = 1.0;
//    double offset = 0.0;
//
//    const cv::Mat & currentImage =
//        imageViewer_->mtfImage();
//
//    imageViewer_->currentMask().copyTo(mask);
//
//    if ( currentImage.depth() == CV_8U ) {
//      currentImage.copyTo(image);
//    }
//    else if ( currentImage.depth() == CV_32F || currentImage.depth() == CV_64F ) {
//
//      const double dstmin = 0, dstmax = 255;
//      double srcmin = 0, srcmax = 1;
//
//      cv::minMaxLoc(currentImage, &srcmin, &srcmax, nullptr, nullptr, mask);
//
//      scale = (dstmax - dstmin) / (srcmax - srcmin);
//      offset = dstmin - scale * srcmin;
//
//      currentImage.convertTo(image, CV_8U, scale, offset);
//    }
//    else {
//      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
//      currentImage.convertTo(image, CV_8U, scale, offset);
//    }
//
//    create_histogram(image, mask,
//        H,
//        hmin, hmax,
//        256,
//        false,
//        false);
//
//    (*hmin -= offset) /= scale;
//    (*hmax -= offset) /= scale;
//  }
}


void QInputSourceView::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::Mat & mtfImage, cv::Mat & displayImage, int ddepth)
{
  if ( !currentImage.empty() ) {

    const DisplayParams & opts =
        displayParams();

    const bool needColormap =
        opts.colormap != COLORMAP_NONE &&
            currentImage.channels() == 1 &&
            ddepth == CV_8U;

    applyMtf(currentImage, needColormap ? cv::noArray() : currentMask,
        mtfImage, ddepth);

    if ( needColormap && !mtfImage.empty() ) {
      applyColorMap(mtfImage, currentMask, displayImage);
    }
    else {
      displayImage = mtfImage;
    }
  }
}

void QInputSourceView::createDisplayPoints(cv::InputArray currentPoints,
    cv::InputArray currentColors,
    cv::InputArray currentMask,
    cv::OutputArray displayPoints,
    cv::OutputArray mtfColors,
    cv::OutputArray displayColors)
{
  DisplayParams & opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  cv::Mat mtfcolors, displaycolors;

  const bool needColormap =
      opts.colormap != COLORMAP_NONE;// &&
          //currentColors.channels() == 1;


  currentPoints.getMat().convertTo(displayPoints, CV_32F);


  adjustMtfRange(mtf, needColormap ? currentColors : cv::noArray(), currentMask, &a);
  mtf->apply(currentColors, mtfcolors, CV_8U);
  restoreMtfRange(mtf, a);

  mtfcolors.copyTo(mtfColors);

  if ( needColormap ) {

    if( mtfcolors.channels() != 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_RGB2GRAY);
    }

    cv::applyColorMap(mtfcolors,
        displaycolors,
        opts.lut);

    if( currentMask.size() == displaycolors.size() ) {
      displaycolors.setTo(0, ~currentMask.getMat());
    }

    cv::cvtColor(displaycolors, displaycolors, cv::COLOR_BGR2RGB);

    displaycolors.copyTo(displayColors);

  }

  else if( mtfcolors.channels() == 1 ) {

    cv::cvtColor(mtfcolors, displayColors, cv::COLOR_GRAY2RGB);
  }

  else {

    mtfcolors.copyTo(displayColors);
  }


  Q_EMIT displayImageChanged();


  //    if( total_points_to_display > 0 ) {
  //
  //      display_points_.reserve(total_points_to_display);
  //
  //      for( const auto &cloud : clouds_ ) {
  //        if( cloud->visible && cloud->points.rows > 0 ) {
  //
  //          const cv::Mat3f &points =
  //              cloud->points;
  //
  //          const double Sx = cloud->Scale.x();
  //          const double Sy = cloud->Scale.y();
  //          const double Sz = cloud->Scale.z();
  //
  //          const double Tx = cloud->Translation.x();
  //          const double Ty = cloud->Translation.y();
  //          const double Tz = cloud->Translation.z();
  //
  //          const double Rx = cloud->Rotation.x();
  //          const double Ry = cloud->Rotation.y();
  //          const double Rz = cloud->Rotation.z();
  //
  //          for( int i = 0; i < points.rows; ++i ) {
  //
  //            const cv::Vec3f & srcp =
  //                points[i][0];
  //
  //            display_points_.emplace_back(srcp[0] * Sx - Tx - sceneOrigin_.x(),
  //                srcp[1] * Sy - Ty - sceneOrigin_.y(),
  //                srcp[2] * Sz - Tz - sceneOrigin_.z());
  //
  //          }
  //        }
  //      }
  //    }
  //
  //    update_display_colors_ = true;
  //  }
  //
  //  if( update_display_colors_ || display_colors_.size() != display_points_.size() ) {
  //
  //    display_colors_.clear();
  //
  //    QMtfDisplay::DisplayParams & opts =
  //        mtfDisplay_.displayParams();
  //
  //    c_pixinsight_mtf &mtf =
  //        opts.mtf;
  //
  //    double imin, imax;
  //
  //    mtf.get_input_range(&imin, &imax);
  //
  //    if( imin >= imax ) {
  //      mtf.set_input_range(0, 255);
  //    }
  //
  //    for( const auto &cloud : clouds_ ) {
  //      if( cloud->visible && cloud->points.rows > 0 ) {
  //
  //        if ( cloud->colors.rows != cloud->points.rows ) {
  //
  //          int gray = mtf.apply(255);
  //
  //          for( int i = 0, n = cloud->points.rows; i < n; ++i ) {
  //            display_colors_.emplace_back(gray, gray, gray);
  //          }
  //
  //        }
  //        else {
  //
  //          const cv::Mat & colors =
  //              cloud->colors;
  //
  //          const int channels =
  //              colors.channels();
  //
  //
  //          for( int i = 0; i < colors.rows; ++i ) {
  //
  //            const cv::Scalar color =
  //                compute_point_color(colors, i, mtf);
  //
  //            if ( channels == 1) {
  //
  //              const int gray =
  //                  std::max(0, std::min(255,
  //                      (int) (color[0] + pointBrightness_)));
  //
  //              display_colors_.emplace_back(gray, gray, gray);
  //            }
  //            else {
  //
  //              const int red =
  //                  std::max(0, std::min(255,
  //                      (int) (color[2] + pointBrightness_)));
  //
  //              const int green =
  //                  std::max(0, std::min(255,
  //                      (int) (color[1] + pointBrightness_)));
  //
  //              const int blue =
  //                  std::max(0, std::min(255,
  //                      (int) (color[0] + pointBrightness_)));
  //
  //              display_colors_.emplace_back(red, green, blue);
  //            }
  //          }
  //        }
  //      }
  //    }
  //
  //    if( imin >= imax ) {
  //      mtf.set_input_range(imin, imax);
  //    }
  //  }
  //
  //  if ( update_display_colors_ ) {
  //    Q_EMIT mtfDisplay_.displayImageChanged();
  //  }
  //
  //  update_display_points_ = false;
  //  update_display_colors_ = false;

}



///////////////
} // namespace cloudview
