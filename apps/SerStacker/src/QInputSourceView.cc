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
#include <core/io/image/c_image_input_source.h>
#include <core/debug.h>


#define ICON_eye  ":/serstacker/icons/eye"


static void init_resources()
{
  Q_INIT_RESOURCE(gui_resources);
}

namespace serstacker {

static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}

static inline bool isOpen(const c_input_source::sptr & source)
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
  //mainLayout_->setMargin(0);
  mainLayout_->setSpacing(0);

  mainLayout_->addLayout(toolbarLayout_ = new QHBoxLayout());
  toolbarLayout_->setContentsMargins(0,0,0,0);
  //toolbarLayout_->setMargin(0);
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

QStackedWidget * QInputSourceView::stackWidget() const
{
  return stackWidget_;
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

            if ( currentFrame_ ) {

              const std::set<DataViewType> & viewTypwes =
                  currentFrame_->get_supported_view_types();

              if ( viewTypwes.size() > 1 ) {

                QMenu menu;

                for ( const auto & viewType : viewTypwes ) {

                  const c_enum_member * m = enum_member(viewType);
                  if ( m ) {

                    menu.addAction(createCheckableAction(QIcon(),
                            m->name.c_str(),
                            m->comment.c_str(),
                            viewType == selectedViewType_,
                            [this, viewType]() {
                              setViewType(viewType);
                            }));
                  }
                }

                if ( !menu.isEmpty() ) {
                  menu.exec(tb->mapToGlobal(QPoint(tb->width() - 4, tb->height() - 4)));
                }
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

  connect(this, &ThisClass::displayChannelsChanged,
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
  return currentSource_ ? currentSource_->cfilename() : QString();
}

IMtfDisplay * QInputSourceView::mtfDisplay()
{
  return this;
}

const IMtfDisplay * QInputSourceView::mtfDisplay() const
{
  return this;
}

void QInputSourceView::setCurrentProcessor(const c_data_frame_processor::sptr & processor)
{
  currentProcessor_ = processor;

  if ( currentFrame_ ) {
    currentFrame_->cleanup();
    processCurrentFrame();
    setViewType(selectedViewType_);
  }
}

const c_data_frame_processor::sptr & QInputSourceView::currentProcessor() const
{
  return currentProcessor_;
}

//void QInputSourceView::setInputSource(const c_input_source::sptr & current_source)
//{
//  closeCurrentSource();
//
//  currentSource_ = current_source;
//  startDisplay();
//
//  Q_EMIT currentFileNameChanged();
//}

const c_input_source::sptr & QInputSourceView::inputSource() const
{
  return currentSource_;
}

const c_input_options * QInputSourceView::inputOptions() const
{
  return &input_options_;
}

c_input_options * QInputSourceView::inputOptions()
{
  return &input_options_;
}

bool QInputSourceView::openFile(const QString & abspath)
{
  closeCurrentSource();

  if ( !abspath.isEmpty() ) {

    const std::string filename =
        abspath.toStdString();

    if( !(currentSource_ = c_input_source::create(filename)) ) {
      CF_ERROR("c_input_source::open('%s') fails", filename.c_str());
      return false;
    }

    // CF_DEBUG("typeid(currentSource_)=%s",  typeid(*currentSource_.get()).name());

    currentSource_->set_input_options(&input_options_);

    startDisplay();

    Q_EMIT currentFileNameChanged();
  }

  return true;
}

void QInputSourceView::reloadCurrentFrame()
{
  if( isOpen(currentSource_) ) {
    if( currentSource_->curpos() > 0 ) {
      currentSource_->seek(currentSource_->curpos() - 1);
    }
    loadNextFrame();
  }
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

  if ( !currentSource_ ) {
    return;
  }

  if ( !currentSource_->is_open() && !currentSource_->open() ) {
    CF_ERROR("currentSource_->open() fails");
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

    if( !currentSource_->read(currentFrame_) ) {
      CF_ERROR("currentSource_->read(currentFrame_) fails");
      return;
    }

    processCurrentFrame();

    setViewType(selectedViewType_);

    Q_EMIT currentFrameChanged();
  }

}

void QInputSourceView::processCurrentFrame()
{
  if( !currentFrame_ ) {
    selectedViewType_ = DataViewType_Image;
    viewSelectionToolbutton_ctl->setEnabled(false);
  }
  else {

    if( currentProcessor_ && !currentProcessor_->process(currentFrame_) ) {
      CF_ERROR("currentProcessor_->process(currentFrame_) fails");
    }

    const std::set<DataViewType> & viewTypes =
        currentFrame_->get_supported_view_types();

    if( viewTypes.empty() ) {
      selectedViewType_ = DataViewType_Image;
    }
    else if( viewTypes.find(selectedViewType_) == viewTypes.end() ) {
      selectedViewType_ = *viewTypes.begin();
    }

    viewSelectionToolbutton_ctl->setEnabled(viewTypes.size() > 1);
  }
}


void QInputSourceView::closeCurrentSource()
{
  if ( currentSource_ ) {
    currentSource_->close();
  }

  currentFrame_.reset();
}

void QInputSourceView::setViewType(DataViewType viewType)
{
  if ( viewType != selectedViewType_ ) {
    selectedViewType_ = viewType;
  }

  if( currentFrame_ ) {

    const auto & new_displays =
        currentFrame_->displayChannels();

    auto & existing_displays =
        this->displays_;

    bool haschages = false;

    for( auto ii = existing_displays.begin(); ii != existing_displays.end(); ) {
      if( new_displays.find(ii->first.toStdString()) != new_displays.end() ) {
        ++ii;
      }
      else {
        ii = existing_displays.erase(ii);
        haschages = true;
      }
    }

    for( auto ii = new_displays.begin(); ii != new_displays.end(); ++ii ) {
      if( existing_displays.find(ii->first.c_str()) == existing_displays.end() ) {

        const auto & c =
            ii->second;

        addDisplay(existing_displays, ii->first.c_str(),
            c.minval,
            c.maxval);

        haschages = true;
      }
    }

    if ( haschages ) {
      Q_EMIT displayChannelsChanged();
    }

    displayCurrentFrame();
  }

}

void QInputSourceView::displayCurrentFrame()
{
  // CF_DEBUG("displayCurrentFrame()");

  cv::Mat image, data, mask;

  if( !displays_.empty() && displays_.find(displayChannel_) == displays_.end() ) {
    displayChannel_ = displays_.begin()->first;
  }

  if( currentFrame_ ) {

    switch (selectedViewType_) {

      case DataViewType_Image:

        currentFrame_->get_data(&selectedViewType_,
            displayChannel_.toStdString(),
            image, data, mask);

        setCurrentView(imageView_);
        imageView_->inputImage() = image;
        imageView_->inputMask() = mask;
        imageView_->updateImage();
        break;

      case DataViewType_PointCloud:

        currentFrame_->get_data(&selectedViewType_,
            displayChannel_.toStdString(),
            image, data, mask);

        setCurrentView(cloudView_);
        cloudView_->setPoints(image, data, mask, false);
        break;

      case DataViewType_TextFile:
        setCurrentView(textView_);
        textView_->showTextFile(currentFrame_->get_filename());
        break;

      default:
        break;
    }
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

//QStringList QInputSourceView::displayChannels() const
//{
//  QStringList sl;
//
//  for ( const auto & p : displays_ ) {
//    sl.append(p.first);
//  }
//
//  return sl;
//
//  //return displayChannels_.data();
//}

void QInputSourceView::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;

  switch (selectedViewType_) {
    case DataViewType_Image:
      getminmax(imageView_->currentImage(), minval, maxval, imageView_->currentMask());
      break;
    case DataViewType_PointCloud:
      getminmax(cloudView_->currentColors(), minval, maxval, cloudView_->currentMask());
      break;
    default:
      break;
  }
}


void QInputSourceView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  switch (selectedViewType_) {
    case DataViewType_Image:

      create_histogram(imageView_->currentImage(),
          imageView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);


      break;

    case DataViewType_PointCloud:

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
    case DataViewType_Image:

      create_histogram(imageView_->mtfImage(),
          imageView_->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);

      break;

    case DataViewType_PointCloud:

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
  if ( currentPoints.empty() ) {

    displayPoints.release();
    mtfColors.release();
    displayColors.release();

    Q_EMIT displayImageChanged();

    return;
  }

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

  if ( !mtfColors.fixedType() || mtfColors.type() == mtfcolors.type() ) {
    mtfcolors.copyTo(mtfColors);
  }
  else {
    mtfcolors.convertTo(mtfColors, mtfColors.type());
  }

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

  else {

    if( mtfcolors.channels() == 1 ) {
      cv::cvtColor(mtfcolors, mtfcolors,
          cv::COLOR_GRAY2BGR);
    }

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



void QInputSourceView::setDebayerAlgorithm(DEBAYER_ALGORITHM algo)
{
  debayerAlgorithm_ = algo;

  if( isOpen(currentSource_) && dynamic_cast<c_image_input_source*>(currentSource_.get()) ) {

    if( currentSource_->curpos() > 0 ) {
      currentSource_->seek(currentSource_->curpos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT debayerAlgorithmChanged();
}

DEBAYER_ALGORITHM QInputSourceView::debayerAlgorithm() const
{
  return debayerAlgorithm_;
}

void QInputSourceView::setDropBadPixels(bool v)
{
  filterBadPixels_ = v;

  if( isOpen(currentSource_) && dynamic_cast<c_image_input_source*>(currentSource_.get()) ) {

    if( currentSource_->curpos() > 0 ) {
      currentSource_->seek(currentSource_->curpos() - 1);
    }

    loadNextFrame();
  }


  Q_EMIT dropBadPixelsChanged();
}

bool QInputSourceView::dropBadPixels() const
{
  return filterBadPixels_;
}

void QInputSourceView::setBadPixelsVariationThreshold(double v)
{
  badPixelsVariationThreshold_ = v;

  if( isOpen(currentSource_) && dynamic_cast<c_image_input_source*>(currentSource_.get()) ) {

    if( currentSource_->curpos() > 0 ) {
      currentSource_->seek(currentSource_->curpos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT badPixelsVariationThresholdChanged();
}

double QInputSourceView::badPixelsVariationThreshold() const
{
  return badPixelsVariationThreshold_;
}



///////////////
} // namespace serstacker
