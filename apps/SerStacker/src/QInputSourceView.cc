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
#include <core/proc/histogram.h>
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

  _mainLayout = new QVBoxLayout(this);
  _mainLayout->setAlignment(Qt::AlignTop);
  setContentsMargins(0, 0, 0, 0);
  _mainLayout->setSpacing(0);

  _mainLayout->addLayout(_toolbarLayout = new QHBoxLayout());
  _toolbarLayout->setContentsMargins(0,0,0,0);
  _toolbarLayout->addWidget(_mainToolbar = createToolbar(this), 10, Qt::AlignLeft);
  _toolbarLayout->addWidget(_imageViewToolbar = createToolbar(this), 10, Qt::AlignRight);
  _toolbarLayout->addWidget(_cloudViewToolbar = createToolbar(this), 10, Qt::AlignRight);
  _toolbarLayout->addWidget(_textViewToolbar = createToolbar(this), 10, Qt::AlignRight);
  _toolbarLayout->addWidget(_rightToolbar = createToolbar(this), 0, Qt::AlignRight);


  _mainLayout->addWidget(_stackWidget = new QStackedWidget(this), 10000);
  _stackWidget->addWidget(_imageView = new QImageSourceView(this));
  _stackWidget->addWidget(_cloudView = new QPointCloudSourceView(this));
  _stackWidget->addWidget(_textView = new QTextSourceView(this));

  _mainLayout->addWidget(_playControls = new QPlaySequenceControl(this), 0, Qt::AlignBottom);

  QObject::connect(_imageView, &QImageSourceView::onPopulateContextMenu,
      this, &ThisClass::populateImageViewContextMenu);

  QObject::connect(_cloudView, &QPointCloudSourceView::glPointSelectionMouseEvent,
       this, &ThisClass::onCloudViewPointSelectionMouseEvent);

  QObject::connect(_playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  QObject::connect(_stackWidget, &QStackedWidget::currentChanged,
      this, &ThisClass::onStackedWidgetCurrentIndexChanged);

  setupMainToolbar();
  setupMtfDisplayFunction();
  setCurrentView(_imageView);
  setCurrentToolbar(_imageViewToolbar);


}

QToolBar * QInputSourceView::toolbar() const
{
  return _mainToolbar;
}

QToolBar * QInputSourceView::imageViewToolbar() const
{
  return _imageViewToolbar;
}

QToolBar * QInputSourceView::cloudViewToolbar() const
{
  return _cloudViewToolbar;
}

QToolBar * QInputSourceView::textViewToolbar() const
{
  return _textViewToolbar;
}

QToolBar * QInputSourceView::rightToolbar() const
{
  return _rightToolbar;
}

QStackedWidget * QInputSourceView::stackWidget() const
{
  return _stackWidget;
}

QImageSourceView * QInputSourceView::imageView() const
{
  return _imageView;
}

QPointCloudSourceView * QInputSourceView::cloudView() const
{
  return _cloudView;
}

QTextSourceView * QInputSourceView::textView() const
{
  return _textView;
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
  _stackWidget->setCurrentWidget(w);
}

QWidget * QInputSourceView::currentView() const
{
  return _stackWidget->currentWidget();
}

void QInputSourceView::setupMainToolbar()
{
  _mainToolbar->addWidget(_viewSelectionToolbutton_ctl =
      createToolButton(getIcon(ICON_eye), "",
          "Select View",
          [this](QToolButton * tb) {

            if ( _currentFrame ) {

              const auto & viewTypwes =
                  _currentFrame->get_available_display_types();

              if ( viewTypwes.size() > 1 ) {

                QMenu menu;

                for ( const auto & viewType : viewTypwes ) {

                  const c_enum_member * m = enum_member(viewType);
                  if ( m ) {

                    menu.addAction(createCheckableAction(QIcon(),
                            m->name.c_str(),
                            m->comment.c_str(),
                            viewType == _selectedViewType,
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
  _imageView->setDisplayFunction(this);
  _cloudView->setDisplayFunction(this);

  connect(_imageView, &QImageViewer::displayImageChanged,
      this, &ThisClass::displayImageChanged);

  connect(this, &ThisClass::displayChannelsChanged,
      this, &ThisClass::displayCurrentFrame);

  connect(this, &ThisClass::parameterChanged,
      [this]() {
        if ( _imageView->isVisible() ) {
          _imageView->updateDisplay();
        }
        else if ( _cloudView->isVisible() ) {
          _cloudView->updateDisplayColors();
        }
      });


}

void QInputSourceView::onStackedWidgetCurrentIndexChanged()
{
  const QWidget *currentWidget =
      currentView();

  if( currentWidget == _imageView ) {
    setCurrentToolbar(_imageViewToolbar);
  }
  else if( currentWidget == _cloudView ) {
    setCurrentToolbar(_cloudViewToolbar);
  }
  else if( currentWidget == _textView ) {
    setCurrentToolbar(_textViewToolbar);
  }

  Q_EMIT currentViewChanged();
}


void QInputSourceView::setCurrentToolbar(QToolBar * toolbar)
{
  if( toolbar == _imageViewToolbar ) {
    _cloudViewToolbar->hide();
    _textViewToolbar->hide();
    _imageViewToolbar->show();
  }
  else if( toolbar == _cloudViewToolbar ) {
    _imageViewToolbar->hide();
    _textViewToolbar->hide();
    _cloudViewToolbar->show();
  }
  else if( toolbar == _textViewToolbar ) {
    _imageViewToolbar->hide();
    _cloudViewToolbar->hide();
    _textViewToolbar->show();
  }
  else {
    _imageViewToolbar->hide();
    _cloudViewToolbar->hide();
    _textViewToolbar->hide();
  }

  _toolbarLayout->update();
}

QToolBar* QInputSourceView::currentToolbar() const
{
  if( _cloudViewToolbar->isVisible() ) {
    return _cloudViewToolbar;
  }

  if( _imageViewToolbar->isVisible() ) {
    return _imageViewToolbar;
  }

  if( _textViewToolbar->isVisible() ) {
    return _textViewToolbar;
  }

  return nullptr;
}


QString QInputSourceView::currentFileName() const
{
  return _currentSource ? _currentSource->cfilename() : QString();
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
  _currentProcessor = processor;

  if ( _currentFrame ) {
    _currentFrame->cleanup();
    processCurrentFrame();
    setViewType(_selectedViewType);
  }
}

const c_data_frame_processor::sptr & QInputSourceView::currentProcessor() const
{
  return _currentProcessor;
}


void QInputSourceView::setPointSelectionMode(QPointSelectionMode * selectionMode)
{
  if( _currentPointSelectionMode ) {
    _currentPointSelectionMode->setActive(this, false);
  }

  if( (_currentPointSelectionMode = selectionMode) ) {
    _currentPointSelectionMode->setActive(this, true);
  }
}

QPointSelectionMode * QInputSourceView::pointSelectionMode() const
{
  return _currentPointSelectionMode;
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
  return _currentSource;
}

const c_input_options * QInputSourceView::inputOptions() const
{
  return &_input_options;
}

c_input_options * QInputSourceView::inputOptions()
{
  return &_input_options;
}

bool QInputSourceView::openFile(const QString & abspath)
{
  closeCurrentSource();

  if ( !abspath.isEmpty() ) {

    const std::string filename =
        abspath.toStdString();

    if( !(_currentSource = c_input_source::create(filename)) ) {
      CF_ERROR("c_input_source::open('%s') fails", filename.c_str());
      return false;
    }

    // CF_DEBUG("typeid(currentSource_)=%s",  typeid(*currentSource_.get()).name());

    _currentSource->set_input_options(&_input_options);

    startDisplay();

    Q_EMIT currentFileNameChanged();
  }

  return true;
}

void QInputSourceView::reloadCurrentFrame()
{
  if( isOpen(_currentSource) ) {
    if( _currentSource->curpos() > 0 ) {
      _currentSource->seek(_currentSource->curpos() - 1);
    }
    loadNextFrame();
  }
}

int QInputSourceView::currentScrollpos() const
{
  return _playControls->curPos();
}

bool QInputSourceView::scrollToFrame(int frameIndex)
{
  if( isOpen(_currentSource) ) {

    if( frameIndex >= 0 ) {
      _currentSource->seek(frameIndex);
    }


    loadNextFrame();

    _playControls->setCurpos(std::max(0,
        _currentSource->curpos() - 1));

    Q_EMIT currentFrameChanged();

    return true;
  }

  return false;
}


void QInputSourceView::onSeek(int pos)
{
  if( isOpen(_currentSource) ) {
    if( pos != _currentSource->curpos() ) {
      _currentSource->seek(pos);
    }
    loadNextFrame();
  }
}

void QInputSourceView::startDisplay()
{
  _imageView->setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  _playControls->setState(QPlaySequenceControl::Stopped);
  _playControls->hide();

  if ( !_currentSource ) {
    return;
  }

  if ( !_currentSource->is_open() && !_currentSource->open() ) {
    CF_ERROR("currentSource_->open() fails");
    return;
  }

  const int num_frames =
      _currentSource->size();

  if ( num_frames < 1 ) {
    QMessageBox::critical(this, "ERROR",
        "Can not determine number of frames and seek range for given source.\n"
        "Image can not be displayed correctly, seems such input source is not supported");
    return;
  }

  if( num_frames > 1 ) {
    _playControls->show();
    _playControls->setSeekRange(0, num_frames - 1);
    _playControls->setCurpos(0);
  }

  loadNextFrame();
}

void QInputSourceView::loadNextFrame()
{
  if( isOpen(_currentSource) ) {

    QWaitCursor wait(this, _currentSource->size() == 1);

    if( !_currentSource->read(_currentFrame) ) {
      CF_ERROR("currentSource_->read(currentFrame_) fails");
      return;
    }

    processCurrentFrame();

    setViewType(_selectedViewType);

    Q_EMIT currentFrameChanged();
  }

}

void QInputSourceView::processCurrentFrame()
{
  if( !_currentFrame ) {
    _selectedViewType = DisplayType_Image;
    _viewSelectionToolbutton_ctl->setEnabled(false);
  }
  else {

    if( _currentProcessor && !_currentProcessor->process(_currentFrame) ) {
      CF_ERROR("currentProcessor_->process(currentFrame_) fails");
    }

    const auto & viewTypes =
        _currentFrame->get_available_display_types();

    if( viewTypes.empty() ) {
      _selectedViewType = DisplayType_Image;
    }
    else if( viewTypes.find(_selectedViewType) == viewTypes.end() ) {
      _selectedViewType = *viewTypes.begin();
    }

    _viewSelectionToolbutton_ctl->setEnabled(viewTypes.size() > 1);
  }
}


void QInputSourceView::closeCurrentSource()
{
  if ( _currentSource ) {
    _currentSource->close();
  }

  _currentFrame.reset();
}

void QInputSourceView::setViewType(DisplayType viewType)
{
  if ( viewType != _selectedViewType ) {
    _selectedViewType = viewType;
  }

  if( _currentFrame ) {

    const auto & new_displays =
        _currentFrame->get_available_data_displays();

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

  if( _currentFrame ) {

    switch (_selectedViewType) {

      case DisplayType_Image: {

        cv::Mat image, data, mask;

        _currentFrame->get_image(displayChannel_.toStdString(),
            image, mask, data);

        setCurrentView(_imageView);
        _imageView->inputImage() = image;
        _imageView->inputMask() = mask;
        _imageView->updateImage();
        break;
      }

      case DisplayType_PointCloud: {

        std::vector<cv::Mat> points, colors, mask;
        std::vector<std::vector<uint64_t>> pids;

        _currentFrame->get_point_cloud(displayChannel_.toStdString(),
            points, colors, mask, &pids);

        setCurrentView(_cloudView);
        _cloudView->setPoints(points, colors, mask, false);
        break;
      }

      case DisplayType_TextFile:
        setCurrentView(_textView);
        _textView->showTextFile(_currentFrame->get_filename());
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

  switch (_selectedViewType) {
    case DisplayType_Image:
      getminmax(_imageView->currentImage(), minval, maxval, _imageView->currentMask());
      break;
    case DisplayType_PointCloud:
      getminmax(_cloudView->currentColors(), minval, maxval, _cloudView->currentMasks());
      break;
    default:
      break;
  }
}


void QInputSourceView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  switch (_selectedViewType) {
    case DisplayType_Image:

      create_histogram(_imageView->currentImage(),
          _imageView->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);


      break;

    case DisplayType_PointCloud:

      create_histogram(_cloudView->currentColors(),
          _cloudView->currentMasks(),
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

  switch (_selectedViewType) {
    case DisplayType_Image:

      create_histogram(_imageView->mtfImage(),
          _imageView->currentMask(),
          H,
          hmin, hmax,
          256,
          false,
          false);

      break;

    case DisplayType_PointCloud:

      create_histogram(_cloudView->mtfColors(),
          cv::noArray(),
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
    cv::InputArray currentMasks,
    cv::OutputArray displayPoints,
    cv::OutputArray mtfColors,
    cv::OutputArray displayColors)
{

//  CF_DEBUG("\n"
//      "ncurrentPoints: %dx%d depth=%d channels=%d  currentColors: %dx%d depth=%d channels=%d",
//      currentPoints.rows(), currentPoints.cols(), currentPoints.depth(), currentPoints.channels(),
//      currentColors.rows(), currentColors.cols(), currentColors.depth(), currentColors.channels()
//      );

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
      opts.colormap != COLORMAP_NONE;

  currentPoints.getMat().convertTo(displayPoints, CV_32F);


  adjustMtfRange(mtf, needColormap ? currentColors : cv::noArray(), currentMasks, &a);
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

    if( currentMasks.size() == displaycolors.size() ) {
      displaycolors.setTo(0, ~currentMasks.getMat());
    }

    cv::cvtColor(displaycolors, displaycolors, cv::COLOR_BGR2RGB);

    displaycolors.copyTo(displayColors);

  }

  else {

    // CF_DEBUG("mtfcolors.empty()=%d mtfcolors.channels()=%d", mtfcolors.empty(), mtfcolors.channels());

    if( mtfcolors.empty() ) {
      displayColors.release();
    }
    else {

      if( mtfcolors.channels() == 1 ) {
        cv::cvtColor(mtfcolors, mtfcolors,
            cv::COLOR_GRAY2BGR);
      }

      if( !displayColors.fixedType() || displayColors.type() == mtfcolors.type() ) {
        mtfcolors.copyTo(displayColors);
      }
      else {
        mtfcolors.convertTo(displayColors, displayColors.type());
      }
    }
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
  _debayerAlgorithm = algo;

  if( isOpen(_currentSource) && dynamic_cast<c_image_input_source*>(_currentSource.get()) ) {

    if( _currentSource->curpos() > 0 ) {
      _currentSource->seek(_currentSource->curpos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT debayerAlgorithmChanged();
}

DEBAYER_ALGORITHM QInputSourceView::debayerAlgorithm() const
{
  return _debayerAlgorithm;
}

void QInputSourceView::setDropBadPixels(bool v)
{
  _filterBadPixels = v;

  if( isOpen(_currentSource) && dynamic_cast<c_image_input_source*>(_currentSource.get()) ) {

    if( _currentSource->curpos() > 0 ) {
      _currentSource->seek(_currentSource->curpos() - 1);
    }

    loadNextFrame();
  }


  Q_EMIT dropBadPixelsChanged();
}

bool QInputSourceView::dropBadPixels() const
{
  return _filterBadPixels;
}

void QInputSourceView::setBadPixelsVariationThreshold(double v)
{
  _badPixelsVariationThreshold = v;

  if( isOpen(_currentSource) && dynamic_cast<c_image_input_source*>(_currentSource.get()) ) {

    if( _currentSource->curpos() > 0 ) {
      _currentSource->seek(_currentSource->curpos() - 1);
    }

    loadNextFrame();
  }

  Q_EMIT badPixelsVariationThresholdChanged();
}

double QInputSourceView::badPixelsVariationThreshold() const
{
  return _badPixelsVariationThreshold;
}

void QInputSourceView::onCloudViewPointSelectionMouseEvent(QEvent::Type eventType, int keyOrMouseButtons,
    Qt::KeyboardModifiers keyboardModifiers, const QPointF & mousePos,
    bool objHit, double objX, double objY, double objZ)
{
  if( _currentPointSelectionMode ) {
    _currentPointSelectionMode->glMouseEvent(this, eventType, keyOrMouseButtons,
        keyboardModifiers, mousePos,
        objHit, objX, objY, objZ);
  }
}


void QInputSourceView::populateImageViewContextMenu(QMenu & menu, const QPoint & viewpos)
{
  _imageView->populateContextMenu(menu, viewpos);
}

///////////////
} // namespace serstacker
