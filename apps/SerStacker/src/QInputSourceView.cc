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
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <core/proc/minmax.h>
#include <core/data_annotation/c_data_annotation_labels.h>
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
  //mainLayout_->setContentsMargins(0,0,0,0);
  //mainLayout_->setMargin(0);
  _mainLayout->setSpacing(0);

  _mainLayout->addLayout(_toolbarLayout = new QHBoxLayout());
  _toolbarLayout->setContentsMargins(0,0,0,0);
  //toolbarLayout_->setMargin(0);
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

  QObject::connect(_cloudView, &QPointCloudSourceView::glPointMouseEvent,
      this, &ThisClass::onCloudViewPointSelection);

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

DisplayType QInputSourceView::currentViewType() const
{
  return _currentViewType;
}

void QInputSourceView::setupMainToolbar()
{
  _mainToolbar->addWidget(viewTypeSelectionToolbutton_ctl =
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
                            viewType == _currentViewType,
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
    _currentFrame->clean_artifacts();
    processCurrentFrame();
    setViewType(_currentViewType);
  }
}

const c_data_frame_processor::sptr & QInputSourceView::currentProcessor() const
{
  return _currentProcessor;
}


void QInputSourceView::setPointSelectionMode(QPointSelectionMode *selectionMode)
{
  if (_pointSelectionMode) {
    _pointSelectionMode->setActive(this, false);
  }

  if ((_pointSelectionMode = selectionMode)) {
    _pointSelectionMode->setActive(this, true);
  }
}

QPointSelectionMode * QInputSourceView::pointSelectionMode() const
{
  return _pointSelectionMode;
}

void QInputSourceView::setDataAnnotationLabels(const c_data_annotation_labels * v)
{
  data_annotation_labels = v;
}

const c_data_annotation_labels * QInputSourceView::dataAnnotationLabels() const
{
  return data_annotation_labels;
}

void QInputSourceView::setDataAnnotationBlendAlpha(double v)
{
  _dataAnnotationBlendAlpha = v;
  _cloudView->updateDisplayColors();
}

double QInputSourceView::dataAnnotationBlendAlpha() const
{
  return _dataAnnotationBlendAlpha;
}

const c_input_source::sptr & QInputSourceView::inputSource() const
{
  return _currentSource;
}

const c_input_options * QInputSourceView::inputOptions() const
{
  return &_inputOptions;
}

c_input_options * QInputSourceView::inputOptions()
{
  return &_inputOptions;
}

bool QInputSourceView::openSource(const QString & abspath)
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

    _currentSource->set_input_options(&_inputOptions);

    startDisplay();

    Q_EMIT currentFileNameChanged();
  }

  return true;
}

const c_data_frame::sptr & QInputSourceView::currentFrame() const
{
  return _currentFrame;
}

const c_input_source::sptr & QInputSourceView::currentSource() const
{
  return _currentSource;
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

void QInputSourceView::reloadCurrentFrame()
{
  if( isOpen(_currentSource) ) {
    if( _currentSource->curpos() > 0 ) {
      _currentSource->seek(_currentSource->curpos() - 1);
    }
    loadNextFrame();
  }
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

    setViewType(_currentViewType);

    Q_EMIT currentFrameChanged();
  }

}

void QInputSourceView::processCurrentFrame()
{
  if( !_currentFrame ) {
    _currentViewType = DisplayType_Image;
    viewTypeSelectionToolbutton_ctl->setEnabled(false);
  }
  else {

    if( _currentProcessor && !_currentProcessor->process(_currentFrame) ) {
      CF_ERROR("currentProcessor_->process(currentFrame_) fails");
    }

    const auto & viewTypes =
        _currentFrame->get_available_display_types();

    if( viewTypes.empty() ) {
      _currentViewType = DisplayType_Image;
    }
    else if( viewTypes.find(_currentViewType) == viewTypes.end() ) {
      _currentViewType = *viewTypes.begin();
    }

    viewTypeSelectionToolbutton_ctl->setEnabled(viewTypes.size() > 1);
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
  if ( viewType != _currentViewType ) {
    _currentViewType = viewType;
  }

  if( _currentFrame ) {

    const auto & new_displays =
        _currentFrame->get_available_image_displays();

    auto & existing_displays =
        this->_displays;

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

  if( !_displays.empty() && _displays.find(_displayChannel) == _displays.end() ) {
    _displayChannel = _displays.begin()->first;
  }

  if( _currentFrame ) {

    switch (_currentViewType) {

      case DisplayType_Image: {

        cv::Mat image, data, mask;

        _currentFrame->get_image(_displayChannel.toStdString(),
            image, mask, data);

        setCurrentView(_imageView);
        _imageView->inputImage() = image;
        _imageView->inputMask() = mask;
        _imageView->updateImage();
        break;
      }

      case DisplayType_PointCloud: {

        cv::Mat points, colors, mask;
        std::vector<uint64_t> pids;

        _currentFrame->get_point_cloud(_displayChannel.toStdString(),
            points, colors, mask, &pids);

        setCurrentView(_cloudView);
        _cloudView->setPoints(std::move(points), std::move(colors), std::move(mask), std::move(pids));
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

  switch (_currentViewType) {
    case DisplayType_Image:
      getminmax(_imageView->currentImage(), minval, maxval, _imageView->currentMask());
      CF_DEBUG("minval=%g maxval=%g", *minval, *maxval);
      break;
    case DisplayType_PointCloud:
      getminmax(_cloudView->currentColors(), minval, maxval, _cloudView->currentMask());
      CF_DEBUG("minval=%g maxval=%g", *minval, *maxval);
      break;
    default:
      break;
  }
}

//
//void QInputSourceView::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
//{
//  switch (_currentViewType) {
//    case DisplayType_Image:
//
//      create_histogram(_imageView->currentImage(),
//          _imageView->currentMask(),
//          H,
//          hmin, hmax,
//          256,
//          false,
//          false);
//
//
//      break;
//
//    case DisplayType_PointCloud:
//
//      create_histogram(_cloudView->currentColors(),
//          _cloudView->currentMask(),
//          H,
//          hmin, hmax,
//          256,
//          false,
//          false);
//
//      break;
//
//    default:
//      H.release();
//      break;
//  }
//
//}


void QInputSourceView::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{

  switch (_currentViewType) {
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
          _cloudView->currentMask(),
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

//    if (data_annotation_labels && _currentFrame->has_point_annotations()) {
//
//    }

  }
}

void QInputSourceView::createDisplayPoints(cv::OutputArray mtfColors,
    std::vector<cv::Vec3f> & displayPoints,
    std::vector<cv::Vec3b> & displayColors)
{

  const cv::Mat &currentPoints =
      _cloudView->currentPoints();

  //CF_DEBUG("currentPoints: %dx%d", currentPoints.rows, currentPoints.cols);

  if ( currentPoints.empty() ) {

    mtfColors.release();
    displayPoints.clear();
    displayColors.clear();

    Q_EMIT displayImageChanged();

    return;
  }

  DisplayParams & opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  cv::Mat mtfcolors, displaycolors;

  const cv::Mat & currentColors =
      _cloudView->currentColors();

  const cv::Mat & currentMask =
      _cloudView->currentMask();

  const bool needColormap =
      opts.colormap != COLORMAP_NONE;


  currentPoints.convertTo(displayPoints, CV_32F);

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
      displaycolors.setTo(0, ~currentMask);
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


  const std::vector<uint64_t> &currentPids =
      _cloudView->currentPids();

  if (data_annotation_labels && !currentPids.empty() && _currentFrame->has_point_annotations()) {

    cv::Vec4b c;

    const int num_colormaps =
        data_annotation_labels->num_colormaps();

    const double global_alpha =
        _dataAnnotationBlendAlpha;

    for (size_t i = 0, n = currentPids.size(); i < n; ++i) {
      for (int cmap = 0; cmap < num_colormaps; ++cmap) {
        if (data_annotation_labels->colormap(cmap)->visible()) {

          const uint8_t lb =
              _currentFrame->point_annotation(currentPids[i], cmap);

          if (lb) {

            if (data_annotation_labels->colormap(cmap)->color_for_label(lb, &c)) {

              const double alpha = global_alpha * c[3] / 255.;
              displayColors[i] = (1 - alpha) * displayColors[i] + alpha * cv::Vec3b(c[2], c[1], c[0]);

            }
          }
        }
      }
    }

  }

  Q_EMIT displayImageChanged();
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

QString QInputSourceView::statusStringForPoint3D(uint64_t pid) const
{
  return "";
}


void QInputSourceView::onContextMenuRequest(const QPointF & mousePos, QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{
  QMenu menu;

  populate3DPointContextMenu(menu,
      currentFrame(),
      mousePos,
      objHit,
      objX,
      objY,
      objZ);

  if (!menu.isEmpty()) {

    menu.exec(cloudView()->mapToGlobal(QPoint(
        mousePos.x(),
        mousePos.y())));
  }



}

void QInputSourceView::onCloudViewPointSelection(const QPointF &mousePos, QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{

  if (mouseEventType == QEvent::MouseButtonPress && mouseButtons == Qt::RightButton) {
    onContextMenuRequest(mousePos, mouseEventType,
        mouseButtons, keyboardModifiers,
        objHit, objX, objY, objZ);
    return;
  }

  if (_pointSelectionMode && _pointSelectionMode->isActive()) {

    return _pointSelectionMode->glMouseEvent(this, mousePos,
        mouseEventType,
        mouseButtons,
        keyboardModifiers,
        objHit, objX, objY, objZ);
  }

  if (objHit) {

    uint64_t pid;

    if (_cloudView->findPointID(objX, objY, objZ, &pid)) {

      if (mouseEventType == QEvent::MouseButtonPress && mouseButtons == Qt::LeftButton) {

        Q_EMIT glPointClick(pid, mousePos, mouseEventType,
            mouseButtons, keyboardModifiers);
      }
    }
  }
}

void QInputSourceView::populateImageViewContextMenu(QMenu & menu, const QPoint & viewpos)
{
  _imageView->populateContextMenu(menu, viewpos);
}

void QInputSourceView::populate3DPointContextMenu(QMenu &menu,
    const c_data_frame::sptr &dataframe,
    const QPointF &mousePos,
    bool objHit,
    double objX,
    double objY,
    double objZ)
{
}



///////////////
} // namespace serstacker
