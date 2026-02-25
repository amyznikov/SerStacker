/*
 * QLivePipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLivePipeline.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qimageview/cv2qt.h>
#include <core/io/image/c_image_input_source.h>
#include <core/mtf/mtf-histogram.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace serimager {


#define ICON_start            ":/serimager/icons/start.png"
#define ICON_stop             ":/serimager/icons/stop.png"
#define ICON_process          ":/serimager/icons/process.png"
#define ICON_menu             ":/serimager/icons/menu.png"
#define ICON_add              ":/serimager/icons/add.png"
#define ICON_delete           ":/serimager/icons/delete.png"
#define ICON_rename           ":/serimager/icons/rename.png"
#define ICON_bayer            ":/gui/icons/bayer.png"

namespace {

typedef std::lock_guard<std::mutex>
  c_guard_lock;

typedef std::unique_lock<std::mutex>
  c_unique_lock;

} // namespace



///////////////////////////////////////////////////////////////////////////////////////////////////


QLiveDisplayMtfFunction::QLiveDisplayMtfFunction(QImageViewer * imageViewer) :
    Base(imageViewer, "QVideoFrameMtfDisplayFunction")
{
}

void QLiveDisplayMtfFunction::getInputDataRange(double * minval, double * maxval) const
{
  c_unique_lock lock(_mutex);
  Base::getInputDataRange(minval, maxval);
}
//
//void QLiveDisplayMtfFunction::getInputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
//{
//  INSTRUMENT_REGION("");
//
//  if ( imageViewer_ ) {
//
//    cv::Mat image, mask;
//
//    double scale = 1.0;
//    double offset = 0.0;
//
//    _mutex.lock();
//    _isBusy = true;
//
//    const cv::Mat & currentImage =
//        imageViewer_->currentImage();
//
//    if ( currentImage.depth() == CV_8U ) {
//      currentImage.copyTo(image);
//    }
//    else {
//      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
//      currentImage.convertTo(image, CV_8U, scale, offset);
//    }
//
//    imageViewer_->currentMask().copyTo(mask);
//    _mutex.unlock();
//
//    create_histogram(image, mask,
//        H,
//        output_hmin, output_hmax,
//        256,
//        false,
//        false);
//
//    _mutex.lock();
//    _isBusy = false;
//    _mutex.unlock();
//
//    (*output_hmin -= offset) /= scale;
//    (*output_hmax -= offset) /= scale;
//  }
//}

void QLiveDisplayMtfFunction::getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
{
//  Base::getOutputHistogramm(H, output_hmin, output_hmax);
//  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    _mutex.lock();
    _isBusy = true;

    const cv::Mat & currentImage =
        imageViewer_->mtfImage();

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else {
      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, CV_8U, scale, offset);
    }

    imageViewer_->currentMask().copyTo(mask);
    _mutex.unlock();

    create_histogram(image, mask,
        H,
        output_hmin, output_hmax,
        256,
        false,
        false);

    (*output_hmin -= offset) /= scale;
    (*output_hmax -= offset) /= scale;

    _mutex.lock();
    _isBusy = false;
    _mutex.unlock();

  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////


QLiveDisplay::QLiveDisplay(QWidget * parent) :
    Base(parent),
    _mtfDisplayFunction(this)
{
  setDisplayFunction(&_mtfDisplayFunction);

  connect(this, &ThisClass::pixmapChanged,
      this, &ThisClass::onPixmapChanged,
      Qt::QueuedConnection);

  connect(&_mtfDisplayFunction, &QMtfDisplay::displayChannelsChanged,
      &_mtfDisplayFunction, &QMtfDisplay::parameterChanged);

  connect(&_mtfDisplayFunction, &QMtfDisplay::parameterChanged,
      [this]() {
        if ( !_mtfDisplayFunction.isBusy() ) {
          Base::updateImage();
        }
      });

  connect(this, &ThisClass::displayImageChanged,
      &_mtfDisplayFunction, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);


  connect(this, &ThisClass::startUpdateLiveDisplayTimer,
      this, &ThisClass::onStartUpdateLiveDisplayTimer,
      Qt::QueuedConnection);

  connect(this, &ThisClass::stopUpdateLiveDisplayTimer,
      this, &ThisClass::onStopUpdateLiveDisplayTimer,
      Qt::QueuedConnection);



  createShapes();
}

QLiveDisplay::~QLiveDisplay()
{

}

void QLiveDisplay::createShapes()
{
  if( !_rectShape ) {

    QRectF rect;

    if( _currentImage.empty() ) {
      rect.setRect(0, 0, 400, 400);
    }
    else {

      rect.setRect(0, 0, _currentImage.cols, _currentImage.rows);

      if( rect.width() > 400 ) {
        rect.setX((rect.left() + rect.right()) / 2 - 200);
        rect.setWidth(400);
      }

      if( rect.height() > 400 ) {
        rect.setY((rect.top() + rect.bottom()) / 2 - 200);
        rect.setHeight(400);
      }
    }

    _rectShape = new QGraphicsRectShape(rect);
    _rectShape->setResizable(true);
    _rectShape->setSnapToPixelGrid(true);
    _rectShape->setFlag(QGraphicsItem::ItemIsMovable, true);
    _rectShape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    _rectShape->setCosmeticPen(Qt::red);
    _rectShape->setVisible(false);
    _scene->addItem(_rectShape);
  }

  if( !_lineShape ) {

    _lineShape = new QGraphicsLineShape(-100, -100, 100, 100);
    _lineShape->setFlag(QGraphicsItem::ItemIsMovable, true);
    _lineShape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    _lineShape->setCosmeticPen(Qt::green);
    _lineShape->setVisible(false);
    _scene->addItem(_lineShape);
  }

  if( !_targetShape ) {
    _targetShape = new QGraphicsTargetShape();
    _targetShape->setFlag(QGraphicsItem::ItemIsMovable, true);
    _targetShape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    _targetShape->setCosmeticPen(Qt::red);
    _targetShape->setVisible(false);
    _scene->addItem(_targetShape);
  }

}

const QLiveDisplayMtfFunction * QLiveDisplay::mtfDisplayFunction() const
{
  return &_mtfDisplayFunction;
}

QLiveDisplayMtfFunction * QLiveDisplay::mtfDisplayFunction()
{
  return &_mtfDisplayFunction;
}


void QLiveDisplay::setFrameProcessor(const c_image_processor::sptr & processor)
{
  _mtfDisplayFunction.mutex().lock();
  Base::_current_processor = processor;

  if ( !_mtfDisplayFunction.isBusy() ) {
    updateImage();
  }

  _mtfDisplayFunction.mutex().unlock();
}

QGraphicsRectShape * QLiveDisplay::rectShape() const
{
  return _rectShape;
}

QGraphicsLineShape * QLiveDisplay::lineShape() const
{
  return _lineShape;
}

QGraphicsTargetShape * QLiveDisplay::targetShape() const
{
  return _targetShape;
}

void QLiveDisplay::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
}

void QLiveDisplay::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);
}

void QLiveDisplay::timerEvent(QTimerEvent * e)
{
  if( e->timerId() == _update_display_timer_id ) {

    if( _update_display_required ) {

      c_unique_lock mtflock(_mtfDisplayFunction.mutex());
      if( !_mtfDisplayFunction.isBusy() ) {

        _update_display_required = false;

        _live_pipeline_lock.lock();

        if( _live_pipeline && _live_pipeline->get_display_image(_inputImage, _inputMask) ) {
          updateCurrentImage();
        }

        _live_pipeline_lock.unlock();
      }
    }

    e->ignore();
    return;
  }

  Base::timerEvent(e);
}


void QLiveDisplay::onStartUpdateLiveDisplayTimer()
{
  _update_display_timer_id = startTimer(100);
}

void QLiveDisplay::onStopUpdateLiveDisplayTimer()
{
  if ( _update_display_timer_id ) {
    killTimer(_update_display_timer_id);
    _update_display_timer_id = 0;
  }
}

void QLiveDisplay::setLivePipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  _live_pipeline_lock.lock();

  if( _live_pipeline ) {

//    if( true ) {
//      c_unique_lock mtflock(mtfDisplayFunction_.mutex());
//      if( !mtfDisplayFunction_.isBusy() && live_pipeline_->get_display_image(inputImage_, inputMask_) ) {
//        updateCurrentImage();
//      }
//      else {
//        inputImage_.release();
//        inputMask_.release();
//      }
//    }

    QImageProcessingPipeline *pp =
        dynamic_cast<QImageProcessingPipeline*>(_live_pipeline.get());
    if( pp ) {
      pp->disconnect(this);
    }

    if( _update_display_timer_id ) {
      Q_EMIT stopUpdateLiveDisplayTimer();
    }

  }


  if( (_live_pipeline = pipeline) ) {

    QImageProcessingPipeline *pp =
        dynamic_cast<QImageProcessingPipeline*>(_live_pipeline.get());

    if( pp ) {
      connect(pp, &QImageProcessingPipeline::frameProcessed,
          [this]() {
            // give chance to GUI thread to call get_display_image()
            _update_display_required = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          });
    }

    _update_display_required = true;
    Q_EMIT startUpdateLiveDisplayTimer();
  }

  _live_pipeline_lock.unlock();
}


void QLiveDisplay::onPixmapChanged()
{
  c_unique_lock lock(_mtfDisplayFunction.mutex());
  scene()->setImage(_pixmap);
  Q_EMIT currentImageChanged();
}

void QLiveDisplay::updateCurrentImage()
{
  if( !_current_processor || _current_processor->empty() ) {

    current_image_lock lock(this);
    _inputImage.copyTo(_currentImage);
    _inputMask.copyTo(_currentMask);
  }
  else {

    cv::Mat tmp_image, tmp_mask;

    _inputImage.copyTo(tmp_image);
    _inputMask.copyTo(tmp_mask);
    _current_processor->process(tmp_image, tmp_mask);

    current_image_lock lock(this);
    _currentImage = tmp_image;
    _currentMask = tmp_mask;
  }

  if ( true ) {
    _mtfDisplayFunction.createDisplayImage(
        _currentImage,
        _currentMask,
        _mtfImage,
        _displayImage,
        CV_8U);
  }

  if ( true ) {
    _pixmap =
        createPixmap(_displayImage, true,
            Qt::NoFormatConversion |
                Qt::ThresholdDither |
                Qt::ThresholdAlphaDither |
                Qt::NoOpaqueDetection);
  }

  Q_EMIT pixmapChanged();

  if ( !_mtfDisplayFunction.isBusy() ) {
    Q_EMIT displayImageChanged();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{
  load_settings();
}

QLivePipelineThread::~QLivePipelineThread()
{
  while ( isRunning() ) {
    if ( _camera ) {
      _camera->disconnect();
    }
    QThread::msleep(20);
  }
}


void QLivePipelineThread::setDebayer(DEBAYER_ALGORITHM algo)
{
  _debayer = algo;
  save_settings();
}

DEBAYER_ALGORITHM QLivePipelineThread::debayer() const
{
  return _debayer;
}

void QLivePipelineThread::setDarkFramePath(const QString & pathfilename)
{
  setDarkFrame(pathfilename);
  save_settings();
}

const QString & QLivePipelineThread::darkFramePath() const
{
  return _darkFramePath;
}

void QLivePipelineThread::setDarkFrame(const QString & pathfilename)
{
  _darkFrameLock.lock();

  _darkFrame.release();

  if ( !(_darkFramePath = pathfilename).isEmpty() ) {

    cv::Mat ignoreMaskIfExists;
    if ( !load_image(_darkFramePath.toStdString(), _darkFrame, ignoreMaskIfExists) ) {
      CF_ERROR("load_image('%s') fails", _darkFramePath.toUtf8().constData());
    }
    else if ( _darkFrameScale != 0 ) {
      cv::multiply(_darkFrame, _darkFrameScale, _darkFrame);
    }
    //    else if (darkFrame_.depth() == CV_32F ) {
    //    }
  }

  _darkFrameLock.unlock();
}

void QLivePipelineThread::setDarkFrameScale(double v)
{
  _darkFrameScale = v;
  if ( !_darkFramePath.isEmpty() ) {
    setDarkFrame(_darkFramePath);
  }
  save_settings();
}

double QLivePipelineThread::darkFrameScale() const
{
  return _darkFrameScale;
}

void QLivePipelineThread::load_settings()
{
  QSettings settigs;
  _debayer = (DEBAYER_ALGORITHM) (settigs.value("QLivePipelineThread/debayer", (int) _debayer).toInt());
  _darkFramePath = settigs.value("QLivePipelineThread/darkFramePath", _darkFramePath).toString();
  _darkFrameScale = settigs.value("QLivePipelineThread/darkFrameScale", _darkFrameScale).toDouble();
}

void QLivePipelineThread::save_settings()
{
  QSettings settigs;
  settigs.setValue("QLivePipelineThread/debayer", (int)_debayer);
  settigs.setValue("QLivePipelineThread/darkFramePath", _darkFramePath);
  settigs.setValue("QLivePipelineThread/darkFrameScale", _darkFrameScale);
}

void QLivePipelineThread::setDisplay(QLiveDisplay * display)
{
  _display = display;
}

QLiveDisplay* QLivePipelineThread::display() const
{
  return _display;
}

void QLivePipelineThread::setCamera(const QImagingCamera::sptr & camera)
{
  // finish(true);

  if( _camera ) {
    disconnect(_camera.get(), nullptr,
        this, nullptr);
  }

  if( (_camera = camera) ) {

    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    if( _camera->state() == QImagingCamera::State_started ) {
      start();
    }
  }
}

const QImagingCamera::sptr & QLivePipelineThread::camera() const
{
  return _camera;
}

void QLivePipelineThread::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  switch (newState) {
    case QImagingCamera::State_started:
      if ( !isRunning() ) {
        start();
      }
      break;
    default:
      break;
  }
}


bool QLivePipelineThread::setPipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  unique_lock lock(_mutex);

  c_image_processing_pipeline::sptr current_pipeline =
      this->_pipeline;

  if( current_pipeline ) {

    this->_pipeline.reset();

    current_pipeline->cancel(true);
    _condvar.wait(lock, [current_pipeline]() {
      return !current_pipeline->is_running();
    });
  }

  this->_pipeline = pipeline;

  if ( !isRunning() ) {
    Q_EMIT pipelineChanged();
  }

  return true;
}

const c_image_processing_pipeline::sptr & QLivePipelineThread::pipeline() const
{
  return _pipeline;
}

void QLivePipelineThread::run()
{
  struct c_camera_input_source :
      public c_image_input_source
  {
    typedef c_camera_input_source this_class;
    typedef c_image_input_source base;
    typedef std::shared_ptr<this_class> sptr;

    QLivePipelineThread * _thread;
    QImagingCamera::sptr _camera;
    int last_frame_index = -1;
    int bpp = -1;
    COLORID colorid = COLORID_UNKNOWN;

    c_camera_input_source(QLivePipelineThread * thread, const QImagingCamera::sptr & camera) :
        base(/*c_input_source::CAMERA, */""),
        _thread(thread),
        _camera(camera)
    {
    }

    bool open() override
    {
      return _camera->state() == QImagingCamera::State_started;
    }

    void close()  override
    {
    }

    bool seek(int pos) override
    {
      return true;
    }

    int curpos() override
    {
      return 0;
    }

    bool is_open() const override
    {
      return _camera->state() == QImagingCamera::State_started;
    }

    bool read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpp) override
    {
      while (_camera->state() == QImagingCamera::State_started) {

        bool haveInputImage = false;

        if( 42 ) {

          QImagingCamera::shared_lock lock(_camera->mutex());

          const std::deque<QCameraFrame::sptr> &deque =
              _camera->deque();

          if( !deque.empty() ) {

            const QCameraFrame::sptr &frame =
                deque.back();

            const int index =
                frame->index();

            if( index > last_frame_index ) {

              last_frame_index = index;
              * output_bpp = bpp = frame->bpp();
              * output_colorid = colorid = frame->colorid();
              frame->image().copyTo(output_frame);

              return true;
            }
          }
        }

        QThread::msleep(20);
      }

      return false;
    }
  };

  struct c_camera_input_sequence :
      public c_input_sequence
  {
    typedef c_camera_input_sequence this_class;
    typedef c_input_sequence base;
    typedef std::shared_ptr<this_class> sptr;

    c_camera_input_source::sptr camera_source;

    c_camera_input_sequence(QLivePipelineThread * thread, const QImagingCamera::sptr & camera)
    {
      camera_source.reset(new c_camera_input_source(thread, camera));
      _all_sources.emplace_back(camera_source);
      _enabled_sources.emplace_back(camera_source);
      _current_source = 0;
      _current_global_pos = 0;

      set_name(get_file_name(camera->display_name().toStdString()));
    }

    bool is_live() const override
    {
      return true;
    }

    bool open() override
    {
      return camera_source->is_open();
    }

    void close(bool /*clear */= false) override
    {
    }

    bool seek(int pos) override
    {
      return true;
    }

    bool is_open() const override
    {
      return camera_source->is_open();
    }


  };

  CF_DEBUG("enter");
  /////////////////////

  const QImagingCamera::sptr camera = _camera;

  if( camera ) {

    c_camera_input_sequence::sptr input_sequence(
        new c_camera_input_sequence(this,
            camera));

    while (camera->state() == QImagingCamera::State_started) {

      QImageProcessingPipeline * pp = nullptr;

      try {

        _mutex.lock();
        c_image_processing_pipeline::sptr pipeline =
            this->_pipeline;
        _mutex.unlock();

        _display->setLivePipeline(pipeline);

        Q_EMIT pipelineChanged();

        if( pipeline ) {

          try {
            if( pipeline->run(input_sequence) ) {
              CF_DEBUG("pipeline finished");
            }
            else {
              CF_ERROR("pipeline->run() fails");
            }
          }
          catch( const std::exception &e ) {
            CF_ERROR("Exception in pipeline->run() : %s", e.what());
          }
          catch (...) {
            CF_ERROR("Unknown Exception in pipeline->run()");
          }

          if( camera->state() == QImagingCamera::State_started ) {
            // most probably there was an error in pipeline->run()
            unique_lock lock(_mutex);
            this->_pipeline.reset();
          }

          _condvar.notify_all();
        }
        else {

          cv::Mat inputImage;
          COLORID colorid;
          int bpp;

          _display->inputMask().release();

          while ( input_sequence->camera_source->read(inputImage, &colorid, &bpp)  ) {

            if ( true ) {
              c_unique_lock lock(_darkFrameLock);

              if( _darkFrameScale != 0 && _darkFrame.size() == inputImage.size() &&
                  _darkFrame.channels() == inputImage.channels() ) {
                inputImage.convertTo(inputImage, _darkFrame.depth());
                cv::subtract(inputImage, _darkFrame, inputImage);
              }
            }

            if( _debayer != DEBAYER_DISABLE && is_bayer_pattern(colorid) ) {

              const DEBAYER_ALGORITHM method =
                  inputImage.depth() == CV_32F ? DEBAYER_NN2 :
                      (DEBAYER_ALGORITHM) _debayer;

              if( ::debayer(inputImage, inputImage, colorid, method) ) {
                colorid = COLORID_BGR;
              }
            }

            if( true ) {
              c_unique_lock mtflock(_display->mtfDisplayFunction()->mutex());
              inputImage.copyTo(_display->inputImage());
              _display->updateCurrentImage();
              QThread::msleep(10);
            }

            if( true ) {
              unique_lock lock(_mutex);
              if( _pipeline != nullptr ) {
                break;
              }
            }
          }
        }
      }
      catch( const std::exception &e ) {
        CF_ERROR("Exception in live thread : %s", e.what());
      }
      catch (...) {
        CF_ERROR("Unknown Exception in live thread");
      }

      _display->setLivePipeline(nullptr);
    }
  }

  QThread::msleep(100);

  CF_DEBUG("leave");
}

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

class QAddPipelineDialogBox :
    public QDialog
{
public:
  typedef QAddPipelineDialogBox ThisClass;
  typedef QDialog Base;

  QAddPipelineDialogBox(QWidget * parent = nullptr);

  QString selectedPipelineName() const;
  QString selectedPipelineClass() const;

protected:
  //QLivePipelineCollection * pipelineCollection_;
  QFormLayout * form_ = nullptr;
  QHBoxLayout * hbox_ = nullptr;
  QLineEditBox * pipelineName_ctl = nullptr;
  QComboBox * pipelineTypeSelector_ctl = nullptr;
  QLabel * pipelineTooltop_ctl = nullptr;
  QPushButton * btnOk_ = nullptr;
  QPushButton * btnCancel_ = nullptr;
};

QAddPipelineDialogBox::QAddPipelineDialogBox(QWidget * parent) :
    Base(parent)
{
  //using factory_item = c_image_processing_pipeline::factory_item;

  setWindowTitle("Select live pipeline");

  form_ = new QFormLayout(this);

  form_->addRow("Name:", pipelineName_ctl = new QLineEditBox(this));
  form_->addRow("Type:", pipelineTypeSelector_ctl = new QComboBox(this));
  form_->addRow(pipelineTooltop_ctl = new QLabel(this));

  form_->addRow(hbox_ = new QHBoxLayout());
  hbox_->addWidget(btnOk_ = new QPushButton("OK"));
  hbox_->addWidget(btnCancel_ = new QPushButton("Cancel"));

  for( const auto &item : c_image_processing_pipeline::registered_classes() ) {
    pipelineTypeSelector_ctl->addItem(item.class_name.c_str(),
        QVariant::fromValue(QString(item.tooltip.c_str())));
  }

  pipelineTooltop_ctl->setTextFormat(Qt::RichText);
  pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());

  connect(btnOk_, &QPushButton::clicked,
      [this]() {
        if ( pipelineTypeSelector_ctl->currentText().isEmpty() ) {
          pipelineTypeSelector_ctl->setFocus();
        }
        else {
          Base::accept();
        }
      });

  connect(btnCancel_, &QPushButton::clicked,
      [this]() {
        Base::reject();
      });

  connect(pipelineTypeSelector_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      [this](int index) {
        pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());
      });

}

QString QAddPipelineDialogBox::selectedPipelineName() const
{
  return pipelineName_ctl->text();
}

QString QAddPipelineDialogBox::selectedPipelineClass() const
{
  return pipelineTypeSelector_ctl->currentText();
}

} // namespace

QLivePipelineSelectionWidget::QLivePipelineSelectionWidget(QWidget * parent) :
    Base(parent)
{

  static const auto createToolbutton =
      [](const QIcon & icon, const QString & text, const QString & tooltip) -> QToolButton* {
        QToolButton * tb = new QToolButton();
        tb->setIcon(icon);
        tb->setText(text);
        tb->setToolTip(tooltip);
        if (!icon.isNull() ) {
          tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
        }
        return tb;
      };


  setFrameShape(QFrame::Shape::NoFrame);

  _layout = new QVBoxLayout(this);

  _layout->addWidget(toolbar_ctl = new QToolBar(this), 0, Qt::AlignTop);
  toolbar_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
  toolbar_ctl->setIconSize(QSize(16, 16));

  toolbar_ctl->addWidget(combobox_ctl = new QComboBox(this));
  combobox_ctl->setEditable(false);
  combobox_ctl->setMinimumContentsLength(16);
  combobox_ctl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  toolbar_ctl->addWidget(startStop_ctl =
      createToolbutton(getIcon(ICON_start),
          "Start",
          "Start / Stop current pipeline"));

  toolbar_ctl->addWidget(menuButton_ctl =
      createToolbutton(getIcon(ICON_menu),
          "Options",
          "Show / Hide options"));


  scrollArea_ctl = new QScrollArea(this);
  scrollArea_ctl->setWidgetResizable(true);
  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea_ctl->setFrameShape(QFrame::NoFrame);
  _layout->addWidget(scrollArea_ctl, 1000);



  connect(combobox_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onPipelinesComboboxCurrentIndexChanged);

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopCtlClicked);

  connect(menuButton_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuCtlClicked);

  updateControls();
}

void QLivePipelineSelectionWidget::setLiveThread(QLivePipelineThread * liveThread)
{
  if( _liveThread ) {
    _liveThread->disconnect(this);
  }

  if ( (_liveThread  = liveThread) ) {

    connect(_liveThread, &QLivePipelineThread::pipelineChanged,
        this, &ThisClass::onPipelineChanged,
        Qt::QueuedConnection);

  }

  updateControls();
}


QLivePipelineThread * QLivePipelineSelectionWidget::liveThread() const
{
  return _liveThread;
}


static std::string default_config_filename_ =
    "~/.config/SerImager/pipelines.cfg";

void QLivePipelineSelectionWidget::loadPipelines(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !_configFilename.empty() ) {
    filename = _configFilename;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for QLivePipelineCollection::load()");
    return;
  }

  // CF_DEBUG("Loading '%s' ...", filename.c_str());

  c_config cfg(filename);

  if( !cfg.read() ) {
    CF_FATAL("QLivePipelineCollection: cfg.read('%s') fails",
        filename.c_str());
    return;
  }

  std::string object_class;
  if( !load_settings(cfg.root(), "object_class", &object_class) || object_class.empty() ) {
    CF_FATAL("[%s] load_settings(object_class) fails", filename.c_str());
    return;
  }

  if( object_class != "QLivePipelineCollection" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return;
  }

  c_config_setting section =
      cfg.root().get("items");

  if( !section || !section.isList() ) {
    CF_FATAL("section 'items' is not found in file '%s''",
        filename.c_str());
    return;
  }

  combobox_ctl->clear();

  const int N =
      section.length();

  for( int i = 0; i < N; ++i ) {

    c_config_setting item =
        section.get_element(i);

    if( item && item.isGroup() ) {

      std::string class_name;

      if( !load_settings(item, "class_name", &class_name) || class_name.empty() ) {
        CF_ERROR("load_settings(class_name) fails for item %d", i);
        continue;
      }

      c_image_processing_pipeline::sptr obj =
          c_image_processing_pipeline::create_instance(class_name,
              "",
              nullptr);

      if ( !obj ) {
        CF_ERROR("c_image_processing_pipeline::create_instance(class_name='%s') fails for item %d",
            class_name.c_str(), i);
        continue;
      }


      if( !obj->serialize(item, false) ) {
        CF_ERROR("obj->serialize() fails for item index %d (class_name='%s')", i,
            class_name.c_str());
        continue;
      }

      combobox_ctl->addItem(obj->name().c_str(),
          QVariant::fromValue(obj));
    }
  }

  _configFilename = filename;
}

void QLivePipelineSelectionWidget::savePipelines(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !_configFilename.empty() ) {
    filename = _configFilename;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for QLivePipelineCollection::save()");
    return;
  }

  //  CF_DEBUG("Saving '%s' ...",
  //      filename.c_str());

  c_config cfg(filename);

  time_t t = time(0);

  if( !save_settings(cfg.root(), "object_class", std::string("QLivePipelineCollection")) ) {
    CF_FATAL("save_settings(object_class) fails");
    return;
  }

  if( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return;
  }

  c_config_setting section =
      cfg.root().add_list("items");

  const int N =
      combobox_ctl->count();

  for( int i = 0; i < N; ++i ) {

    const c_image_processing_pipeline::sptr obj =
        combobox_ctl->itemData(i).value<c_image_processing_pipeline::sptr>();

    if( !obj ) {
      continue;
    }

    if( !obj->serialize(section.add_group(), true) ) {
      CF_ERROR("obj->serialize() fails for obj '%s' class '%s' ", obj->get_class_name().c_str(), obj->cname());
      continue;
    }
  }

  if( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails",
        cfg.filename().c_str());
    return;
  }

  _configFilename = filename;
}

c_image_processing_pipeline::sptr QLivePipelineSelectionWidget::selectedPipeline() const
{
  return combobox_ctl->currentData().value<c_image_processing_pipeline::sptr>();
}

void QLivePipelineSelectionWidget::onupdatecontrols()
{
  if ( !_liveThread ) {
    setEnabled(false);
  }
  else {

    if( _liveThread->pipeline() ) {
      combobox_ctl->setEnabled(false);
      menuButton_ctl->setEnabled(false);
      startStop_ctl->setIcon(getIcon(ICON_stop));
      startStop_ctl->setEnabled(true);
    }
    else {
      combobox_ctl->setEnabled(true);
      menuButton_ctl->setEnabled(true);
      startStop_ctl->setIcon(getIcon(ICON_start));
      startStop_ctl->setEnabled(selectedPipeline() != nullptr);
    }

    setEnabled(true);
  }
}


void QLivePipelineSelectionWidget::onPipelinesComboboxCurrentIndexChanged(int)
{
  QImageProcessingPipeline *p =
      nullptr;

  QPipelineSettingsWidget *currentWidget =
      dynamic_cast<QPipelineSettingsWidget*>(scrollArea_ctl->widget());

  if( currentWidget ) {
    currentWidget->setCurrentPipeline(nullptr);
    currentWidget = nullptr;
  }

  c_image_processing_pipeline::sptr pipeline =
      selectedPipeline();

  if( pipeline && (p = dynamic_cast<QImageProcessingPipeline*>(pipeline.get())) ) {

    const QString className =
        pipeline->get_class_name().c_str();

    const auto pos =
        std::find_if(_settingsWidgets.begin(), _settingsWidgets.end(),
            [className](const QPipelineSettingsWidget * obj) {
              return obj->pipelineClass() == className;
            });

    if( pos != _settingsWidgets.end() ) {
      currentWidget = *pos;
    }
    else if( !(currentWidget = p->createSettingsWidget(this)) ) {
      CF_ERROR("pipeline->createSettingsWidgget() fails for pipeline class '%s' ",
          className.toUtf8().constData());
    }
    else {
      _settingsWidgets.append(currentWidget);

      QObject::connect(currentWidget, &QSettingsWidget::parameterChanged,
          [this]() {
            savePipelines();
          });
    }
  }

  if( currentWidget ) {
    currentWidget->setCurrentPipeline(p);
  }

  if( scrollArea_ctl->widget() != currentWidget ) {

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->hide();
    }

    scrollArea_ctl->takeWidget();
    scrollArea_ctl->setWidget(currentWidget);

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->show();
    }
  }
}

void QLivePipelineSelectionWidget::onStartStopCtlClicked()
{
  if( _liveThread ) {
    if( _liveThread->pipeline() ) {
      _liveThread->setPipeline(nullptr);
    }
    else {
      _liveThread->setPipeline(selectedPipeline());
    }
  }
}

void QLivePipelineSelectionWidget::onPipelineChanged()
{
  updateControls();
}

void QLivePipelineSelectionWidget::onMenuCtlClicked()
{
  QMenu menu;

  menu.addAction(getIcon(ICON_add), "Add pipeline...",
      this, &ThisClass::onAddLivePipelineClicked);

  menu.addAction(getIcon(ICON_rename), "Rename pipeline......",
      this, &ThisClass::onRenameLivePipelineClicked);

  menu.addSeparator();

  menu.addAction(getIcon(ICON_delete), "Delete pipeline...",
      this, &ThisClass::onRemoveLivePipelineClicked);


  menu.exec(menuButton_ctl->mapToGlobal(QPoint(
      menuButton_ctl->width() / 2,
      menuButton_ctl->height() / 2)));

}

void QLivePipelineSelectionWidget::onAddLivePipelineClicked()
{
  QAddPipelineDialogBox dialogbox(this);

  if( dialogbox.exec() == QDialog::Accepted ) {

    QString name =
        dialogbox.selectedPipelineName();

    const QString pipeline_type =
        dialogbox.selectedPipelineClass();

    if( !pipeline_type.isEmpty() ) {

      if( name.isEmpty() ) {

        for( int i = 1; i < 10000; ++i ) {
          name = qsprintf("%s%d", pipeline_type.toUtf8().constData(), i);
          if ( combobox_ctl->findText(name) < 0 ) {
            break;
          }
        }
      }

      else if( combobox_ctl->findText(name) >= 0 ) {

        for( int i = 1; i < 10000; ++i ) {
          QString newname = qsprintf("%s%d", name.toUtf8().constData(), i);
          if ( combobox_ctl->findText(newname) < 0 ) {
            name = newname;
            break;
          }
        }
      }


      c_image_processing_pipeline::sptr pipeline =
          c_image_processing_pipeline::create_instance(pipeline_type.toStdString(),
              name.toStdString());

      if( !pipeline ) {
        QMessageBox::critical(this, "ERROR",
            qsprintf("pipelineCollection_->addPipeline(%s: %s) fails",
                pipeline_type.toUtf8().constData(),
                name.toUtf8().constData()));
      }
      else {
        combobox_ctl->addItem(pipeline->cname(), QVariant::fromValue(pipeline));
        combobox_ctl->setCurrentIndex(combobox_ctl->count() - 1);
        startStop_ctl->setEnabled(_liveThread && _liveThread->isRunning() );
        savePipelines();
      }
    }
  }
}

void QLivePipelineSelectionWidget::onRemoveLivePipelineClicked()
{
  c_image_processing_pipeline::sptr pipeline =
      selectedPipeline();

  if ( pipeline ) {

    const int resp =
        QMessageBox::warning(this, "Remove Live Pipeline",
            qsprintf("Confirmation required:\n"
                "Are you sure to remove pipeline %s ?",
                pipeline->cname()),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);

    if ( resp != QMessageBox::Yes ) {
      return;
    }

    QImageProcessingPipeline * pp =
        dynamic_cast<QImageProcessingPipeline *>(pipeline.get());

    if( pp ) {
      for( int i = 0, n = _settingsWidgets.size(); i < n; ++i ) {
        if( _settingsWidgets[i]->currentPipeline() == pp ) {
          _settingsWidgets[i]->setCurrentPipeline(nullptr);
        }
      }
    }

    int index = combobox_ctl->findText(pipeline->cname());
    if ( index < 0 ) {
      CF_ERROR("ERROR: combobox_ctl->findData(pipeline=%s) fails", pipeline->cname());
    }
    else {
      combobox_ctl->removeItem(index);
    }

    if ( combobox_ctl->currentIndex() < 0 ) {
      startStop_ctl->setEnabled(false);
    }

    savePipelines();
  }
}

void QLivePipelineSelectionWidget::onRenameLivePipelineClicked()
{
  c_image_processing_pipeline::sptr pipeline =
      selectedPipeline();

  if( pipeline ) {

    while (42) {

      const QString currentName =
          pipeline->cname();

      const QString newName =
          QInputDialog::getText(this,
              "Rename Pipeline",
              "New name:",
              QLineEdit::Normal,
              currentName);

      if( newName.isEmpty() ) {
        break;
      }

      if( combobox_ctl->findText(newName) < 0 ) {

        pipeline->set_name(newName.toStdString());

        const int index = combobox_ctl->findText(currentName);
        if( index >= 0 ) {
          combobox_ctl->setItemText(index, newName);
        }

        savePipelines();

        Q_EMIT parameterChanged();
        break;
      }

      QMessageBox::warning(this,
          "Rename Pipeline",
          "Error: Pipeline '%s' already exists.\n"
              "Enter another name.");
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////


QLiveThreadSettingsWidget::QLiveThreadSettingsWidget(QWidget * parent)  :
    ThisClass(nullptr, parent)
{
}

QLiveThreadSettingsWidget::QLiveThreadSettingsWidget(QLivePipelineThread * liveThread, QWidget * parent) :
    Base(parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Default debayer:",
          "Select debayer algorithm for bayer patterns",
          [this](DEBAYER_ALGORITHM v) {
            if ( _opts ) {
              _opts->setDebayer(v);
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            if ( _opts ) {
              *v = _opts->debayer();
              return true;
            }
            return false;
          });

  darkframe_ctl =
      add_browse_for_path("",
          "Dark frame:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & v) {
            if ( _opts ) {
              _opts->setDarkFramePath(v);
            }
          },
          [this](QString * v) {
            if ( _opts ) {
              *v = _opts->darkFramePath();
              return true;
            }
            return false;
          });

  darkFrameScale_ctl =
      add_numeric_box<double>("darkFrameScale",
          "",
          [this](double v) {
            if ( _opts ) {
              _opts->setDarkFrameScale(v);
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->darkFrameScale();
              return true;
            }
            return false;
          });

  updateControls();
}

void QLiveThreadSettingsWidget::setLiveThread(QLivePipelineThread * liveThread)
{
  setOpts(liveThread);
}

QLivePipelineThread * QLiveThreadSettingsWidget::liveThread() const
{
  return opts();
}

QLiveThreadSettingsDialogBox::QLiveThreadSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(ICON_bayer));
  setWindowTitle("Frame Processor Options");

  _layout = new QVBoxLayout(this);
  _layout->addWidget(_setiingsWidget = new QLiveThreadSettingsWidget(this));
}


void QLiveThreadSettingsDialogBox::setLiveThread(QLivePipelineThread * liveThread)
{
  _setiingsWidget->setLiveThread(liveThread);
}

QLivePipelineThread * QLiveThreadSettingsDialogBox::liveThread() const
{
  return _setiingsWidget->liveThread();
}

void QLiveThreadSettingsDialogBox::closeEvent(QCloseEvent * e)
{
  hide();
}

void QLiveThreadSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QLiveThreadSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
bool read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpc) override
{
  cv::Mat inputImage;

  while (_camera->state() == QImagingCamera::State_started) {

    QThread::msleep(20);

    bool haveInputImage = false;

    if( 42 ) {

      QImagingCamera::shared_lock lock(_camera->mutex());

      const std::deque<QCameraFrame::sptr> &deque =
          _camera->deque();

      if( !deque.empty() ) {

        const QCameraFrame::sptr &frame =
            deque.back();

        const int index =
            frame->index();

        if( index > last_frame_index ) {

          last_frame_index = index;
          bpp = frame->bpp();
          colorid = frame->colorid();
          frame->image().copyTo(inputImage);

          haveInputImage = true;
        }
      }
    }

    if( haveInputImage ) {

      _thread->_darkFrameLock.lock();
      if( _thread->_darkFrame.size() == inputImage.size()
          && _thread->_darkFrame.channels() == inputImage.channels() ) {

        inputImage.convertTo(inputImage, _thread->_darkFrame.depth());

        if( _thread->_darkFrameScale != 0 ) {
          cv::subtract(inputImage, _thread->_darkFrame, inputImage);
        }
      }
      _thread->_darkFrameLock.unlock();

      if( _thread->_debayer != DEBAYER_DISABLE && is_bayer_pattern(colorid) ) {

        const DEBAYER_ALGORITHM method =
            inputImage.depth() == CV_32F ?
                DEBAYER_NN2 :
                (DEBAYER_ALGORITHM) _thread->_debayer;

        if( ::debayer(inputImage, inputImage, colorid, method) ) {
          colorid = COLORID_BGR;
        }
      }

      output_frame = inputImage;
      *output_colorid = colorid;
      *output_bpc = bpp;

      return true;
    }
  }

  return false;
}
#endif

} /* namespace serimager */
