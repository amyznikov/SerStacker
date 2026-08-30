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


///////////////////////////////////////////////////////////////////////////////////////////////////

template<typename Predicate>
static inline bool waitUntil(QWaitCondition & cond, QMutex & mutex, Predicate && pred, unsigned long timeout = ULONG_MAX)
{
  while (!pred()) {
    if( !cond.wait(&mutex, timeout) ) {
      return pred();
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//QLivePipeline::QLivePipeline(const QString & name, QLivePipelineThread * parent) :
//    Base(name, parent), _liveThread(parent)
//{
//}
//
//bool QLivePipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
//{
//  if( !display_frame.needed() ) {
//    display_frame.release();
//  }
//  else {
//    bool processed = false;
//    if( is_bayer_pattern(_current_colorid) ) {
//      if( _liveThread && _liveThread->debayer() != DEBAYER_DISABLE ) {
//        if( debayer(_current_image, display_frame, _current_colorid, _liveThread->debayer()) ) {
//          processed = true;
//        }
//      }
//    }
//    if( !processed ) {
//      _current_image.copyTo(display_frame);
//    }
//  }
//
//  if( display_mask.needed() ) {
//
//    if ( display_frame.empty() || _current_mask.empty()  ) {
//      display_mask.release();
//    }
//    else if (_current_mask.size() != display_frame.size() ) {
//      cv::resize(_current_mask, display_mask, display_frame.size(), 0, 0, cv::INTER_NEAREST);
//    }
//    else {
//      _current_mask.copyTo(display_mask);
//    }
//  }
//
//  return true;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLiveDisplay::QLiveDisplay(QWidget * parent) :
  Base(parent),
  MtfDisplayFunction(this)
{
  QImageEditor::setDisplayFunction(this);

  QObject::connect(mtfDisplayEvents(), &QMtfDisplayEvents::displayChannelsChanged,
      mtfDisplayEvents(), &QMtfDisplayEvents::parameterChanged);

  QObject::connect(mtfDisplayEvents(), &QMtfDisplayEvents::parameterChanged,
      this, &Base::updateImage);

  QObject::connect(this, &QImageViewer::displayImageChanged,
      mtfDisplayEvents(), &QMtfDisplayEvents::displayImageChanged,
      Qt::QueuedConnection);

//  QObject::connect(this, &ThisClass::inputImageReady, this,
//      [this]() {
//        try {
//          Base::updateImage();
//        }
//        catch (const std::exception& e) {
//          CF_ERROR("Exception in updateImage: %s", e.what());
//        }
//        catch (...) {
//          CF_ERROR("Unknown exception in updateImage");
//        }
//
//        // Make a short delay to allow GUI thread process user I/O
//         _frameReleaseTimer.start();
//        // _canAcceptFrame = true;
//      }, Qt::QueuedConnection);


  QObject::connect(this, &Base::onPopulateContextMenu,
      this, &Base::populateContextMenu);

  createShapes();
}

QLiveDisplay::~QLiveDisplay()
{
}

void QLiveDisplay::createShapes()
{
  if( !_roiShape ) {

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

    _roiShape = new QGraphicsRectShape(rect);
    _roiShape->setResizable(true);
    _roiShape->setSnapToPixelGrid(true);
    _roiShape->setFlag(QGraphicsItem::ItemIsMovable, true);
    _roiShape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    _roiShape->setCosmeticPen(Qt::red);
    _roiShape->setVisible(false);
    _scene->addItem(_roiShape);
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

QGraphicsRectShape * QLiveDisplay::roiShape() const
{
  return _roiShape;
}

QGraphicsLineShape * QLiveDisplay::lineShape() const
{
  return _lineShape;
}

QGraphicsTargetShape * QLiveDisplay::targetShape() const
{
  return _targetShape;
}

void QLiveDisplay::setInputSource(INPUT_SOURCE v)
{
  _inputSource = v;
  toggleUpdateTimer();
}

QLiveDisplay::INPUT_SOURCE QLiveDisplay::inputSource() const
{
  return _inputSource;
}

void QLiveDisplay::setCamera(const QImagingCamera::sptr & camera)
{
  if( _camera ) {
    disconnect(_camera.get(), nullptr,
        this, nullptr);
  }

  if( (_camera = camera) ) {
    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);
  }

  toggleUpdateTimer();
}

const QImagingCamera::sptr& QLiveDisplay::camera() const
{
  return _camera;
}

void QLiveDisplay::onCameraStateChanged(QImagingCamera::State /*oldState*/, QImagingCamera::State /*newState*/)
{
  _lastCameraFrameIndex = -1;
  toggleUpdateTimer();
}

void QLiveDisplay::setCurrentPipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  _lastCameraFrameIndex = -1;
  // _pipelineFrameReady = false;

  if( QImageProcessingPipeline * qpp = dynamic_cast<QImageProcessingPipeline*>(_currentPipeline.get()) ) {
    qpp->disconnect(this);
  }

  if( (_currentPipeline = pipeline) ) {
    if( QImageProcessingPipeline * qpp = dynamic_cast<QImageProcessingPipeline*>(_currentPipeline.get()) ) {
      QObject::connect(qpp, &QImageProcessingPipeline::frameProcessed, this,
          [this]() {
            _pipelineFrameReady = true;
          }, Qt::QueuedConnection);
    }
  }

  toggleUpdateTimer();
}

const c_image_processing_pipeline::sptr & QLiveDisplay::currentPipeline() const
{
  return _currentPipeline;
}

void QLiveDisplay::setPaused(bool v)
{
  _paused = v;
  toggleUpdateTimer();
}

bool QLiveDisplay::paused() const
{
  return _paused;
}

void QLiveDisplay::setDebayer(DEBAYER_ALGORITHM algo)
{
  _debayer_algo = algo;
  saveSettings();
}

DEBAYER_ALGORITHM QLiveDisplay::debayer() const
{
  return _debayer_algo;
}

void QLiveDisplay::setCameraUpdateInterval(int v)
{
  _cameraUpdateInterval = v;
  toggleUpdateTimer();
}

int QLiveDisplay::cameraUpdateInterval() const
{
  return _cameraUpdateInterval;
}

void QLiveDisplay::setPipelineUpdateInterval(int v)
{
  _pipelineUpdateInterval = v;
  toggleUpdateTimer();
}

int QLiveDisplay::pipelineUpdateInterval() const
{
  return _pipelineUpdateInterval;
}

void QLiveDisplay::toggleUpdateTimer()
{
  int requiredUpdateInterval = 0;

  if( !_paused ) {
    if( _currentPipeline && _inputSource == INPUT_SOURCE_PIPELINE ) {
      requiredUpdateInterval = std::max(10, _pipelineUpdateInterval);
    }
    else if( _camera && _camera->state() == QImagingCamera::State_started ) {
      requiredUpdateInterval = std::max(10, _cameraUpdateInterval);
    }
  }

  if( requiredUpdateInterval ) {
    if( _updateTimerId < 0 || requiredUpdateInterval != _currentUpdateInterval ) {
      if( _updateTimerId >= 0 ) {
        killTimer(_updateTimerId);
      }
      _updateTimerId = startTimer(requiredUpdateInterval);
      _currentUpdateInterval = requiredUpdateInterval;
      _lastCameraFrameIndex = -1;
    }
  }
  else if( _updateTimerId >= 0 ) {
    killTimer(_updateTimerId);
    _updateTimerId = -1;
    _currentUpdateInterval = 0;
     _lastCameraFrameIndex = -1;
  }
}

void QLiveDisplay::timerEvent(QTimerEvent *e)
{
  if (e->timerId() != _updateTimerId) {
    Base::timerEvent(e);
  }

  // Pipeline display has higher priority
  cv::Mat image, mask;
  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;

  if ( _currentPipeline && _inputSource == INPUT_SOURCE_PIPELINE ) {
    if( _pipelineFrameReady ) {
      _currentPipeline->get_display(image, mask);
    }
  }
  else if ( _camera && _camera->state() == QImagingCamera::State_started  ) {
    if( true ) {
      QImagingCamera::shared_lock lock(_camera->mutex());
      const auto & deque = _camera->deque();
      if( !deque.empty() ) {
        const QCameraFrame::sptr & freshestFrame = deque.back();
        const int freshestIndex = freshestFrame->index();
        if( freshestIndex > _lastCameraFrameIndex ) {
          _lastCameraFrameIndex = freshestIndex;
          freshestFrame->image().copyTo(image);
          bpp = freshestFrame->bpp();
          colorid = freshestFrame->colorid();
        }
      }
    }
  }

  if( !image.empty() ) {
    try {
      if( !is_bayer_pattern(colorid) ) {
        Base::editImage(image, mask, false);
      }
      else {
        ::debayer(image, Base::_inputImage, colorid, _debayer_algo);
        Base::_inputMask = mask;
        Base::updateImage();
      }
    }
    catch( const std::exception & e ) {
      CF_ERROR("Exception in updateImage: %s", e.what());
    }
    catch( ... ) {
      CF_ERROR("Unknown exception in updateImage");
    }
  }
}

void QLiveDisplay::loadSettings(const QString & prefix)
{
  const QSettings settings;
  loadSettings(settings, prefix);
}

void QLiveDisplay::saveSettings(const QString & prefix)
{
  QSettings settings;
  saveSettings(settings, prefix);
}

void QLiveDisplay::loadSettings(const QSettings & settings, const QString & prefix)
{
  const QString PREFIX = prefix.isEmpty() ? "QLiveDisplay" : prefix;
  _debayer_algo = (DEBAYER_ALGORITHM) settings.value(QString("%1/debayer").arg(prefix), (int) _debayer_algo).toInt();
  _cameraUpdateInterval = settings.value(QString("%1/cameraUpdateInterval").arg(prefix), (int) _cameraUpdateInterval).toInt();
  _pipelineUpdateInterval = settings.value(QString("%1/pipelineUpdateInterval").arg(prefix), (int) _pipelineUpdateInterval).toInt();
}

void QLiveDisplay::saveSettings(QSettings & settings, const QString & prefix)
{
  const QString PREFIX = prefix.isEmpty() ? "QLiveDisplay" : prefix;
  settings.setValue(QString("%1/debayer").arg(prefix), (int) _debayer_algo);
  settings.setValue(QString("%1/cameraUpdateInterval").arg(prefix), (int) _cameraUpdateInterval);
  settings.setValue(QString("%1/pipelineUpdateInterval").arg(prefix), (int) _pipelineUpdateInterval);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{
}

QLivePipelineThread::~QLivePipelineThread()
{
  if ( true ) {
    QMutexLocker lock(&_lock);
    if( _currentPipeline ) {
      // Stop current pipeline if running
      _currentPipeline->cancel(true);
    }
    _stopRequested = true;
    _condvar.wakeAll();
  }

  while (isRunning()) {
    QThread::msleep(200);
  }
}


void QLivePipelineThread::setFrameQualityEstimator(QFrameQualityEstimation * estimator)
{
  QMutexLocker lock(&_lock);
  _qualityEstimator = estimator;
}

QFrameQualityEstimation* QLivePipelineThread::frameQualityEstimator() const
{
  return _qualityEstimator;
}

void QLivePipelineThread::setCamera(const QImagingCamera::sptr & camera)
{
  bool shouldRun = false;

  if( true ) {
    QMutexLocker lock(&_lock);
    if( _currentPipeline ) {
      // Stop current pipeline if running
      _currentPipeline->cancel(true);
    }

    if( _camera ) {
      disconnect(_camera.get(), nullptr, this, nullptr);
    }

    if( (_camera = camera) ) {
      connect(_camera.get(), &QImagingCamera::stateChanged,
          this, &ThisClass::onCameraStateChanged,
          Qt::QueuedConnection);

      if( _camera->state() == QImagingCamera::State_started ) {
        shouldRun = true;
      }
    }
    _condvar.wakeAll();
  }

  if( shouldRun && !isRunning() ) {
    start();
  }
}

const QImagingCamera::sptr & QLivePipelineThread::camera() const
{
  return _camera;
}

void QLivePipelineThread::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  _condvar.wakeAll();
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

void QLivePipelineThread::setPipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  QMutexLocker lock(&_lock);

  if( _currentPipeline ) {
    // Stop current pipeline if running
    _currentPipeline->cancel(true);
  }

  _currentPipeline = pipeline;
  _condvar.wakeAll();

  Q_EMIT pipelineChanged();
}

const c_image_processing_pipeline::sptr & QLivePipelineThread::pipeline() const
{
  return _currentPipeline;
}

void QLivePipelineThread::run()
{
  struct c_camera_input_source : public c_image_input_source
  {
    typedef c_camera_input_source this_class;
    typedef c_image_input_source base;
    typedef std::shared_ptr<this_class> sptr;

    QLivePipelineThread * _liveThread;
    QImagingCamera::sptr _camera;
    int last_frame_index = -1;
    int bpp = -1;
    COLORID colorid = COLORID_UNKNOWN;

    c_camera_input_source(QLivePipelineThread * thread, const QImagingCamera::sptr & camera) :
      base(""), _liveThread(thread),_camera(camera)
    {}
    bool open() final {
      return _camera->state() == QImagingCamera::State_started;
    }
    void close() final {
    }
    bool seek(int pos) final {
      return true;
    }
    int curpos() final {
      return 0;
    }
    bool is_open() const final {
      return _camera->state() == QImagingCamera::State_started;
    }
    bool read(cv::OutputArray output_image, cv::OutputArray output_mask,
        enum COLORID * output_colorid,
        int * output_bpp) final
    {
      while (_camera->state() == QImagingCamera::State_started) {

        std::vector<QCameraFrame::sptr> local_frames;
        int freshest_index = -1;

        // Quick grab of camera frames under a short shared_lock
        if( true ) {
          QImagingCamera::shared_lock lock(_camera->mutex());
          const auto & deque = _camera->deque();
          if( deque.empty() ) {
            _camera->condvar().wait(lock);
            continue;
          }

          const QCameraFrame::sptr & freshest_frame = deque.back();
          freshest_index = freshest_frame->index();

          if( freshest_index <= last_frame_index ) {
            _camera->condvar().wait(lock);
            continue;
          }

          // Activate the estimator only if a lag is detected AND this option is enabled from GUI

          const int LAG_THRESHOLD = 3;
          const int frames_in_queue = freshest_index - last_frame_index;

          QFrameQualityEstimation * estimator = _liveThread->frameQualityEstimator();

          if( frames_in_queue <= LAG_THRESHOLD || !estimator || !estimator->isEnabled() ) {
            // no lags detected or estimator is disabled - just take the most recent frame
            local_frames.push_back(freshest_frame);
          }
          else {
            local_frames.reserve(deque.size());
            for( const auto & frame : deque ) {
              if( frame->index() > last_frame_index ) {
                local_frames.emplace_back(frame);
              }
            }
          }
        }

        if (local_frames.empty()) {
          continue;
        }

        // Select target frame
        QCameraFrame::sptr selected_frame = nullptr;

        if( local_frames.size() < 2 ) {
          // Standard mode without lags - take the single available frame
          selected_frame = local_frames.front();
        }
        else {
          // Request the pointer again (in case the user switched it to the GUI in a split second)
          QFrameQualityEstimation * estimator = _liveThread->frameQualityEstimator();

          if( estimator && estimator->isEnabled() ) {
            // Quality calculation is performed here without blocking the camera queue
            const int best_local_idx = estimator->estimateFrameQuality(local_frames);
            if( best_local_idx >= 0 && best_local_idx < (int) local_frames.size() ) {
              selected_frame = local_frames[best_local_idx];
            }
          }

          // If the estimator suddenly returns an error, take the latest one from the pack
          if( !selected_frame ) {
            selected_frame = local_frames.back();
          }
        }

        // Advance the index to the most recent frame,
        // thereby discarding (dropping) all the missed ones
        last_frame_index = freshest_index;
        selected_frame->image().copyTo(output_image);
        *output_bpp = bpp = selected_frame->bpp();
        *output_colorid = colorid = selected_frame->colorid();

        return true;
      }

      return false;
    }
  };

  struct c_camera_input_sequence : public c_input_sequence
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
      set_name(get_file_name(camera->name().toStdString()));
    }
    bool is_live() const final {
      return true;
    }
    bool open() final {
      return camera_source->is_open();
    }
    void close(bool /*clear */= false) final {
    }
    bool seek(int pos) final {
      return true;
    }
    bool is_open() const final {
      return camera_source->is_open();
    }
  };

  CF_DEBUG("enter");

  while (!_stopRequested) {
     QImagingCamera::sptr camera;
     c_image_processing_pipeline::sptr currentPipeline;

     if ( true ) {
       QMutexLocker lock(&_lock);
       while (!_stopRequested && (!_currentPipeline || !_camera || _camera->state() != QImagingCamera::State_started)) {
         _condvar.wait(&_lock);
       }
       if (_stopRequested) {
         break;
       }
       camera = _camera;
       currentPipeline = _currentPipeline;
     }


     c_camera_input_sequence::sptr input_sequence(new c_camera_input_sequence(this, camera));

     // Blocking call. Will emit QImageProcessingPipeline::frameProcessed() from inside.
     CF_DEBUG("call currentPipeline->run()");
     if (currentPipeline->run(input_sequence)) {
       CF_DEBUG("currentPipeline finished");
     }
     else {
       CF_ERROR("currentPipeline->run() fails");
     }

     _condvar.wakeAll();
   }

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
  QFormLayout * _form = nullptr;
  QHBoxLayout * _hbox = nullptr;
  QLineEditBox * pipelineName_ctl = nullptr;
  QComboBox * pipelineTypeSelector_ctl = nullptr;
  QLabel * pipelineTooltop_ctl = nullptr;
  QPushButton * _btnOk = nullptr;
  QPushButton * _btnCancel = nullptr;
};

QAddPipelineDialogBox::QAddPipelineDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Select live pipeline");

  _form = new QFormLayout(this);

  _form->addRow("Name:", pipelineName_ctl = new QLineEditBox(this));
  _form->addRow("Type:", pipelineTypeSelector_ctl = new QComboBox(this));
  _form->addRow(pipelineTooltop_ctl = new QLabel(this));

  _form->addRow(_hbox = new QHBoxLayout());
  _hbox->addWidget(_btnOk = new QPushButton("OK"));
  _hbox->addWidget(_btnCancel = new QPushButton("Cancel"));

  for( const auto &item : c_image_processing_pipeline::registered_classes() ) {
    pipelineTypeSelector_ctl->addItem(item.class_name.c_str(),
        QVariant::fromValue(QString(item.tooltip.c_str())));
  }

  pipelineTooltop_ctl->setTextFormat(Qt::RichText);
  pipelineTooltop_ctl->setText(pipelineTypeSelector_ctl->currentData().toString());

  QObject::connect(_btnOk, &QPushButton::clicked,
      this, [this]() {
        if ( pipelineTypeSelector_ctl->currentText().isEmpty() ) {
          pipelineTypeSelector_ctl->setFocus();
        }
        else {
          Base::accept();
        }
      });

  QObject::connect(_btnCancel, &QPushButton::clicked,this, &Base::reject);

  QObject::connect(pipelineTypeSelector_ctl,
      static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, [this](int index) {
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

static std::string _default_config_filename =
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
    filename = _default_config_filename;
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

  c_config_setting section = cfg.root().get("items");
  if( !section || !section.isList() ) {
    CF_FATAL("section 'items' is not found in file '%s''",
        filename.c_str());
    return;
  }

  combobox_ctl->clear();

  const int N = section.length();
  for( int i = 0; i < N; ++i ) {

    c_config_setting item = section.get_element(i);
    if( item && item.isGroup() ) {

      std::string class_name;

      if( !load_settings(item, "class_name", &class_name) || class_name.empty() ) {
        CF_ERROR("load_settings(class_name) fails for item %d", i);
        continue;
      }

      c_image_processing_pipeline::sptr obj =
          c_image_processing_pipeline::create_instance(class_name, "",
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
    filename = _default_config_filename;
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

  c_config_setting section = cfg.root().add_list("items");
  const int N = combobox_ctl->count();
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
      //menuButton_ctl->setEnabled(false);
      startStop_ctl->setIcon(getIcon(ICON_stop));
      startStop_ctl->setEnabled(true);
    }
    else {
      combobox_ctl->setEnabled(true);
      //menuButton_ctl->setEnabled(true);
      startStop_ctl->setIcon(getIcon(ICON_start));
      startStop_ctl->setEnabled(selectedPipeline() != nullptr);
    }

    setEnabled(true);
  }
}


void QLivePipelineSelectionWidget::onPipelinesComboboxCurrentIndexChanged(int)
{
  QImageProcessingPipeline *p = nullptr;

  QPipelineSettingsWidget *currentWidget = dynamic_cast<QPipelineSettingsWidget*>(scrollArea_ctl->widget());
  if( currentWidget ) {
    currentWidget->setCurrentPipeline(nullptr);
    currentWidget = nullptr;
  }

  c_image_processing_pipeline::sptr pipeline = selectedPipeline();

  if( pipeline && (p = dynamic_cast<QImageProcessingPipeline*>(pipeline.get())) ) {

    const QString className = pipeline->get_class_name().c_str();
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
          this, [this]() {
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
  if ( !_liveThread ) {
    return;
  }


  const auto activePipeline = _liveThread->pipeline();
  if (!activePipeline) {

    QMenu menu;

    menu.addAction(getIcon(ICON_add), "Add pipeline...",
        this, &ThisClass::onAddLivePipelineClicked);

    menu.addAction(getIcon(ICON_rename), "Rename pipeline......",
        this, &ThisClass::onRenameLivePipelineClicked);

    menu.addSeparator();

    menu.addAction(getIcon(ICON_delete), "Delete pipeline...",
        this, &ThisClass::onRemoveLivePipelineClicked);

    if ( !menu.isEmpty() ) {
      menu.exec(menuButton_ctl->mapToGlobal(QPoint(
          menuButton_ctl->width() / 2,
          menuButton_ctl->height() / 2)));
    }

  }
  else {
    CF_DEBUG("pipeline is active");

    const c_enum_member *display_types = activePipeline->get_preview_displays();

    if( display_types ) {

      QMenu menu;

      const int display_type = activePipeline->preview_display();
      int items_count = 0;

      for( ; !display_types->name.empty(); ++display_types ) {
        QAction *action = menu.addAction(display_types->name.c_str());
        action->setData(QVariant::fromValue(display_types->value));
        action->setCheckable(true);
        action->setChecked(display_type == display_types->value);
        ++items_count;
      }

      if( items_count > 1 ) {

        QAction * action =
            menu.exec(menuButton_ctl->mapToGlobal(QPoint(menuButton_ctl->width() / 2,
                menuButton_ctl->height() / 2)));

        if( action ) {
          const int selected_display_type = action->data().value<int>();
          if( display_type != selected_display_type ) {
            activePipeline->set_preview_display(selected_display_type);
          }
        }
      }
    }

  }

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
  c_image_processing_pipeline::sptr pipeline = selectedPipeline();
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
  c_image_processing_pipeline::sptr pipeline = selectedPipeline();

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


QLiveDisplaySettingsWidget::QLiveDisplaySettingsWidget(QWidget * parent)  :
    ThisClass(nullptr, parent)
{
}

QLiveDisplaySettingsWidget::QLiveDisplaySettingsWidget(QLiveDisplay * liveDisplay, QWidget * parent) :
    Base(parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Default debayer:",
          "Select debayer algorithm for bayer patterns",
          [this](DEBAYER_ALGORITHM v) {
            if ( _opts ) {
              _opts->setDebayer(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            return _opts ? *v = _opts->debayer(), true : false;
          });

  cameraUpdateInterval_ctl =
      add_numeric_box<int>("cameraUpdateInterval [ms]:",
          "",
          [this](int v) {
            if ( _opts && _opts->cameraUpdateInterval() != v ) {
              _opts->setCameraUpdateInterval(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            return _opts ? *v = _opts->cameraUpdateInterval(), true : false;
          });

  pipelineUpdateInterval =
      add_numeric_box<int>("pipelineUpdateInterval [ms]:",
          "",
          [this](int v) {
            if ( _opts && _opts->pipelineUpdateInterval() != v ) {
              _opts->setPipelineUpdateInterval(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            return _opts ? *v = _opts->pipelineUpdateInterval(), true : false;
          });

  updateControls();
}

void QLiveDisplaySettingsWidget::setLiveDisplay(QLiveDisplay * liveThread)
{
  setOpts(liveThread);
}

QLiveDisplay * QLiveDisplaySettingsWidget::liveDisplay() const
{
  return opts();
}

QLiveDisplaySettingsDialogBox::QLiveDisplaySettingsDialogBox(QWidget * parent) :
    Base("Live Display Options", parent)
{
  setWindowIcon(getIcon(ICON_bayer));
}

void QLiveDisplaySettingsDialogBox::setLiveDisplay(QLiveDisplay * liveDisplay)
{
  Base::setOpts(liveDisplay);
}

QLiveDisplay * QLiveDisplaySettingsDialogBox::liveDisplay() const
{
  return Base::opts();
}

void QLiveDisplaySettingsDialogBox::closeEvent(QCloseEvent * e)
{
  hide();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */

