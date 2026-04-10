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

  QObject::connect(this, &ThisClass::inputImageReady, this,
      [this]() {
        try {
          Base::updateImage();
        }
        catch (const std::exception& e) {
          CF_ERROR("Exception in updateImage: %s", e.what());
        }
        catch (...) {
          CF_ERROR("Unknown exception in updateImage");
        }
        _canAcceptFrame = true;
      }, Qt::QueuedConnection);

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

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{
  loadSettings();
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
  saveSettings();
}

DEBAYER_ALGORITHM QLivePipelineThread::debayer() const
{
  return _debayer;
}

void QLivePipelineThread::setEnableDarkFrame(bool v)
{
  _enableDarkFrame = v;
  if ( _darkFrame.empty() && !_darkFramePath.isEmpty() ) {
    setDarkFrame(_darkFramePath);
  }
}

bool QLivePipelineThread::enableDarkFrame() const
{
  return _enableDarkFrame;
}

void QLivePipelineThread::setDarkFramePath(const QString & pathfilename)
{
  setDarkFrame(pathfilename);
  saveSettings();
}

const QString & QLivePipelineThread::darkFramePath() const
{
  return _darkFramePath;
}

void QLivePipelineThread::setDarkFrame(const QString & pathfilename)
{
  QMutexLocker lock(&_darkFrameLock);

  _darkFrame.release();

  if( !(_darkFramePath = pathfilename).isEmpty() ) {

    CF_DEBUG("load darkFrame");

    cv::Mat unusedMask;
    if( !load_image(_darkFramePath.toStdString(), _darkFrame, unusedMask) ) {
      CF_ERROR("load_image('%s') fails", _darkFramePath.toUtf8().constData());
    }
    else if( _darkFrame.depth() != CV_32F ) {
      CF_DEBUG("_darkFrame.convertTo");
      _darkFrame.convertTo(_darkFrame, CV_32F, _darkFrameScale);
    }
    else if( _darkFrameScale != 1 ) {
      CF_DEBUG("cv::multiply");
      cv::multiply(_darkFrame, _darkFrameScale, _darkFrame);
    }

    CF_DEBUG("_darkFrame: %dx%d %d channels depth=%d",
        _darkFrame.cols, _darkFrame.rows,
        _darkFrame.channels(),
        _darkFrame.depth());
  }
}

void QLivePipelineThread::setDarkFrameScale(double v)
{
  _darkFrameScale = v;
  if ( !_darkFramePath.isEmpty() ) {
    setDarkFrame(_darkFramePath);
  }
  saveSettings();
}

double QLivePipelineThread::darkFrameScale() const
{
  return _darkFrameScale;
}

void QLivePipelineThread::loadSettings()
{
  QSettings settigs;
  _debayer = (DEBAYER_ALGORITHM) (settigs.value("QLivePipelineThread/debayer", (int) _debayer).toInt());
  _darkFramePath = settigs.value("QLivePipelineThread/darkFramePath", _darkFramePath).toString();
  _darkFrameScale = settigs.value("QLivePipelineThread/darkFrameScale", _darkFrameScale).toDouble();
}

void QLivePipelineThread::saveSettings()
{
  QSettings settings;
  settings.setValue("QLivePipelineThread/debayer", (int)_debayer);
  settings.setValue("QLivePipelineThread/darkFramePath", _darkFramePath);
  settings.setValue("QLivePipelineThread/darkFrameScale", _darkFrameScale);
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

void QLivePipelineThread::setPipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  QMutexLocker lock(&_lock);

  _userPipeline = pipeline;

  Q_EMIT pipelineChanged();

  if( _currentPipeline ) { // Stop current pipeline if running
    _currentPipeline->cancel(true);
    waitUntil(_condvar, _lock, [pp = _currentPipeline]() {
      return !pp->is_running();
    });
  }
}

void QLivePipelineThread::setCurrentPipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  if( QImageProcessingPipeline * qpp = dynamic_cast<QImageProcessingPipeline*>(_currentPipeline.get()) ) {
    qpp->disconnect(this);
  }

  if( (_currentPipeline = pipeline) ) {
    if( QImageProcessingPipeline * qpp = dynamic_cast<QImageProcessingPipeline*>(_currentPipeline.get()) ) {
      // Copy image from pipeline and Notify QLiveDisplay on frame ready
      QObject::connect(qpp, &QImageProcessingPipeline::frameProcessed, this,
          [this]() {
            if ( _display && _display->_canAcceptFrame && _display->currentImageLock().tryLock(5) ) {
              if ( _currentPipeline->get_display(_display->inputImage(), _display->inputMask()) ) {
                _display->_canAcceptFrame = false;
                Q_EMIT _display->inputImageReady();
              }
              _display->currentImageLock().unlock();
            }
          }, Qt::DirectConnection);
    }
  }

  Q_EMIT pipelineChanged();
}

const c_image_processing_pipeline::sptr & QLivePipelineThread::pipeline() const
{
  return _userPipeline;
}

void QLivePipelineThread::run()
{
  struct c_camera_input_source :
      public c_image_input_source
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
    bool read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpp) final
    {
      while (_camera->state() == QImagingCamera::State_started) {

        bool haveInputImage = false;

        if( 42 ) {

          QImagingCamera::shared_lock lock(_camera->mutex());
          const auto &deque = _camera->deque();
          if( !deque.empty() ) {

            const QCameraFrame::sptr &frame = deque.back();
            const int index = frame->index();
            if( index <= last_frame_index ) {
              continue;
            }

            last_frame_index = index;
            frame->image().copyTo(output_frame);

            if ( _liveThread->_enableDarkFrame ) {
              QMutexLocker lock(&_liveThread->_darkFrameLock);

              const cv::Mat & darkFrame = _liveThread->_darkFrame;
              if( darkFrame.size() == output_frame.size() && darkFrame.channels() == output_frame.channels() ) {
                if ( output_frame.depth() != darkFrame.depth() ) {
                  output_frame.convertTo(output_frame, darkFrame.depth());
                }
                cv::subtract(output_frame, darkFrame, output_frame);
              }
            }

            const DEBAYER_ALGORITHM algo = _liveThread->_debayer;
            if ( is_bayer_pattern(frame->colorid()) && algo != DEBAYER_DISABLE  ) {
              ::debayer(output_frame, output_frame, frame->colorid(), algo);
              * output_colorid = colorid = COLORID_BGR;
            }
            else {
              * output_colorid = colorid = frame->colorid();
            }

            * output_bpp = bpp = frame->bpp();
            return true;
          }
        }
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
  /////////////////////

  const QImagingCamera::sptr camera = this->_camera;

  if( camera ) {

    QLivePipeline::sptr dummyPipeline(new QLivePipeline("dummyPipeline", this));

    c_camera_input_sequence::sptr input_sequence(new c_camera_input_sequence(this, camera));

    while (camera->state() == QImagingCamera::State_started) {

      try {

        if ( true ) {
          QMutexLocker lock(&_lock);

          if ( _userPipeline ) {
            setCurrentPipeline(_userPipeline);
          }
          else if ( _currentPipeline != dummyPipeline ) {
            setCurrentPipeline(dummyPipeline);
          }
        }

        try {
          // Blocking call. Will emit QImageProcessingPipeline::frameProcessed() from inside.
          if( _currentPipeline->run(input_sequence) ) {
            CF_DEBUG("_currentPipeline finished");
          }
          else {
            CF_ERROR("_currentPipeline->run() fails");
          }
        }
        catch( const std::exception &e ) {
          CF_ERROR("Exception in activePipeline->run() : %s", e.what());
        }
        catch (...) {
          CF_ERROR("Unknown Exception in activePipeline->run()");
        }

        _condvar.wakeAll();
      }
      catch( const std::exception &e ) {
        CF_ERROR("Exception in live thread : %s", e.what());
      }
      catch (...) {
        CF_ERROR("Unknown Exception in live thread");
      }
    }

    setCurrentPipeline(nullptr);
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
            return _opts ? *v = _opts->debayer(), true : false;
          });

  enable_darkframe_ctl =
      add_checkbox("Subtract dark frame",
          "Enable dark frame subtraction",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setEnableDarkFrame(checked);
            }
          },
          [this](bool * checked) {
            return _opts ? * checked = _opts->enableDarkFrame(), true : false;
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
            return _opts ? *v = _opts->darkFramePath(),  true : false;
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
            return _opts ? *v = _opts->darkFrameScale(), true : false;
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

} /* namespace serimager */

