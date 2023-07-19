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
  c_unique_lock lock(mutex_);
  Base::getInputDataRange(minval, maxval);
}

void QLiveDisplayMtfFunction::getInputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
{
  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    mutex_.lock();
    isBusy_ = true;

    const cv::Mat & currentImage =
        imageViewer_->currentImage();

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else {
      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, scale, offset);
    }

    imageViewer_->currentMask().copyTo(mask);
    mutex_.unlock();

    create_histogram(image, mask,
        H,
        output_hmin, output_hmax,
        256,
        false,
        false);

    mutex_.lock();
    isBusy_ = false;
    mutex_.unlock();

    (*output_hmin -= offset) /= scale;
    (*output_hmax -= offset) /= scale;
  }
}

void QLiveDisplayMtfFunction::getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
{
//  Base::getOutputHistogramm(H, output_hmin, output_hmax);
//  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    mutex_.lock();
    isBusy_ = true;

    const cv::Mat & currentImage =
        imageViewer_->mtfImage();

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else {
      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, scale, offset);
    }

    imageViewer_->currentMask().copyTo(mask);
    mutex_.unlock();

    create_histogram(image, mask,
        H,
        output_hmin, output_hmax,
        256,
        false,
        false);

    (*output_hmin -= offset) /= scale;
    (*output_hmax -= offset) /= scale;

    mutex_.lock();
    isBusy_ = false;
    mutex_.unlock();

  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////


QLiveDisplay::QLiveDisplay(QWidget * parent) :
    Base(parent),
    mtfDisplayFunction_(this)
{
  setDisplayFunction(&mtfDisplayFunction_);

  connect(this, &ThisClass::pixmapChanged,
      this, &ThisClass::onPixmapChanged,
      Qt::QueuedConnection);

  connect(&mtfDisplayFunction_, &QMtfDisplay::parameterChanged,
      [this]() {
        if ( !mtfDisplayFunction_.isBusy() ) {
          Base::updateImage();
        }
      });

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
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
  if( !rectShape_ ) {

    QRectF rect;

    if( currentImage_.empty() ) {
      rect.setRect(0, 0, 400, 400);
    }
    else {

      rect.setRect(0, 0, currentImage_.cols, currentImage_.rows);

      if( rect.width() > 400 ) {
        rect.setX((rect.left() + rect.right()) / 2 - 200);
        rect.setWidth(400);
      }

      if( rect.height() > 400 ) {
        rect.setY((rect.top() + rect.bottom()) / 2 - 200);
        rect.setHeight(400);
      }
    }

    rectShape_ = new QGraphicsRectShape(rect);
    rectShape_->setResizable(true);
    rectShape_->setSnapToPixelGrid(true);
    rectShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    rectShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    rectShape_->setCosmeticPen(Qt::red);
    rectShape_->setVisible(false);
    scene_->addItem(rectShape_);
  }

  if( !lineShape_ ) {

    lineShape_ = new QGraphicsLineShape(-100, -100, 100, 100);
    lineShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    lineShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    lineShape_->setCosmeticPen(Qt::green);
    lineShape_->setVisible(false);
    scene_->addItem(lineShape_);
  }

  if( !targetShape_ ) {
    targetShape_ = new QGraphicsTargetShape();
    targetShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    targetShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    targetShape_->setCosmeticPen(Qt::red);
    targetShape_->setVisible(false);
    scene_->addItem(targetShape_);
  }

}

const QLiveDisplayMtfFunction * QLiveDisplay::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

QLiveDisplayMtfFunction * QLiveDisplay::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}


void QLiveDisplay::setFrameProcessor(const c_image_processor::sptr & processor)
{
  mtfDisplayFunction_.mutex().lock();
  Base::current_processor_ = processor;

  if ( !mtfDisplayFunction_.isBusy() ) {
    updateImage();
  }

  mtfDisplayFunction_.mutex().unlock();
}

QGraphicsRectShape * QLiveDisplay::rectShape() const
{
  return rectShape_;
}

QGraphicsLineShape * QLiveDisplay::lineShape() const
{
  return lineShape_;
}

QGraphicsTargetShape * QLiveDisplay::targetShape() const
{
  return targetShape_;
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
  if( e->timerId() == update_display_timer_id_ ) {

    if( update_display_required_ ) {

      c_unique_lock mtflock(mtfDisplayFunction_.mutex());
      if( !mtfDisplayFunction_.isBusy() ) {

        update_display_required_ = false;

        live_pipeline_lock_.lock();

        if( live_pipeline_ && live_pipeline_->get_display_image(inputImage_, inputMask_) ) {
          updateCurrentImage();
        }

        live_pipeline_lock_.unlock();
      }
    }

    e->ignore();
    return;
  }

  Base::timerEvent(e);
}


void QLiveDisplay::onStartUpdateLiveDisplayTimer()
{
  update_display_timer_id_ = startTimer(100);
}

void QLiveDisplay::onStopUpdateLiveDisplayTimer()
{
  if ( update_display_timer_id_ ) {
    killTimer(update_display_timer_id_);
    update_display_timer_id_ = 0;
  }
}

void QLiveDisplay::setLivePipeline(const c_image_processing_pipeline::sptr & pipeline)
{
  live_pipeline_lock_.lock();

  if( live_pipeline_ ) {

    if( true ) {
      c_unique_lock mtflock(mtfDisplayFunction_.mutex());
      if( !mtfDisplayFunction_.isBusy() && live_pipeline_->get_display_image(inputImage_, inputMask_) ) {
        updateCurrentImage();
      }
    }

    if( update_display_timer_id_ ) {
      Q_EMIT stopUpdateLiveDisplayTimer();
    }

    QImageProcessingPipeline *pp =
        dynamic_cast<QImageProcessingPipeline*>(live_pipeline_.get());
    if( pp ) {
      pp->disconnect(this);
    }
  }

  if( (live_pipeline_ = pipeline) ) {

    QImageProcessingPipeline *pp =
        dynamic_cast<QImageProcessingPipeline*>(live_pipeline_.get());

    if( pp ) {
      connect(pp, &QImageProcessingPipeline::frameProcessed,
          [this]() {
            // give chance to GUI thread to call get_display_image()
            update_display_required_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          });
    }

    update_display_required_ = true;
    Q_EMIT startUpdateLiveDisplayTimer();
  }

  live_pipeline_lock_.unlock();
}


void QLiveDisplay::onPixmapChanged()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());
  scene()->setImage(pixmap_);
  Q_EMIT currentImageChanged();
}

void QLiveDisplay::updateCurrentImage()
{
  if( !current_processor_ || current_processor_->empty() ) {
    current_image_lock lock(this);
    inputImage_.copyTo(currentImage_);
    inputMask_.copyTo(currentMask_);
  }
  else {

    cv::Mat tmp_image, tmp_mask;

    inputImage_.copyTo(tmp_image);
    inputMask_.copyTo(tmp_mask);
    current_processor_->process(tmp_image, tmp_mask);

    current_image_lock lock(this);
    currentImage_ = tmp_image;
    currentMask_ = tmp_mask;
  }

  mtfDisplayFunction_.createDisplayImage(
      currentImage_,
      currentMask_,
      mtfImage_,
      displayImage_,
      CV_8U);

  pixmap_ =
      createPixmap(displayImage_, true,
          Qt::NoFormatConversion |
              Qt::ThresholdDither |
              Qt::ThresholdAlphaDither |
              Qt::NoOpaqueDetection);

  Q_EMIT pixmapChanged();

  if ( !mtfDisplayFunction_.isBusy() ) {
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
    if ( camera_ ) {
      camera_->disconnect();
    }
    QThread::msleep(20);
  }
}


void QLivePipelineThread::setDebayer(DEBAYER_ALGORITHM algo)
{
  debayer_ = algo;
  save_settings();
}

DEBAYER_ALGORITHM QLivePipelineThread::debayer() const
{
  return debayer_;
}

void QLivePipelineThread::setDarkFramePath(const QString & pathfilename)
{
  setDarkFrame(pathfilename);
  save_settings();
}

const QString & QLivePipelineThread::darkFramePath() const
{
  return darkFramePath_;
}

void QLivePipelineThread::setDarkFrame(const QString & pathfilename)
{
  darkFrameLock_.lock();

  darkFrame_.release();

  if ( !(darkFramePath_ = pathfilename).isEmpty() ) {

    cv::Mat ignoreMaskIfExists;
    if ( !load_image(darkFramePath_.toStdString(), darkFrame_, ignoreMaskIfExists) ) {
      CF_ERROR("load_image('%s') fails", darkFramePath_.toUtf8().constData());
    }
    else if ( darkFrameScale_ != 0 ) {
      cv::multiply(darkFrame_, darkFrameScale_, darkFrame_);
    }
    //    else if (darkFrame_.depth() == CV_32F ) {
    //    }
  }

  darkFrameLock_.unlock();
}

void QLivePipelineThread::setDarkFrameScale(double v)
{
  darkFrameScale_ = v;
  if ( !darkFramePath_.isEmpty() ) {
    setDarkFrame(darkFramePath_);
  }
  save_settings();
}

double QLivePipelineThread::darkFrameScale() const
{
  return darkFrameScale_;
}

void QLivePipelineThread::load_settings()
{
  QSettings settigs;
  debayer_ = (DEBAYER_ALGORITHM) (settigs.value("QLivePipelineThread/debayer", (int) debayer_).toInt());
  darkFramePath_ = settigs.value("QLivePipelineThread/darkFramePath", darkFramePath_).toString();
  darkFrameScale_ = settigs.value("QLivePipelineThread/darkFrameScale", darkFrameScale_).toDouble();
}

void QLivePipelineThread::save_settings()
{
  QSettings settigs;
  settigs.setValue("QLivePipelineThread/debayer", (int)debayer_);
  settigs.setValue("QLivePipelineThread/darkFramePath", darkFramePath_);
  settigs.setValue("QLivePipelineThread/darkFrameScale", darkFrameScale_);
}

void QLivePipelineThread::setDisplay(QLiveDisplay * display)
{
  display_ = display;
}

QLiveDisplay* QLivePipelineThread::display() const
{
  return display_;
}

void QLivePipelineThread::setCamera(const QImagingCamera::sptr & camera)
{
  // finish(true);

  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }

  if( (camera_ = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    if( camera_->state() == QImagingCamera::State_started ) {
      start();
    }
  }
}

const QImagingCamera::sptr & QLivePipelineThread::camera() const
{
  return camera_;
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
  unique_lock lock(mutex_);

  c_image_processing_pipeline::sptr current_pipeline =
      this->pipeline_;

  if( current_pipeline ) {

    this->pipeline_.reset();

    current_pipeline->cancel(true);
    condvar_.wait(lock, [current_pipeline]() {
      return !current_pipeline->is_running();
    });
  }

  this->pipeline_ = pipeline;

  if ( !isRunning() ) {
    Q_EMIT pipelineChanged();
  }

  return true;
}

const c_image_processing_pipeline::sptr & QLivePipelineThread::pipeline() const
{
  return pipeline_;
}

void QLivePipelineThread::run()
{
  struct c_camera_input_source :
      public c_input_source
  {
    typedef c_camera_input_source this_class;
    typedef c_input_source base;
    typedef std::shared_ptr<this_class> sptr;

    QLivePipelineThread * thread_;
    QImagingCamera::sptr camera_;
    int last_frame_index = -1;
    int bpp = -1;
    COLORID colorid = COLORID_UNKNOWN;

    c_camera_input_source(QLivePipelineThread * thread, const QImagingCamera::sptr & camera) :
        base(c_input_source::CAMERA, ""),
        thread_(thread),
        camera_(camera)
    {
    }

    bool open() override
    {
      return camera_->state() == QImagingCamera::State_started;
    }

    void close()  override
    {
    }

    bool seek(int pos) override
    {
      return true;
    }

    bool is_open() const override
    {
      return camera_->state() == QImagingCamera::State_started;
    }

    bool read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpc) override
    {
      cv::Mat inputImage;

      while (camera_->state() == QImagingCamera::State_started) {

        QThread::msleep(30);

        bool haveInputImage = false;

        if( 42 ) {

          QImagingCamera::shared_lock lock(camera_->mutex());

          const std::deque<QCameraFrame::sptr> &deque =
              camera_->deque();

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

          thread_->darkFrameLock_.lock();
          if( thread_->darkFrame_.size() == inputImage.size()
              && thread_->darkFrame_.channels() == inputImage.channels() ) {

            inputImage.convertTo(inputImage, thread_->darkFrame_.depth());

            if( thread_->darkFrameScale_ != 0 ) {
              cv::subtract(inputImage, thread_->darkFrame_, inputImage);
            }
          }
          thread_->darkFrameLock_.unlock();

          if( thread_->debayer_ != DEBAYER_DISABLE && is_bayer_pattern(colorid) ) {

            const DEBAYER_ALGORITHM method =
                inputImage.depth() == CV_32F ?
                    DEBAYER_NN2 :
                    (DEBAYER_ALGORITHM) thread_->debayer_;

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
      all_sources_.emplace_back(camera_source);
      enabled_sources_.emplace_back(camera_source);
      current_source_ = 0;
      current_global_pos_ = 0;

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

  const QImagingCamera::sptr camera = camera_;

  if( camera ) {

    c_camera_input_sequence::sptr input_sequence(
        new c_camera_input_sequence(this,
            camera));

    while (camera->state() == QImagingCamera::State_started) {

      QImageProcessingPipeline * pp = nullptr;

      try {

        mutex_.lock();
        c_image_processing_pipeline::sptr pipeline =
            this->pipeline_;
        mutex_.unlock();

        display_->setLivePipeline(pipeline);

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
            unique_lock lock(mutex_);
            this->pipeline_.reset();
          }

          condvar_.notify_all();
        }
        else {

          //cv::Mat image;
          COLORID colorid;
          int bpp;

          while (input_sequence->camera_source->read(display_->inputImage(), &colorid, &bpp)) {

            if( true ) {
              c_unique_lock mtflock(display_->mtfDisplayFunction()->mutex());
              if( !display_->mtfDisplayFunction()->isBusy() ) {
                display_->updateCurrentImage();
              }
            }

            if( true ) {
              unique_lock lock(mutex_);
              if( pipeline_ != nullptr ) {
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

      display_->setLivePipeline(nullptr);
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

  layout_ = new QVBoxLayout(this);

  layout_->addWidget(toolbar_ctl = new QToolBar(this), 0, Qt::AlignTop);
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
  layout_->addWidget(scrollArea_ctl, 1000);



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
  if( liveThread_ ) {
    liveThread_->disconnect(this);
  }

  if ( (liveThread_  = liveThread) ) {

    connect(liveThread_, &QLivePipelineThread::pipelineChanged,
        this, &ThisClass::onPipelineChanged,
        Qt::QueuedConnection);

  }

  updateControls();
}


QLivePipelineThread * QLivePipelineSelectionWidget::liveThread() const
{
  return liveThread_;
}


static std::string default_config_filename_ =
    "~/.config/SerImager/pipelines.cfg";

void QLivePipelineSelectionWidget::loadPipelines(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
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

  config_filename_ = filename;
}

void QLivePipelineSelectionWidget::savePipelines(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
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

  config_filename_ = filename;
}

c_image_processing_pipeline::sptr QLivePipelineSelectionWidget::selectedPipeline() const
{
  return combobox_ctl->currentData().value<c_image_processing_pipeline::sptr>();
}

void QLivePipelineSelectionWidget::onupdatecontrols()
{
  if ( !liveThread_ ) {
    setEnabled(false);
  }
  else {

    if( liveThread_->pipeline() ) {
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
        std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
            [className](const QPipelineSettingsWidget * obj) {
              return obj->pipelineClass() == className;
            });

    if( pos != settingsWidgets_.end() ) {
      currentWidget = *pos;
    }
    else if( !(currentWidget = p->createSettingsWidget(this)) ) {
      CF_ERROR("pipeline->createSettingsWidgget() fails for pipeline class '%s' ",
          className.toUtf8().constData());
    }
    else {
      settingsWidgets_.append(currentWidget);

      connect(currentWidget, &QSettingsWidget::parameterChanged,
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
  if( liveThread_ ) {
    if( liveThread_->pipeline() ) {
      liveThread_->setPipeline(nullptr);
    }
    else {
      liveThread_->setPipeline(selectedPipeline());
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
        startStop_ctl->setEnabled(liveThread_ && liveThread_->isRunning() );
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
      for( int i = 0, n = settingsWidgets_.size(); i < n; ++i ) {
        if( settingsWidgets_[i]->currentPipeline() == pp ) {
          settingsWidgets_[i]->setCurrentPipeline(nullptr);
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
    Base("QDisplayFrameProcessorSettings", parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Default debayer:",
          "Select debayer algorithm for bayer patterns",
          [this](DEBAYER_ALGORITHM v) {
            if ( liveThread_ ) {
              liveThread_->setDebayer(v);
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            if ( liveThread_ ) {
              *v = liveThread_->debayer();
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
            if ( liveThread_ ) {
              liveThread_->setDarkFramePath(v);
            }
          },
          [this](QString * v) {
            if ( liveThread_ ) {
              *v = liveThread_->darkFramePath();
              return true;
            }
            return false;
          });

  darkFrameScale_ctl =
      add_numeric_box<double>("darkFrameScale",
          "",
          [this](double v) {
            if ( liveThread_ ) {
              liveThread_->setDarkFrameScale(v);
            }
          },
          [this](double * v) {
            if ( liveThread_ ) {
              *v = liveThread_->darkFrameScale();
              return true;
            }
            return false;
          });

  updateControls();
}

void QLiveThreadSettingsWidget::setLiveThread(QLivePipelineThread * liveThread)
{
  liveThread_ = liveThread;
  updateControls();
}

QLivePipelineThread * QLiveThreadSettingsWidget::liveThread() const
{
  return liveThread_;
}

void QLiveThreadSettingsWidget::onupdatecontrols()
{
  if( !liveThread_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

QLiveThreadSettingsDialogBox::QLiveThreadSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(ICON_bayer));
  setWindowTitle("Frame Processor Options");

  layout_ = new QVBoxLayout(this);
  layout_->addWidget(setiingsWidget_ = new QLiveThreadSettingsWidget(this));
}


void QLiveThreadSettingsDialogBox::setLiveThread(QLivePipelineThread * liveThread)
{
  setiingsWidget_->setLiveThread(liveThread);
}

QLivePipelineThread * QLiveThreadSettingsDialogBox::liveThread() const
{
  return setiingsWidget_->liveThread();
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
