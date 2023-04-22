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

void QLiveDisplay::onPixmapChanged()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());
  scene()->setImage(pixmap_);
  Q_EMIT currentImageChanged();
}

void QLiveDisplay::showVideoFrame(const cv::Mat & image, COLORID colorid, int bpp)
{
  c_unique_lock mtflock(mtfDisplayFunction_.mutex());

  image.copyTo(inputImage_);
  inputMask_.release();


  if( !current_processor_ || current_processor_->empty() ) {
    current_image_lock lock(this);
    currentImage_ = inputImage_;
    currentMask_ = inputMask_;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////


QLivePipeline::QLivePipeline(const QString & name, QObject * parent) :
    Base(parent),
    name_(name)
{
}

std::mutex& QLivePipeline::mutex()
{
  return mtx_;
}

void QLivePipeline::setName(const QString & name)
{
  name_ = name;
}

const QString & QLivePipeline::name() const
{
  return name_;
}

bool QLivePipeline::serialize(c_config_setting settings, bool save)
{
  return true;
}

bool QLivePipeline::canceled()
{
  return canceled_;
}

void QLivePipeline::setRunning(bool v)
{
  if( running_ != v ) {
    running_ = v;
    Q_EMIT runningStateChanged(v);
  }
}

bool QLivePipeline::isRunning() const
{
  return running_;
}

void QLivePipeline::set_canceled(bool v)
{
  canceled_ = v;
}

bool QLivePipeline::initialize_pipeline()
{
  canceled_ = false;
  return true;
}

void QLivePipeline::cleanup_pipeline()
{
}


bool QLivePipeline::convert_image(const cv::Mat & src, COLORID src_colorid, int src_bpp,
    cv::Mat * dst, COLORID dst_colorid, int dst_depth) const
{
  const cv::Mat *s = nullptr;
  int max_bpp;

  if( dst_colorid != src_colorid ) {

    if( is_bayer_pattern(dst_colorid) ) {
      CF_ERROR("Invalid argument: dst_colorid is bayer patern %s",
          toString(dst_colorid));
      return false;
    }
  }

  if( dst_depth < 0 ) {
    dst_depth = src.depth();
  }

  if( !is_bayer_pattern(src_colorid) ) {
    s = &src;
  }
  else if( !debayer(src, *dst, src_colorid) ) {
    CF_ERROR("debayer(src_colorid=%s) fails", toString(src_colorid));
    return false;
  }
  else {
    s = dst;
    src_colorid = COLORID_BGR;
  }

  if( s->depth() != dst_depth ) {

    double scale, offset;
    get_scale_offset(s->depth(), src_bpp, dst_depth, &scale, &offset);
    s->convertTo(*dst, dst_depth, scale, offset);
  }
  else if( src_bpp > 0 && src_bpp < (max_bpp = get_max_bpp_for_pixel_depth(s->depth())) ) {
    // scale required
    s->convertTo(*dst, dst_depth, 1 << (max_bpp - src_bpp), 0);
  }

  if( src_colorid == dst_colorid ) {
    if( s != dst ) {
      s->copyTo(*dst);
    }
  }
  else {
    switch (src_colorid) {
      case COLORID_MONO:

        switch (dst_colorid) {
          case COLORID_MONO:
            s->copyTo(*dst);
            break;
          case COLORID_RGB:
            cv::cvtColor(*s, *dst, cv::COLOR_GRAY2RGB);
            break;
          case COLORID_BGR:
            cv::cvtColor(*s, *dst, cv::COLOR_GRAY2BGR);
            break;
          case COLORID_BGRA:
            cv::cvtColor(*s, *dst, cv::COLOR_GRAY2BGRA);
            break;
          default:
            s->copyTo(*dst);
            break;
        }
        break;

      case COLORID_RGB:
        switch (dst_colorid) {
          case COLORID_MONO:
            cv::cvtColor(*s, *dst, cv::COLOR_RGB2GRAY);
            break;
          case COLORID_RGB:
            s->copyTo(*dst);
            break;
          case COLORID_BGR:
            cv::cvtColor(*s, *dst, cv::COLOR_RGB2BGR);
            break;
          case COLORID_BGRA:
            cv::cvtColor(*s, *dst, cv::COLOR_RGB2BGRA);
            break;
          default:
            s->copyTo(*dst);
            break;
        }
        break;

      case COLORID_BGR:
        switch (dst_colorid) {
          case COLORID_MONO:
            cv::cvtColor(*s, *dst, cv::COLOR_BGR2GRAY);
            break;
          case COLORID_RGB:
            cv::cvtColor(*s, *dst, cv::COLOR_BGR2RGB);
            break;
          case COLORID_BGR:
            s->copyTo(*dst);
            break;
          case COLORID_BGRA:
            cv::cvtColor(*s, *dst, cv::COLOR_BGR2BGRA);
            break;
          default:
            s->copyTo(*dst);
            break;
        }
        break;

      case COLORID_BGRA:
        switch (dst_colorid) {
          case COLORID_MONO:
            cv::cvtColor(*s, *dst, cv::COLOR_BGRA2GRAY);
            break;
          case COLORID_RGB:
            cv::cvtColor(*s, *dst, cv::COLOR_BGRA2RGB);
            break;
          case COLORID_BGR:
            cv::cvtColor(*s, *dst, cv::COLOR_BGRA2BGR);
            break;
          case COLORID_BGRA:
            s->copyTo(*dst);
            break;
          default:
            s->copyTo(*dst);
            break;
        }
        break;
      default:
        s->copyTo(*dst);
        break;
    }
  }

  return true;
}

std::string QLivePipeline::create_output_path(const std::string & output_directory) const
{
  std::string output_path;

  if( output_directory.empty() ) {

    output_path =
        ssprintf("./%s",
            name_.toUtf8().constData());
  }
  else if( !is_absolute_path(output_directory) ) {

    output_path =
        ssprintf("./%s",
            output_directory.c_str());
  }
  else {
    output_path =
        output_directory;
  }

  return output_path;
}


std::string QLivePipeline::generate_output_file_name(const std::string & output_path,
    const std::string & ufilename,
    const std::string & postfix,
    const std::string & suffix) const
{
  std::string output_file_name =
      ufilename;

  const std::string dateTimeString =
      QDateTime::currentDateTime().toString("yyyy.MM.dd-hh-mm-ss").toStdString();

  if( output_file_name.empty() ) {

    output_file_name =
        ssprintf("%s/%s.%s.%s%s",
            output_path.c_str(),
            name_.toUtf8().constData(),
            dateTimeString.c_str(),
            postfix.c_str(),
            suffix.empty() ? ".avi" :
                suffix.c_str());
  }
  else {

    std::string file_directory;
    std::string file_name;
    std::string file_suffix;

    split_pathfilename(output_file_name,
        &file_directory,
        &file_name,
        &file_suffix);

    if( file_directory.empty() ) {
      file_directory = output_path;
    }
    else if( !is_absolute_path(file_directory) ) {
      file_directory =
          ssprintf("%s/%s",
              output_path.c_str(),
              file_directory.c_str());
    }

    if( file_name.empty() ) {
      file_name =
          ssprintf("%s.%s.%s",
              name_.toUtf8().constData(),
              dateTimeString.c_str(),
              postfix.c_str());
    }

    if( file_suffix.empty() ) {
      file_suffix =
          suffix.empty() ? ".avi" :
              suffix;
    }

    output_file_name =
        ssprintf("%s/%s%s",
            file_directory.c_str(),
            file_name.c_str(),
            file_suffix.c_str());
  }

  return output_file_name;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{

  connect(this, &ThisClass::restartAfterException,
      this, &ThisClass::onRestartAfterException,
      Qt::QueuedConnection);

}

QLivePipelineThread::~QLivePipelineThread()
{
  finish(true);
}


const QImagingCamera::sptr & QLivePipelineThread::camera() const
{
  return camera_;
}

void QLivePipelineThread::setCamera(const QImagingCamera::sptr & camera)
{
  if ( isRunning() ) {
    finish(true);
  }

  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }

  if( (camera_ = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged/*,
        Qt::QueuedConnection*/);

    startPipeline(pipeline_);
  }

}

void QLivePipelineThread::onRestartAfterException()
{
  finish(true);
  startPipeline(nullptr);
}

void QLivePipelineThread::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  switch (newState) {
    case QImagingCamera::State_started:
      startPipeline(pipeline_);
      break;
    default:
      finish(true);
      pipeline_ = nullptr;
      break;
  }
}

void QLivePipelineThread::setDisplay(QLiveDisplay * display)
{
  display_ = display;
}

QLiveDisplay* QLivePipelineThread::display() const
{
  return display_;
}

QLivePipeline* QLivePipelineThread::currentPipeline() const
{
  return pipeline_;
}

void QLivePipelineThread::setDebayer(DEBAYER_ALGORITHM algo)
{
  debayer_ = algo;
}

DEBAYER_ALGORITHM QLivePipelineThread::debayer() const
{
  return debayer_;
}


bool QLivePipelineThread::startPipeline(QLivePipeline * pipeline)
{
  if( isRunning() ) {
    CF_DEBUG("finish()");
    finish(true);
    CF_DEBUG("finish() OK");
  }

  this->finish_ = false;
  this->pipeline_ = pipeline;

  if( !display_ ) {
    CF_ERROR("ERROR: no display was specified, can not start");
    return false;
  }

  if( !camera_ ) {
    CF_ERROR("ERROR: no camera specified, can not start");
    return false;
  }

  if( camera_->state() != QImagingCamera::State_started ) {
    // CF_ERROR("ERROR: camera is not started");
    return false;
  }

  CF_DEBUG("Base::start()");
  Base::start();

  CF_DEBUG("leave");

  return true;
}

void QLivePipelineThread::finish(bool wait)
{
  finish_ = true;

  if( wait ) {
    while (isRunning()) {
      Base::msleep(100);
    }
  }
}

void QLivePipelineThread::run()
{
  CF_DEBUG("enter");

  cv::Mat inputImage;
  bool haveException = false;

  int last_frame_index = -1;
  int bpp;
  COLORID colorid;

  try {

    if( pipeline_ ) {
      if( !pipeline_->initialize_pipeline() ) {
        CF_ERROR("pipeline_->initialize_pipeline() fails");
        return;
      }
      pipeline_->setRunning(true);
    }

    while (!finish_) {

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

        if( debayer_ != DEBAYER_DISABLE && is_bayer_pattern(colorid) ) {
          if( ::debayer(inputImage, inputImage, colorid, debayer_) ) {
            colorid = COLORID_BGR;
          }
        }

        if( !pipeline_ ) {
          display_->showVideoFrame(inputImage, colorid, bpp);
        }
        else {

          if( !pipeline_->process_frame(inputImage, colorid, bpp) ) {
            CF_ERROR("pipeline_->processFrame() fails");
            break;
          }

          if( !pipeline_->get_display_image(&inputImage, &colorid, &bpp) ) {
            CF_ERROR("pipeline_->getDisplayImage() fails");
            break;
          }

          display_->showVideoFrame(inputImage, colorid, bpp);
        }
      }

      QThread::msleep(30);
    }
  }
  catch( const std::exception &e )
  {
    haveException = true;
    CF_ERROR("Exception in live thread : %s", e.what());
  }

  if( pipeline_ ) {
    try {
      pipeline_->setRunning(false);
      pipeline_->cleanup_pipeline();
    }
    catch( const std::exception &e )
    {
      haveException = true;
      CF_ERROR("Exception in pipeline_->cleanup_pipeline() : %s", e.what());
    }
  }


  if ( haveException ) {
    Q_EMIT restartAfterException(nullptr);
  }


  CF_DEBUG("leave");
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const QList<QLivePipelineCollection::PipelineType*>& QLivePipelineCollection::pipelineTypes() const
{
  return pipelineTypes_;
}

bool QLivePipelineCollection::addPipelineClassFactory(const QString & className, const QString & tooltip,
    const PipelineType::PipelineFactoryFunction & factory,
    const PipelineType::SettingsFactoryFunction & settingsFactory)
{
  const auto pos =
      std::find_if(pipelineTypes_.begin(), pipelineTypes_.end(),
          [className](const PipelineType * obj) {
            return obj->className() == className;
          });

  if( pos != pipelineTypes_.end() ) {
    CF_ERROR("ERROR: Requested pipeline type '%s' already registered",
        className.toUtf8().constData());
    return false;
  }

  pipelineTypes_.append(new PipelineType(className,
      tooltip,
      factory,
      settingsFactory));

  return true;
}

const QLivePipelineCollection::PipelineType * QLivePipelineCollection::findPipelineClassFactory(const QString & className) const
{
  const auto pos =
      std::find_if(pipelineTypes_.begin(), pipelineTypes_.end(),
          [className](const PipelineType * obj) {
            return obj->className() == className;
          });

  return pos == pipelineTypes_.end() ? nullptr : *pos;
}


QLivePipeline * QLivePipelineCollection::addPipeline(const QString & className, const QString & objName)
{
  const auto pos =
      std::find_if(pipelineTypes_.begin(), pipelineTypes_.end(),
          [className](const PipelineType * obj) {
            return obj->className() == className;
          });

  if( pos == pipelineTypes_.end() ) {
    CF_ERROR("ERROR: pipeline type '%s' not registered",
        className.toUtf8().constData());
    return nullptr;
  }

  QLivePipeline * pipeline =
      (*pos)->createInstance(objName);

  if( !pipeline ) {
    CF_ERROR("ERROR: PipelineType->createInstance() fails for pipeline type '%s' name '%s'",
        className.toUtf8().constData(), objName.toUtf8().constData());
    return nullptr;
  }


  this->append(pipeline);

  save();

  return pipeline;
}

bool QLivePipelineCollection::removePipeline(QLivePipeline * pipeline)
{
  const int pos =
      indexOf(pipeline);

  if ( pos >= 0 ) {
    removeAt(pos);
    save();
    return true;
  }

  return false;
}


QLivePipeline * QLivePipelineCollection::findPipeline(const QString & name) const
{
  const auto pos =
      std::find_if(begin(), end(),
          [name](const QLivePipeline * obj) {
            return obj->name() == name;
          });

  return pos == end() ? nullptr : *pos;
}


std::string QLivePipelineCollection::default_config_filename_ =
    "~/.config/SerImager/pipelines.cfg";

void QLivePipelineCollection::load(const std::string & cfgfilename)
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

  const int n =
      section.length();

  // FIXME: very crazy hack  !!!!!
  for( auto p : *this ) {
    delete p;
  }
  Base::clear();
  Base::reserve(n);

  for( int i = 0; i < n; ++i ) {

    c_config_setting item =
        section.get_element(i);

    if( item && item.isGroup() ) {

      std::string objtype, objname;

      QLivePipeline *obj = nullptr;

      if( !load_settings(item, "objtype", &objtype) || objtype.empty() ) {
        CF_ERROR("load_settings(objtype) fails for item %d", i);
        continue;
      }

      if( !load_settings(item, "objname", &objname) || objname.empty() ) {
        CF_ERROR("load_settings(objname) fails for item %d", i);
        continue;
      }

      const QString objClassName(objtype.c_str());

      const auto * factory =
          findPipelineClassFactory(objtype.c_str());

      if( !factory ) {
        CF_ERROR("ERROR: findPipelineClassFactory(objtype='%s') fails for item %d",
            objtype.c_str(), i);
        continue;
      }

      if( !(obj = factory->createInstance(objname.c_str())) ) {
        CF_ERROR("ERROR: factory->createInstance(objtype='%s' objname='%s')  fails",
            objtype.c_str(), objname.c_str());
        continue;
      }

      if( !obj->serialize(item, false) ) {
        CF_ERROR("obj->serialize() fails for item index %d (%s:%s)", i,
            objtype.c_str(), objname.c_str());
        delete obj;
        continue;
      }

      Base::append(obj);
    }
  }

  config_filename_ = filename;
}

void QLivePipelineCollection::save(const std::string & cfgfilename) const
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

  for ( QLivePipeline * obj : *this ) {

    std::string objtype =
        obj->getClassName().toStdString();

    if( objtype.empty() ) {
      CF_ERROR("FIXME: obj->getClassName() returns empty string (objname='%s')",
          obj->name().toUtf8().constData());
      continue;
    }

    c_config_setting item =
        section.add_group();

    if ( !save_settings(item, "objtype", objtype) ) {
      CF_ERROR("save_settings(objtype) fails");
      continue;
    }

    if ( !save_settings(item, "objname", obj->name().toStdString()) ) {
      CF_ERROR("save_settings(objname) fails");
      continue;
    }

    if( !obj->serialize(item, true) ) {
      CF_ERROR("obj->serialize(ty[e=%s name=%s) fails", objtype, obj->name().toUtf8().constData());
      continue;
    }

  }

  if( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return;
  }

  config_filename_ = filename;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

class QAddLivePipelineDialogBox :
    public QDialog
{
public:
  typedef QAddLivePipelineDialogBox ThisClass;
  typedef QDialog Base;

  QAddLivePipelineDialogBox(QLivePipelineCollection * pipelineCollection,
      QWidget * parent = nullptr);

  QString selectedPipelineName() const;
  QString selectedPipelineClass() const;

protected:
  QLivePipelineCollection * pipelineCollection_;
  QFormLayout * form_ = nullptr;
  QHBoxLayout * hbox_ = nullptr;
  QLineEditBox * pipelineName_ctl = nullptr;
  QComboBox * pipelineTypeSelector_ctl = nullptr;
  QLabel * pipelineTooltop_ctl = nullptr;
  QPushButton * btnOk_ = nullptr;
  QPushButton * btnCancel_ = nullptr;
};

QAddLivePipelineDialogBox::QAddLivePipelineDialogBox(QLivePipelineCollection * pipelineCollection, QWidget * parent) :
    Base(parent),
    pipelineCollection_(pipelineCollection)
{
  setWindowTitle("Select live pipeline");

  form_ = new QFormLayout(this);

  form_->addRow("Name:", pipelineName_ctl = new QLineEditBox(this));
  form_->addRow("Type:", pipelineTypeSelector_ctl = new QComboBox(this));
  form_->addRow(pipelineTooltop_ctl = new QLabel(this));

  form_->addRow(hbox_ = new QHBoxLayout());
  hbox_->addWidget(btnOk_ = new QPushButton("OK"));
  hbox_->addWidget(btnCancel_ = new QPushButton("Cancel"));

  for( const auto &item : pipelineCollection->pipelineTypes() ) {
    pipelineTypeSelector_ctl->addItem(item->className(),
        QVariant::fromValue(QString(item->tooltip())));
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

QString QAddLivePipelineDialogBox::selectedPipelineName() const
{
  return pipelineName_ctl->text();
}

QString QAddLivePipelineDialogBox::selectedPipelineClass() const
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

    connect(liveThread_, &QThread::started,
        this, &ThisClass::onLiveThreadStateChanged);

    connect(liveThread_, &QThread::finished,
        this, &ThisClass::onLiveThreadStateChanged);

  }

  updateControls();
}


QLivePipelineThread * QLivePipelineSelectionWidget::liveThread() const
{
  return liveThread_;
}


void QLivePipelineSelectionWidget::setPipelineCollection(QLivePipelineCollection * pipelines)
{
  pipelineCollection_ = pipelines;
  populatepipelines();
  updateControls();
}

QLivePipelineCollection * QLivePipelineSelectionWidget::pipelines() const
{
  return pipelineCollection_;
}



QLivePipeline* QLivePipelineSelectionWidget::selectedPipeline() const
{
  return combobox_ctl->currentData().value<QLivePipeline*>();
}

void QLivePipelineSelectionWidget::populatepipelines()
{
  combobox_ctl->clear();

  if ( pipelineCollection_ ) {
    for ( QLivePipeline * p : *pipelineCollection_ ) {
      combobox_ctl->addItem(p->name(), QVariant::fromValue(p));
    }
  }
}

void QLivePipelineSelectionWidget::onLiveThreadStateChanged()
{
  updateControls();
}

void QLivePipelineSelectionWidget::onupdatecontrols()
{
  if ( !pipelineCollection_ || !liveThread_ ) {
    setEnabled(false);
  }
  else {

    if ( !liveThread_->isRunning() ) {

      combobox_ctl->setEnabled(true);
      menuButton_ctl->setEnabled(true);

      startStop_ctl->setIcon(getIcon(ICON_start));
      startStop_ctl->setEnabled(false);

    }

    else if ( liveThread_->currentPipeline() ) {

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
  QLivePipelineSettingsWidget *currentWidget =
      nullptr;

  QLivePipeline* pipeline =
      selectedPipeline();

  if ( pipeline ) {

    const QString className =
        pipeline->getClassName();

    const auto pos =
        std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
            [className](const QLivePipelineSettingsWidget * obj) {
              return className == obj->pipelineClassName();
            });

    if ( pos != settingsWidgets_.end() ) {
      currentWidget = *pos;
    }
    else {

      const auto *factory =
          pipelineCollection_->findPipelineClassFactory(className);

      if( !factory ) {
        CF_FATAL("PipelineClassFactory not regsistered for pipeline class '%s' name '%s'",
            pipeline->getClassName().toUtf8().constData(),
            pipeline->name().toUtf8().constData());
      }
      else if( !(currentWidget = factory->createSettingsWidgget(this)) ) {
        CF_ERROR("factory->createSettingsWidgget() fails for pipeline class '%s' ",
            className.toUtf8().constData());
      }
      else {
        settingsWidgets_.append(currentWidget);

        //  connect(currentWidget, &QLivePipelineSettingsWidget::parameterChanged,
        //    this, &ThisClass::parameterChanged);

        connect(currentWidget, &QLivePipelineSettingsWidget::parameterChanged,
            [this]() {
              if ( pipelineCollection_ ) {
                pipelineCollection_->save();
              }
            });
      }
    }

    if ( currentWidget ) {
      currentWidget->setCurrentPipeline(pipeline);
    }
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

    if( liveThread_->currentPipeline() ) {
      liveThread_->startPipeline(nullptr);
    }
    else {
      QLivePipeline *pipeline = selectedPipeline();
      liveThread_->startPipeline(pipeline);
    }
  }
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
  if (!pipelineCollection_ ) {
    return;
  }

  QAddLivePipelineDialogBox dialogbox(pipelineCollection_, this);

  if( dialogbox.exec() == QDialog::Accepted ) {

    QString name =
        dialogbox.selectedPipelineName();

    const QString pipeline_type =
        dialogbox.selectedPipelineClass();

    if( !pipeline_type.isEmpty() ) {

      if( name.isEmpty() ) {

        for( int i = 1; i < 10000; ++i ) {
          name = qsprintf("%s%d", pipeline_type.toUtf8().constData(), i);
          if( !pipelineCollection_->findPipeline(name) ) {
            break;
          }
        }
      }

      else if( pipelineCollection_->findPipeline(name) ) {

        for( int i = 1; i < 10000; ++i ) {
          QString newname = qsprintf("%s%d", name.toUtf8().constData(), i);
          if( !pipelineCollection_->findPipeline(newname) ) {
            name = newname;
            break;
          }
        }
      }

      QLivePipeline * pipeline =
          pipelineCollection_->addPipeline(pipeline_type, name);

      if( !pipeline ) {
        QMessageBox::critical(this, "ERROR",
            qsprintf("pipelineCollection_->addPipeline(%s: %s) fails",
                pipeline_type.toUtf8().constData(),
                name.toUtf8().constData()));
      }
      else {
        combobox_ctl->addItem(pipeline->name(), QVariant::fromValue(pipeline));
        combobox_ctl->setCurrentIndex(combobox_ctl->count() - 1);
        startStop_ctl->setEnabled(liveThread_ && liveThread_->isRunning() );
      }
    }
  }
}

void QLivePipelineSelectionWidget::onRemoveLivePipelineClicked()
{
  QLivePipeline* pipeline =
      selectedPipeline();

  if ( pipeline ) {

    const int resp =
        QMessageBox::warning(this, "Remove Live Pipeline",
            qsprintf("Confirmation required:\n"
                "Are you sure to remove pipeline %s ?",
                pipeline->name().toUtf8().constData()),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);

    if ( resp != QMessageBox::Yes ) {
      return;
    }

    for( int i = 0, n = settingsWidgets_.size(); i < n; ++i ) {
      if( settingsWidgets_[i]->currentPipeline() == pipeline ) {
        settingsWidgets_[i]->setCurrentPipeline(nullptr);
      }
    }

    if( !pipelineCollection_->removePipeline(pipeline) ) {
      CF_ERROR("ERROR: pipelineCollection_->removePipeline('%s') fails",
          pipeline->name().toUtf8().constData());
    }

    const int comboIndex =
        combobox_ctl->findData(QVariant::fromValue(pipeline));

    if ( comboIndex >= 0 ) {
      combobox_ctl->removeItem(comboIndex);
    }
    else {
      CF_ERROR("ERROR: combobox_ctl->findData(pipeline=%s) fails",
          pipeline->name().toUtf8().constData());
    }

    delete pipeline;

    if ( combobox_ctl->currentIndex() < 0 ) {
      startStop_ctl->setEnabled(false);
    }

  }
}

void QLivePipelineSelectionWidget::onRenameLivePipelineClicked()
{
  QLivePipeline *pipeline =
      selectedPipeline();

  if( pipeline ) {

    while (42) {

      const QString currentName =
          pipeline->name();

      const QString newName =
          QInputDialog::getText(this,
              "Rename Pipeline",
              "New name:",
              QLineEdit::Normal,
              currentName);

      if( newName.isEmpty() ) {
        break;
      }

      if( !pipelineCollection_->findPipeline(newName) ) {

        pipeline->setName(newName);

        const int index = combobox_ctl->findText(currentName);
        if( index >= 0 ) {
          combobox_ctl->setItemText(index, newName);
        }

        pipelineCollection_->save();

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
