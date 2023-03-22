/*
 * QLivePipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLivePipeline.h"
#include "pipeline/QLiveStereoCalibration/QLiveStereoCalibrationOptions.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <core/proc/pixtype.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace serimager {


#define ICON_start            ":/qserimager/icons/start.png"
#define ICON_stop             ":/qserimager/icons/stop.png"
#define ICON_process          ":/qserimager/icons/process.png"
#define ICON_menu             ":/qserimager/icons/menu.png"
#define ICON_add              ":/qserimager/icons/add.png"
#define ICON_delete           ":/qserimager/icons/delete.png"
#define ICON_rename           ":/qserimager/icons/rename.png"

namespace {

typedef std::lock_guard<std::mutex>
  c_guard_lock;

typedef std::unique_lock<std::mutex>
  c_unique_lock;

} // namespace


///////////////////////////////////////////////////////////////////////////////////////////////////
QLivePipeline::QLivePipeline(const QString & name, QObject * parent) :
    Base(parent),
    name_(name)
{
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

bool QLivePipeline::convertImage(const cv::Mat & src, COLORID src_colorid, int src_bpp,
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

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{

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

  camera_ = camera;
  if( (camera_ = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    startPipeline(pipeline_);
  }

}

void QLivePipelineThread::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  switch (newState) {
    case QImagingCamera::State_started:
      startPipeline(pipeline_);
      break;
    default:
      finish(true);
      break;
  }
}

void QLivePipelineThread::setDisplay(QVideoFrameDisplay * display)
{
  display_ = display;
}

QVideoFrameDisplay* QLivePipelineThread::display() const
{
  return display_;
}

QLivePipeline* QLivePipelineThread::currentPipeline() const
{
  return pipeline_;
}

bool QLivePipelineThread::startPipeline(QLivePipeline * pipeline)
{
  if( isRunning() ) {
    finish(true);
  }

  this->pipeline_ = pipeline;
  this->finish_ = false;

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

  Base::start();

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
  bool haveInputImage;

  int last_frame_index = -1;
  int bpp;
  COLORID colorid;

  while (!finish_) {

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

      if ( !pipeline_ ) {
        display_->showVideoFrame(inputImage, colorid, bpp);
      }
      else {
        pipeline_->processFrame(inputImage, colorid, bpp);
        pipeline_->getDisplayImage(&inputImage, &colorid, &bpp);
        display_->showVideoFrame(inputImage, colorid, bpp);
      }


    }

    QThread::msleep(30);
  }
  CF_DEBUG("leave");
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const QList<QLivePipelineCollection::PipelineType*>& QLivePipelineCollection::pipelineTypes() const
{
  return pipelineTypes_;
}

bool QLivePipelineCollection::addPipelineType(const QString & name, const QString & tooltip,
    const PipelineType::factoryFunction & factory)
{
  const auto pos =
      std::find_if(pipelineTypes_.begin(), pipelineTypes_.end(),
          [name](const PipelineType * obj) {
            return obj->name() == name;
          });

  if( pos != pipelineTypes_.end() ) {
    CF_ERROR("ERROR: Requested pipeline type '%s' already registered",
        name.toUtf8().constData());
    return false;
  }

  pipelineTypes_.append(new PipelineType(name, tooltip, factory));

  return true;
}

QLivePipeline * QLivePipelineCollection::addPipeline(const QString & type, const QString & name)
{
  const auto pos =
      std::find_if(pipelineTypes_.begin(), pipelineTypes_.end(),
          [type](const PipelineType * obj) {
            return obj->name() == type;
          });

  if( pos == pipelineTypes_.end() ) {
    CF_ERROR("ERROR: pipeline type '%s' not registered",
        type.toUtf8().constData());
    return nullptr;
  }

  QLivePipeline * pipeline =
      (*pos)->createInstance(name);

  if( !pipeline ) {
    CF_ERROR("ERROR: PipelineType->createInstance() fails for pipeline type '%s' name '%s'",
        type.toUtf8().constData(), name.toUtf8().constData());
    return nullptr;
  }


  this->append(pipeline);

  save();

  return pipeline;
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
  if( !load_settings(cfg.root(), "object_class", &object_class) ) {
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

      if( objtype == "QLiveStereoCalibrationPipeline" ) {
        obj = new QLiveStereoCalibrationPipeline(objname.c_str());
      }
      else {
        CF_ERROR("Unknown objtype '%s' specified for item %d", objtype.c_str(), i);
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

  CF_DEBUG("Saving '%s' ...",
      filename.c_str());

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

    std::string objtype;

    if( dynamic_cast<QLiveStereoCalibrationPipeline*>(obj) ) {
      objtype = "QLiveStereoCalibrationPipeline";
    }
    else {
      CF_ERROR("FIXME: implement adequate class factory please !!!!");
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
    pipelineTypeSelector_ctl->addItem(item->name(),
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
          "Start / Stop currenet pipeline"));

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

    if ( liveThread_->currentPipeline() ) {

      QWidget * w = scrollArea_ctl->widget();
      if ( w ) {
        w->setEnabled(false);
      }

      combobox_ctl->setEnabled(false);
      menuButton_ctl->setEnabled(false);

      startStop_ctl->setIcon(getIcon(ICON_stop));
      startStop_ctl->setEnabled(true);

    }
    else if( liveThread_->camera() && liveThread_->camera()->state() == QImagingCamera::State_started ) {

      QWidget *w = scrollArea_ctl->widget();
      if( w ) {
        w->setEnabled(true);
      }

      combobox_ctl->setEnabled(true);
      menuButton_ctl->setEnabled(true);

      startStop_ctl->setIcon(getIcon(ICON_start));
      startStop_ctl->setEnabled(true);
    }
    else {
      QWidget *w = scrollArea_ctl->widget();
      if( w ) {
        w->setEnabled(true);
      }

      combobox_ctl->setEnabled(true);
      menuButton_ctl->setEnabled(true);

      startStop_ctl->setIcon(getIcon(ICON_start));
      startStop_ctl->setEnabled(false);

    }


    setEnabled(true);
  }
}


void QLivePipelineSelectionWidget::onPipelinesComboboxCurrentIndexChanged(int)
{
  QWidget *currentWidget = nullptr;

  QLivePipeline* pipeline = selectedPipeline();

  if ( pipeline ) {

    if( QLiveStereoCalibrationPipeline *stereoCalibration =
        dynamic_cast<QLiveStereoCalibrationPipeline*>(pipeline) ) {

      if( !stereoCalibrationOptions_ctl ) {
        stereoCalibrationOptions_ctl = new QLiveStereoCalibrationOptions(this);
        connect(stereoCalibrationOptions_ctl, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( pipelineCollection_ ) {
                CF_DEBUG("pipelineCollection_->save()");
                pipelineCollection_->save();
              }
            });
      }

      currentWidget = stereoCalibrationOptions_ctl;
      stereoCalibrationOptions_ctl->setPipeline(stereoCalibration);
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

//  if ( currentWidget ) {
//
//    if ( QImageProcessingPipeline::isRunning() && pipeline == QImageProcessingPipeline::current_pipeline() ) {
//      currentWidget->setEnabled(false);
//    }
//    else {
//      currentWidget->setEnabled(true);
//    }
//  }

}

void QLivePipelineSelectionWidget::onStartStopCtlClicked()
{
  if( liveThread_ ) {

    if( liveThread_->currentPipeline() ) {
      CF_DEBUG("liveThread_->startPipeline(nullptr)");
      liveThread_->startPipeline(nullptr);
    }
    else {
      QLivePipeline *pipeline = selectedPipeline();

      CF_DEBUG("liveThread_->startPipeline(pipeline=%p)", pipeline);

      liveThread_->startPipeline(pipeline);
    }

  }
}

void QLivePipelineSelectionWidget::onMenuCtlClicked()
{
  QMenu menu;

  menu.addAction(getIcon(ICON_add), "Add pipeline...",
      this, &ThisClass::onAddLivePipelineClicked);

  menu.addAction(getIcon(ICON_add), "Rename pipeline......",
      this, &ThisClass::onRenameLivePipelineClicked);

  menu.addSeparator();

  menu.addAction(getIcon(ICON_add), "Delete pipeline...",
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
      }
    }
  }
}

void QLivePipelineSelectionWidget::onRemoveLivePipelineClicked()
{

}

void QLivePipelineSelectionWidget::onRenameLivePipelineClicked()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */
