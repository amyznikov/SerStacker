/*
 * QLivePipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLivePipeline.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <core/proc/pixtype.h>
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

bool QLivePipeline::convertImage(const cv::Mat & src, COLORID src_colorid, int src_bpp,
    cv::Mat * dst, COLORID dst_colorid, int dst_depth) const
{
  const cv::Mat *bayer = nullptr;
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

  if( src.depth() != dst_depth ) {

    double scale, offset;
    get_scale_offset(src.depth(), src_bpp, dst_depth, &scale, &offset);
    src.convertTo(*dst, dst_depth, scale, offset);
    bayer = dst;

  }
  else if( src_bpp > 0 && src_bpp < (max_bpp = get_max_bpp_for_pixel_depth(src.depth())) ) {
    // scale required
    src.convertTo(*dst, dst_depth, 1 << (max_bpp - src_bpp), 0);
    bayer = dst;
  }
  else {
    bayer = &src;
  }

  if( is_bayer_pattern(src_colorid) ) {
    debayer(*bayer, *dst, src_colorid);
    src_colorid = COLORID_BGR;
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


void QLivePipelineCollection::load()
{

}

void QLivePipelineCollection::save()
{

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

  connect(combobox_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onPipelinesComboboxCurrentIndexChanged);

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopCtlClicked);

  connect(menuButton_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuCtlClicked);

  updateControls();
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

void QLivePipelineSelectionWidget::onupdatecontrols()
{
  if ( !pipelineCollection_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}


void QLivePipelineSelectionWidget::onPipelinesComboboxCurrentIndexChanged(int)
{
  CF_DEBUG("H");

}

void QLivePipelineSelectionWidget::onStartStopCtlClicked()
{
  CF_DEBUG("H");

  QLivePipeline * selectedPipeline =
      this->selectedPipeline();

  CF_DEBUG("selectedPipeline: %p", selectedPipeline);

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
