/*
 * QFFStreams.cc
 *
 *  Created on: Mar 17, 2023
 *      Author: amyznikov
 */

#include "QFFStreams.h"

namespace serimager {

void QFFStreams::registerMetaTypes()
{
  static bool initialized = false;
  if ( !initialized ) {
    //#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    qRegisterMetaType<QFFMPEGCameraParameters>("QFFMPEGCameraParameters");
    qRegisterMetaType<serimager::QFFMPEGCameraParameters>("serimager::QFFMPEGCameraParameters");
    qRegisterMetaType<QList<QFFMPEGCameraParameters>>("QList<QFFMPEGCameraParameters>");
    qRegisterMetaType<QList<serimager::QFFMPEGCameraParameters>>("QList<serimager::QFFMPEGCameraParameters>");
    qRegisterMetaTypeStreamOperators<QFFMPEGCameraParameters>("QFFMPEGCameraParameters");
    qRegisterMetaTypeStreamOperators<serimager::QFFMPEGCameraParameters>("serimager::QFFMPEGCameraParameters");
    qRegisterMetaTypeStreamOperators<QList<QFFMPEGCameraParameters>>("QList<QFFMPEGCameraParameters>");
    qRegisterMetaTypeStreamOperators<QList<serimager::QFFMPEGCameraParameters>>("QList<serimager::QFFMPEGCameraParameters>");
    //#endif
    initialized = true;
  }

}

QFFStreams * QFFStreams::instance()
{
  static QFFStreams * _instance = new QFFStreams();
  return _instance;
}

QFFStreams::QFFStreams()
{
}

const QList<QImagingCamera::sptr> & QFFStreams::streams()
{
  return instance()->streams_;
}

void QFFStreams::add(const QFFMPEGCamera::sptr & stream)
{
  instance()->streams_.append(stream);
  instance()->save();
  Q_EMIT instance()->streamsChaged();
}

void QFFStreams::remove(const QFFMPEGCamera::sptr & stream)
{
  QList<QImagingCamera::sptr> & streams =
      instance()->streams_;

  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {
    if( ii->get() == stream.get() ) {
      streams.erase(ii);
      Q_EMIT instance()->streamsChaged();
      return;
    }
  }
}

bool QFFStreams::exist(const QImagingCamera::sptr & stream)
{
  const QList<QImagingCamera::sptr> & streams =
      instance()->streams_;

  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {
    if( ii->get() == stream.get() ) {
      return true;
    }
  }

  return false;
}

bool QFFStreams::exist(const QString & streamName)
{
  const QList<QImagingCamera::sptr> & streams =
      instance()->streams_;

  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {

    const QFFMPEGCamera::sptr camera =
        std::dynamic_pointer_cast<QFFMPEGCamera>(*ii);

    if( camera && camera->name() == streamName ) {
      return true;
    }
  }

  return false;
}


void QFFStreams::save()
{
  const QList<QImagingCamera::sptr> & streams =
      instance()->streams_;

  QList<QFFMPEGCameraParameters> cameras;

  for( const QImagingCamera::sptr &stream : streams ) {

    const QFFMPEGCamera::sptr camera =
        std::dynamic_pointer_cast<QFFMPEGCamera>(stream);

    if( camera ) {

      const QFFMPEGCameraParameters params = {
          .name = camera->name(),
          .url = camera->url(),
          .opts = camera->opts()
      };

      cameras.append(params);
    }
  }

  QSettings().setValue("QFFStreams",
      QVariant::fromValue(cameras));
}

void QFFStreams::load()
{
  QList<QImagingCamera::sptr> & streams =
      instance()->streams_;

  const QList<QFFMPEGCameraParameters> cameras =
      QSettings().value("QFFStreams",
          QVariant::fromValue(QList<QFFMPEGCameraParameters>()))
            .value<QList<QFFMPEGCameraParameters>>();

  streams.clear();

  for( const QFFMPEGCameraParameters &c : cameras ) {
    streams.append(QFFMPEGCamera::create(c.name, c.url, c.opts));
  }

  Q_EMIT instance()->streamsChaged();
}


QFFStreamsDialogBox::QFFStreamsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("FFmpeg streams");

  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->addWidget(streams_ctl = new QFFStreamsWidget(this));
}


QFFStreamListWidget::QFFStreamListWidget(QWidget * parent) :
    Base(parent)
{
  static const auto create_toolbutton =
      [](const QIcon & icon, const QString & text, const QString & tooltip, QWidget * parent = nullptr) -> QToolButton* {

        QToolButton * tb = new QToolButton(parent);

        //tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
        // tb->setIconSize(QSize(22, 22));

        if ( !text.isEmpty() ) {
          tb->setText(text);
        }

        if ( !icon.isNull() ) {
          tb->setIcon(icon);
        }

        tb->setToolTip(tooltip);
        tb->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

        return tb;
      };


  hbox = new QHBoxLayout(this);
  vbox = new QVBoxLayout();

  vbox->setAlignment(Qt::AlignTop);

  vbox->addWidget(addStream_ctl =
      create_toolbutton(QIcon(), "Add", "Add new ffmpeg input stream"),
      0, Qt::AlignTop);

  vbox->addWidget(removeStream_ctl =
      create_toolbutton(QIcon(), "Remove", "Remove ffmpeg stream from list"),
      0, Qt::AlignTop);

  removeStream_ctl->setEnabled(false);


  hbox->addLayout(vbox);
  hbox->addWidget(list_ctl = new QListWidget());

  list_ctl->setViewMode(QListView::ViewMode::ListMode);
  list_ctl->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  list_ctl->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);


  connect(QFFStreams::instance(), &QFFStreams::streamsChaged,
      this, &ThisClass::updateStreamList);

  connect(addStream_ctl, &QToolButton::clicked,
      this, &ThisClass::onAddStreamClicked);

  connect(removeStream_ctl, &QToolButton::clicked,
      this, &ThisClass::onRemoveStreamClicked);

  connect(list_ctl, &QListWidget::currentItemChanged,
      this, &ThisClass::onCurrentListItemChanged);

  updateStreamList();
}


void QFFStreamListWidget::updateStreamList()
{
  list_ctl->clear();

  for( const auto &stream : QFFStreams::streams() ) {

    QFFMPEGCamera::sptr camera =
        std::dynamic_pointer_cast<QFFMPEGCamera>(stream);

    if( camera ) {

      QListWidgetItem *item =
          new QListWidgetItem(camera->name());

      item->setData(Qt::UserRole,
          QVariant::fromValue(camera));

      list_ctl->addItem(item);
    }
  }
}

void QFFStreamListWidget::onAddStreamClicked()
{
  QString streamName;

  for( int i = 0; i < 10000; ++i ) {
    streamName = QString("stream%1").arg(i);
    if( !QFFStreams::exist(streamName) ) {
      break;
    }
  }

  QFFMPEGCamera::sptr camera =
      QFFMPEGCamera::create(streamName, "","");

  if ( camera ) {
    QFFStreams::add(camera);
    updateStreamList();
    selectStream(camera);
  }

}

void QFFStreamListWidget::onRemoveStreamClicked()
{

}

void QFFStreamListWidget::onCurrentListItemChanged(QListWidgetItem * current, QListWidgetItem * previous)
{
  if( !current ) {
    removeStream_ctl->setEnabled(false);
  }
  else {
    current->setSelected(true);
    removeStream_ctl->setEnabled(true);
  }

  Q_EMIT selectedStreamChanged();
}

QFFMPEGCamera::sptr QFFStreamListWidget::selectedStream() const
{
  QListWidgetItem *currentItem =
      list_ctl->currentItem();

  return currentItem ?
      std::dynamic_pointer_cast<QFFMPEGCamera>(currentItem->
          data(Qt::UserRole).value<QFFMPEGCamera::sptr>()) :
      nullptr;
}

void QFFStreamListWidget::selectStream(const QFFMPEGCamera::sptr & camera)
{
  for ( int i = 0, n = list_ctl->count(); i < n; ++i ) {

    QListWidgetItem * item =
        list_ctl->item(i);

    QImagingCamera::sptr c =
        item->data(Qt::UserRole).value<QFFMPEGCamera::sptr>();

    if ( c == camera ) {
      list_ctl->setCurrentItem(item);
      break;
    }
  }
}

QFFStreamsWidget::QFFStreamsWidget(QWidget * parent) :
    Base(parent)
{
  ///
  streamName_ctl =
      add_textbox("Name:",
          "",
          [this](const QString & value) {
            if ( _selectedStream ) {
              _selectedStream->setName(value);
              QFFStreams::save();
            }
          },
          [this](QString * value) {
            if ( _selectedStream ) {
              *value = _selectedStream->name();
              return true;
            }
            return false;
          });

  ///
  addRow("URL:", streamUrl_ctl =
      new QFFMPEGCameraUrlWidget());

  connect(streamUrl_ctl, &QFFMPEGCameraUrlWidget::urlChanged,
      [this]() {
        if ( !updatingControls() && _selectedStream ) {
          _selectedStream->setUrl(streamUrl_ctl->url());
          QFFStreams::save();
        }
      });

  ///

  streamOpts_ctl =
      add_textbox("Options:",
          "",
          [this](const QString & value) {
            if ( _selectedStream ) {
              _selectedStream->setOpts(value);
              QFFStreams::save();
            }
          },
          [this](QString * value) {
            if ( _selectedStream ) {
              *value = _selectedStream->opts();
              return true;
            }
            return false;
          });


  ///
  addRow(list_ctl =
      new QFFStreamListWidget());

  connect(list_ctl, & QFFStreamListWidget::selectedStreamChanged,
      this, &ThisClass::onSelectedStreamChanged);

  ///
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _selectedStream ) {
          streamUrl_ctl->setUrl(_selectedStream->url());
        }
        //    Base::onupdatecontrols();
      });

  connect(this, &ThisClass::enablecontrols,
      [this]() {
          const bool enable = _selectedStream != nullptr;
          streamName_ctl->setEnabled(enable);
          streamUrl_ctl->setEnabled(enable);
          streamOpts_ctl->setEnabled(enable);
          //    Base::onupdatecontrols();
      });

  updateControls();
}

void QFFStreamsWidget::onSelectedStreamChanged()
{
  if( _selectedStream ) {
    QObject::disconnect(_selectedStream.get(),
        nullptr, this, nullptr);
  }

  if( (_selectedStream = list_ctl->selectedStream()) ) {
    QObject::connect(_selectedStream.get(), &QFFMPEGCamera::parametersChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}

//void QFFStreamsWidget::onupdatecontrols()
//{
//  const bool enable =
//      _selectedStream != nullptr;
//
//  if ( enable ) {
//    streamUrl_ctl->setUrl(_selectedStream->url());
//  }
//
//  streamName_ctl->setEnabled(enable);
//  streamUrl_ctl->setEnabled(enable);
//  streamOpts_ctl->setEnabled(enable);
//
//
//
//
//  Base::onupdatecontrols();
//}

} // namespace serimager

