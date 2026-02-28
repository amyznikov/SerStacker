/*
 * QLCSCTStreams.cc
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#include "QLCSCTPStreams.h"

namespace serimager {

void QLCSCTPStreams::registerMetaTypes()
{
  static bool initialized = false;
  if( !initialized ) {
    //#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    qRegisterMetaType<QLCSCTPCameraParameters>("QLCSCTPCameraParameters");
    qRegisterMetaType<serimager::QLCSCTPCameraParameters>("serimager::QLCSCTPCameraParameters");
    qRegisterMetaType<QList<QLCSCTPCameraParameters>>("QList<QLCSCTPCameraParameters>");
    qRegisterMetaType<QList<serimager::QLCSCTPCameraParameters>>("QList<serimager::QLCSCTPCameraParameters>");
    qRegisterMetaTypeStreamOperators<QLCSCTPCameraParameters>("QLCSCTPCameraParameters");
    qRegisterMetaTypeStreamOperators<serimager::QLCSCTPCameraParameters>("serimager::QLCSCTPCameraParameters");
    qRegisterMetaTypeStreamOperators<QList<QLCSCTPCameraParameters>>("QList<QLCSCTPCameraParameters>");
    qRegisterMetaTypeStreamOperators<QList<serimager::QLCSCTPCameraParameters>>("QList<serimager::QLCSCTPCameraParameters>");
    //#endif
    initialized = true;
  }
}


QLCSCTPStreams * QLCSCTPStreams::instance()
{
  static QLCSCTPStreams * _instance = new QLCSCTPStreams();
  return _instance;
}

QLCSCTPStreams::QLCSCTPStreams()
{
}

const QList<QImagingCamera::sptr> & QLCSCTPStreams::streams()
{
  return instance()->_streams;
}

void QLCSCTPStreams::add(const QLCSCTPCamera::sptr & stream)
{
  instance()->_streams.append(stream);
  instance()->save();
  Q_EMIT instance()->streamsChaged();
}

void QLCSCTPStreams::remove(const QLCSCTPCamera::sptr & stream)
{
  QList<QImagingCamera::sptr> & streams = instance()->_streams;

  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {
    if( ii->get() == stream.get() ) {
      streams.erase(ii);
      instance()->save();
      Q_EMIT instance()->streamsChaged();
      return;
    }
  }
}

bool QLCSCTPStreams::exist(const QImagingCamera::sptr & stream)
{
  const QList<QImagingCamera::sptr> & streams = instance()->_streams;
  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {
    if( ii->get() == stream.get() ) {
      return true;
    }
  }

  return false;
}

bool QLCSCTPStreams::exist(const QString & streamName)
{
  const QList<QImagingCamera::sptr> & streams = instance()->_streams;
  for( auto ii = streams.begin(); ii != streams.end(); ++ii ) {

    const QLCSCTPCamera::sptr camera =
        std::dynamic_pointer_cast<QLCSCTPCamera>(*ii);

    if( camera && camera->name() == streamName ) {
      return true;
    }
  }

  return false;
}


void QLCSCTPStreams::save()
{
  const QList<QImagingCamera::sptr> & streams = instance()->_streams;

  QList<QLCSCTPCameraParameters> cameras;

  for( const QImagingCamera::sptr &stream : streams ) {

    const QLCSCTPCamera::sptr camera =
        std::dynamic_pointer_cast<QLCSCTPCamera>(stream);

    if( camera ) {

      const QLCSCTPCameraParameters params = {
          .name = camera->name(),
          .url = camera->url()
      };

      cameras.append(params);
    }
  }

  QSettings().setValue("QLCSCTPStreams",
      QVariant::fromValue(cameras));
}

void QLCSCTPStreams::load()
{
  QList<QImagingCamera::sptr> & streams =
      instance()->_streams;

  const QList<QLCSCTPCameraParameters> cameras =
      QSettings().value("QLCSCTPStreams",
          QVariant::fromValue(QList<QLCSCTPCameraParameters>()))
            .value<QList<QLCSCTPCameraParameters>>();

  streams.clear();

  for( const QLCSCTPCameraParameters &c : cameras ) {
    streams.append(QLCSCTPCamera::create(c.name, c.url));
  }

  Q_EMIT instance()->streamsChaged();
}


QLCSCTPStreamsDialogBox::QLCSCTPStreamsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("LCSCTP streams");

  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->addWidget(streams_ctl = new QLCSCTPStreamsWidget(this));
}


QLCSCTPStreamListWidget::QLCSCTPStreamListWidget(QWidget * parent) :
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
      create_toolbutton(QIcon(), "Add", "Add new lcsctp input stream"),
      0, Qt::AlignTop);

  vbox->addWidget(removeStream_ctl =
      create_toolbutton(QIcon(), "Remove", "Remove lcsctp stream from list"),
      0, Qt::AlignTop);

  removeStream_ctl->setEnabled(false);


  hbox->addLayout(vbox);
  hbox->addWidget(list_ctl = new QListWidget());

  list_ctl->setViewMode(QListView::ViewMode::ListMode);
  list_ctl->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  list_ctl->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);


  connect(QLCSCTPStreams::instance(), &QLCSCTPStreams::streamsChaged,
      this, &ThisClass::updateStreamList);

  connect(addStream_ctl, &QToolButton::clicked,
      this, &ThisClass::onAddStreamClicked);

  connect(removeStream_ctl, &QToolButton::clicked,
      this, &ThisClass::onRemoveStreamClicked);

  connect(list_ctl, &QListWidget::currentItemChanged,
      this, &ThisClass::onCurrentListItemChanged);

  updateStreamList();
}


void QLCSCTPStreamListWidget::updateStreamList()
{
  list_ctl->clear();

  for( const auto &stream : QLCSCTPStreams::streams() ) {
    QLCSCTPCamera::sptr camera = std::dynamic_pointer_cast<QLCSCTPCamera>(stream);
    if( camera ) {
      QListWidgetItem *item = new QListWidgetItem(camera->name());
      item->setData(Qt::UserRole, QVariant::fromValue(camera));
      list_ctl->addItem(item);
    }
  }
}

void QLCSCTPStreamListWidget::onAddStreamClicked()
{
  QString streamName;

  for( int i = 0; i < 10000; ++i ) {
    streamName = QString("stream%1").arg(i);
    if( !QLCSCTPStreams::exist(streamName) ) {
      break;
    }
  }

  QLCSCTPCamera::sptr camera =
      QLCSCTPCamera::create(streamName, "");

  if ( camera ) {
    QLCSCTPStreams::add(camera);
    updateStreamList();
    selectStream(camera);
  }

}

void QLCSCTPStreamListWidget::onRemoveStreamClicked()
{
  QListWidgetItem * currentItem = list_ctl->currentItem();
  if ( currentItem ) {
    QLCSCTPCamera::sptr camera = currentItem->data(Qt::UserRole).value<QLCSCTPCamera::sptr>();// std::dynamic_pointer_cast<QLCSCTPCamera>();
    if( camera ) {

      const int responce =
          QMessageBox::question(this, "Confirmation required",
              QString("Remove stream %1?").arg(currentItem->text()));
      if ( responce == QMessageBox::Yes ) {
        QLCSCTPStreams::remove(camera);
        updateStreamList();
      }
    }
  }
}

void QLCSCTPStreamListWidget::onCurrentListItemChanged(QListWidgetItem * current, QListWidgetItem * previous)
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

QLCSCTPCamera::sptr QLCSCTPStreamListWidget::selectedStream() const
{
  QListWidgetItem *currentItem =
      list_ctl->currentItem();

  return currentItem ?
      std::dynamic_pointer_cast<QLCSCTPCamera>(currentItem->
          data(Qt::UserRole).value<QLCSCTPCamera::sptr>()) :
      nullptr;
}

void QLCSCTPStreamListWidget::selectStream(const QLCSCTPCamera::sptr & camera)
{
  for ( int i = 0, n = list_ctl->count(); i < n; ++i ) {
    QListWidgetItem * item = list_ctl->item(i);
    QImagingCamera::sptr c = item->data(Qt::UserRole).value<QLCSCTPCamera::sptr>();
    if ( c == camera ) {
      list_ctl->setCurrentItem(item);
      break;
    }
  }
}

QLCSCTPStreamsWidget::QLCSCTPStreamsWidget(QWidget * parent) :
    Base(parent)
{
  ///
  streamName_ctl =
      add_textbox("Name:",
          "",
          [this](const QString & value) {
            if ( _selectedStream ) {
              _selectedStream->setName(value);
              QLCSCTPStreams::save();
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
  addRow("URL:", streamUrl_ctl = new QLCSCTPUrlWidget());
  QObject::connect(streamUrl_ctl, &QLCSCTPUrlWidget::urlChanged,
      [this]() {
        if ( !updatingControls() && _selectedStream ) {
          _selectedStream->setUrl(streamUrl_ctl->url());
          QLCSCTPStreams::save();
        }
      });
  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _selectedStream ) {
          streamUrl_ctl->setUrl(_selectedStream->url());
        }
      });


  ///
  addRow(list_ctl = new QLCSCTPStreamListWidget());
  connect(list_ctl, & QLCSCTPStreamListWidget::selectedStreamChanged,
      this, &ThisClass::onSelectedStreamChanged);

  ///

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        const bool enable = _selectedStream != nullptr;
        streamName_ctl->setEnabled(enable);
        streamUrl_ctl->setEnabled(enable);
      });

  updateControls();
}

void QLCSCTPStreamsWidget::onSelectedStreamChanged()
{
  if( _selectedStream ) {
    QObject::disconnect(_selectedStream.get(),
        nullptr, this, nullptr);
  }

  if( (_selectedStream = list_ctl->selectedStream()) ) {
    QObject::connect(_selectedStream.get(), &QLCSCTPCamera::parametersChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}

//void QLCSCTPStreamsWidget::onupdatecontrols()
//{
//  const bool enable =
//      selectedStream_ != nullptr;
//
//  if ( enable ) {
//    streamUrl_ctl->setUrl(selectedStream_->url());
//  }
//
//  streamName_ctl->setEnabled(enable);
//  streamUrl_ctl->setEnabled(enable);
//
//  Base::onupdatecontrols();
//}

} /* namespace serimager */
