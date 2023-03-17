/*
 * QFFStreams.h
 *
 *  Created on: Mar 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFFStreams_h__
#define __QFFStreams_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QFFMPEGCameraUrlWidget.h"
#include "QFFMPEGCamera.h"

namespace serimager {

class QFFStreamListWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QFFStreamListWidget ThisClass;
  typedef QWidget Base;

  QFFStreamListWidget(QWidget * parent = nullptr);

  QFFMPEGCamera::sptr selectedStream() const;

  void selectStream(const QFFMPEGCamera::sptr & camera);

Q_SIGNALS:
  void selectedStreamChanged();

protected Q_SLOTS:
  void updateStreamList();
  void onAddStreamClicked();
  void onRemoveStreamClicked();
  void onCurrentListItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

protected:

protected:
  QHBoxLayout * hbox = nullptr;
  QVBoxLayout * vbox = nullptr;
  QListWidget * list_ctl = nullptr;
  QToolButton * addStream_ctl = nullptr;
  QToolButton * removeStream_ctl = nullptr;
};

class QFFStreamsWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFFStreamsWidget ThisClass;
  typedef QSettingsWidget Base;

  QFFStreamsWidget(QWidget * parent);

protected:
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onSelectedStreamChanged();

protected:
  QFFMPEGCamera::sptr selectedStream_;
  QLineEditBox * streamName_ctl = nullptr;
  QFFMPEGCameraUrlWidget * streamUrl_ctl = nullptr;
  QLineEditBox * streamOpts_ctl = nullptr;
  QFFStreamListWidget * list_ctl = nullptr;
};

class QFFStreamsDialogBox:
    public QDialog
{
public:
  typedef QFFStreamsDialogBox ThisClass;
  typedef QDialog Base;

  QFFStreamsDialogBox(QWidget * parent = nullptr);

protected:
  QFFStreamsWidget * streams_ctl = nullptr;
};

class QFFStreams :
    public QObject
{
  Q_OBJECT;
public:

  static QFFStreams * instance();

  static const QList<QImagingCamera::sptr> & streams();

  static void add(const QFFMPEGCamera::sptr & stream);
  static void remove(const QFFMPEGCamera::sptr & stream);
  static bool exist(const QImagingCamera::sptr & stream);
  static bool exist(const QString & streamName);
  static void save();
  static void load();

Q_SIGNALS:
  void streamsChaged();

protected: // Indented to be a singleton
  QFFStreams();

protected:
  static QList<QImagingCamera::sptr> streams_;
};



} // namespace serimager

#endif /* __QFFStreams_h__ */
