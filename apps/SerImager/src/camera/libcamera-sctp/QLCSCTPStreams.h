/*
 * QLCSCTStreams.h
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLCSCTStreams_h__
#define __QLCSCTStreams_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QLCSCTPCamera.h"
#include "QLCSCTPUrlWidget.h"

namespace serimager {

class QLCSCTPStreamListWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QLCSCTPStreamListWidget ThisClass;
  typedef QWidget Base;

  QLCSCTPStreamListWidget(QWidget * parent = nullptr);

  QLCSCTPCamera::sptr selectedStream() const;

  void selectStream(const QLCSCTPCamera::sptr & camera);

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

class QLCSCTPStreamsWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLCSCTPStreamsWidget ThisClass;
  typedef QSettingsWidget Base;

  QLCSCTPStreamsWidget(QWidget * parent);

protected:
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onSelectedStreamChanged();

protected:
  QLCSCTPCamera::sptr selectedStream_;
  QLineEditBox * streamName_ctl = nullptr;
  QLCSCTPUrlWidget * streamUrl_ctl = nullptr;
  QLCSCTPStreamListWidget * list_ctl = nullptr;
};

class QLCSCTPStreamsDialogBox:
    public QDialog
{
public:
  typedef QLCSCTPStreamsDialogBox ThisClass;
  typedef QDialog Base;

  QLCSCTPStreamsDialogBox(QWidget * parent = nullptr);

protected:
  QLCSCTPStreamsWidget * streams_ctl = nullptr;
};

class QLCSCTPStreams :
    public QObject
{
  Q_OBJECT;
public:

  static void registerMetaTypes();
  static QLCSCTPStreams * instance();

  static const QList<QImagingCamera::sptr> & streams();

  static void add(const QLCSCTPCamera::sptr & stream);
  static void remove(const QLCSCTPCamera::sptr & stream);
  static bool exist(const QImagingCamera::sptr & stream);
  static bool exist(const QString & streamName);
  static void save();
  static void load();

Q_SIGNALS:
  void streamsChaged();

protected: // Indented to be a singleton
  QLCSCTPStreams();

protected:
  QList<QImagingCamera::sptr> streams_;
};

} /* namespace serimager */

#endif /* __QLCSCTStreams_h__ */
