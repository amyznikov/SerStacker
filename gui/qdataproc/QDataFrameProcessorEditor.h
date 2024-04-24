/*
 * QDataFrameProcessorEditor.h
 *
 *  Created on: Dec 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDataFrameProcessorEditor_h__
#define __QDataFrameProcessorEditor_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/UpdateControls.h>
#include <core/dataproc/c_data_frame_processor.h>

class QDataFrameProcessorEditor :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QDataFrameProcessorEditor ThisClass;
  typedef QFrame Base;

  QDataFrameProcessorEditor(QWidget * parent = nullptr);

  void setCurrentProcessor(const c_data_frame_processor::sptr & p);
  const c_data_frame_processor::sptr & currentProcessor() const;

Q_SIGNALS:
  void parameterChanged();

protected Q_SLOTS:
  void onTreeItemChanged(QTreeWidgetItem *item, int column);
  void onCurrentTreeItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
  void onMoveCurrentProcessorDown();
  void onMoveCurrentProcessorUp();
  void onAddImageProcessor();
  void onRemoveCurrentProcessor();
  void updateItemSizeHint(QTreeWidgetItem * item);

protected:
  void onupdatecontrols() override;
  QTreeWidgetItem * insertRoutine(int index,
      const c_data_frame_processor_routine::sptr & routine);

protected:
  c_data_frame_processor::sptr currentProcessor_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QTreeWidget * tree_ctl = nullptr;

  QAction * moveDownAction_ = nullptr;
  QAction * moveUpAction_ = nullptr;
  QAction * addProcAction_ = nullptr;
  QAction * removeProcAction_ = nullptr;
};



class QDataFrameRoutineOptionsControl :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QDataFrameRoutineOptionsControl ThisClass;
  typedef QSettingsWidget Base;

  static QDataFrameRoutineOptionsControl * create(const c_data_frame_processor_routine::sptr & routine,
      QWidget * parent = nullptr);

protected:
  QDataFrameRoutineOptionsControl(const c_data_frame_processor_routine::sptr & routine,
      QWidget * parent = nullptr);

  virtual void setupControls();
  void onupdatecontrols() override;

protected:
  c_data_frame_processor_routine::sptr routine_;
  std::map<QWidget*, std::function<bool()>> state_ctls_;
};



class QDataFrameProcessorSelector :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QDataFrameProcessorSelector ThisClass;
  typedef QFrame Base;

  QDataFrameProcessorSelector(QWidget * parent = nullptr);

  c_data_frame_processor::sptr currentProcessor() const;

  QString selected_processor() const;
  void set_selected_processor(const QString & name);

  bool imageProcessingEnabled() const;

Q_SIGNALS:
  void parameterChanged();

protected:
  void onupdatecontrols() override;
  void updatecurrentprocessor();

protected Q_SLOTS:
  void onProcessorSelectorCurrentIndexChanged(int);
  void addProcessor();
  void deleteCurrentProcessor();
  void renameCurrentProcessor();
  void addCopyOfCurrentProcessor();

protected:
  c_data_frame_processor::sptr currentProcessor_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QDataFrameProcessorEditor * processor_ctl = nullptr;

  QCheckBox * enabled_ctl = nullptr;
  QComboBox * selector_ctl = nullptr;
  QToolButton * selectorMenu_ctl = nullptr;
};



class QDataFrameProcessorSelectionCombo :
    public QComboBox
{
  Q_OBJECT;
public:
  typedef QDataFrameProcessorSelectionCombo ThisClass;
  typedef QComboBox Base;

  QDataFrameProcessorSelectionCombo(QWidget * parent = nullptr);

  bool setCurrentProcessor(const c_data_frame_processor::sptr & processor);
  c_data_frame_processor::sptr currentProcessor() const;
  c_data_frame_processor::sptr processor(int index) const;

public slots:
  void refresh();
};


class QDataFrameProcessorsCollection :
    public QObject
{
  Q_OBJECT;
public:
  typedef QDataFrameProcessorsCollection ThisClass;
  typedef QObject Base;

  static bool load();
  static bool save();
  static int size();
  static bool empty();
  static const c_data_frame_processor::sptr & item(int pos);
  static void add(const c_data_frame_processor::sptr & p, bool emit_notify = true);
  static bool insert(int pos, const c_data_frame_processor::sptr & p, bool emit_notify = true);
  static bool remove(const c_data_frame_processor::sptr & p, bool emit_notify = true);
  static bool remove_at(int pos, bool emit_notify = true);
  static int indexof(const c_data_frame_processor::sptr & p);
  static int indexof(const std::string & name);
  static int indexof(const QString & name);

  // for QObject::connect()
  static QDataFrameProcessorsCollection * instance();

Q_SIGNALS:
  void collectionChanged();

protected:
  QDataFrameProcessorsCollection();
  virtual ~QDataFrameProcessorsCollection();

protected:
  static QDataFrameProcessorsCollection instance_;
};



#endif /* __QDataFrameProcessorEditor_h__ */
