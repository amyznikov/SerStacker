/*
 * QImageProcessorChainEditor.h
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageProcessorChainEditor_h__
#define __QImageProcessorChainEditor_h__

//#include "QImageProcessorRoutineSettings.h"
#include <gui/widgets/QSettingsWidgetTemplate.h>
#include <gui/widgets/UpdateControls.h>
#include <core/improc/c_image_processor.h>

class QImageProcessorChainEditor :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImageProcessorChainEditor ThisClass;
  typedef QFrame Base;

  QImageProcessorChainEditor(QWidget * parent = nullptr);

  void setCurrentProcessor(const c_image_processor::sptr & p);
  const c_image_processor::sptr & currentProcessor() const;

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
  QTreeWidgetItem * insertProcessorItem(int index, const c_image_processor_routine::ptr & routine);

protected:
  c_image_processor::sptr _currentProcessor;

  QVBoxLayout * _lv = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QTreeWidget * tree_ctl = nullptr;

  QAction * _moveDownAction = nullptr;
  QAction * _moveUpAction = nullptr;
  QAction * _addProcAction = nullptr;
  QAction * _removeProcAction = nullptr;
};

class QImageProcessorSettingsControl2 :
    public QSettingsWidgetTemplate<c_image_processor_routine>
{
  Q_OBJECT;
public:
  typedef QImageProcessorSettingsControl2 ThisClass;
  typedef QSettingsWidgetTemplate<c_image_processor_routine> Base;

  static QImageProcessorSettingsControl2 * create(const c_image_processor_routine::ptr & processor,
      QWidget * parent = nullptr);

protected:
  QImageProcessorSettingsControl2(const c_image_processor_routine::ptr & processor, QWidget * parent = nullptr);
  virtual void setupControls();

protected:
  c_image_processor_routine::ptr _processor;
};

#endif /* __QImageProcessorChainEditor_h__ */
