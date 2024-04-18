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
#include <gui/widgets/QSettingsWidget.h>
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

  void set_current_processor(const c_image_processor::sptr & p);
  const c_image_processor::sptr & current_processor() const;

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
  c_image_processor::sptr current_processor_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QTreeWidget * tree_ctl = nullptr;

  QAction * moveDownAction_ = nullptr;
  QAction * moveUpAction_ = nullptr;
  QAction * addProcAction_ = nullptr;
  QAction * removeProcAction_ = nullptr;
};


class QImageProcessorSettingsControl :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorSettingsControl ThisClass;
  typedef QSettingsWidget Base;

  static QImageProcessorSettingsControl * create(const c_image_processor_routine::ptr & processor,
      QWidget * parent = nullptr);

protected:
  QImageProcessorSettingsControl(const c_image_processor_routine::ptr & processor, QWidget * parent = nullptr);
  virtual void setupControls();
  void onupdatecontrols() override;

protected:
  c_image_processor_routine::ptr processor_;
};

#endif /* __QImageProcessorChainEditor_h__ */
