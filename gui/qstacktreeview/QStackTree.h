/*
 * QStackSequencesTree.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __QStackSequencesTree_h__
#define __QStackSequencesTree_h__

#include <QtWidgets/QtWidgets>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QStackTreeView;

class QStackTree
  : public QWidget
{
  Q_OBJECT;
public:
  typedef QStackTree ThisClass;
  typedef QWidget Base;

  QStackTree(QWidget * parent = Q_NULLPTR);

  void set_stacklist(const c_image_stacks_collection::ptr & stacks);
  const c_image_stacks_collection::ptr & stacklist() const;

  void applyInputOptionsToAll(const c_input_options & options);
  void applyMasterFrameOptionsToAll(const c_master_frame_options & options);
  void applyROISelectionOptionsToAll(const c_roi_selection_options & options);
  void applyFrameUpscaleOptionsToAll(const c_frame_upscale_options & options);
  void applyFrameAccumulationOptionsToAll(const c_frame_accumulation_options & options);
  void applyFrameRegistrationOptionsToAll(const c_frame_registration_options & options);
  void applyOutputOptionsToAll(const c_image_stacking_output_options & options);
  void applyAllStackOptionsToAll(const c_image_stacking_options::ptr & currentStack);

  const QList<QAction * > & toolbarActions() const;

signals:
  void stackCollectionChanged();
  void currentItemChanged(const c_image_stacking_options::ptr & stack, const c_input_source::ptr & inputSource);
  void itemDoubleClicked(const c_image_stacking_options::ptr & stack, const c_input_source::ptr & inputSource);
  //void currentInputSourceChanged(const c_input_source::ptr & input_source);
  //void inputSourceDoubleClicked(const c_input_source::ptr & input_source);
  void showStackOptionsClicked(const c_image_stacking_options::ptr & stack);
  void stackNameChanged(const c_image_stacking_options::ptr & stack);
  void stackSourcesChanged(const c_image_stacking_options::ptr & stack);
  void stackDeleted(const c_image_stacking_options::ptr & stack);

public slots:
  void updateStackName(const c_image_stacking_options::ptr & ppline);
  void addNewStack();
  void addSourcesToCurrentStack();
  void deleteSelectedItems();
  void onItemChanged(QTreeWidgetItem *item, int column);
  void onCurrentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);
  void onItemDoubleClicked(QTreeWidgetItem *item, int column);
  void onCustomContextMenuRequested(const QPoint &pos);
  void onShowStackOptionsClicked();
  void onStartStackingClicked();
  void onStartAllStackingClicked();
  void onStopStackingClicked();
  void onStackingThreadStarted();
  void onStackingThreadFinishing();
  void onStackingThreadFinished();
  bool startNextStacking();

protected:
  void updateControls();
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QStackTreeView * treeView_ = Q_NULLPTR;

  QAction * addStackAction = Q_NULLPTR;
  QAction * addSourcesAction = Q_NULLPTR;
  QAction * deleteItemAction = Q_NULLPTR;
  QAction * showStackOptionsAction = Q_NULLPTR;
  //QAction * startStopStackingAction = Q_NULLPTR;

  QMenu *  startStacking = Q_NULLPTR;
  QAction * startStackingMenuAction = Q_NULLPTR;
  QAction * startAction = Q_NULLPTR;
  QAction * startAllAction = Q_NULLPTR;
  QAction * stopStacking = Q_NULLPTR;

  QList<QAction * > toolbarActions_;

  enum {
    ProcessIdle,
    ProcessSingleStack,
    ProcessBatch,
  } currentProcessingMode_ = ProcessIdle;

  bool isItemChangeFeedBack_ = false;
};

class QStackTreeView
    : public QTreeWidget
{
  Q_OBJECT;
public:
  typedef QStackTreeView ThisClass;
  typedef QTreeWidget Base;
  friend class QStackTree;

  class QStackItem;
  class QInputSourceItem;

  QStackTreeView(QWidget * parent = Q_NULLPTR);

  void set_stacklist(const c_image_stacks_collection::ptr & stacks);
  const c_image_stacks_collection::ptr & stacklist() const;

  QStackItem * findStackItem(const QString & name) const;
  QStackItem * findStackItem(const c_image_stacking_options::ptr & stack) const;
  QInputSourceItem * findInputSourceItem(QStackItem * stackItem, const QString & name) const;

signals:
  void stackCollectionChanged();

protected slots:
  void updateStackName(const c_image_stacking_options::ptr & stack);
  void onAddNewStack();
  void onAddSourcesToCurrentStackingOptions();
  void onDeleteSelectedItems();

protected:
  void populateTreeView();
  QTreeWidgetItem * addStackItem(const c_image_stacking_options::ptr & stack);
  QTreeWidgetItem * addNewStack(const QString & name = QString());
  void deleteItems(QList<QTreeWidgetItem*> & items);

  void keyPressEvent(QKeyEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;

  Qt::DropActions supportedDropActions() const override;
  void dragEnterEvent(QDragEnterEvent *event) override;
  void dragMoveEvent(QDragMoveEvent *event) override;
  void dropEvent(QDropEvent *event) override;
  int dropSources(QDropEvent *e, QStackItem * targetStackItem, QTreeWidgetItem * targetItem);
  bool dropSource(QDropEvent *e, const QUrl & url, QStackItem * targetStackItem, QTreeWidgetItem * targetItem);


protected:
  c_image_stacks_collection::ptr stacklist_;
};

#endif /* __QStackSequencesTree_h__ */
