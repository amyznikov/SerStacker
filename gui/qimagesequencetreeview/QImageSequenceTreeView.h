/*
 * QImageSequencesTreeView.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __QImageSequencesTreeView_h__
#define __QImageSequencesTreeView_h__

#include <QtWidgets/QtWidgets>
#include <core/pipeline/c_image_processing_pipeline.h>

class QImageSequenceTreeView;
class QImageSequenceTree;

enum {
  QImageSequenceTreeItemType = QTreeWidgetItem::UserType + 1,
  QInputSourceTreeItemType = QTreeWidgetItem::UserType + 2,
};

class QImageSequenceTreeItem :
    public QTreeWidgetItem
{
public:
  typedef QImageSequenceTreeItem ThisClass;
  typedef QTreeWidgetItem Base;

  QImageSequenceTreeItem(QTreeWidget * treeview, const c_image_sequence::sptr & image_sequence);
  void refreshInputSources();

  const c_image_sequence::sptr& image_sequence() const;
  void set_image_sequence(const c_image_sequence::sptr & image_sequence);

protected:
  c_image_sequence::sptr image_sequence_;
};

class QInputSourceTreeItem :
    public QTreeWidgetItem
{
public:
  typedef QInputSourceTreeItem ThisClass;
  typedef QTreeWidgetItem Base;

  QInputSourceTreeItem(const c_input_source::ptr & input_source, QTreeWidgetItem * parent = nullptr );

  const c_input_source::ptr & input_source() const;
  void set_input_source(const c_input_source::ptr & input_source ) ;

  void setCkecked(bool v);
  void updateCheckState();

protected:
  c_input_source::ptr input_source_;
};


class QImageSequenceTreeView :
    public QTreeWidget
{
  Q_OBJECT;
public:
  typedef QImageSequenceTreeView ThisClass;
  typedef QTreeWidget Base;
  friend class QImageSequenceTree;


  QImageSequenceTreeView(QWidget * parent = Q_NULLPTR);

  void set_image_sequence_collection(const c_image_sequence_collection::sptr & collection);
  const c_image_sequence_collection::sptr & image_sequence_collection() const;
  void refresh();

  QImageSequenceTreeItem * findImageSequenceItem(const QString & name) const;
  QImageSequenceTreeItem * findImageSequenceItem(const c_image_sequence::sptr & image_sequence) const;
  QInputSourceTreeItem * findInputSourceItem(QImageSequenceTreeItem * stackItem, const QString & name) const;

  bool setUpdatingControls(bool v);
  bool updatingControls() const;

Q_SIGNALS:
  void imageSequenceCollectionChanged();
  void imageSequenceNameChanged(const c_image_sequence::sptr & image_sequence);
  void imageSequenceSourcesChanged(const c_image_sequence::sptr & image_sequence);

protected Q_SLOTS:
  void updateImageSequenceName(const c_image_sequence::sptr & image_sequence);
  void onAddNewImageSequence();
  void onAddSourcesToCurrentImageSequence();
  void onDeleteSelectedItems();
  void onItemChanged(QTreeWidgetItem *item, int column);

protected:
  void populateTreeView();
  QTreeWidgetItem * addImageSequenceItem(const c_image_sequence::sptr & image_sequence);
  QTreeWidgetItem * addNewImageSequence(const QString & name = QString());
  void deleteItems(QList<QTreeWidgetItem*> & items);

  void keyPressEvent(QKeyEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;

  Qt::DropActions supportedDropActions() const override;
  void dragEnterEvent(QDragEnterEvent *event) override;
  void dragMoveEvent(QDragMoveEvent *event) override;
  void dropEvent(QDropEvent *event) override;
  int dropSources(QDropEvent *e, QImageSequenceTreeItem * targetStackItem, QTreeWidgetItem * targetItem);
  bool dropSource(QDropEvent *e, const QUrl & url, QImageSequenceTreeItem * targetStackItem, QTreeWidgetItem * targetItem);


protected:
  c_image_sequence_collection::sptr image_sequence_collection_;
  bool updatingControls_ = false;
};

class QImageSequenceTree :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QImageSequenceTree ThisClass;
  typedef QWidget Base;

  QImageSequenceTree(QWidget * parent = Q_NULLPTR);

  void set_image_sequence_collection(const c_image_sequence_collection::sptr & image_sequence_collection);
  const c_image_sequence_collection::sptr & image_sequence_collection() const;

  c_input_source::ptr getCurrentInputSource(c_image_sequence::sptr * parent_sequence = nullptr) const;


  void refresh();

  const QList<QAction * > & toolbarActions() const;


Q_SIGNALS:
  void imageSequenceCollectionChanged();
  void currentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::ptr & inputSource);
  void itemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::ptr & inputSource);
  void showImageSequenceOptionsClicked(const c_image_sequence::sptr & sequence);
  void imageSequenceNameChanged(const c_image_sequence::sptr & sequence);
  void imageSequenceSourcesChanged(const c_image_sequence::sptr & sequence);
  void imageSequenceDeleted(const c_image_sequence::sptr & sequence);

public Q_SLOTS:
  void updateImageSequenceName(const c_image_sequence::sptr & ppline);
  void addNewImageSequence();
  void addSourcesToCurrentImageSequence();
  void deleteSelectedItems();
  void onCurrentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);
  void onItemDoubleClicked(QTreeWidgetItem *item, int column);
  void onCustomContextMenuRequested(const QPoint &pos);
  void onShowStackOptionsClicked();
  void onStartStackingClicked();
  void onStartAllStackingClicked();
  void onStopStackingClicked();
  void onPipelineThreadStarted();
  void onPipelineThreadFinishing();
  void onPipelineThreadFinished();
  bool startNextStacking();

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QImageSequenceTreeView * treeView_ = Q_NULLPTR;

  QAction * addImageSequenceAction = Q_NULLPTR;
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
};


#endif /* __QImageSequencesTreeView_h__ */
