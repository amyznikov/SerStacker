/*
 * QInputSequencesTreeView.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#ifndef __QImageSequencesTreeView_h__
#define __QImageSequencesTreeView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomdock/QCustomDock.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/widgets/UpdateControls.h>

class QInputSequencesTreeView;
class QInputSequencesTree;

enum {
  QInputSequenceTreeItemType = QTreeWidgetItem::UserType + 1,
  QInputSourceTreeItemType = QTreeWidgetItem::UserType + 2,
};

class QInputSequenceTreeItem :
    public QTreeWidgetItem
{
public:
  typedef QInputSequenceTreeItem ThisClass;
  typedef QTreeWidgetItem Base;

  QInputSequenceTreeItem(QTreeWidget * treeview, const c_image_sequence::sptr & image_sequence);
  void refreshInputSources();

  const c_image_sequence::sptr& input_sequence() const;
  void set_input_sequence(const c_image_sequence::sptr & image_sequence);

  bool isRunnng() const;


protected:
  c_image_sequence::sptr input_sequence_;
};

class QInputSourceTreeItem :
    public QTreeWidgetItem
{
public:
  typedef QInputSourceTreeItem ThisClass;
  typedef QTreeWidgetItem Base;

  QInputSourceTreeItem(const c_input_source::sptr & input_source, QTreeWidgetItem * parent = nullptr );

  const c_input_source::sptr & input_source() const;
  void set_input_source(const c_input_source::sptr & input_source ) ;

  void setCkecked(bool v);
  void updateCheckState();

protected:
  c_input_source::sptr input_source_;
};


class QInputSequencesTreeView :
    public QTreeWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QInputSequencesTreeView ThisClass;
  typedef QTreeWidget Base;
  friend class QInputSequencesTree;


  QInputSequencesTreeView(QWidget * parent = nullptr);

  QInputSequenceTreeItem * getImageSequenceItem(QTreeWidgetItem * item) const;
  QInputSequenceTreeItem * findImageSequenceItem(const QString & name) const;
  QInputSequenceTreeItem * findImageSequenceItem(const c_input_sequence::sptr & image_sequence) const;
  QInputSourceTreeItem * findInputSourceItem(QInputSequenceTreeItem * stackItem, const QString & name) const;

  void loadSequences(const std::string & cfgfilename = "");
  void saveSequences(const std::string & cfgfilename = "");

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
  //void populateTreeView();
  QInputSequenceTreeItem * addImageSequenceItem(const c_image_sequence::sptr & image_sequence);
  QInputSequenceTreeItem * addNewImageSequence(const QString & name = QString());
  void deleteItems(QList<QTreeWidgetItem*> & items);

  void keyPressEvent(QKeyEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;

  Qt::DropActions supportedDropActions() const override;
  void dragEnterEvent(QDragEnterEvent *event) override;
  void dragMoveEvent(QDragMoveEvent *event) override;
  void dropEvent(QDropEvent *event) override;
  int dropSources(QDropEvent *e, QInputSequenceTreeItem * targetStackItem, QTreeWidgetItem * targetItem);
  bool dropSource(QDropEvent *e, const QUrl & url, QInputSequenceTreeItem * targetStackItem, QTreeWidgetItem * targetItem);


protected:
  std::string config_filename_;
  static std::string default_config_filename_;
};

class QInputSequencesTree :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QInputSequencesTree ThisClass;
  typedef QWidget Base;

  QInputSequencesTree(QWidget * parent = nullptr);

  void loadSequences(const std::string & cfgfilename = "");
  void saveSequences(const std::string & cfgfilename = "");

  c_input_source::sptr getCurrentInputSource(c_image_sequence::sptr * parent_sequence = nullptr) const;

  const QList<QAction * > & toolbarActions() const;

  void getSelectedSequences(std::vector<c_image_sequence::sptr> * sequences) const;


Q_SIGNALS:
  void imageSequenceCollectionChanged();
  void currentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & inputSource);
  void itemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & inputSource);
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
  void onShowPipelineOptionsClicked();
  void onStartPipelineClicked();
  void onStartAllPipelinesClicked();
  void onStopPipelineClicked();
  void onPipelineThreadStarting();
  void onPipelineThreadStarted();
  void onPipelineThreadFinishing();
  void onPipelineThreadFinished();
  bool startNextStacking();

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QVBoxLayout * vbox_ = nullptr;
  QInputSequencesTreeView * treeView_ = nullptr;

  QAction * addImageSequenceAction = nullptr;
  QAction * addSourcesAction = nullptr;
  QAction * deleteItemAction = nullptr;
  QAction * showStackOptionsAction = nullptr;
  //QAction * startStopStackingAction = nullptr;

  QMenu *  startMenu = nullptr;
  QAction * startMenuAction = nullptr;
  QAction * startAction = nullptr;
  QAction * startAllAction = nullptr;
  QAction * stopAction = nullptr;

  QList<QAction * > toolbarActions_;

  enum {
    ProcessIdle,
    ProcessSingleStack,
    ProcessBatch,
  } currentProcessingMode_ = ProcessIdle;
};



class QInputSequenceTreeDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QInputSequenceTreeDock ThisClass;
  typedef QCustomDockWidget Base;

  QInputSequenceTreeDock(const QString &title, QWidget * parent = nullptr);

  QInputSequencesTree * treeView() const;

protected:
  QInputSequencesTree * treeView_ = nullptr;
};


QInputSequenceTreeDock * addInputSequenceTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);


#endif /* __QImageSequencesTreeView_h__ */
