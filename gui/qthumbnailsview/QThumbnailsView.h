/*
 * QThumbnailsView.h
 *
 *  Created on: Sep 29, 2019
 *      Author: amyznikov
 */

#ifndef __QThumbnailsView_h__
#define __QThumbnailsView_h__

#include <QtWidgets/QtWidgets>
#include "QSearchImageFiles.h"
#include "QThumbnailExtractor.h"
#include "QThumbnailsQuickFilterOptions.h"

class QThumbnailsListWidget :
    public QListWidget
{
  Q_OBJECT;
public:
  typedef QThumbnailsListWidget ThisClass;
  typedef QListWidget Base;

  QThumbnailsListWidget(QWidget * parent = Q_NULLPTR);

  int zoom(void) const;
  void setZoom(int z);

  void setQuickFilter(const QString & v, Qt::MatchFlags flags);
  void clearQuickFilter();
  const QString & quickFilter() const;
  Qt::MatchFlags quickFilterMatchingFlags() const;

  void addIcon(const QIcon & icon, const QString & fullPathName, const QVariant & data);
  void updateIcon(const QIcon & icon, const QString & fullPathName, const QVariant & data);
  void clear();

  QString currentItemPath();
  void selectNextIcon();
  void selectPrevIcon();

  bool deleteFiles(const QList<QListWidgetItem*> & selectedItems, bool askConfirmation = true);
  void selectAll(bool includeHiddenItems = false);

  void scheduleUpdateItemsLayout();
  void updateItemsLayout();

signals:
  //void keyPressed(QKeyEvent * e);
  void enterPressed(QListWidgetItem * current_item);
  void zoomChanged(int z);
  void currentIconChanged();
  void iconDoubleClicked(const QString & name);

protected:
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *event) override;
  //bool event(QEvent *e) override;

protected:
  virtual void onZoomChanged(void);
  bool matchQuickFilter(const QString & text);
  void quickFilterUpdateItemsVisibility(void);

private:
  QString quickFilter_;
  Qt::MatchFlags quickFilterMatchingFlags_ = Qt::MatchContains;
  int currentZoom_ = 0;
};


class QThumbnailsView
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QThumbnailsView ThisClass;
  typedef QWidget Base;

  QThumbnailsView(QWidget * parent = Q_NULLPTR);

  bool setCurrentPath(const QString & path, bool refreshNow = true);
  const QString & currentPath() const;

  void populateContextMenu(QMenu * menu, const QPoint &pos);
  QPoint contextMenuPosToGlobal(const QPoint & pos) const;

  bool moveToBads(const QString & pathfilename );


public slots:
  void displayPath(const QString & path);
  void reload();
  void cancelPendingUpdates();
  void selectNextIcon();
  void selectPrevIcon();

signals:
  void iconDoubleClicked(const QString & fullPathName);
  void iconEnterPressed(const QString & fullPathName);
  void currentIconChanged(const QString & fullPathName);
  void showInDirTreeRequested(const QString & abspath);
  void customContextMenuRequested(const QPoint &pos);

  //void openImageSequenceRequested(const QStringList & fullPathNames);
  //void currentDirectoryChanged();


private:
  void updateProgressIndicator();
  void updateCurrentStackWidget();
  void startUpdate();
  void refreshWhiteListTextMessage();

private slots:
  void onSearchImageFilesStarted();
  void onSearchImageFilesFinished();
  void onImageFileFound(int rid, const QString fullPathName);
  void onThumbnailExtractorStarted();
  void onThumbnailExtractorFinished();
  void onThumbnailExtrated(int rid, const QIcon & icon, const QString & fullPathName);
  void extractMissingThumbiails();
  void onCurrentWidgetChanged(int);
  void onCurrentItemChanged(QListWidgetItem */*current*/, QListWidgetItem */*previous*/);
  void onItemDoubleClicked(QListWidgetItem *item);
  void onItemEnterPressed(QListWidgetItem *item);
  void onShowQuickFilter();
  void showQuickFilter(const QString & wildcard = "");
  void clearQuickFilter();

  //  void onCustomContextMenuRequested(const QPoint &pos);
  //  void onViewItemAdded(QIconListViewInterface * view);
  //  void onViewItemRemoved(QIconListViewInterface * view);
  //  void onCurrentViewChanged(QIconListViewInterface * current, QIconListViewInterface * previous);
  //  void updateViewSelectorAction();

  //  void onViewItemDoubleClicked(QIconListViewInterface * view, const QString & item);
  //  void onViewItemCurrentIconChanged(QIconListViewInterface * view);
  //  void onViewItemCustomContextMenuRequested(QIconListViewInterface * view, const QPoint & pos);

private:
  QString currentPath_;
  QSearchImageFiles searchImageFiles_;
  QThumbnailExtractor thumbnailExtractor_;
  int lastSearchImageFilesRID_ = -1;

  QVBoxLayout * layout_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QStackedWidget * stack_ = Q_NULLPTR;
  QLabel * whiteSheet_ = Q_NULLPTR;
  QThumbnailsListWidget * listWidget_ = Q_NULLPTR;

  QAction * showInDirTreeAction_ = Q_NULLPTR;
  QAction * refreshAction_ = Q_NULLPTR;
  QLabel * currentPathLabel_ = Q_NULLPTR;
  QAction * quickFilterAction_ = Q_NULLPTR;
  //QAction * clearFilterAction_ = Q_NULLPTR;
  QThumbnailsQuickFilterDialogBox * quickfilterDialogBox_ = Q_NULLPTR;
  bool ignoreQuickFilterAction_ = false;
};

#endif /* __QThumbnailsView_h__ */
