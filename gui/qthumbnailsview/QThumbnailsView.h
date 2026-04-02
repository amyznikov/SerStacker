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

  QThumbnailsListWidget(QWidget * parent = nullptr);

  int zoom(void) const;
  void setZoom(int z);

  void setQuickFilter(const QString & v, Qt::MatchFlags flags, bool invertMatch);
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

Q_SIGNALS:
  void enterPressed(QListWidgetItem * current_item);
  void zoomChanged(int z);
  void currentIconChanged();
  void iconDoubleClicked(const QString & name);

protected:
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *event) override;

protected:
  virtual void onZoomChanged(void);
  bool matchQuickFilter(const QString & text);
  void quickFilterUpdateItemsVisibility(void);

private:
  QString _quickFilter;
  Qt::MatchFlags _quickFilterMatchingFlags = Qt::MatchContains;
  bool _quickFilterInvertMatch = false;
  int _currentZoom = 0;
};


class QThumbnailsView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QThumbnailsView ThisClass;
  typedef QWidget Base;

  QThumbnailsView(QWidget * parent = nullptr);

  bool setCurrentPath(const QString & path, bool refreshNow = true);
  const QString & currentPath() const;

  void populateContextMenu(QMenu * menu, const QPoint &pos);
  QPoint contextMenuPosToGlobal(const QPoint & pos) const;

  bool moveToBads(const QString & pathfilename );


public Q_SLOTS:
  void displayPath(const QString & path);
  void reload();
  void cancelPendingUpdates();
  void selectNextIcon();
  void selectPrevIcon();

Q_SIGNALS:
  void iconDoubleClicked(const QString & fullPathName);
  void iconEnterPressed(const QString & fullPathName);
  void currentIconChanged(const QString & fullPathName);
  void showInDirTreeRequested(const QString & abspath);
  void customContextMenuRequested(const QPoint &pos);

private:
  void updateProgressIndicator();
  void updateCurrentStackWidget();
  void startUpdate();
  void refreshWhiteListTextMessage();

private Q_SLOTS:
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

private:
  QString _currentPath;
  QSearchImageFiles _searchImageFiles;
  QThumbnailExtractor _thumbnailExtractor;
  int _lastSearchImageFilesRID = -1;

  QVBoxLayout * _layout = nullptr;
  QToolBar * _toolbar = nullptr;
  QStackedWidget * _stack = nullptr;
  QLabel * _whiteSheet = nullptr;
  QThumbnailsListWidget * _listWidget = nullptr;

  QAction * _showInDirTreeAction = nullptr;
  QAction * _refreshAction = nullptr;
  QLabel * _currentPathLabel = nullptr;
  QAction * _quickFilterAction = nullptr;
  QThumbnailsQuickFilterDialogBox * _quickfilterDialogBox = nullptr;
  bool _ignoreQuickFilterAction = false;
};

#endif /* __QThumbnailsView_h__ */
