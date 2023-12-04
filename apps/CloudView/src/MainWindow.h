/*
 * MainWindow.h
 *
 *  Created at Sat, November 18, 2023
 *      Author: amyznikov
 */
#pragma once
#ifndef __qcloudview_main_window_h__
#define __qcloudview_main_window_h__

#include <gui/mainwindow/QMainAppWindow.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include "QCloudViewDatasetCollectionsView.h"
#include "QCloudSequenceView.h"
#include "QCloudViewImageEditor.h"


//#include <gui/qtextview/QTextFileViewer.h>
//#include <gui/qcloudview/QCloudViewer.h>
//#include <gui/qcloudview/QCloudViewSettings.h>
//#include <gui/qimageview/QImageViewOptions.h>
//#include <gui/qgraphicsshape/QShapesButton.h>
//#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
//#include <gui/qimagesequencetreeview/QImageSequencesTreeView.h>
//#include <gui/widgets/QScaleSelectionButton.h>
//#include <gui/qpipeline/QPipelineOptionsView.h>
//#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
//#include <gui/qinputsequenceview/QInputSequenceView.h>
//#include "QAppSettings.h"
//#include "QPipelineProgressView.h"
//#include "QSerStackerImageEditor.h"

namespace cloudview {
///////////////////////////////////////////////////////////////////////////////


class MainWindow :
    public QMainAppWindow
{
  Q_OBJECT;
public:
  typedef MainWindow ThisClass;
  typedef QMainAppWindow Base;

  MainWindow();
  ~MainWindow();

private:
  void setupMainMenu();
  void setupStatusbar();
  void setupFileSystemTreeView();
  void setupThumbnailsView();
  void setupDatasetView();
  void setupCloudSequenceView();

private:
  void updateWindowTittle();
  void openImage(const QString & abspath);

  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;

  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);



private:
  QStackedWidget * centralStackedWidget = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;
  QCloudSequenceView * cloudSequenceView = nullptr;
  QCloudViewImageEditor * imageView = nullptr;
  QCloudViewer * cloudView = nullptr;
  QTextFileViewer * textView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;

  QCloudViewDatasetCollectionsView * datasetView = nullptr;
  QCloudViewDatasetCollectionsDock * datasetViewDock = nullptr;


  QAction * quitAppAction = nullptr;



  // status bar
  QLabel * statusbarMousePosLabel_ctl = nullptr;
  QLabel * statusbarShapesLabel_ctl = nullptr;
  QToolButton * statusbarShowLog_ctl = nullptr;

};


///////////////////////////////////////////////////////////////////////////////
}  // namespace cloudview
#endif /* __qcloudview_main_window_h__ */
