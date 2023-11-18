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
  void setupFileSystemTreeView();

private:
  void updateWindowTittle();

  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;

  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);


private:
  QStackedWidget * centralStackedWidget = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;


  QAction * quitAppAction = nullptr;
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace cloudview
#endif /* __qcloudview_main_window_h__ */
