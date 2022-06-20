/*
 * MainWindow.h
 *
 *  Created on: Tuesday, December 1, 2020
 *      Author: amyznikov
 */
#pragma once
#ifndef __qskystacker_main_window_h__
#define __qskystacker_main_window_h__

#include <QtWidgets/QtWidgets>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qmtf/QMtfControl.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qstackingthread/QStackingProgressView.h>
#include <gui/qstackingoptions/QStackOptions.h>
#include <gui/qstacktreeview/QStackTreeViewDock.h>
#include <gui/qimproc/QImageProcessorSelector.h>
#include "QImageEditor.h"

#if HAVE_QGLViewer // Should come from CMakeLists.txt
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qcloudview/QCloudViewSettings.h>
#endif // HAVE_QGLViewer

namespace qserstacker {
///////////////////////////////////////////////////////////////////////////////


class MainWindow
    : public QMainWindow
{
  Q_OBJECT;
public:
  typedef MainWindow ThisClass;
  typedef QMainWindow Base;

  MainWindow();
  ~MainWindow();

private:
  void saveGeometry();
  void restoreGeometry();
  void saveState();
  void restoreState();
  void configureImageViewerToolbars();
  void configureTextViewerToolbars();
  void configureCloudViewerToolbars();
  void createDisplaySettingsControl();
  bool eventFilter(QObject *watched, QEvent *event) override;

public slots:
  void onSaveCurrentImageAs();
  void onSaveCurrentDisplayImageAs();
  void onLoadStackConfig();

private slots:
  void updateWindowTittle();
  void openImage(const QString & abspath);
  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_stacking_options::ptr & currentStack,
      const c_input_source::ptr & currentInputSource);
  void onStackTreeItemDoubleClicked(const c_image_stacking_options::ptr & stack,
      const c_input_source::ptr & inputSource);
  void onDisplaySettingsMenuActionClicked(bool checked);

//  void onInputSourceDoubleClicked(const c_input_source::ptr & input_source);
//  void onCurrentInputSourceChanged(const c_input_source::ptr & input_source);
  void onShowStackOptionsClicked(const c_image_stacking_options::ptr & ppline);
  void onStackingThreadStarted();
  void onStackingThreadFinished();

  void saveCurrentWork();

private:
  c_image_stacks_collection::ptr stacklist_ = c_image_stacks_collection::create();

  QStackedWidget * centralStackedWidget = Q_NULLPTR;
  QMtfControlDialogBox * mtfControl = Q_NULLPTR;
  QThumbnailsView * thumbnailsView = Q_NULLPTR;
  QImageEditor * imageEditor = Q_NULLPTR;
  QTextFileViewer * textViewer = Q_NULLPTR;

#if HAVE_QGLViewer
  QCloudViewer * cloudViewer = Q_NULLPTR;
  QCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = Q_NULLPTR;
#endif // HAVE_QGLViewer

  QStackOptions * stackOptionsView = Q_NULLPTR;
  QStackingProgressView * stackProgressView = Q_NULLPTR;

  QFileSystemTreeDock * fileSystemTreeDock = Q_NULLPTR;

  QStackTreeViewDock * stackTreeDock = Q_NULLPTR;
  QStackTree * stackTreeView = Q_NULLPTR;

  QCustomDockWidget * imageProcessorSelectorDock = Q_NULLPTR;
  QImageProcessorSelector * imageProcessorSelector = Q_NULLPTR;

  QMenu * fileMenu = Q_NULLPTR;
  QMenu * viewMenu = Q_NULLPTR;
  QMenu * editMenu = Q_NULLPTR;

  QAction * quitAppAction = Q_NULLPTR;
  QAction * saveImageAsAction = Q_NULLPTR;
  QAction * saveDisplayImageAsAction = Q_NULLPTR;
  QAction * loadStackAction = Q_NULLPTR;
  QAction * setReferenceFrameAction = Q_NULLPTR;
  QAction * copyDisplayImageAction = Q_NULLPTR;
  QAction * displaySettingsMenuAction = Q_NULLPTR;
  QAction * editMaskAction = Q_NULLPTR;

};


///////////////////////////////////////////////////////////////////////////////
}  // namespace qserstacker
#endif /* __qskystacker_main_window_h__ */
