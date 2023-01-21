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
#include <gui/qimageview/QImageViewOptions.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qfocus/QFocusGraph.h>
#include <gui/qimagestats/QImageStatistics.h>
#include "focus/QImageFocusMeasure.h"
#include "QImageEditor.h"
#include "QAppSettings.h"

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
  void createImageViewOptionsControl();
  void setupFocusGraph();
  void setupRoiOptions();
  bool eventFilter(QObject *watched, QEvent *event) override;

public Q_SLOTS:
  void onSaveCurrentImageAs();
  void onSaveCurrentDisplayImageAs();
  void onSaveCurrentImageMask();
  void onLoadCurrentImageMask();
  void onLoadStackConfig();
  void onViewGeneralSettings();

private Q_SLOTS:
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

  QStackedWidget * centralStackedWidget = nullptr;
  QMtfControlDialogBox * mtfControl = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;
  QImageEditor * imageEditor = nullptr;
  QTextFileViewer * textViewer = nullptr;
  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
  QGeneralAppSettingsDialogBox * appSettingsDlgBox = nullptr;

#if HAVE_QGLViewer
  QCloudViewer * cloudViewer = nullptr;
  QCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;
#endif // HAVE_QGLViewer

  QStackOptions * stackOptionsView = nullptr;
  QStackingProgressView * stackProgressView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;

  QStackTreeViewDock * stackTreeDock = nullptr;
  QStackTree * stackTreeView = nullptr;

  QCustomDockWidget * imageProcessorSelectorDock = nullptr;
  QImageProcessorSelector * imageProcessorSelector = nullptr;


  QImageFocusMeasure * focusMeasure_ = nullptr;
  QFocusGraph * focusGraph_ = nullptr;
  QFocusGraphDock * focusGraphDock_ = nullptr;

  QAction * showRoiAction_ = nullptr;
  QToolButton * roiActionsButton_ = nullptr;
  QGraphicsRectShapeSettingsDialogBox * roiOptionsDialogBox_ = nullptr;
  QImageStatisticsDisplayDialogBox * imageStatisticsDialogBox_ = nullptr;
  QMenu roiActionsMenu_;


  QMenu * fileMenu = nullptr;
  QMenu * viewMenu = nullptr;
  QMenu * editMenu = nullptr;

  QAction * quitAppAction = nullptr;
  QAction * saveImageAsAction = nullptr;
  QAction * saveDisplayImageAsAction = nullptr;
  QAction * saveImageMaskAction = nullptr;
  QAction * loadStackAction = nullptr;
  QAction * setReferenceFrameAction = nullptr;
  QAction * copyDisplayImageAction = nullptr;
  QAction * displaySettingsMenuAction = nullptr;
  QAction * editMaskAction = nullptr;
  QAction * loadImageMaskAction = nullptr;
  QAction * badframeAction = nullptr;
  QAction * viewGeneralSettingsAction = nullptr;
  QAction * closeImageViewAction_ = nullptr;

};


///////////////////////////////////////////////////////////////////////////////
}  // namespace qserstacker
#endif /* __qskystacker_main_window_h__ */
