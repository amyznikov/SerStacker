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
//#include <gui/widgets/QLogWidget.h>
//#include <gui/widgets/QCustomDock.h>
//#include <gui/widgets/QVideoSceneView.h>
//#include <gui/widgets/QImageViewer.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qmtfcontrols/QMtfControlDialogBox.h>
#include <gui/qstackingthread/QStackingProgressView.h>
#include <gui/qstackingoptions/QStackOptions.h>
#include <gui/qstacktreeview/QStackTreeViewDock.h>
#include "../../../gui/qimproc/QImageProcessorCollectionSettings.h"
#include "c_display_function.h"

//#include <gui/widgets/QMTFImageLevelsWidget.h>
//#include <gui/pipeline/QLibrawOutputParamsWidget.h>
//#include <gui/pipeline/QStakerBatchEditor.h>
//#include <gui/pipeline/QPipelineSettings.h>
//#include <gui/pipeline/QStackingProgressView.h>
//#include <gui/pipeline/QStackingThread.h>

//
//#include "QStackListTree.h"
//#include "QFrameStackingPipelineView.h"


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

public slots:

private slots:
  void updateWindowTittle();
  void openImage(const QString & abspath);
  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_stacking_options::ptr & currentStack,
      const c_input_source::ptr & currentInputSource);
  void onStackTreeItemDoubleClicked(const c_image_stacking_options::ptr & stack,
      const c_input_source::ptr & inputSource);

//  void onInputSourceDoubleClicked(const c_input_source::ptr & input_source);
//  void onCurrentInputSourceChanged(const c_input_source::ptr & input_source);
  void onShowStackOptionsClicked(const c_image_stacking_options::ptr & ppline);
  void onStackingThreadStarted();
  void onStackingThreadFinished();
  void updateImageProcessingOptions(QImageEditor * e);

  //void onPipelineTreeViewCurrentItemChanged();
//  void onPipelineItemPressed(const QFrameStackingPipeline::ptr & ppline);
//  void onPipelineItemClicked(const QFrameStackingPipeline::ptr & ppline);
//  void onPipelineItemDoubleClicked(const QFrameStackingPipeline::ptr & ppline);
//  void onPipelineItemActivated(const QFrameStackingPipeline::ptr & ppline);
//  void onPipelineItemEntered(const QFrameStackingPipeline::ptr & ppline);
  //void onAddStack();

private:
  c_image_stacks_collection::ptr stacklist_ = c_image_stacks_collection::create();
  c_image_processor_collection::ptr image_processors_ = c_image_processor_collection::create();
  c_display_function image_display_function_;

  QStackedWidget * centralStackedWidget = Q_NULLPTR;
  QThumbnailsView * thumbnailsView = Q_NULLPTR;
  QImageFileEditor * imageEditor = Q_NULLPTR;
  QTextFileViewer * textViewer = Q_NULLPTR;
  QStackOptions * stackOptionsView = Q_NULLPTR;
  //QScrollArea * stackingOptionsScrollArea = Q_NULLPTR;
  QStackingProgressView * stackProgressView = Q_NULLPTR;

  QMtfControlDialogBox * imageLevelsDialogBox = Q_NULLPTR;

  QFileSystemTreeDock * fileSystemTreeDock = Q_NULLPTR;

  QStackTreeViewDock * stackTreeDock = Q_NULLPTR;
  QStackTree * stackTreeView = Q_NULLPTR;

  QCustomDockWidget * imageProcessorSettingsDock = Q_NULLPTR;
  QImageProcessorCollectionSettings * imageProcessorSettings = Q_NULLPTR;
  QImageEditor * currentImageEditor = Q_NULLPTR;

  QMenu * fileMenu = Q_NULLPTR;
  QMenu * viewMenu = Q_NULLPTR;
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace qserstacker
#endif /* __qskystacker_main_window_h__ */
