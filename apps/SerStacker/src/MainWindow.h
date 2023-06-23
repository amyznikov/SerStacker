/*
 * MainWindow.h
 *
 *  Created on: Tuesday, December 1, 2020
 *      Author: amyznikov
 */
#pragma once
#ifndef __qskystacker_main_window_h__
#define __qskystacker_main_window_h__

#include <gui/mainwindow/QMainAppWindow.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qcloudview/QCloudViewSettings.h>
#include <gui/qimageview/QImageViewOptions.h>
#include <gui/qgraphicsshape/QShapesButton.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qimagesequencetreeview/QImageSequenceTreeDock.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/qpipelineoptions/QPipelineOptionsView.h>
#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
#include "QImageEditor.h"
#include "QAppSettings.h"
#include "QPipelineProgressView.h"

namespace serstacker {
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
  void setupPipelineTypes();
  void setupMainMenu();
  void setupFileSystemTreeView();
  void setupThumbnailsView();
  void setupStackTreeView();
  void setupStackOptionsView();
  void setupImageEditor();
  void setupTextViewer();
  void stupCloudViewer();
  void setupRoiOptions();
  void setupImageViewOptions();
  void setupDisplayImageVideoWriter();
  void setupStatusbar();

private Q_SLOTS:
  void updateWindowTittle();
  void onSaveCurrentImageAs();
  void onSaveCurrentDisplayImageAs();
  void onSaveCurrentImageMask();
  void onLoadCurrentImageMask();
  void onLoadStackConfig();
  void onViewInputOptions();
  void onStackProgressViewTextChanged();
  void openImage(const QString & abspath);
  void onImageEditorVisibilityChanged(bool visible);
  void onImageEditorCurrentFileNameChanged();
  void onImageEditorCurrentImageChanged();
  void onImageEditorDisplayImageChanged();
  void onImageEditorCheckIfBadFrameSelected();
  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void updateMeasurements();

  //  void onInputSourceDoubleClicked(const c_input_source::ptr & input_source);
  //  void onCurrentInputSourceChanged(const c_input_source::ptr & input_source);
  void onShowImageSequenceOptions(const c_image_sequence::sptr & sequence);
  void onStackingThreadStarted();
  void onStackingThreadFinished();

  void saveCurrentWork();

private :
  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;
  void onImageProcessorParameterChanged() override;
  void onMeasureRightNowRequested() override;

private:
  c_image_sequence_collection::sptr image_sequences_ =
      c_image_sequence_collection::sptr(new c_image_sequence_collection());

  QStackedWidget * centralStackedWidget = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;
  QImageEditor * imageEditor = nullptr;
  QTextFileViewer * textViewer = nullptr;
  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
  QGeneralAppSettingsDialogBox * inputOptionsDlgBox = nullptr;

  QCloudViewer * cloudViewer = nullptr;
  QCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;

  QPipelineProgressView * pipelineProgressView = nullptr;
  QPipelineOptionsView * pipelineOptionsView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;

  QImageSequenceTree * sequencesTreeView = nullptr;
  QImageSequenceTreeDock * sequencesTreeDock = nullptr;



  QAction * showRoiAction_ = nullptr;
  QToolButton * roiActionsButton_ = nullptr;
  QGraphicsRectShapeSettingsDialogBox * roiOptionsDialogBox_ = nullptr;
  QMenu roiActionsMenu_;


  QAction * quitAppAction = nullptr;
  QAction * saveImageAsAction = nullptr;
  QAction * saveDisplayImageAsAction = nullptr;
  QAction * saveImageMaskAction = nullptr;
  QAction * loadStackAction = nullptr;
  QAction * setReferenceFrameAction = nullptr;
  QAction * copyDisplayImageAction = nullptr;
  QAction * copyDisplayViewportAction = nullptr;
  //QAction * displaySettingsMenuAction = nullptr;
  QAction * editMaskAction = nullptr;
  QAction * loadImageMaskAction = nullptr;
  QAction * badframeAction = nullptr;
  QAction * viewInputOptionsAction = nullptr;
  QAction * closeImageViewAction_ = nullptr;

  QAction * selectPreviousFileAction_ = nullptr;
  QAction * selectNextFileAction_ = nullptr;
  QAction * reloadCurrentFileAction_ = nullptr;
  QAction * showImageProcessorSettingsAction_ = nullptr;

  QLabel * imageNameLabel_ctl = nullptr;
  QLabel * imageSizeLabel_ctl = nullptr;
  QScaleSelectionButton * scaleSelection_ctl = nullptr;
  QShapesButton * shapes_ctl = nullptr;

  QLabel * mousePosLabel_ctl = nullptr;
  QLabel * shapesLabel_ctl = nullptr;
  QToolButton * showLog_ctl = nullptr;

  ///
  QDisplayVideoWriter diplayImageWriter_;
  QToolButton* displayImageVideoWriterToolButton_ = nullptr;
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
#endif /* __qskystacker_main_window_h__ */
