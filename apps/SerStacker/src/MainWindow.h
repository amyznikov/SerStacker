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
#include <gui/qimagesequencetreeview/QImageSequencesTreeView.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/qpipeline/QPipelineOptionsView.h>
#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
#include <gui/qinputsequenceview/QInputSequenceView.h>
#include "QAppSettings.h"
#include "QPipelineProgressView.h"
#include "QSerStackerImageEditor.h"

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
  void setupPipelines();
  void setupMainMenu();
  void setupFileSystemTreeView();
  void setupThumbnailsView();
  void setupStackTreeView();
  void setupStackOptionsView();
  void setupInputSequenceView();
  void setupDisplayImageVideoWriter();
  void setupStatusbar();
  void showImageViewOptions(bool show);

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
  void checkIfBadFrameSelected();

  void onCurrentViewVisibilityChanged();
  void onCurrentViewDisplayImageChanged();
  void onImageViewCurrentImageChanged();

  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);

  void updateMeasurements();
  void updateProfileGraph(QGraphicsItem * lineItem = nullptr) override;

  void onShowImageSequenceOptions(const c_image_sequence::sptr & sequence);
  void onPipelineThreadStarted();
  void onPipelineThreadFinished();

  void onShowCloudViewSettingsDialogBoxActionClicked(bool checked);

  void saveCurrentWork();

private :
  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;
  void onImageProcessorParameterChanged() override;
  void onMeasureRightNowRequested() override;

private:
  QStackedWidget * centralStackedWidget = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;
  QInputSequenceView * inputSequenceView = nullptr;
  QSerStackerImageEditor * imageView = nullptr;
  QCloudViewer * cloudView = nullptr;
  QTextFileViewer * textView = nullptr;
  QPipelineProgressView * pipelineProgressView = nullptr;

  QGeneralAppSettingsDialogBox * inputOptionsDlgBox = nullptr;

  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
  QCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;
  QPipelineOptionsView * pipelineOptionsView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;
  QImageSequencesTree * sequencesTreeView = nullptr;
  QImageSequenceTreeDock * sequencesTreeViewDock = nullptr;


  QGraphicsRectShapeSettingsDialogBox * roiOptionsDialogBox_ = nullptr;
  QAction * showRoiOptionsAction = nullptr;
  QAction * showRoiRectangleAction = nullptr;
  QMenu roiActionsMenu_;


  QAction * quitAppAction = nullptr;
  QAction * saveImageAsAction = nullptr;
  QAction * saveDisplayImageAsAction = nullptr;
  QAction * saveImageMaskAction = nullptr;
  QAction * loadStackAction = nullptr;
  QAction * setReferenceFrameAction = nullptr;
  QAction * copyDisplayImageAction = nullptr;
  QAction * copyDisplayViewportAction = nullptr;
  QAction * editMaskAction = nullptr;
  QAction * loadImageMaskAction = nullptr;
  QAction * badframeAction = nullptr;
  QAction * viewInputOptionsAction = nullptr;



  QAction * selectPreviousFileAction_ = nullptr;
  QAction * selectNextFileAction = nullptr;
  QAction * reloadCurrentFileAction = nullptr;
  QAction * showImageProcessorSettingsAction = nullptr;
  QAction * showCloudViewSettingsDialogBoxAction = nullptr;

  QLabel * currentFileNameLabel_ctl = nullptr;
  QLabel * imageSizeLabel_ctl = nullptr;
  QScaleSelectionButton * scaleSelection_ctl = nullptr;
  QShapesButton * shapes_ctl = nullptr;

  QLabel * statusbarMousePosLabel_ctl = nullptr;
  QLabel * statusbarShapesLabel_ctl = nullptr;
  QToolButton * statusbarShowLog_ctl = nullptr;

  ///
  QDisplayVideoWriter diplayImageWriter_;
  QToolButton* displayImageVideoWriterToolButton_ = nullptr;
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
#endif /* __qskystacker_main_window_h__ */
