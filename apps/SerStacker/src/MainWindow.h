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
#include <gui/qlogwidget/QLogWidget.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qmtf/QMtfControl.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qcloudview/QCloudViewSettings.h>
#include <gui/qimproc/QImageProcessorSelector.h>
#include <gui/qimageview/QImageViewOptions.h>
#include <gui/qgraphicsshape/QShapesButton.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qimagesequencetreeview/QImageSequenceTreeDock.h>
#include <gui/qfocus/QFocusGraph.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/qpipelineoptions/QPipelineOptionsView.h>
#include <gui/qmeasure/QImageStatistics.h>
#include "focus/QImageFocusMeasure.h"
#include "QImageEditor.h"
#include "QAppSettings.h"
#include "QPipelineProgressView.h"

namespace serstacker {
///////////////////////////////////////////////////////////////////////////////


class MainWindow :
    public QMainWindow
{
  Q_OBJECT;
public:
  typedef MainWindow ThisClass;
  typedef QMainWindow Base;

  MainWindow();
  ~MainWindow();

private:
  void saveState();
  void restoreState();
  void setupPipelineTypes();
  void setupMainMenu();
  void setupLogWidget();
  void setupFileSystemTreeView();
  void setupThumbnailsView();
  void setupStackTreeView();
  void setupStackOptionsView();
  void setupImageProcessorSelector();
  void setupImageEditor();
  void setupTextViewer();
  void stupCloudViewer();
  void setupFocusGraph();
  void setupRoiOptions();

  void createDisplaySettingsControl();
  void createImageViewOptionsControl();
  bool eventFilter(QObject *watched, QEvent *event) override;

public Q_SLOTS:
  void onSaveCurrentImageAs();
  void onSaveCurrentDisplayImageAs();
  void onSaveCurrentImageMask();
  void onLoadCurrentImageMask();
  void onLoadStackConfig();
  void onViewGeneralSettings();
  void onStackProgressViewTextChanged();

private Q_SLOTS:
  void updateWindowTittle();
  void openImage(const QString & abspath);
  void onImageEditorCurrentImageChanged();
  void onImageEditorVisibilityChanged(bool visible);
  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onDisplaySettingsMenuActionClicked(bool checked);

//  void onInputSourceDoubleClicked(const c_input_source::ptr & input_source);
//  void onCurrentInputSourceChanged(const c_input_source::ptr & input_source);
  void onShowImageSequenceOptions(const c_image_sequence::sptr & sequence);
  void onStackingThreadStarted();
  void onStackingThreadFinished();

  void saveCurrentWork();

private:
  c_image_sequence_collection::sptr image_sequences_ =
      c_image_sequence_collection::sptr(new c_image_sequence_collection());

  QStackedWidget * centralStackedWidget = nullptr;
  QMtfControlDialogBox * mtfControl = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;
  QImageEditor * imageEditor = nullptr;
  QTextFileViewer * textViewer = nullptr;
  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
  QGeneralAppSettingsDialogBox * appSettingsDlgBox = nullptr;


  QLogWidget * logwidget_ctl = nullptr;
  QCustomDockWidget * logwidgetDock_ = nullptr;

  QCloudViewer * cloudViewer = nullptr;
  QCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;

  QPipelineProgressView * pipelineProgressView = nullptr;
  QPipelineOptionsView * pipelineOptionsView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;

  QImageSequenceTree * sequencesTreeView = nullptr;
  QImageSequenceTreeDock * sequencesTreeDock = nullptr;

  QCustomDockWidget * imageProcessorSelectorDock = nullptr;
  QImageProcessorSelector * imageProcessorSelector = nullptr;

  QImageFocusMeasure * focusMeasure_ = nullptr;
  QFocusGraph * focusGraph_ = nullptr;
  QFocusGraphDock * focusGraphDock_ = nullptr;

  QAction * showRoiAction_ = nullptr;
  QToolButton * roiActionsButton_ = nullptr;
  QGraphicsRectShapeSettingsDialogBox * roiOptionsDialogBox_ = nullptr;
  QImageStatisticsDisplayDialogBox * measureDialogBox_ = nullptr;
  QMenu roiActionsMenu_;


  QMenu * menuFile_ = nullptr;
  QMenu * menuEdit_ = nullptr;
  QMenu * menuView_ = nullptr;

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

  QAction * selectPreviousFileAction_ = nullptr;
  QAction * selectNextFileAction_ = nullptr;
  QAction * reloadCurrentFileAction_ = nullptr;
  QAction * showImageProcessorSettingsAction_ = nullptr;

  QLabel * imageNameLabel_ctl = nullptr;
  QLabel * imageSizeLabel_ctl = nullptr;
  QScaleSelectionButton * scaleSelection_ctl = nullptr;
  QShapesButton * shapes_ctl = nullptr;

};


///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
#endif /* __qskystacker_main_window_h__ */
