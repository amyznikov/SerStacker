/*
 * MainWindow.h
 *
 *  Created on: Tuesday, December 1, 2020
 *      Author: amyznikov
 */
#pragma once
#ifndef __qdso_main_window_h__
#define __qdso_main_window_h__

#include <gui/mainwindow/QMainAppWindow.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qcloudview/QCloudViewSettings.h>
#include <gui/qglview/QGLViewPlanarGridSettings.h>
#include <gui/qimageview/QImageViewOptions.h>
#include <gui/qgraphicsshape/QShapesButton.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qimagesequencetreeview/QImageSequencesTreeView.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/qpipeline/QPipelineOptionsView.h>
#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
#include <gui/qinputoptions/QInputOptions.h>

#include "QDSOCloudView.h"
#include "QDSOImageView.h"
#include "QDSOThread.h"

//#include "QPipelineProgressView.h"
//#include "QInputSourceView.h"
//#include "QProgressImageViewer.h"

namespace qdso {
///////////////////////////////////////////////////////////////////////////////


class MainWindow :
    public QMainAppWindow,
    public dso::c_dso_display
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
  void setupMainToolbar();
  void setupDSOImageViews();
  void setupFileSystemTreeView();
  void setupThumbnailsView();
  void setupStackTreeView();
  void setupStackOptionsView();
  void setupInputSequenceView();
  void setupPipelineProgressView();
  void setupStatusbar();
  void showImageViewOptions(bool show);
  void setupDSOThread();

private Q_SLOTS:
  void updateWindowTittle();
  void onOpenDatasetConfig();
  bool openDatasetConfig(const QString & configPathFileName);
  void onStartStopDSOThread();

  void onSaveCurrentImageAs();
  void onSaveCurrentDisplayImageAs();
  void onSaveCurrentImageMask();
  void onLoadCurrentImageMask();
  void onLoadStackConfig();
  void onViewInputOptions();
  void onStackProgressViewTextChanged();
  void openImage(const QString & abspath);
  void checkIfBadFrameSelected();
  void onWriteDisplayVideo();

  void onCurrentViewVisibilityChanged();
  void onCurrentViewDisplayImageChanged();
  void onImageViewCurrentImageChanged();

  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);

  void updateMeasurements();
  void updateProfileGraph(QGraphicsItem * lineItem = nullptr) override;
  void onShowProfileGraphActionTriggered(bool checked) override;

  void onShowImageSequenceOptions(const c_image_sequence::sptr & sequence);
  void onPipelineThreadStarted();
  void onPipelineThreadFinished();

  void onShowCloudViewSettingsDialogBoxActionClicked(bool checked);


  void saveCurrentWork();

protected:
  bool needDisplayInputFrame() const override;
  void displayInputFrame(const dso::c_image_and_exposure & image, int id) override;

  bool needPushDepthImage() const override;
  void pushDepthImage(const cv::Mat & image) override;


  bool needDisplayKeyframe() const override;
  void displayKeyframe(const dso::FrameHessian* frame, bool _final, const dso::CalibHessian * HCalib) override;

  bool needDisplayKeyframes() const override;
  void displayKeyframes(const std::vector<dso::FrameHessian*>  & frames, const dso::CalibHessian * HCalib) override;

private :
  void closeEvent(QCloseEvent *event) override;
  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;
  void onImageProcessorParameterChanged() override;
  void onDataframeProcessorParameterChanged() override;
  void onMeasureRightNowRequested() override;
  void saveShapes(QSettings & settings);
  void loadShapes(const QSettings & settings);
  IMtfDisplay * getCurrentMtfDisplay() override;

private:
  c_dso_dataset_reader::uptr dataset_;
  QDSOThread dsoThread;

  QStackedWidget * centralStackedWidget = nullptr;
  QDSOCloudView * cloudView = nullptr;


//  QThumbnailsView * thumbnailsView = nullptr;
//  QInputSourceView * inputSourceView = nullptr;
//  QImageSourceView * imageView = nullptr;
//  QPointCloudSourceView * cloudView = nullptr;
//  QTextSourceView * textView = nullptr;
//

  QToolBar * mainToolbar_ = nullptr;
  QMenu * menuViewDsoImages = nullptr;

  QDSOImageView * dsoInputFrameView = nullptr;
  QDSOImageViewDock * dsoInputFrameViewDock = nullptr;

  QDSOImageView * dsoDepthImageView = nullptr;
  QDSOImageViewDock * dsoDepthImageViewDock = nullptr;

  QDSOImageView * dsoKeyframeView = nullptr;
  QDSOImageViewDock * dsoKeyframeViewDock = nullptr;



  QAction * openDsoDatasetAction = nullptr;
  QAction * startStopDSOThreadAction = nullptr;
  QAction * quitAppAction = nullptr;

//  QPipelineProgressView * pipelineProgressView = nullptr;
//  QProgressImageViewer * pipelineProgressImageView = nullptr;

//  QInputOptionsDialogBox * inputOptionsDlgBox = nullptr;
//
//  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
//  QPipelineOptionsView * pipelineOptionsView = nullptr;
//
//  QFileSystemTreeDock * fileSystemTreeDock = nullptr;
//  QImageSequencesTree * sequencesTreeView = nullptr;
//  QImageSequenceTreeDock * sequencesTreeViewDock = nullptr;
//
//
//  QGraphicsRectShapeSettingsDialogBox * roiOptionsDialogBox_ = nullptr;
//  QAction * showRoiOptionsAction = nullptr;
//  QAction * showRoiRectangleAction = nullptr;
//  QMenu roiActionsMenu_;
//
//
//  QAction * saveImageAsAction = nullptr;
//  QAction * saveDisplayImageAsAction = nullptr;
//  QAction * saveImageMaskAction = nullptr;
//  QAction * loadStackAction = nullptr;
//  QAction * setReferenceFrameAction = nullptr;
//  QAction * copyDisplayImageAction = nullptr;
//  QAction * copyDisplayViewportAction = nullptr;
//  QToolButton * editMaskAction = nullptr;
//  QAction * loadImageMaskAction = nullptr;
//  QAction * badframeAction = nullptr;
//  QAction * viewInputOptionsAction = nullptr;
//
//
//
//  QAction * selectPreviousFileAction_ = nullptr;
//  QAction * selectNextFileAction = nullptr;
//  QAction * reloadCurrentFileAction = nullptr;
//  QAction * showImageProcessorSettingsAction = nullptr;
  QAction * showCloudViewSettingsDialogBoxAction = nullptr;
  QGLViewPlanarGridSettingsDialogBox * glGridSettingsDialog = nullptr;
  QGlPointCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;

//
//  QLabel * currentFileNameLabel_ctl = nullptr;
//  QLabel * imageSizeLabel_ctl = nullptr;
//  QScaleSelectionButton * scaleSelection_ctl = nullptr;
//  QShapesButton * shapes_ctl = nullptr;
//
  QLabel * statusbarMousePosLabel_ctl = nullptr;
  QLabel * statusbarShapesLabel_ctl = nullptr;
  QToolButton * statusbarShowLog_ctl = nullptr;
//
//  ///
//  QDisplayVideoWriter diplayImageWriter_;
//  QToolButton* displayImageVideoWriterToolButton_ = nullptr;
//  bool lockDiplayImageWriter_ = false;
//
//  ///
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace qdso
#endif /* __qdso_main_window_h__ */
