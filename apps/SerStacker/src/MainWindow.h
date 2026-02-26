/*
 * MainWindow.h
 *
 *  Created on: Tuesday, December 1, 2020
 *      Author: amyznikov
 */
#pragma once
#ifndef __qserstacker_main_window_h__
#define __qserstacker_main_window_h__

#include <gui/mainwindow/QMainAppWindow.h>
#include <gui/qfilesystemtreeview/QFileSystemTreeDock.h>
#include <gui/qthumbnailsview/QThumbnailsView.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qglview/QGLPointCloudView.h>
#include <gui/qglview/QGLViewPlanarGridSettings.h>
#include <gui/qimageview/QImageViewOptions.h>
#include <gui/qgraphicsshape/QShapesButton.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include <gui/qpipeline/QPipelineOptionsView.h>
#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
#include <gui/qinputoptions/QInputOptions.h>
#include <gui/qinputsequencetreeview/QInputSequencesTreeView.h>

#include "QPipelineProgressView.h"
#include "QInputSourceView.h"
#include "QProgressImageViewer.h"
#include "QGeoMapView.h"
#include "QPointSelectionMode.h"

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
  void setupPipelineProgressView();
  void setupStatusbar();
  void setupGeoView();
  void showImageViewOptions(bool show);


private Q_SLOTS:
  void updateWindowTittle();
  void onLoadGpxTrack();
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
  void onCurrentViewFrameChanged();
  void onCurrentViewDisplayImageChanged();
  void onImageViewCurrentImageChanged();

  void onFileSystemTreeCustomContextMenuRequested(const QPoint & pos, const QFileInfoList &);
  void onThumbnailsViewCustomContextMenuRequested(const QPoint &pos);
  void onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);
  void onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source);

  void updateMeasureChannels();
  void updateMeasurements();
  void updateProfileGraph(QGraphicsItem * lineItem = nullptr) final;
  void onShowProfileGraphActionTriggered(bool checked) final;

  void onShowImageSequenceOptions(const c_image_sequence::sptr & sequence);
  void onPipelineThreadStarted();
  void onPipelineThreadFinished();

  void onShowCloudViewSettingsDialogBoxActionClicked(bool checked);
  void onShowCloudSettingsDialogBoxActionClicked(bool checked);

  void onOpenVideoFileRequested(const QString & filename, int scrollToIndex = -1);

  void toggleFindTextDialogBox(bool fshow);

  void saveCurrentWork();

private :
  void closeEvent(QCloseEvent *event) final;
  void onSaveState(QSettings & settings) final;
  void onRestoreState(QSettings & settings) final;
  void onMtfControlVisibilityChanged(bool visible) final;
  void onImageProcessorParameterChanged() final;
  void onDataframeProcessorParameterChanged() final;
  void onMeasureRightNowRequested() final;
  void onUpdateAvailableMeasureDataChannelsRequired() final;
  void saveShapes(QSettings & settings);
  void loadShapes(const QSettings & settings);
  IMtfDisplay * getCurrentMtfDisplay() final;

private:
  QStackedWidget * centralStackedWidget = nullptr;
  QThumbnailsView * thumbnailsView = nullptr;

  QInputSourceView * inputSourceView = nullptr;
  QImageSourceView * imageView = nullptr;
  QPointCloudSourceView * cloudView = nullptr;
  QTextSourceView * textView = nullptr;

  QPipelineProgressView * pipelineProgressView = nullptr;
  QProgressImageViewer * pipelineProgressImageView = nullptr;

  QInputOptionsDialogBox * inputOptionsDlgBox = nullptr;

  QImageViewOptionsDlgBox * imageViewOptionsDlgBox = nullptr;
  QPointCloudViewSettingsDialogBox * cloudViewSettingsDialogBox = nullptr;
  QGlPointCloudSettingsDialogBox * cloudSettingsDialogBox = nullptr;
  QPipelineOptionsView * pipelineOptionsView = nullptr;

  QFileSystemTreeDock * fileSystemTreeDock = nullptr;
  QInputSequencesTree * sequencesTreeView = nullptr;
  QInputSequenceTreeDock * sequencesTreeViewDock = nullptr;

  QGraphicsRectShapeSettingsDialogBox * _roiOptionsDialogBox = nullptr;
  QAction * showRoiOptionsAction = nullptr;
  QAction * showRoiRectangleAction = nullptr;
  QMenu _roiActionsMenu;


  QAction * quitAppAction = nullptr;
  QAction * saveImageAsAction = nullptr;
  QAction * saveDisplayImageAsAction = nullptr;
  QAction * saveImageMaskAction = nullptr;
  QAction * loadStackAction = nullptr;
  QAction * setReferenceFrameAction = nullptr;
  QAction * copyDisplayImageAction = nullptr;
  QAction * copyDisplayViewportAction = nullptr;
  QToolButton * editMaskAction = nullptr;
  QAction * loadImageMaskAction = nullptr;
  QAction * badframeAction = nullptr;
  QAction * viewInputOptionsAction = nullptr;
  QShortcut * badframeActionShortcut = nullptr;



  QAction * selectPreviousFileAction = nullptr;
  QAction * selectNextFileAction = nullptr;
  QAction * reloadCurrentFileAction = nullptr;
  QAction * showImageProcessorSettingsAction = nullptr;
  QAction * showCloudViewSettingsDialogBoxAction = nullptr;
  QAction * showCloudSettingsDialogBoxAction = nullptr;

  QLabel * currentFileNameLabel_ctl = nullptr;
  QLabel * imageSizeLabel_ctl = nullptr;
  QScaleSelectionButton * scaleSelection_ctl = nullptr;
  QShapesButton * shapes_ctl = nullptr;

  QLabel * statusbarMousePosLabel_ctl = nullptr;
  QLabel * statusbarShapesLabel_ctl = nullptr;
  QToolButton * statusbarShowLog_ctl = nullptr;

  ///
  QDisplayVideoWriter _diplayImageWriter;
  QToolButton* _displayImageVideoWriterToolButton = nullptr;
  bool _lockDiplayImageWriter = false;

  ///
  QGLViewPlanarGridSettingsDialogBox * _glGridSettingsDialog = nullptr;

  ///
  QGeoMapView * geoView = nullptr;
  QGeoMapViewDock * geoViewDock = nullptr;
  QAction * onLoadGpsTrackAction = nullptr;


  // QPointSelectionMode
  QPointSelection3DRulerMode _pointSelection3DRulerMode;
  QToolButton * _pointSelectionModeToolbutton = nullptr;
  QList<QAction *> _pointSelectionModeActions;
  QMenu _pointSelectionModeMenu;

  // QFindTextDialog
  QFindTextDialog * findTextDialog = nullptr;
  QAction* showFindTextDialogAction = nullptr;
  QShortcut* findNextShortcut = nullptr;
  QShortcut* findPrevShortcut = nullptr;
};


///////////////////////////////////////////////////////////////////////////////
}  // namespace serstacker
#endif /* __qserstacker_main_window_h__ */
