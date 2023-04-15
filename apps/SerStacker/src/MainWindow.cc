/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/QToolbarSpacer.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qimagesave/QImageSaveOptions.h>
#include <gui/qthumbnailsview/QThumbnails.h>
#include <gui/qpipelinethread/QImageProcessingPipeline.h>
#include <core/io/load_image.h>
#include <core/pipeline/c_image_stacking_pipeline.h>
#include <core/pipeline/c_camera_calibration_pipeline.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>
#include <core/pipeline/c_regular_stereo_pipeline.h>
#include <core/debug.h>

namespace serstacker {
///////////////////////////////////////////////////////////////////////////////


#define ICON_reload       ":/gui/icons/reload"
#define ICON_prev         ":/gui/icons/prev"
#define ICON_next         ":/gui/icons/next"
#define ICON_like         ":/gui/icons/like"
#define ICON_dislike      ":/gui/icons/dislike"
#define ICON_close        ":/gui/icons/close"
#define ICON_histogram    ":/gui/icons/histogram"
#define ICON_marker_blue  ":/gui/icons/marker-blue"
#define ICON_reference    ":/gui/icons/reference"
#define ICON_options      ":/gui/icons/options"
#define ICON_mask         ":/gui/icons/mask"
#define ICON_frame        ":/gui/icons/frame"
#define ICON_badframe     ":/gui/icons/badframe"

#define ICON_copy         ":/gui/icons/copy"
#define ICON_delete       ":/gui/icons/delete"

#define ICON_roi          ":/serstacker/icons/roi.png"

namespace  {

inline bool is_visible(QWidget * w)
{
  return w && w->isVisible();
}

}  // namespace


MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/serstacker/icons/app-icon.png"));
  updateWindowTittle();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(thumbnailsView = new QThumbnailsView(this));
  centralStackedWidget->addWidget(imageEditor = new QImageEditor(this));
  centralStackedWidget->addWidget(textViewer = new QTextFileViewer(this));
  centralStackedWidget->addWidget(pipelineOptionsView = new QPipelineOptionsView(this));
  centralStackedWidget->addWidget(cloudViewer = new QCloudViewer(this));

  connect(centralStackedWidget, &QStackedWidget::currentChanged,
      [this]() {
        if ( cloudViewSettingsDialogBox && cloudViewSettingsDialogBox->isVisible() ) {
          if ( !cloudViewer->isVisible() ) {
            cloudViewSettingsDialogBox->hide();
          }
        }
      });



  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  setupPipelineTypes();
  setupMainMenu();
  setupLogWidget();
  setupMtfControls();
  setupFileSystemTreeView();
  setupThumbnailsView();
  setupStackTreeView();
  setupStackOptionsView();
  setupImageProcessingControls();
  //setupImageProcessorSelector();
  setupImageEditor();
  setupTextViewer();
  stupCloudViewer();
  setupMeasures();
  setupRoiOptions();


  tabifyDockWidget(fileSystemTreeDock, sequencesTreeDock);
  //tabifyDockWidget(sequencesTreeDock, imageProcessorSelectorDock);
  tabifyDockWidget(sequencesTreeDock, imageProcessorDock_);


  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::started,
      this, &ThisClass::onStackingThreadStarted);

  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::finished,
      this, &ThisClass::onStackingThreadFinished);


  restoreState();

  imageEditor->set_current_processor(imageProcessor_ctl->current_processor());
  image_sequences_->load();
  sequencesTreeView->set_image_sequence_collection(image_sequences_);
}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("SerStacker");
}

void MainWindow::onSaveState(QSettings & settings)
{
  Base::onSaveState(settings);

  if( fileSystemTreeDock ) {
    settings.setValue("fileSystemTree/absoluteFilePath",
        fileSystemTreeDock->currentAbsoluteFilePath());
  }
}

void MainWindow::onRestoreState(QSettings & settings)
{
  Base::onRestoreState(settings);

  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->displayPath(settings.value(
        "fileSystemTree/absoluteFilePath").toString());
  }
}


void MainWindow::setupPipelineTypes()
{
  c_image_processing_pipeline::register_class(
      c_image_stacking_pipeline::class_name(),
      c_image_stacking_pipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new c_image_stacking_pipeline(name, input_sequence));
      });

  c_image_processing_pipeline::register_class(
      c_camera_calibration_pipeline::class_name(),
      c_camera_calibration_pipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new c_camera_calibration_pipeline(name, input_sequence));
      });

  c_image_processing_pipeline::register_class(
      c_stereo_calibration_pipeline::class_name(),
      c_stereo_calibration_pipeline::tooltip(),
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new c_stereo_calibration_pipeline(name, input_sequence));
      });

  c_image_processing_pipeline::register_class(
      c_regular_stereo_pipeline::class_name(),
      "c_rstereo_calibration_pipeline",
      [](const std::string & name, const c_input_sequence::sptr & input_sequence) {
        return c_image_processing_pipeline::sptr(new c_regular_stereo_pipeline(name, input_sequence));
      });

}

void MainWindow::setupMainMenu()
{
  Base::setupMainMenu();

  //
  // File
  //

  menuFile_->addAction(reloadCurrentFileAction_ =
      createAction(getIcon(ICON_reload),
          "Reload",
          "Reload current file from disk (Ctrl+R)",
          [this]() {
            QWidget * w = centralStackedWidget->currentWidget();
            if ( w == imageEditor ) {
              imageEditor->openImage(imageEditor->currentFileName());
            }
            else if ( w == textViewer ) {
              textViewer->showTextFile(textViewer->currentFileName());
            }
            else if ( w == thumbnailsView ) {
              thumbnailsView->reload();
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_R),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile_->addAction(selectPreviousFileAction_ =
      createAction(getIcon(ICON_prev),
          "Previous (Ctrl+PgUp)",
          "Select previous file (Ctrl+PgUp)",
          [this]() {
            thumbnailsView->selectPrevIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile_->addAction(selectNextFileAction_ =
      createAction(getIcon(ICON_next),
          "Next (Ctrl+PgDown)",
          "Select next file (Ctrl+PgUp)",
          [this]() {
            thumbnailsView->selectNextIcon();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown),
              this, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuFile_->addSeparator();

  saveImageAsAction =
      menuFile_->addAction("Save current image as...",
          this, &ThisClass::onSaveCurrentImageAs);

  saveImageAsAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentImage().empty());

  saveDisplayImageAsAction =
      menuFile_->addAction("Save current display image as...",
          this, &ThisClass::onSaveCurrentDisplayImageAs);

  saveDisplayImageAsAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentImage().empty());

  saveImageMaskAction =
      menuFile_->addAction("Save current image mask...",
          this, &ThisClass::onSaveCurrentImageMask);

  saveImageMaskAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentMask().empty());

  loadImageMaskAction =
      menuFile_->addAction("Set current image mask from file...",
          this, &ThisClass::onLoadCurrentImageMask);

  loadImageMaskAction->setEnabled(is_visible(imageEditor) &&
      !imageEditor->currentImage().empty());

  menuFile_->addSeparator();

  loadStackAction =
      menuFile_->addAction("Load stack config...",
          this, &ThisClass::onLoadStackConfig);


  menuFile_->addSeparator();

  quitAppAction =
      menuFile_->addAction("Quit",
          this, &ThisClass::close);


  //
  // Edit
  //

  //
  // View
  //
  menuView_->addAction(viewGeneralSettingsAction =
      createCheckableAction(QIcon(),
          "General Settings...",
          "Configure general app options",
          this, &ThisClass::onViewGeneralSettings));


  pipelineProgressView = new QPipelineProgressView(this);
  menuBar()->setCornerWidget(pipelineProgressView, Qt::TopRightCorner );
  pipelineProgressView->hide();

  connect(pipelineProgressView, &QPipelineProgressView::progressTextChanged,
      this, &ThisClass::onStackProgressViewTextChanged,
      Qt::QueuedConnection);

}

void MainWindow::onStackProgressViewTextChanged()
{
  // FIXME : this is ugly temporary hotfix

  const QSize hint = pipelineProgressView->sizeHint();
  const QSize size = pipelineProgressView->size();

  // CF_DEBUG("sizeHint: %dx%d size=%dx%d", hint.width(), hint.height(), size.width(), size.height());
  if( size != hint ) {
    menuBar()->adjustSize();
  }
}

void MainWindow::setupFileSystemTreeView()
{
  fileSystemTreeDock =
      addFileSystemTreeDock(this, Qt::LeftDockWidgetArea,
          "fileSystemTreeDock",
          "Directory Tree",
          menuView_);

  fileSystemTreeDock->raise();

  connect(fileSystemTreeDock, &QFileSystemTreeDock::currentDirectoryChanged,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if ( cloudViewer ) {
          cloudViewer->clear();
        }

        centralStackedWidget->setCurrentWidget(thumbnailsView);
        thumbnailsView->displayPath(abspath);
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::directoryItemPressed,
      [this](const QString & abspath) {

        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if ( cloudViewer ) {
          cloudViewer->clear();
        }

        if ( centralStackedWidget->currentWidget() != thumbnailsView ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
          if ( thumbnailsView->currentPath() != abspath ) {
            thumbnailsView->displayPath(abspath);
          }
        }
      });

  connect(fileSystemTreeDock, &QFileSystemTreeDock::customContextMenuRequested,
      this, &ThisClass::onFileSystemTreeCustomContextMenuRequested);

}

void MainWindow::setupThumbnailsView()
{

  connect(thumbnailsView, &QThumbnailsView::showInDirTreeRequested,
      [this](const QString & abspath) {
        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if (cloudViewer) {
          cloudViewer->clear();
        }
        if ( fileSystemTreeDock ) {
          fileSystemTreeDock->show(),
          fileSystemTreeDock->raise(),
          fileSystemTreeDock->displayPath(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::currentIconChanged,
      [this](const QString & abspath) {
        if ( pipelineProgressView ) {
          pipelineProgressView->setImageViewer(nullptr);
        }
        if ( imageEditor ) {
          imageEditor->clear();
        }
        if ( textViewer ) {
          textViewer->clear();
        }
        if (cloudViewer) {
          cloudViewer->clear();
        }
        if ( imageEditor && imageEditor->isVisible() ) {
          openImage(abspath);
        }
        else if ( textViewer && textViewer->isVisible() ) {
          openImage(abspath);
        }
        else if ( cloudViewer && cloudViewer->isVisible() ) {
          openImage(abspath);
        }
      });

  connect(thumbnailsView, &QThumbnailsView::iconDoubleClicked,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::iconEnterPressed,
      this, &ThisClass::openImage);

  connect(thumbnailsView, &QThumbnailsView::customContextMenuRequested,
      this, &ThisClass::onThumbnailsViewCustomContextMenuRequested);

}

void MainWindow::setupStackTreeView()
{
  sequencesTreeDock =
      addImageSequenceTreeDock(this,
          Qt::LeftDockWidgetArea,
          "sequencesTreeDock",
          "Sequences",
          menuView_);

  sequencesTreeView =
      sequencesTreeDock->treeView();

  connect(sequencesTreeView, &QImageSequenceTree::currentItemChanged,
      this, &ThisClass::onStackTreeCurrentItemChanged);

  connect(sequencesTreeView, &QImageSequenceTree::itemDoubleClicked,
      this, &ThisClass::onStackTreeItemDoubleClicked);

  connect(sequencesTreeView, &QImageSequenceTree::showImageSequenceOptionsClicked,
      this, &ThisClass::onShowImageSequenceOptions);

  connect(sequencesTreeView, &QImageSequenceTree::imageSequenceCollectionChanged,
        this, &ThisClass::saveCurrentWork );

  connect(sequencesTreeView, &QImageSequenceTree::imageSequenceSourcesChanged,
      [this](const c_image_sequence::sptr & sequence) {
        if ( pipelineOptionsView->isVisible() && pipelineOptionsView->current_sequence() == sequence ) {
          // fixme: temporary hack to force update options views
          pipelineOptionsView->set_current_sequence(sequence);
        }
        saveCurrentWork();
  });

  connect(sequencesTreeView, &QImageSequenceTree::imageSequenceNameChanged,
      this, &ThisClass::saveCurrentWork);

}

void MainWindow::setupStackOptionsView()
{
  connect(pipelineOptionsView, &QPipelineOptionsView::closeWindowRequested,
      [this]() {
        if ( !QImageProcessingPipeline::isRunning() ) {
          centralStackedWidget->setCurrentWidget(thumbnailsView);
        }
        else {
          centralStackedWidget->setCurrentWidget(imageEditor);
          pipelineProgressView->setImageViewer(imageEditor);
        }
      });

  connect(pipelineOptionsView, &QPipelineOptionsView::parameterChanged,
      this, &ThisClass::saveCurrentWork);

  connect(pipelineOptionsView, &QPipelineOptionsView::cloneCurrentPipelineRequested,
      [this]() {

        QWaitCursor wait (this);

        std::vector<c_image_sequence::sptr> selected_sequences;

        sequencesTreeView->getSelectedSequences(&selected_sequences);

        if ( !selected_sequences.empty() ) {
          if ( pipelineOptionsView->cloneCurrentPipeline(selected_sequences) ) {
            saveCurrentWork();
          }
        }
      });

}

void MainWindow::setupImageEditor()
{
  QToolBar *toolbar;
  QStatusBar *statusbar;

  toolbar = imageEditor->embedToolbar();
  statusbar = imageEditor->embedStatusbar();

  imageEditor->addAction(copyDisplayImageAction =
      createAction(QIcon(),
          "Copy display image to clipboard (Ctrl+c)",
          "Copy display image to clipboard (Ctrl+c)",
          [this]() {
            imageEditor->copyDisplayImageToClipboard();
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_C),
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  menuEdit_->addAction(copyDisplayImageAction);

  connect(imageEditor, &QImageEditor::currentImageChanged,
      this, &ThisClass::onImageEditorCurrentImageChanged);

  connect(imageEditor, &QImageViewer::visibilityChanged,
      this, &ThisClass::onImageEditorVisibilityChanged);


  ///
  /// Configure image editor toolbar
  ///

  static QIcon badframeIcon;
  if( badframeIcon.isNull() ) {
    badframeIcon.addPixmap(getPixmap(ICON_frame), QIcon::Normal, QIcon::Off);
    badframeIcon.addPixmap(getPixmap(ICON_badframe), QIcon::Normal, QIcon::On);
  }

  toolbar->addAction(selectPreviousFileAction_);
  toolbar->addAction(selectNextFileAction_);
  toolbar->addAction(reloadCurrentFileAction_);

  toolbar->addSeparator();

  toolbar->addAction(createAction(getIcon(ICON_dislike),
      "Bad",
      "Move current image to the .bads subfolder (Ctrl+DEL)",
      [this]() {
        thumbnailsView->moveToBads(imageEditor->currentFileName());
      },
      new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Delete),
          imageEditor, nullptr, nullptr,
          Qt::WindowShortcut)));

  toolbar->addAction(badframeAction =
      createCheckableAction(badframeIcon,
          "Bad Frame",
          "Mark / Unmark current frame as bad (Ctrl+A)",
          [this](bool checked) {
            if ( imageEditor->isVisible() ) {

              const c_input_sequence::sptr & input_sequence = imageEditor->input_sequence();
              if ( input_sequence ) {

                c_input_source::sptr source = input_sequence->current_source();
                if ( source ) {
                  source->set_badframe(input_sequence->current_pos() - 1, checked);
                  source->save_badframes();
                }
              }
            }
          },
          new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_A),
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  toolbar->addAction(setReferenceFrameAction =
      createAction(getIcon(ICON_reference),
          "Make reference",
          "Make this frame reference",
          [this]() {
            if ( imageEditor->isVisible() && sequencesTreeView->isVisible() ) {

              c_input_source::sptr selected_source;
              c_image_sequence::sptr selected_sequence;

              selected_source =
                  sequencesTreeView->getCurrentInputSource(
                      &selected_sequence);

              if ( selected_source && selected_sequence ) {

                const c_input_sequence::sptr & currentSequence =
                    imageEditor->input_sequence();

                if ( currentSequence && currentSequence->current_source()->filename() == selected_source->filename() ) {

                  const c_image_processing_pipeline::sptr pipeline =
                      selected_sequence->current_pipeline();

                  if ( pipeline ) {
                    pipeline->set_master_source(selected_source->filename());
                    pipeline->set_master_frame_index(currentSequence->current_pos() - 1);
                  }
                  else {

                    QMessageBox::warning(this, "warning",
                        qsprintf("No current pipeline is selected,\n"
                            "master frame is not assigned\n"
                            " %s: %d",
                            __FILE__, __LINE__) );
                  }
                }
              }
            }
          }));

  toolbar->addSeparator();

  toolbar->addWidget(imageNameLabel_ctl = new QLabel(""));
  imageNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  toolbar->addWidget(imageSizeLabel_ctl = new QLabel(""));
  imageSizeLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();
  toolbar->addWidget(new QToolbarSpacer());

  toolbar->addAction(editMaskAction =
      createCheckableAction(getIcon(ICON_mask),
          "Mask",
          "View / Edit image mask",
          [this](bool checked) {
            if ( imageEditor->isVisible() ) {

              if ( checked ) {
                setupImageViewOptions();
              }
              else if ( imageViewOptionsDlgBox ) {
                delete imageViewOptionsDlgBox;
                imageViewOptionsDlgBox = nullptr;
              }
            }
          }));

  toolbar->addWidget(shapes_ctl =
      new QShapesButton(imageEditor->sceneView(),
          this));

  toolbar->addAction(showMtfControlAction_);

  toolbar->addWidget(scaleSelection_ctl = new QScaleSelectionButton(this));
  scaleSelection_ctl->setScaleRange(QImageSceneView::MIN_SCALE, QImageSceneView::MAX_SCALE);
  connect(scaleSelection_ctl, &QScaleSelectionButton::scaleChanged,
      [this](int v) {
        imageEditor->setViewScale(v);
      });

  toolbar->addAction(closeImageViewAction_ =
      createAction(getIcon(ICON_close),
          "Close",
          "Close window",
          [this]() {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          },
          new QShortcut(QKeySequence::Cancel,
              imageEditor, nullptr, nullptr,
              Qt::WindowShortcut)));

  connect(imageEditor, &QImageFileEditor::onMouseMove,
      [this, statusbar](QMouseEvent * e) {
        statusbar->showMessage(imageEditor->statusStringForPixel(e->pos()));
      });

  connect(imageEditor->scene(), &QImageScene::graphicsItemChanged,
      [this, statusbar](QGraphicsItem * item) {

        QGraphicsLineShape * lineShape = nullptr;
        QGraphicsRectShape * rectShape = nullptr;

        if ( (lineShape = dynamic_cast<QGraphicsLineShape * >(item)) ) {

          const QLineF line = lineShape->sceneLine();

          const QPointF p1 = line.p1();
          const QPointF p2 = line.p2();
          const double length = hypot(p2.x()-p1.x(), p2.y()-p1.y());
          const double angle = atan2(p2.y()-p1.y(), p2.x()-p1.x());

          statusbar->showMessage(qsprintf("p1: (%g %g)  p2: (%g %g)  length: %g  angle: %g deg",
                  p1.x(), p1.y(), p2.x(), p2.y(), length, angle * 180 / M_PI));

        }
        else if ( (rectShape = dynamic_cast<QGraphicsRectShape* >(item))) {

          const QRectF rc = rectShape->mapToScene(rectShape->rect()).boundingRect();

          const QPointF p1 = rc.topLeft();
          const QPointF p2 = rc.bottomRight();
          const QPointF center = rc.center();
          const double width = rc.width();
          const double height = rc.height();

          statusbar->showMessage(
              qsprintf("RECT: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                  p1.x(), p1.y(),
                  p2.x(), p2.y(),
                  width, height,
                  center.x(), center.y()));

        }
      });

}


void MainWindow::setupTextViewer()
{
  QToolBar *toolbar;
  QLabel *imageNameLabel;
  QLabel *imageSizeLabel;

  toolbar = textViewer->toolbar();

  toolbar->addAction(selectPreviousFileAction_);
  toolbar->addAction(selectNextFileAction_);
  toolbar->addAction(reloadCurrentFileAction_);

  toolbar->addSeparator();

  toolbar->addWidget(imageNameLabel = new QLabel(""));
  imageNameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  const auto onTextViewerCurrentSourceChanged =
      [this, imageNameLabel]() {

        const QString abspath = textViewer->currentFileName();
        imageNameLabel->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());

      };

  connect(textViewer, &QTextFileViewer::currentFileNameChanged,
      onTextViewerCurrentSourceChanged);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());

  toolbar->addAction(createAction(getIcon(ICON_close),
      "Close",
      "Close window",
      [this]() {
        textViewer->clear();
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      },
      new QShortcut(QKeySequence::Cancel,
          textViewer, nullptr, nullptr,
          Qt::WindowShortcut)));

}


void MainWindow::onImageEditorCurrentImageChanged()
{
  const bool isvisible =
      imageEditor->isVisible();

  onImageEditorVisibilityChanged(isvisible);

  if ( isvisible ) {

    const QString abspath =
        imageEditor->currentFileName();

    imageNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
    imageSizeLabel_ctl->setText(QString("%1x%2").arg(imageEditor->currentImage().cols).arg(imageEditor->currentImage().rows));

    if ( is_visible(mtfControl_) ) {

      if( mtfControl_->mtfDisplaySettings() != imageEditor->mtfDisplayFunction() ) {
        mtfControl_->setMtfDisplaySettings(imageEditor->mtfDisplayFunction());
      }

      if ( imageEditor->currentFileName().isEmpty() ) {
        mtfControl_->setWindowTitle("Adjust Display Levels ...");
      }
      else {
        mtfControl_->setWindowTitle(QFileInfo(imageEditor->currentFileName()).fileName());
      }
    }

    updateMeasurements();
  }
}

void MainWindow::onImageProcessorParameterChanged()
{
  Base::onImageProcessorParameterChanged();
  imageEditor->set_current_processor(imageProcessor_ctl->current_processor());
}

void MainWindow::onMtfControlVisibilityChanged(bool visible)
{
  Base::onMtfControlVisibilityChanged(visible);

  if( visible && is_visible(imageEditor) ) {
    mtfControl_->setMtfDisplaySettings(imageEditor->mtfDisplayFunction());
  }
}


void MainWindow::onImageEditorVisibilityChanged(bool isvisible)
{
  const bool hasimage =
      !imageEditor->currentImage().empty();

  const bool hasmask =
      !imageEditor->currentMask().empty();

  const bool hasdisplayimage =
      !imageEditor->displayImage().empty();

  saveImageAsAction->setEnabled(isvisible && hasimage);
  saveDisplayImageAsAction->setEnabled(isvisible && hasdisplayimage);
  copyDisplayImageAction->setEnabled(isvisible && hasdisplayimage);
  saveImageMaskAction->setEnabled(isvisible && hasmask);
  loadImageMaskAction->setEnabled(isvisible && hasimage);

  bool check_badframe_action = false;

  if( isvisible ) {

    const c_input_sequence::sptr &input_sequence =
        imageEditor->input_sequence();

    if( input_sequence ) {

      c_input_source::sptr source =
          input_sequence->current_source();

      if( source ) {
        check_badframe_action =
            source->is_badframe(input_sequence->current_pos() - 1);
      }
    }
  }

  if( badframeAction->isChecked() != check_badframe_action ) {
    badframeAction->setChecked(check_badframe_action);
  }
}


void MainWindow::stupCloudViewer()
{
  QToolBar * toolbar;
  QAction * action;
  QLabel * imageNameLabel_ctl;
  QLabel * imageSizeLabel_ctl;
  QShortcut * shortcut;

  toolbar = cloudViewer->toolbar();

  toolbar->addAction(action = new QAction(getIcon(ICON_prev), "Previous"));
  action->setToolTip("Load previous image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectPrevIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_next), "Next"));
  action->setToolTip("Load next image from list");
  action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  connect(action, &QAction::triggered, [this]() {
    thumbnailsView->selectNextIcon();
  });


  toolbar->addAction(action = new QAction(getIcon(ICON_reload), "Reload"));
  action->setToolTip("Reaload current image from disk");
  connect(action, &QAction::triggered, [this]() {
    QWaitCursor wait(this);
    cloudViewer->openPlyFile(cloudViewer->currentFileName());
  });


  toolbar->addSeparator();


  toolbar->addWidget(imageNameLabel_ctl = new QLabel(""));
  imageNameLabel_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse);

  toolbar->addSeparator();

  const auto onCloudViewerCurrentSourceChanged =
      [this, imageNameLabel_ctl] () {
        const QString abspath = cloudViewer->currentFileName();
        imageNameLabel_ctl->setText(abspath.isEmpty() ? "" : QFileInfo(abspath).fileName());
      };

  connect(cloudViewer, &QCloudViewer::currentFileNameChanged,
      onCloudViewerCurrentSourceChanged);

  toolbar->addSeparator();

  toolbar->addWidget(new QToolbarSpacer());



  toolbar->addAction(action = new QAction(getIcon(ICON_options), "Options"));
  action->setToolTip("Configure cloud view options");
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, &QAction::triggered,
      [this, action](bool checked) {

        if ( checked && !cloudViewSettingsDialogBox ) {

          cloudViewSettingsDialogBox = new QCloudViewSettingsDialogBox(this);
          cloudViewSettingsDialogBox->setCloudViewer(cloudViewer);
          connect(cloudViewSettingsDialogBox, &QCloudViewSettingsDialogBox::visibilityChanged,
              action, &QAction::setChecked);
        }

        if ( cloudViewSettingsDialogBox ) {
          if ( !checked ) {
            cloudViewSettingsDialogBox->hide();
          }
          else {
            cloudViewSettingsDialogBox->setWindowTitle(QFileInfo(cloudViewer->currentFileName()).fileName());
            cloudViewSettingsDialogBox->showNormal();
          }
        }
      });


  toolbar->addAction(action = new QAction(getIcon(ICON_close), "Close"));
  action->setShortcut(QKeySequence::Cancel);
  action->setToolTip("Close window");
  connect(action, &QAction::triggered, [this]() {
    cloudViewer->clear();
    centralStackedWidget->setCurrentWidget(thumbnailsView);
  });
}

void MainWindow::updateMeasurements()
{
  if( !QMeasureProvider::requested_measures().empty() && imageEditor->roiRectShape()->isVisible() ) {

    QImageViewer::current_image_lock lock(imageEditor);
    if ( !imageEditor->currentImage().empty() ) {

      QMeasureProvider::compute(imageEditor->currentImage(),
          imageEditor->currentMask(),
          imageEditor->roiRectShape()->iSceneRect());

    }
  }
}

void MainWindow::setupRoiOptions()
{
  QAction * action;

  ///

  roiOptionsDialogBox_ =
      new QGraphicsRectShapeSettingsDialogBox("ROI rectangle options",
          imageEditor->roiRectShape(),
          this);

  roiOptionsDialogBox_->loadParameters();

  roiActionsMenu_.addAction(action =
      createCheckableAction(QIcon(),
          "ROI Options..",
          "Configure ROI rectangle options",
          [this](bool checked) {
            roiOptionsDialogBox_->setVisible(checked);
          }));

  connect(roiOptionsDialogBox_, &QGraphicsRectShapeSettingsDialogBox::visibilityChanged,
      [this, action](bool visible) {
        action->setChecked(visible);
        if ( visible ) {
          imageEditor->roiRectShape()->setVisible(true);
        }
      });

  roiActionsMenu_.addAction(showMeasuresSettingsAction_);
  roiActionsMenu_.addAction(showMeasuresDisplayAction_);
  roiActionsMenu_.addAction(showMeasuresGraphAction_);

  ///

  connect(imageEditor->roiRectShape(), &QGraphicsShape::itemChanged,
      [this]() {

        QGraphicsRectShape * shape =
            imageEditor->roiRectShape();

        const QRectF rc =
            shape->sceneRect();

        const QPointF p1 = rc.topLeft();
        const QPointF p2 = rc.bottomRight();
        const QPointF center = rc.center();
        const double width = rc.width();
        const double height = rc.height();

        imageEditor->statusbar()->showMessage(
            qsprintf("ROI: p1=(%g %g) p2=(%g %g) size=(%g x %g) center=(%g %g)",
                p1.x(), p1.y(),
                p2.x(), p2.y(),
                width, height,
                center.x(), center.y()));

        updateMeasurements();
      });


  ///

  QToolBar * toolbar = imageEditor->toolbar();
  if ( toolbar ) {

    showRoiAction_ =
        createCheckableAction(getIcon(ICON_roi),
            "ROI Rectangle",
            "Show / Hide ROI rectangle",
            [this](bool checked) {
              imageEditor->roiRectShape()->setVisible(checked);
            });

    toolbar->insertWidget(closeImageViewAction_,
        roiActionsButton_ =
            createToolButtonWithPopupMenu(showRoiAction_,
                &roiActionsMenu_));

    connect(imageEditor->roiRectShape(), &QGraphicsObject::visibleChanged,
        [this]() {
          showRoiAction_->setChecked(imageEditor->roiRectShape()->isVisible());
        });

    showRoiAction_->setChecked(imageEditor->roiRectShape()->isVisible());
  }

}

void MainWindow::setupImageViewOptions()
{
  if( !imageViewOptionsDlgBox ) {

    imageViewOptionsDlgBox = new QImageViewOptionsDlgBox(this);
    imageViewOptionsDlgBox->setImageViewer(imageEditor);

    connect(imageViewOptionsDlgBox, &QImageViewOptionsDlgBox::visibilityChanged,
        [this](bool visible) {
          if ( editMaskAction->isChecked() != visible ) {
            editMaskAction->setChecked(visible);
          }
        });

    connect(imageViewOptionsDlgBox, &QImageViewOptionsDlgBox::finished,
        [this](int) {
          delete imageViewOptionsDlgBox;
          imageViewOptionsDlgBox = nullptr;
        });

    imageViewOptionsDlgBox->show();
  }
}


void MainWindow::openImage(const QString & abspath)
{
  QWaitCursor wait(this);

  if ( pipelineProgressView ) {
    pipelineProgressView->setImageViewer(nullptr);
  }

  imageEditor->clear();
  textViewer->clear();
  cloudViewer->clear();

  const QString suffix =
      QFileInfo(abspath).suffix();

  if ( isTextFileSuffix(suffix) ) {
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
  }
  else if ( isPlyFileSuffix(suffix) ) {
#if 1
    if ( cloudViewer->openPlyFile(abspath) ) {
      centralStackedWidget->setCurrentWidget(cloudViewer);
    }
    else {
      centralStackedWidget->setCurrentWidget(textViewer);
      textViewer->showTextFile(abspath);
    }
#else
    centralStackedWidget->setCurrentWidget(textViewer);
    textViewer->showTextFile(abspath);
#endif
  }
  else {
    centralStackedWidget->setCurrentWidget(imageEditor);
    imageEditor->openImage(abspath);
  }
}



void MainWindow::onFileSystemTreeCustomContextMenuRequested(const QPoint & pos,
    const QFileInfoList & selectedItems )
{
  QMenu menu;
  QAction * act;
  QString path, name;


  if ( fileSystemTreeDock ) {
    fileSystemTreeDock->fillContextMenu(menu, selectedItems);
  }

  menu.addSeparator();

//  QFileInfo curretPath(path = fileSystemTreeDock->currentAbsoluteFilePath());
//  if ( curretPath.isDir() && !wkspace->batches((name = curretPath.fileName()).toStdString()) ) {
//    menu.addAction(act = new QAction("Create batch here..."));
//    connect(act, &QAction::triggered, [this, path, name]() {
//      this->addBatch(path, name);
//    });
//  }

  if ( !menu.isEmpty() ) {
    menu.exec(pos);
  }
}



void MainWindow::onThumbnailsViewCustomContextMenuRequested(const QPoint & pos)
{
  QMenu poupupMenu;
  thumbnailsView->populateContextMenu(&poupupMenu, pos);
  if( !poupupMenu.isEmpty() ) {
    poupupMenu.exec(thumbnailsView->contextMenuPosToGlobal(pos));
  }
}

void MainWindow::onStackTreeCurrentItemChanged(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source)
{
  if ( source ) {
    centralStackedWidget->setCurrentWidget(imageEditor);
    imageEditor->openImage(source->filename());
  }
  else if ( sequence ) {
    if ( centralStackedWidget->currentWidget() == thumbnailsView ) {
      thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
    }
    else {
      pipelineOptionsView->set_current_sequence(sequence);
      centralStackedWidget->setCurrentWidget(pipelineOptionsView);
    }
  }

}

void MainWindow::onStackTreeItemDoubleClicked(const c_image_sequence::sptr & sequence, const c_input_source::sptr & source)
{
  if ( QImageProcessingPipeline::isRunning() ) {

    if ( source ) {
      pipelineProgressView->setImageViewer(nullptr);
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(source->filename());
    }
    else if ( sequence ) {

      QWidget * currentCentralWidget =
          centralStackedWidget->currentWidget();

      if ( sequence == QImageProcessingPipeline::current_sequence() ) {

        if ( currentCentralWidget == imageEditor ) {

          if ( !pipelineProgressView->imageViewer() ) {
            pipelineProgressView->setImageViewer(imageEditor);
          }
          else {
            pipelineProgressView->setImageViewer(nullptr);
            pipelineOptionsView->set_current_sequence(sequence);
            centralStackedWidget->setCurrentWidget(pipelineOptionsView);
          }

        }
        else if ( currentCentralWidget == pipelineOptionsView ) {
          if ( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
          else {
            centralStackedWidget->setCurrentWidget(imageEditor);
            pipelineProgressView->setImageViewer(imageEditor);
          }
        }
        else {
          centralStackedWidget->setCurrentWidget(imageEditor);
          pipelineProgressView->setImageViewer(imageEditor);
        }
      }
      else {

        if ( currentCentralWidget == pipelineOptionsView ) {
          if ( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
            centralStackedWidget->setCurrentWidget(thumbnailsView);
          }
        }
        else {
          pipelineOptionsView->set_current_sequence(sequence);
          centralStackedWidget->setCurrentWidget(pipelineOptionsView);
        }
      }
    }

  }
  else if ( source ) {
    if ( centralStackedWidget->currentWidget() == imageEditor ) {
      centralStackedWidget->setCurrentWidget(thumbnailsView);
      thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false);
    }
    else {
      centralStackedWidget->setCurrentWidget(imageEditor);
      imageEditor->openImage(source->filename());
    }
  }
  else if ( sequence ) {
    if ( centralStackedWidget->currentWidget() == pipelineOptionsView ) {
      if( thumbnailsView->setCurrentPath(sequence->get_display_path().c_str(), false) ) {
        centralStackedWidget->setCurrentWidget(thumbnailsView);
      }
    }
    else {
      pipelineOptionsView->set_current_sequence(sequence);
      centralStackedWidget->setCurrentWidget(pipelineOptionsView);
    }
  }

}


void MainWindow::onShowImageSequenceOptions(const c_image_sequence::sptr & sequence)
{
  if ( sequence ) {

    if ( pipelineProgressView ) {
      pipelineProgressView->setImageViewer(nullptr);
    }

    pipelineOptionsView->set_current_sequence(sequence);
    centralStackedWidget->setCurrentWidget(pipelineOptionsView);
  }

}

void MainWindow::onStackingThreadStarted()
{
  if ( centralStackedWidget->currentWidget() != imageEditor ) {
    centralStackedWidget->setCurrentWidget(imageEditor);
  }

  imageEditor->clear();
  pipelineProgressView->setImageViewer(imageEditor);

  if ( !pipelineProgressView->isVisible() ) {
    pipelineProgressView->show();
  }
}

void MainWindow::onStackingThreadFinished()
{
  if ( pipelineProgressView ) {

    // it may be that there is next task in queue,
    // don't blink with this dialog box
    QTimer::singleShot(1000,
        [this]() {

          if ( !QImageProcessingPipeline::isRunning() ) {
            if ( pipelineProgressView->isVisible() ) {
              pipelineProgressView->hide();
            }
            if ( pipelineOptionsView->isVisible() ) {
              pipelineOptionsView->setEnabled(true);
            }
          }
        });
  }
}

void MainWindow::onSaveCurrentImageAs()
{
  if ( !imageEditor->isVisible() || imageEditor->currentImage().empty() ) {
    return;
  }

  QString savedFileName = saveImageFileAs(this,
      imageEditor->currentImage(),
      imageEditor->currentMask(),
      imageEditor->current_processor(),
      imageEditor->currentFileName());

  if ( !savedFileName.isEmpty() ) {
    imageEditor->setCurrentFileName(savedFileName);
  }
}

void MainWindow::onSaveCurrentDisplayImageAs()
{
  if( imageEditor->isVisible() && !imageEditor->displayImage().empty() ) {

    saveImageFileAs(this,
        imageEditor->displayImage(),
        cv::Mat(),
        nullptr,
        imageEditor->currentFileName());
  }
}

void MainWindow::onSaveCurrentImageMask()
{
  if( imageEditor->isVisible() && !imageEditor->currentMask().empty() ) {

    saveImageFileAs(this,
        imageEditor->currentMask(),
        cv::Mat(),
        nullptr,
        QString("%1.mask.png").arg(imageEditor->currentFileName()));
  }
}

void MainWindow::onLoadCurrentImageMask()
{
  if( imageEditor->isVisible() && !imageEditor->currentImage().empty() ) {

    static const QString keyName = "lastImageMask";

    QSettings settings;

    static const QString filter =
        "Image files *.tiff *.tif *.png *.jpg (*.tiff *.tif *.png *.jpg);;\n"
        "All files (*);;";

    QString fileName =
        QFileDialog::getOpenFileName(this,
            "Select binary image", settings.value(keyName).toString(),
            filter,
            nullptr);

    if ( fileName.isEmpty() ) {
      return;
    }

    settings.setValue(keyName, fileName);

    cv::Mat image;

    if ( !load_image(fileName.toStdString(), image) ) {
      QMessageBox::critical(this, "Error",
          "load_image() fails.\n"
          "Can not load image from specified file");
      return;
    }

    if( image.type() != CV_8UC1 || image.size() != imageEditor->currentImage().size() ) {
      QMessageBox::critical(this, "Error",
          QString("Not appropriate mask image: %1x%2 depth=%3 channels=%4.\n"
              "Must be CV_8UC1 of the same size as current image (%5x%6)")
              .arg(image.cols)
              .arg(image.rows)
              .arg(image.depth())
              .arg(image.channels())
              .arg(imageEditor->currentImage().cols)
              .arg(imageEditor->currentImage().rows));
      return;
    }

    imageEditor->setMask(image, false);

  }
}


void MainWindow::saveCurrentWork()
{
  QWaitCursor wait (this);

  if( image_sequences_->indexof(pipelineOptionsView->current_sequence()) < 0 ) {
    pipelineOptionsView->set_current_sequence(nullptr);
  }

  image_sequences_->save();
}

void MainWindow::onLoadStackConfig()
{
  static const QString loadStackConfigSavedPathKeyName =
      "loadStackConfigSavedPath";

  QSettings settings;

  QString savedPathFileName =
      settings.value(loadStackConfigSavedPathKeyName).toString();

  const QString filter =
      "Config files (*.cfg) ;;"
      "All files (*.*)";

  QStringList selectedFileNames =
      QFileDialog::getOpenFileNames(this,
          "Select stack config files",
          savedPathFileName,
          filter,
          nullptr,
          QFileDialog::ReadOnly);


  if ( selectedFileNames.isEmpty() ) {
    return;
  }

  settings.setValue(loadStackConfigSavedPathKeyName,
      selectedFileNames[0]);

  bool hasChanges = false;

  for ( int i = 0, n = selectedFileNames.size(); i < n; ++i ) {

    c_image_sequence::sptr sequence =
        c_image_sequence::load(selectedFileNames[i].toStdString());

    if ( !sequence ) {

      if ( i == n - 1 ) {
        QMessageBox::critical(this,
            "ERROR",
            QString("Can not load %1.\nSee error log for details.").arg(selectedFileNames[i]));
        break;
      }

      const int responce =
          QMessageBox::critical(this, "ERROR",
              QString("Can not load %1.\n"
                  "See error log for details.\n"
                  "Continue loading ?").arg(selectedFileNames[i]),
              QMessageBox::Yes | QMessageBox::No);

      if ( responce != QMessageBox::Yes ) {
        break;
      }

      continue;
    }


    int pos = image_sequences_->indexof(sequence->name());
    if ( pos < 0 ) {
      image_sequences_->add(sequence);
      hasChanges = true;
    }
    else {

      const int responce =
          QMessageBox::critical(this, "ERROR",
              QString("Stack with name '%1' already exists.\n"
                  "Replace existing ?").arg(QString(sequence->cname())),
              QMessageBox::Yes | QMessageBox::No);

      if ( responce == QMessageBox::Yes  ) {
        image_sequences_->set(pos, sequence);
        hasChanges = true;
      }
    }
  }

  if ( hasChanges ) {
    sequencesTreeView->refresh();
  }
}

void MainWindow::onViewGeneralSettings()
{
  if( !appSettingsDlgBox ) {
    appSettingsDlgBox = new QGeneralAppSettingsDialogBox(this);
    appSettingsDlgBox->setImageEditor(imageEditor);
    connect(appSettingsDlgBox, &QGeneralAppSettingsDialogBox::visibilityChanged,
        viewGeneralSettingsAction, &QAction::setChecked);
  }

  if ( !appSettingsDlgBox->isVisible() ){
    appSettingsDlgBox->show();
  }
  else {
    appSettingsDlgBox->hide();
  }
}


///////////////////////////////////////////////////////////////////////////////
}  // namespace qskystacker
